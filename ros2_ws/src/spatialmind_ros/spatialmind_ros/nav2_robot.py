"""Async Nav2 ActionClient wrapper using a separate rclpy executor thread.

ROS 2 nodes can run beside asyncio without nested spins/deadlocks.
Requires localization and actual perception topics before missions start.
"""
from __future__ import annotations

import asyncio
import json
import threading
import time

from spatialmind.physical import MetricObservation, MotionResult, ObjectEstimate, Pose2D


class Nav2MetricRobot:
    def __init__(self, *, map_frame="map", observation_topic="/spatialmind/observation_json",
                 navigation_action="navigate_to_pose", observation_timeout_s=5.0):
        import rclpy
        from rclpy.action import ActionClient
        from rclpy.executors import MultiThreadedExecutor
        from nav2_msgs.action import NavigateToPose
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import Odometry
        from std_msgs.msg import String
        if not rclpy.ok():
            rclpy.init()
        self.rclpy=rclpy
        self.GoalStatus=GoalStatus
        self.NavigateToPose=NavigateToPose
        self.node=rclpy.create_node("spatialmind_metric_robot")
        self.action=ActionClient(self.node,NavigateToPose,navigation_action)
        self.map_frame=map_frame
        self.timeout_s=observation_timeout_s
        self._pose=Pose2D(0,0,frame_id=map_frame)
        self._localized=False
        self._goal=None
        self._feedback_distance=None
        self._odom_previous=None
        self._odom_distance_m=0.0
        self._odom_count=0
        self._last_nav_finished=0.0
        self._observations=[]
        self._observation_lock=threading.Lock()
        self.node.create_subscription(PoseWithCovarianceStamped,
                                      "/amcl_pose",self._on_pose,10)
        self.node.create_subscription(String,observation_topic,self._on_obs,10)
        self.node.create_subscription(Odometry,"/odom",self._on_odom,10)
        self.executor=MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.node)
        self.thread=threading.Thread(target=self.executor.spin,daemon=True)
        self.thread.start()

    @property
    def pose(self):
        return self._pose

    def _on_pose(self,msg):
        if msg.header.frame_id!=self.map_frame:
            return
        position=msg.pose.pose.position
        orientation=msg.pose.pose.orientation
        import math
        yaw=math.atan2(2*(orientation.w*orientation.z+orientation.x*orientation.y),
                        1-2*(orientation.y*orientation.y+orientation.z*orientation.z))
        variance=max(0.,msg.pose.covariance[0]+msg.pose.covariance[7])
        self._pose=Pose2D(position.x,position.y,yaw,self.map_frame,
                          time.time(),variance)
        self._localized=True

    def _on_odom(self,msg):
        p=msg.pose.pose.position
        current=(p.x,p.y)
        if self._odom_previous is not None:
            import math
            delta=math.hypot(current[0]-self._odom_previous[0],
                             current[1]-self._odom_previous[1])
            if delta<=1.0:
                self._odom_distance_m+=delta
                self._odom_count+=1
        self._odom_previous=current

    def _on_obs(self,msg):
        try:
            payload=json.loads(msg.data)
            if payload.get("frame_id",self.map_frame)!=self.map_frame:
                return
            stamp=float(payload.get("stamp",time.time()))
            det=tuple(ObjectEstimate(
                item["label"],
                Pose2D(item["x"],item["y"],frame_id=self.map_frame,
                       stamp=stamp,
                       position_variance=float(item.get("variance",0.05))),
                float(item["confidence"]),
                item.get("track_id"),item.get("evidence_ref"),
            ) for item in payload["detections"])
            localized=Pose2D(self._pose.x,self._pose.y,self._pose.yaw,
                             self.map_frame,stamp,self._pose.position_variance)
            obs=MetricObservation(
                localized,det,
                frozenset(tuple(c) for c in payload.get("visible_cells",[])),
                float(payload.get("coverage_quality",0)),source="ros-rgbd",
            )
            with self._observation_lock:
                self._observations.append((time.monotonic(),obs))
                self._observations=self._observations[-3:]
        except (ValueError,TypeError,KeyError):
            self.node.get_logger().warning("Rejected malformed observation payload")

    async def _rclpy_future(self,future,timeout_s:float):
        loop=asyncio.get_running_loop()
        wrapped=loop.create_future()
        def callback(done):
            try:
                result=done.result()
            except Exception as exc:
                loop.call_soon_threadsafe(lambda: not wrapped.done() and wrapped.set_exception(exc))
            else:
                loop.call_soon_threadsafe(lambda: not wrapped.done() and wrapped.set_result(result))
        future.add_done_callback(callback)
        return await asyncio.wait_for(wrapped,timeout=timeout_s)

    async def navigate(self,target:Pose2D,*,timeout_s:float=120):
        if target.frame_id!=self.map_frame:
            return MotionResult(False,"frame_mismatch")
        if not self._localized or self._pose.position_variance>2:
            return MotionResult(False,"localization_unavailable")
        start=self.pose
        started=time.monotonic()
        baseline_odom=self._odom_distance_m
        ready=await asyncio.to_thread(self.action.wait_for_server,timeout_sec=5)
        if not ready:
            return MotionResult(False,"nav2_action_unavailable")
        goal=self.NavigateToPose.Goal()
        goal.pose.header.frame_id=self.map_frame
        goal.pose.header.stamp=self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x=target.x
        goal.pose.pose.position.y=target.y
        import math
        goal.pose.pose.orientation.z=math.sin(target.yaw/2)
        goal.pose.pose.orientation.w=math.cos(target.yaw/2)
        try:
            handle=await self._rclpy_future(self.action.send_goal_async(
                goal,feedback_callback=self._on_feedback),5)
            if handle is None or not handle.accepted:
                return MotionResult(False,"goal_rejected")
            self._goal=handle
            answer=await self._rclpy_future(handle.get_result_async(),timeout_s)
            success=answer.status==self.GoalStatus.STATUS_SUCCEEDED
            near_goal=self.pose.distance(target)<=0.65
            reason="arrived" if success and near_goal else (
                "pose_verification_failed" if success else "nav2_failed")
            return MotionResult(success and near_goal,reason,
                                time.monotonic()-started,
                                max(0,self._odom_distance_m-baseline_odom)
                                if self._odom_count else start.distance(self.pose),
                                str(handle.goal_id))
        except asyncio.TimeoutError:
            await self.stop()
            return MotionResult(False,"navigation_timeout",time.monotonic()-started,
                                max(0,self._odom_distance_m-baseline_odom)
                                if self._odom_count else start.distance(self.pose))
        finally:
            self._last_nav_finished=time.monotonic()
            self._goal=None

    def _on_feedback(self,message):
        feedback=getattr(message,"feedback",None)
        if feedback:
            self._feedback_distance=getattr(feedback,"distance_remaining",None)

    async def observe(self):
        started=time.monotonic()
        while time.monotonic()-started<self.timeout_s:
            with self._observation_lock:
                observations=list(self._observations)
            if observations:
                at,obs=observations[-1]
                if (at>=self._last_nav_finished
                        and time.monotonic()-at<self.timeout_s
                        and obs.pose.distance(self.pose)<0.75):
                    return obs
            await asyncio.sleep(.05)
        raise TimeoutError("RGB-D perception stream not available or stale")

    async def stop(self):
        handle=self._goal
        if handle is not None:
            try:
                await self._rclpy_future(handle.cancel_goal_async(),3)
            except (asyncio.TimeoutError,Exception):
                self.node.get_logger().warning("Unable to confirm Nav2 cancellation")

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        self.thread.join(timeout=2)
