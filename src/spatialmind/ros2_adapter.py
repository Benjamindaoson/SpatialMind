"""Optional ROS 2/Nav2 transport adapter.

Requires a running ROS 2 Jazzy environment and a separate perception node
publishing spatialmind/observation_json. This is an integration boundary,
not an RGB-D or Gazebo end-to-end implementation.
"""
from __future__ import annotations
import json
from typing import Any
from spatialmind.models import Detection, NavigationResult, Observation, Point

class Nav2RobotAdapter:
    """Synchronous Nav2 NavigateToPose action client; not a safety controller."""
    def __init__(self, *, resolution: float = 1.0, origin_x: float = 0.0,
                 origin_y: float = 0.0, frame_id: str = "map",
                 timeout_sec: float = 60.0) -> None:
        try:
            import rclpy
            from action_msgs.msg import GoalStatus
            from geometry_msgs.msg import PoseWithCovarianceStamped
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            from std_msgs.msg import String
        except ImportError as exc:
            raise RuntimeError(
                "ROS 2 missing. Source /opt/ros/jazzy/setup.bash "
                "and install nav2_msgs, action_msgs, geometry_msgs and std_msgs."
            ) from exc
        if resolution <= 0 or timeout_sec <= 0:
            raise ValueError("resolution and timeout must be positive")
        self.rclpy, self.GoalStatus, self.NavigateToPose = rclpy, GoalStatus, NavigateToPose
        self.resolution, self.origin_x, self.origin_y = resolution, origin_x, origin_y
        self.frame_id, self.timeout_sec = frame_id, timeout_sec
        if not rclpy.ok():
            rclpy.init()
        self.node = rclpy.create_node("spatialmind_nav2_adapter")
        self.action = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        self._pose = Point(0, 0)
        self._observation: Observation | None = None
        self._goal_handle: Any = None
        self._amcl_sub = self.node.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_pose, 10)
        self._obs_sub = self.node.create_subscription(
            String, "/spatialmind/observation_json", self._on_observation, 10)

    @property
    def pose(self) -> Point:
        return self._pose

    def _grid_point(self, x: float, y: float) -> Point:
        return Point(round((x - self.origin_x) / self.resolution),
                     round((y - self.origin_y) / self.resolution))

    def _on_pose(self, msg: Any) -> None:
        p = msg.pose.pose.position
        self._pose = self._grid_point(p.x, p.y)

    def _on_observation(self, msg: Any) -> None:
        payload = json.loads(msg.data)
        detections = tuple(
            Detection(item["label"], self._grid_point(item["x"], item["y"]),
                      float(item.get("confidence", 1.0)), item.get("track_id"),
                      item.get("room"))
            for item in payload.get("detections", []))
        visible = frozenset(
            self._grid_point(cell["x"], cell["y"])
            for cell in payload.get("visible_cells", []))
        self._observation = Observation(
            self._pose, payload.get("room"), detections, visible, source="ros2")

    def navigate(self, target: Point) -> NavigationResult:
        if not self.action.wait_for_server(timeout_sec=5.0):
            return NavigationResult(False, 0, "nav2_server_unavailable")
        goal = self.NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.origin_x + target.x * self.resolution
        goal.pose.pose.position.y = self.origin_y + target.y * self.resolution
        goal.pose.pose.orientation.w = 1.0
        future = self.action.send_goal_async(goal)
        self.rclpy.spin_until_future_complete(self.node, future, timeout_sec=self.timeout_sec)
        if not future.done():
            return NavigationResult(False, 0, "send_goal_timeout")
        handle = future.result()
        if handle is None or not handle.accepted:
            return NavigationResult(False, 0, "nav2_rejected")
        self._goal_handle = handle
        outcome = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, outcome, timeout_sec=self.timeout_sec)
        if not outcome.done():
            self.stop()
            return NavigationResult(False, 0, "nav2_result_timeout")
        result = outcome.result()
        succeeded = result.status == self.GoalStatus.STATUS_SUCCEEDED
        error = getattr(result.result, "error_msg", "") or "nav2_failed"
        self._goal_handle = None
        return NavigationResult(succeeded, 0, "ok" if succeeded else error)

    def observe(self) -> Observation:
        self.rclpy.spin_once(self.node, timeout_sec=0.1)
        if self._observation is None:
            raise RuntimeError(
                "No perception observation. Publish map-frame detections and "
                "visible_cells to /spatialmind/observation_json.")
        observation = self._observation
        self._observation = None
        return observation

    def stop(self) -> None:
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    def close(self) -> None:
        self.stop()
        self.node.destroy_node()
