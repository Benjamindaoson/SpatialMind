#!/usr/bin/env python3
"""ROS2/Gazebo live-navigation acceptance probe.

This deliberately fails unless a *real* Nav2 action reaches a destination and
AMCL localization reports that the robot has moved. Endpoint availability alone
is not sufficient evidence of an embodied robot navigation loop.
"""
from __future__ import annotations

import argparse
import math
import time


def main() -> int:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav2_msgs.action import NavigateToPose
    from action_msgs.msg import GoalStatus

    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, default=-3.0)
    parser.add_argument("--y", type=float, default=-4.0)
    parser.add_argument("--timeout", type=float, default=150)
    parser.add_argument("--min-motion", type=float, default=0.35)
    parser.add_argument("--tolerance", type=float, default=0.65)
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("spatialmind_live_nav_acceptance")
    client = ActionClient(node, NavigateToPose, "/navigate_to_pose")
    poses = []
    def on_pose(msg):
        p = msg.pose.pose.position
        if msg.header.frame_id == "map":
            poses.append((p.x, p.y, time.monotonic()))
    sub = node.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", on_pose, 10)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    started = time.monotonic()
    def pump_until(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return True
            executor.spin_once(timeout_sec=0.2)
        return predicate()

    try:
        if not pump_until(lambda: bool(poses), min(60, args.timeout)):
            raise RuntimeError("AMCL did not publish a map-frame localization pose")
        initial = poses[-1][:2]
        if not client.wait_for_server(timeout_sec=15):
            raise RuntimeError("Nav2 NavigateToPose action server unavailable")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.x
        goal.pose.pose.position.y = args.y
        goal.pose.pose.orientation.w = 1.
        send = client.send_goal_async(goal)
        if not pump_until(send.done, 15):
            raise RuntimeError("Nav2 did not acknowledge goal")
        handle = send.result()
        if not handle.accepted:
            raise RuntimeError("Nav2 rejected target")
        result = handle.get_result_async()
        if not pump_until(result.done, args.timeout):
            handle.cancel_goal_async()
            raise RuntimeError("Nav2 navigation timed out")
        status = result.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 action returned non-success status {status}")
        if not poses:
            raise RuntimeError("Missing localization trace")
        final = poses[-1][:2]
        displacement = math.dist(initial, final)
        error = math.dist(final, (args.x, args.y))
        if displacement < args.min_motion:
            raise RuntimeError(f"Movement unverified: {displacement:.3f}m")
        if error > args.tolerance:
            raise RuntimeError(f"Goal not reached by AMCL: {error:.3f}m")
        import json
        print(json.dumps({
            "status": "passed",
            "sensor": "/amcl_pose",
            "action": "/navigate_to_pose",
            "initial_map_pose_m": initial,
            "final_map_pose_m": final,
            "goal_map_pose_m": [args.x, args.y],
            "displacement_m": round(displacement, 3),
            "goal_error_m": round(error, 3),
            "duration_s": round(time.monotonic()-started, 2),
            "localization_samples": len(poses),
            "note": "Gazebo navigation only; this does not validate RGB-D perception."
        }, indent=2))
        return 0
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
