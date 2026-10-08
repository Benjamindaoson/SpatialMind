#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
ros2 action list -t | grep -F '/navigate_to_pose [nav2_msgs/action/NavigateToPose]'
ros2 topic list | grep -Fx '/amcl_pose'
echo "Nav2 and localization endpoints detected."
if ros2 topic list | grep -Fx '/spatialmind/observation_json'; then
  echo "SpatialMind perception topic detected."
else
  echo "No perception stream: object search is not ready."
fi
