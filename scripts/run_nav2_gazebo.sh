#!/usr/bin/env bash
# Baseline Gazebo Harmonic + Nav2 simulation on ROS2 Jazzy.
set -eo pipefail
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "Install ROS2 Jazzy first; see docs/ROS2_GAZEBO.md" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
set -u
export TURTLEBOT3_MODEL=waffle
exec ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True use_rviz:=False "$@"
