#!/usr/bin/env bash
# Requires Ubuntu 24.04, ROS2 Jazzy, Nav2, Gazebo Harmonic and xacro.
# Starts navigation in generated world; vision bridge is launched separately.
set -eo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS2 Jazzy missing" >&2
  exit 2
fi
source /opt/ros/jazzy/setup.bash
set -u
python3 "$ROOT/scripts/generate_gazebo_scene.py" --out "$ROOT/simulation/generated"
SIM_SHARE="$(ros2 pkg prefix --share nav2_minimal_tb3_sim 2>/dev/null || true)"
if [[ -z "$SIM_SHARE" ]]; then
  echo "Required ROS2 package nav2_minimal_tb3_sim was not found." >&2
  exit 2
fi
ROBOT_XACRO="$SIM_SHARE/urdf/gz_waffle.sdf.xacro"
if [[ ! -f "$ROBOT_XACRO" ]]; then
  echo "Find the installed gz_waffle.sdf.xacro and pass it to build_rgbd_robot.py." >&2
  echo "Run official nav2_bringup tb3_simulation_launch.py as fallback." >&2
  exit 2
fi
python3 "$ROOT/scripts/build_rgbd_robot.py" \
  --xacro "$ROBOT_XACRO" --output "$ROOT/simulation/generated/robot_rgbd.sdf"
export TURTLEBOT3_MODEL=waffle
exec ros2 launch nav2_bringup tb3_simulation_launch.py \
  "world:=$ROOT/simulation/generated/room_world.sdf.xacro" \
  "map:=$ROOT/simulation/generated/map.yaml" \
  "robot_sdf:=$ROOT/simulation/generated/robot_rgbd.sdf" \
  x_pose:=-4.0 y_pose:=-4.0 headless:=True use_rviz:=False
