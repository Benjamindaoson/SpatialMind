#!/usr/bin/env bash
# Evidence-first rosbag2 collection; install rosbag2 on ROS2 Jazzy.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
set -u
mkdir -p artifacts/rosbags
exec ros2 bag record -o "artifacts/rosbags/spatialmind_$(date +%Y%m%d_%H%M%S)" \
  /tf /tf_static /clock /odom /amcl_pose /scan \
  /rgbd_camera/image /rgbd_camera/depth_image \
  /rgbd_camera/camera_info /spatialmind/observation_json \
  /navigate_to_pose/_action/feedback \
  /navigate_to_pose/_action/status
