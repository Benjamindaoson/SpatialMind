# SpatialMind ROS2 Jazzy + Gazebo Harmonic Integration

**Status:** Source integration and deterministic scene generator are provided. Current hosted CI has **not** run actual ROS2/Gazebo navigation or any physical hardware trials. Software tests must not be represented as physical results.

## 1. Requirements

Ubuntu 24.04, ROS2 Jazzy, Gazebo Harmonic, Navigation2, nav2_minimal_tb3_sim, ros_gz_bridge, message_filters, tf2_ros, sensor_msgs and Python 3.11+.

Official documentation:
- https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
- https://gazebosim.org/docs/harmonic/ros2_integration/
- https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_bringup

Example setup **after installing ROS2**:

~~~bash
source /opt/ros/jazzy/setup.bash
sudo apt update
sudo apt install ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-minimal-tb3-sim ros-jazzy-ros-gz \
  ros-jazzy-message-filters ros-jazzy-tf2-ros \
  ros-jazzy-nav2-msgs python3-colcon-common-extensions

# From repository root:
python3 scripts/generate_gazebo_scene.py --out simulation/generated
export PYTHONPATH="$PWD/src:$PYTHONPATH"
cd ros2_ws
colcon build --symlink-install --packages-select spatialmind_ros
source install/setup.bash
cd ..
~~~

Ensure the installed Nav2 distribution supports the world and robot_sdf launch arguments:
~~~bash
ros2 launch nav2_bringup tb3_simulation_launch.py --show-args
~~~

## 2. Bring up separate processes

**Terminal A: generated world, occupancy grid, calibrated semantic points and Nav2**

~~~bash
source /opt/ros/jazzy/setup.bash
bash scripts/bringup_spatialmind.sh
~~~

**Terminal B: bridge Gazebo RGB-D messages, camera optical TF and pixel/depth perception**

~~~bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
ros2 launch spatialmind_ros sensors.launch.py
~~~

**Terminal C: verify endpoints and execute real Nav2 action goals**

~~~bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
export PYTHONPATH="$PWD/src:$PYTHONPATH"
bash scripts/smoke_ros2.sh
ros2 topic hz /rgbd_camera/image
ros2 topic hz /rgbd_camera/depth_image
ros2 topic hz /spatialmind/observation_json
ros2 run spatialmind_ros spatialmind_nav_agent \
  --semantic-map simulation/generated/semantic_map.json \
  --kind find --target "blue toolbox" --max-actions 15 --timeout 240
~~~

The ROS package connects to actual Nav2 NavigateToPose Actions and /amcl_pose localization, not toy-grid navigation. Physical observations are produced from sensor_msgs image/depth + CameraInfo + TF2; the agent does not read Gazebo object world truth.

## 3. Visual grounding path

~~~mermaid
flowchart LR
    A["Gazebo rendered RGB-D"] --> B["ros_gz_bridge"]
    B --> C["Synchronize + TF2"]
    C --> D["Pixel detection + real depth"]
    D --> E["Map object estimate + visible cells"]
    E --> F["Temporal belief memory"]
    F --> G["Async Agent"]
    G --> H["Nav2 NavigateToPose"]
    H --> I["Localization / result"]
~~~

The reference detector finds controlled red/blue regions, **not arbitrary real-world objects**. Replace it with a calibrated object detector/VLM + tracker for open-vocabulary scenes.

The injected camera publishes front_rgbd_optical_frame. The sensors launcher supplies a base_link → optical-frame static transform. **Check its actual direction, translation, RGB/depth alignment and projection against a known landmark before allowing negative evidence or real autonomous navigation.** Robot simulation revisions may use a different sensor frame convention.

## 4. Rosbag and provenance

~~~bash
bash scripts/record_rosbag.sh
ros2 bag info artifacts/rosbags/EXPERIMENT_ID
~~~

Persist git SHA, ROS distro/package versions, world map revision, camera intrinsics/extrinsics, random seed, model settings, task ID, navigation action IDs, RGB-D data, TF and AMCL, agent events, plan decisions, failure causes and outcomes.

## 5. Fault isolation and safety

Check in this order: Gazebo camera topics; ROS bridged topics; CameraInfo/TF2 alignment; map→odom→base_link transforms; AMCL convergence and covariance; semantic waypoint reachability; Nav2 action status; physical observation stream; temporal memory changes.

Never use task-level stop as a hardware E-stop. Before real-robot operation, implement and test independent hardware stop, geofencing, watchdogs, actuator and collision protections.

## 6. Unverified elements

- No ROS2/Gazebo runtime executable exists in the authoring environment; real package import, bringup and camera simulation remain unverified.
- Gazebo-generated assets are XML/PGM validated by unit tests, not by a real renderer.
- Exact Gazebo camera sensor topic names and TF axes must be checked on target ROS2 version.
- No real robot test, learned re-ID, SLAM retraining, VLA fine-tuning or safety certification is claimed.
