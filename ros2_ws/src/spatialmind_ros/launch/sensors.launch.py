"""Launch Gazebo RGB-D image bridge, optical-frame TF and perception pipeline."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share=get_package_share_directory("spatialmind_ros")
    bridge=os.path.join(share,"config","bridge_rgbd.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("bridge_config",default_value=bridge),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ros_gz_bridge"),
                             "launch","ros_gz_bridge.launch.py")),
            launch_arguments={"config_file":LaunchConfiguration("bridge_config")}.items(),
        ),
        Node(package="tf2_ros",executable="static_transform_publisher",
             arguments=[
                 "--x","0.12","--y","0","--z","0.24",
                 "--roll","-1.5707963","--pitch","0","--yaw","-1.5707963",
                 "--frame-id","base_link",
                 "--child-frame-id","front_rgbd_optical_frame",
             ],parameters=[{"use_sim_time":True}]),
        Node(package="spatialmind_ros",executable="spatialmind_rgbd_bridge",
             parameters=[{"use_sim_time":True}],output="screen"),
    ])
