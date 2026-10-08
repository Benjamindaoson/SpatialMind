"""Measured RGB-D bridge launch."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    names=("rgb_topic","depth_topic","info_topic","map_frame")
    defaults=("/rgbd_camera/image","/rgbd_camera/depth_image",
              "/rgbd_camera/camera_info","map")
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in zip(names,defaults)],
        Node(package="spatialmind_ros",executable="spatialmind_rgbd_bridge",
             name="spatialmind_rgbd_bridge",output="screen",
             parameters=[{key:LaunchConfiguration(key) for key in names},
                         {"use_sim_time":True}]),
    ])
