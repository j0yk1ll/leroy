"""Experimental metric video depth (do not run concurrently with DepthART for same target topic)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("leroy_perception"),
        "config", "metric_video_depth.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument(
            "device", default_value="cpu", choices=["cpu", "cuda", "auto"]
        ),
        Node(
            package="leroy_perception",
            executable="metric_video_depth_node",
            name="metric_video_depth_anything",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                {"device": LaunchConfiguration("device")},
            ],
        ),
    ])
