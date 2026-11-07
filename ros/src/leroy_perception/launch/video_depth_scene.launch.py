"""Visual-only dense depth view for Metric Video Depth Anything.

Consumes an existing /video_depth_anything/depth publisher. The output is
in the camera optical frame and is NEVER forwarded to Nav2/costmaps.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("leroy_perception"),
        "config", "video_depth_scene.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        Node(
            package="leroy_perception",
            executable="depth_obstacles",
            name="experimental_video_depth_cloud",
            output="screen",
            parameters=[
                LaunchConfiguration("config"),
                # Explicitly override any config in order to fail closed.
                {
                    "publish_obstacles": False,
                    "extrinsics_validated": False,
                    "ground_reference_validated": False,
                },
            ],
        ),
    ])
