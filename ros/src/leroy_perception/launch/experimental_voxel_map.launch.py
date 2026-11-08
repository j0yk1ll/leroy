"""Visualization-only bounded voxel map, never connected to Nav2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("leroy_perception"),
        "config", "experimental_voxel_map.yaml",
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        Node(
            package="leroy_perception",
            executable="experimental_voxel_mapper",
            name="experimental_voxel_mapper",
            output="screen",
            parameters=[LaunchConfiguration("config")],
        ),
    ])
