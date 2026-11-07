"""Read-only experimental 3D voxel -> 2D occupancy-grid preview."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("leroy_navigation"),
        "config", "experimental_nav2_preview.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        Node(
            package="leroy_navigation",
            executable="experimental_nav2_grid_preview",
            name="leroy_experimental_nav2_grid_preview",
            output="screen",
            parameters=[LaunchConfiguration("config")]),
    ])
