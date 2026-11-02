"""Read-only RViz2 dashboard for the native XRSLAM and Nav2 topics."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("leroy_navigation"), "rviz", "navigation.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(package="rviz2", executable="rviz2", name="leroy_rviz",
             arguments=["-d", config], output="screen",
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
