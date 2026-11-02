"""Read-only RViz visualization with optional camera feature overlay."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("leroy_navigation"), "rviz", "navigation.rviz")
    return LaunchDescription([
        DeclareLaunchArgument("image_topic", default_value="/camera/image_raw"),
        DeclareLaunchArgument("use_features", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="leroy_navigation", executable="feature_overlay",
            name="leroy_feature_overlay", output="screen",
            condition=IfCondition(LaunchConfiguration("use_features")),
            parameters=[{"image_topic": LaunchConfiguration("image_topic"),
                         "use_sim_time": LaunchConfiguration("use_sim_time")}]),
        Node(package="rviz2", executable="rviz2", name="leroy_rviz",
             arguments=["-d", config], output="screen",
             parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}]),
    ])
