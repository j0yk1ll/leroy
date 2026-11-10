"""Explicitly opt-in relative ZipDepth, with no metric or motor authority."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("leroy_perception"),
        "config", "zipdepth_relative.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=config),
        DeclareLaunchArgument(
            "python_executable", default_value="/opt/vda-venv/bin/python3",
            description="Interpreter with ROS, torch, and upstream ZipDepth installed"),
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("python_executable"),
                "-m", "leroy_perception.zipdepth_relative_node",
                "--ros-args", "--params-file", LaunchConfiguration("config"),
            ],
            name="zipdepth_relative",
            output="screen",
        ),
    ])
