"""Experimental metric video depth (do not run concurrently with DepthART for same target topic)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


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
        DeclareLaunchArgument(
            "python_executable", default_value="/opt/vda-venv/bin/python3",
            description="Python interpreter with VDA, ROS and PyTorch dependencies",
        ),
        # Invoke the installed Python module directly. ROS's Node action
        # requires a libexec wrapper even when a Python prefix is specified;
        # deployments that have the package module but lack that wrapper
        # otherwise abort the entire bringup.
        ExecuteProcess(
            cmd=[
                LaunchConfiguration("python_executable"),
                "-m", "leroy_perception.metric_video_depth_node",
                "--ros-args",
                "-r", "__node:=metric_video_depth_anything",
                "--params-file", LaunchConfiguration("config"),
                "-p", ["device:=", LaunchConfiguration("device")],
            ],
            name="metric_video_depth_anything",
            output="screen",
        ),
    ])
