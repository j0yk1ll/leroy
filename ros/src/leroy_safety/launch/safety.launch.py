"""Read-only safety middleware deployment; no motor output."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="leroy_safety", executable="safety_supervisor",
             name="leroy_safety_supervisor", output="screen"),
    ])
