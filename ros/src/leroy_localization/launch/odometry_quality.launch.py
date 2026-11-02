"""Optional read-only odometry diagnostic node."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="leroy_localization", executable="leroy_odometry_quality",
             name="leroy_odometry_quality", output="screen"),
    ])
