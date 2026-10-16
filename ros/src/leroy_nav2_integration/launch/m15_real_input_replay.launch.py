"""M15 only: disconnected replay of real-shaped ROS sensor interfaces.

Two ROS nodes. No XRSLAM binary, DepthART driver, physical /odom,
external /tf, hardware bridge, controller action or real /cmd_vel.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build(context, *args, **kwargs):
    mode = LaunchConfiguration('fault').perform(context)
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if (mode not in ('depth', 'odom', 'tracking') or
            any(os.environ.get(k) != v for k, v in required.items())):
        raise RuntimeError('M15 only accepts offline replay fault fixture')
    return [
        Node(
            package='leroy_nav2_integration',
            executable='m15_replay_source',
            name='m15_replay_source',
            parameters=[{'fault_mode': mode}],
            output='screen',
        ),
        Node(
            package='leroy_nav2_integration',
            executable='m15_real_input_gate',
            name='m15_real_input_gate',
            parameters=[{'profile': 'offline_replay'}],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fault', default_value='depth',
            choices=['depth', 'odom', 'tracking']),
        OpaqueFunction(function=_build),
    ])
