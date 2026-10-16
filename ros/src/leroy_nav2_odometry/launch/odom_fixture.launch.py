"""M3: synthetic wheel odometry and TF, no Nav2 controller or motor bridge."""

import os

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    environment = os.environ
    expected = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(k) != v for k, v in expected.items()):
        raise RuntimeError('M3 synthetic odometry requires isolated Nav2 environment')
    return []


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package='leroy_nav2_odometry',
            executable='synthetic_odom',
            name='synthetic_odom',
            output='screen',
        ),
    ])
