"""Only raw optical-frame diagnostics; no obstacle authority or motor path."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    if (os.environ.get('LEROY_NAV2_DRY_RUN') != '1'
            or os.environ.get('ROS_DOMAIN_ID') != '167'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('M2 diagnostics require offline Nav2 container, domain 167')
    return []


def generate_launch_description():
    share = get_package_share_directory('leroy_nav2_perception')
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package='leroy_nav2_perception',
            executable='depth_obstacles',
            name='depth_obstacles',
            parameters=[os.path.join(share, 'config', 'depth_diagnostics.yaml')],
            output='screen',
        ),
    ])
