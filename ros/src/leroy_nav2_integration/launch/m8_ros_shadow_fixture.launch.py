"""M8: connect M7 shadow classification to M6 FAKE ROS streams.

Use M6's already-tested isolated Nav2 fixture (no FollowPath action).
The extra producer only publishes a new fictional diagnostic topic.
The shadow observer is run manually as a READ-ONLY ros2 run process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


VALID_FAULTS = ('depth', 'odom')


def _guard(context, *args, **kwargs):
    if LaunchConfiguration('fault').perform(context) not in VALID_FAULTS:
        raise RuntimeError('M8 only supports synthetic depth or odom fault')
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError('M8 requires isolated synthetic Nav2 Podman')
    return []


def generate_launch_description():
    share = get_package_share_directory('leroy_nav2_integration')
    existing = os.path.join(share, 'launch', 'm6_fault_fixture.launch.py')
    return LaunchDescription([
        DeclareLaunchArgument(
            'fault', default_value='depth', choices=['depth', 'odom']),
        OpaqueFunction(function=_guard),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(existing),
            launch_arguments={'fault': LaunchConfiguration('fault')}.items(),
        ),
        Node(
            package='leroy_nav2_integration',
            executable='synthetic_shadow_candidate',
            name='m8_synthetic_candidate',
            output='screen',
        ),
    ])
