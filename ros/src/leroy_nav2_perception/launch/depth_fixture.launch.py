"""Isolated synthetic TF/depth/pointcloud and stand-alone Nav2 costmap.

No live sensors, no navigation controller, no velocity producers, no motors.
Even the costmap lifecycle remains unconfigured until manual activation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    if (os.environ.get('LEROY_NAV2_DRY_RUN') != '1'
            or os.environ.get('ROS_DOMAIN_ID') != '167'
            or os.environ.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST'):
        raise RuntimeError('Synthetic fixture requires isolated Nav2 container')
    return []


def generate_launch_description():
    share = get_package_share_directory('leroy_nav2_perception')
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package='leroy_nav2_perception',
            executable='synthetic_depth_fixture',
            output='screen',
        ),
        Node(
            package='leroy_nav2_perception',
            executable='depth_obstacles',
            name='depth_obstacles',
            parameters=[os.path.join(share, 'config', 'fixture_obstacles.yaml')],
            output='screen',
        ),
        Node(
            # Experimental synthetic-only lifetime-order runner.
            package='leroy_nav2_costmap_fixture',
            executable='costmap_fixture',
            name='costmap',
            parameters=[os.path.join(share, 'config', 'fixture_costmap.yaml')],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_depth_fixture',
            parameters=[{'autostart': False, 'node_names': ['costmap']}],
            output='screen',
        ),
    ])
