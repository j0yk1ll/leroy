"""M5 isolated synthetic MPPI FollowPath: no live sensors or motors.

CRITICAL: the only controller output topic is DIAGNOSTIC. All variations
of cmd_vel are remapped; never include a bridge or hardware driver.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


PROPOSED_CMD = '/leroy_nav2/diagnostics/proposed_cmd_vel'


def _guard(context, *args, **kwargs):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError('M5 controller can run only in isolated synthetic graph')
    return []


def generate_launch_description():
    perception = get_package_share_directory('leroy_nav2_perception')
    integration = get_package_share_directory('leroy_nav2_integration')
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package='leroy_nav2_odometry',
            executable='synthetic_odom',
            name='synthetic_odom',
            output='screen',
        ),
        Node(
            package='leroy_nav2_perception',
            executable='synthetic_depth_fixture',
            output='screen',
        ),
        Node(
            package='leroy_nav2_perception',
            executable='depth_obstacles',
            name='depth_obstacles',
            parameters=[os.path.join(
                perception, 'config', 'fixture_obstacles.yaml')],
            output='screen',
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            parameters=[os.path.join(
                integration, 'config', 'm5_controller.yaml')],
            output='screen',
            remappings=[
                # Catch relative and absolute names. Never allow /cmd_vel.
                ('cmd_vel', PROPOSED_CMD),
                ('/cmd_vel', PROPOSED_CMD),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_m5_fixture',
            parameters=[{
                'autostart': False,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),
    ])
