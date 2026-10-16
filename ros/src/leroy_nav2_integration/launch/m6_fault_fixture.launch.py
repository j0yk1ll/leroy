"""M6 isolated periodic sensor blackout: no actions, bridge, or motor commands.

This tests a read-only diagnostic audit, NOT Nav2's ability to stop output.
A controller may be activated manually, but FollowPath is NEVER sent.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DIAGNOSTIC_CMD_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'
VALID_MODES = ('depth', 'odom')


def require_fixture(mode, environment):
    if mode not in VALID_MODES:
        raise RuntimeError('M6 fault must be depth or odom')
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(k) != v for k, v in required.items()):
        raise RuntimeError('M6 fixture requires isolated Nav2 environment')


def _build_nodes(context, *args, **kwargs):
    fault = LaunchConfiguration('fault').perform(context)
    require_fixture(fault, os.environ)
    perception = get_package_share_directory('leroy_nav2_perception')
    integration = get_package_share_directory('leroy_nav2_integration')

    return [
        Node(
            package=('leroy_nav2_integration'
                     if fault == 'odom' else 'leroy_nav2_odometry'),
            executable=('fault_odom_fixture'
                        if fault == 'odom' else 'synthetic_odom'),
            name='synthetic_odom',
            output='screen',
        ),
        Node(
            package=('leroy_nav2_integration'
                     if fault == 'depth' else 'leroy_nav2_perception'),
            executable=('fault_depth_fixture'
                        if fault == 'depth' else 'synthetic_depth_fixture'),
            name='synthetic_depth_fixture',
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
                ('cmd_vel', DIAGNOSTIC_CMD_TOPIC),
                ('/cmd_vel', DIAGNOSTIC_CMD_TOPIC),
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_m6_fixture',
            parameters=[{
                'autostart': False,
                'node_names': ['controller_server'],
            }],
            output='screen',
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'fault', default_value='depth',
            choices=['depth', 'odom'],
            description='Which synthetic source pauses cyclically'),
        OpaqueFunction(function=_build_nodes),
    ])
