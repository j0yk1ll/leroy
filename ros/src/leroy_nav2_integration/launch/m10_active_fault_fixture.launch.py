"""M10: one synthetic FollowPath can overlap a TRIGGERED fake sensor outage.

Exactly five disconnected fixture processes, with all controller
velocity outputs hard-remapped to an UNCONSUMED diagnostics topic.
Nothing triggers a fault or sends an action merely by launching.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DIAGNOSTIC_CMD_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'
MODES = ('depth', 'odom')


def _build(context, *args, **kwargs):
    mode = LaunchConfiguration('fault').perform(context)
    guard = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if mode not in MODES or any(os.environ.get(k) != v for k, v in guard.items()):
        raise RuntimeError('M10 requires one selected fake fault, isolated Podman')
    perception = get_package_share_directory('leroy_nav2_perception')
    integration = get_package_share_directory('leroy_nav2_integration')
    return [
        Node(
            package=('leroy_nav2_integration'
                     if mode == 'odom' else 'leroy_nav2_odometry'),
            executable=('m10_fake_odom'
                        if mode == 'odom' else 'synthetic_odom'),
            name='synthetic_odom',
            output='screen',
        ),
        Node(
            package=('leroy_nav2_integration'
                     if mode == 'depth' else 'leroy_nav2_perception'),
            executable=('m10_fake_depth'
                        if mode == 'depth' else 'synthetic_depth_fixture'),
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
            remappings=[
                ('cmd_vel', DIAGNOSTIC_CMD_TOPIC),
                ('/cmd_vel', DIAGNOSTIC_CMD_TOPIC),
            ],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_m10_fixture',
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
            'fault', default_value='depth', choices=['depth', 'odom']),
        OpaqueFunction(function=_build),
    ])
