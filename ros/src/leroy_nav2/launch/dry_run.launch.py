"""Milestone-1 Nav2 inspection only. Must run in an OFFLINE Podman network."""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Deliberately immutable: no argument lets a user redirect this to /cmd_vel.
PROPOSED_CMD_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'


def require_dry_run(mode, environment):
    """Fail closed on accidental hardware/passive deployment attempts.

    This check is defense in depth, NOT proof of network or hardware isolation.
    The required primary boundary is Podman --network=none, with no devices or
    mounts connected to the hardware container.
    """
    if mode != 'simulation':
        raise RuntimeError(
            'Milestone 1 supports simulation only; passive_live/hardware forbidden'
        )
    if environment.get('LEROY_NAV2_DRY_RUN') != '1':
        raise RuntimeError('Set LEROY_NAV2_DRY_RUN=1 in the isolated Nav2 image')
    if environment.get('ROS_DOMAIN_ID') != '167':
        raise RuntimeError('Milestone-1 dry run requires ROS_DOMAIN_ID=167')
    if environment.get('ROS_AUTOMATIC_DISCOVERY_RANGE') != 'LOCALHOST':
        raise RuntimeError('Require ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST')


def _guard(context, *args, **kwargs):
    require_dry_run(LaunchConfiguration('mode').perform(context), os.environ)
    return [LogInfo(msg='Leroy Nav2 DIAGNOSTIC ONLY: controller inactive; '
                        'physical network isolation must be verified externally')]


def generate_launch_description():
    params = str(
        Path(get_package_share_directory('leroy_nav2')) / 'config' / 'simulation.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='simulation',
                              description='Only simulation is supported in M1'),
        OpaqueFunction(function=_guard),
        # No planner, BT navigator, behaviors, smoother, motor bridge, camera,
        # /odom source, TF authority, or motor-compatible publisher is launched.
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params],
            remappings=[
                ('cmd_vel', PROPOSED_CMD_TOPIC),
                ('/cmd_vel', PROPOSED_CMD_TOPIC),
            ],
        ),
        # autostart=False deliberately leaves the controller unconfigured.
        # No physical/simulated odometry or transform is fabricated in M1.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_leroy_nav2',
            output='screen',
            parameters=[{
                'autostart': False,
                'node_names': ['controller_server'],
            }],
        ),
    ])
