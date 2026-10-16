"""M4 offline TF chain: odom->base_link->fake camera, depth->rolling costmap.

No live sensors, no planner, no controller, no action server, no motors.
The synthetic-only costmap starts UNCONFIGURED until manual activation.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _guard(context, *args, **kwargs):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(key) != val for key, val in required.items()):
        raise RuntimeError('M4 synthetic fixture requires isolated Nav2 environment')
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
            # M2's tested callback-group cleanup fix; original Nav2
            # costmap executable is not used by this offline fixture.
            package='leroy_nav2_costmap_fixture',
            executable='costmap_fixture',
            name='costmap',
            parameters=[os.path.join(
                integration, 'config', 'rolling_costmap.yaml')],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_m4_fixture',
            parameters=[{'autostart': False, 'node_names': ['costmap']}],
            output='screen',
        ),
    ])
