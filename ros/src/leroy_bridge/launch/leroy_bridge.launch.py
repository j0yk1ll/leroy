from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('leroy_bridge'),
        'config',
        'leroy_bridge.yaml',
    )

    return LaunchDescription([
        Node(
            package='leroy_bridge',
            executable='leroy_bridge_node',
            name='leroy_bridge',
            output='screen',
            parameters=[config],
        ),
    ])
