from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('leroy_mcu'),
        'config',
        'leroy_mcu.yaml',
    )

    return LaunchDescription([
        Node(
            package='leroy_mcu',
            executable='leroy_mcu_node',
            name='leroy_mcu',
            output='screen',
            parameters=[config],
        ),
    ])
