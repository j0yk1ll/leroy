from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('xrslam_ros2'))
    return LaunchDescription([
        DeclareLaunchArgument('relocalization_profile', default_value='off',
                              description='off for initial VIO tests; v1 needs EigenPlaces model env'),
        DeclareLaunchArgument('params_file',
                              default_value=str(share / 'config' / 'leroy_adapter.yaml')),
        DeclareLaunchArgument('slam_config_path',
                              default_value=str(share / 'config' / 'leroy_slam.yaml')),
        SetEnvironmentVariable('XRSLAM_RELOCALIZATION_PROFILE',
                               LaunchConfiguration('relocalization_profile')),
        Node(package='xrslam_ros2', executable='xrslam_ros2_node', name='xrslam_ros2',
             output='screen', parameters=[LaunchConfiguration('params_file'),
                                          {'slam_config_path': LaunchConfiguration('slam_config_path')}]),
    ])
