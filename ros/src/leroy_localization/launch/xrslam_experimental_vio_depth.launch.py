"""Experimental asynchronous VIO-depth commissioning (no Nav2 or motors).

Requires running /camera/image_rect, /camera/camera_info, /imu/data_raw
and ROS static camera/IMU transforms plus timestamped 32FC1 predicted depth. All four IMU noise covariance diagonals
and camera-to-IMU time offset must be supplied from measured Leroy calibration.
No default EuRoC sensor noise or zero time offset is permitted.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    slam_config = str(Path(get_package_share_directory("leroy_localization")) /
                      "config" / "leroy_experimental_rgbd_imu_slam.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("slam_yaml",
                              default_value=slam_config),
        DeclareLaunchArgument("camera_imu_offset_sec"),
        DeclareLaunchArgument("imu_cov_g_diagonal"),
        DeclareLaunchArgument("imu_cov_a_diagonal"),
        DeclareLaunchArgument("imu_cov_bg_diagonal"),
        DeclareLaunchArgument("imu_cov_ba_diagonal"),
        Node(
            package="leroy_localization",
            executable="leroy_xrslam_node",
            name="leroy_xrslam",
            output="screen",
            parameters=[{
                "slam_yaml": LaunchConfiguration("slam_yaml"),
                "image_topic": "/camera/image_rect",
                "camera_info_topic": "/camera/camera_info",
                "camera_optical_frame": "camera_optical_frame",
                "imu_topic": "/imu/data_raw",
                "imu_frame": "imu_link",
                "base_frame": "base_link",
                "camera_imu_offset_sec": ParameterValue(LaunchConfiguration("camera_imu_offset_sec"), value_type=float),
                "imu_cov_g_diagonal": ParameterValue(LaunchConfiguration("imu_cov_g_diagonal"), value_type=float),
                "imu_cov_a_diagonal": ParameterValue(LaunchConfiguration("imu_cov_a_diagonal"), value_type=float),
                "imu_cov_bg_diagonal": ParameterValue(LaunchConfiguration("imu_cov_bg_diagonal"), value_type=float),
                "imu_cov_ba_diagonal": ParameterValue(LaunchConfiguration("imu_cov_ba_diagonal"), value_type=float),
                "depth_topic": "/video_depth_anything/depth",
                "async_depth_stage_a1": True,
                "experimental_vio_depth": True,
                "experimental_metric_preview": False,
                "pose_convention_validated": False,
                "enable_map_tf": False,
            }],
        ),
    ])
