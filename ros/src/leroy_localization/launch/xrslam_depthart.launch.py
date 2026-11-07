"""Optional DepthART -> native XRSLAM RGB-D visualization; no motor or odom authority.

Requires running /camera/image_rect, /camera/camera_info and sensor TF.
This launch deliberately does not start cameras, Nav2, or motor control.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    perception = Path(get_package_share_directory("leroy_perception"))
    depthart = str(perception / "launch" / "depthart.launch.py")

    return LaunchDescription([
        DeclareLaunchArgument("device", default_value="cpu"),
        DeclareLaunchArgument("depthart_config", default_value=str(perception / "config" / "depthart.yaml")),
        DeclareLaunchArgument("slam_yaml", default_value="/opt/xrslam/configs/rgbd_slam.yaml"),
        DeclareLaunchArgument("device_template_yaml", default_value="/opt/xrslam/configs/euroc_sensor.yaml"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(depthart),
            launch_arguments={
                "config": LaunchConfiguration("depthart_config"),
                "device": LaunchConfiguration("device"),
            }.items(),
        ),
        Node(
            package="leroy_localization",
            executable="leroy_xrslam_node",
            name="leroy_xrslam",
            output="screen",
            parameters=[{
                "slam_yaml": LaunchConfiguration("slam_yaml"),
                "device_template_yaml": LaunchConfiguration("device_template_yaml"),
                "image_topic": "/camera/image_rect",
                "camera_info_topic": "/camera/camera_info",
                "depth_topic": "/depthart/depth",
                "camera_optical_frame": "camera_optical_frame",
                "depth_wait_ms": 3000,
                "min_depth_valid_fraction": 0.01,
                # RGB-D pose conventions and depth scale are unverified on Leroy.
                "pose_convention_validated": False,
                "enable_map_tf": False,
            }],
        ),
    ])
