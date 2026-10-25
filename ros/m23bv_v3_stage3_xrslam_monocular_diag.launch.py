"""Stage 3 real camera -> actual XRSLAM monocular-frame submission.

Diagnostic ONLY. The URDF camera/IMU extrinsics remain PROVISIONAL.
No MCU mapping, no IMU subscription, no metric pose, no map TF or motors.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

UPSTREAM_EUROC_TEMPLATE = "/opt/xrslam/configs/euroc_sensor.yaml"
STAGE3_S = "/opt/leroy_stage3_slam.yaml"


def _guard(_context, *args, **_kwargs):
    env = {
        "M23BV_STAGE3_XRSLAM_MONOCULAR_DIAG": "YES",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    for key, required in env.items():
        if os.environ.get(key) != required:
            raise RuntimeError("BLOCKED_STAGE3_XRSLAM_ENV_" + key)
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/video2"):
        raise RuntimeError("BLOCKED_STAGE3_XRSLAM_DEVICE_VISIBLE")
    for file in (STAGE3_S, UPSTREAM_EUROC_TEMPLATE):
        if not os.path.isfile(file):
            raise RuntimeError("BLOCKED_STAGE3_XRSLAM_CONFIG_MISSING:" + file)
    return []


def generate_launch_description():
    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("leroy_description"),
            "launch", "description.launch.py",
        ])),
    )
    xrslam = Node(
        package="leroy_xrslam",
        executable="leroy_xrslam_node",
        name="leroy_xrslam",
        output="screen",
        parameters=[{
            "slam_yaml": STAGE3_S,
            # Upstream EuRoC template supplies UNVALIDATED noise defaults.
            # The adapter overwrites camera intrinsics from real CameraInfo
            # and camera-to-body extrinsics from the PROVISIONAL robot URDF.
            "device_template_yaml": UPSTREAM_EUROC_TEMPLATE,
            "image_topic": "/camera/image_rect",
            "camera_info_topic": "/camera/camera_info",
            "imu_topic": "/imu/data_raw",
            "base_frame": "base_link",
            "camera_optical_frame": "camera_optical_frame",
            "imu_frame": "imu_link",
            "odom_frame": "odom",
            "pose_convention_validated": False,
            "enable_map_tf": False,
        }],
    )
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        robot_tf,
        xrslam,
    ])
