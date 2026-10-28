"""Real front camera -> rectified image -> CPU DepthART -> raw optical cloud.

This launch is non-actuating and intentionally DOES NOT start the bridge,
motor authority, XRSLAM, planner or controller. Never use with motor power.
"""
import os
import stat

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _guard(_context, *args, **kwargs):
    required = {
        "LEROY_LIVE_PERCEPTION_ONLY": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }
    for key, value in required.items():
        if os.environ.get(key) != value:
            raise RuntimeError("live perception requires " + key + "=" + value)
    if os.path.exists("/dev/ttyACM0"):
        raise RuntimeError("motor-capable STM32 serial must not be mapped")
    camera = "/dev/video2"
    if not os.path.exists(camera) or not stat.S_ISCHR(os.stat(camera).st_mode):
        raise RuntimeError("verified camera device /dev/video2 must be mapped")
    if not os.path.isfile("/config/camera/front_camera.yaml"):
        raise RuntimeError("physical camera calibration file is missing")
    return []


def generate_launch_description():
    depthart_share = get_package_share_directory("leroy_depthart")
    perception_share = get_package_share_directory("leroy_nav2_perception")
    return LaunchDescription([
        OpaqueFunction(function=_guard),
        Node(
            package="usb_cam", executable="usb_cam_node_exe",
            namespace="/camera", name="usb_cam", output="screen",
            parameters=[{
                "video_device": "/dev/video2",
                "image_width": 640, "image_height": 480,
                "framerate": 30.0, "io_method": "mmap",
                "pixel_format": "mjpeg2rgb",
                "camera_name": "front_camera",
                "frame_id": "camera_optical_frame",
                "camera_info_url": "file:///config/camera/front_camera.yaml",
            }],
        ),
        Node(
            package="image_proc", executable="rectify_node",
            namespace="/camera", name="rectify_node", output="screen",
            remappings=[("image", "image_raw"),
                        ("camera_info", "camera_info")],
        ),
        Node(
            package="leroy_depthart", executable="depthart_node",
            name="depthart", output="screen",
            parameters=[
                os.path.join(depthart_share, "config", "depthart.yaml"),
                {"device": "cpu"},
            ],
        ),
        Node(
            package="leroy_nav2_perception", executable="depth_obstacles",
            name="depth_obstacles", output="screen",
            parameters=[os.path.join(
                perception_share, "config", "depth_diagnostics.yaml")],
        ),
    ])
