"""M23K camera-only comparison: USB camera WITHOUT image_proc rectifier.

No MCU, IMU, motors or recorder. Uses exact original M20 640x480
framerate, capture, and calibration settings; only selects alternate
explicitly approved pixel format for diagnostic comparison.
"""
import os
import stat

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23K_OPERATOR_CAMERA_ONLY_APPROVED": "YES",
}
MODES = {
    "raw_only_mjpeg": "mjpeg2rgb",
    "raw_only_yuyv": "yuyv2rgb",
}


def select_pixel_format(environ):
    for key, expected in EXPECTED_ENV.items():
        if environ.get(key) != expected:
            raise RuntimeError("M23K raw-only requires " + key + "=" + expected)
    variant = environ.get("M23K_PROBE_VARIANT")
    if variant not in MODES:
        raise ValueError("unsupported raw-only camera mode")
    return MODES[variant]


def _camera_only(_context, *args, **kwargs):
    fmt = select_pixel_format(os.environ)
    if not stat.S_ISCHR(os.stat("/dev/video2").st_mode):
        raise RuntimeError("front USB camera device absent")
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/ttyACM1"):
        raise RuntimeError("raw-only camera probe refuses STM32 device")
    if not os.path.isfile("/config/camera/front_camera.yaml"):
        raise RuntimeError("existing front camera calibration missing")
    return [Node(
        package="usb_cam", executable="usb_cam_node_exe",
        namespace="camera", name="usb_cam", output="screen",
        parameters=[{
            "video_device": "/dev/video2",
            "image_width": 640, "image_height": 480,
            "framerate": 30.0, "io_method": "mmap",
            "pixel_format": fmt,
            "camera_name": "front_camera",
            "frame_id": "camera_optical_frame",
            "camera_info_url": "file:///config/camera/front_camera.yaml",
        }]
    )]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_camera_only)])
