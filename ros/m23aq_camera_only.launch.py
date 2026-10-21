"""M23AQ: DORMANT BY DEFAULT, camera-only manual-review launch candidate.

This file MUST NOT be invoked until the separately documented physical
authorization gate is approved. This source change alone authorizes
ZERO camera / serial / motor / VIO activity and is never installed into
the main robot launcher.
"""
import os
import re

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

OPENCV_PREFIX = "/opt/m23aj-opencv/lib"
CALIBRATION = "/config/camera/front_camera.yaml"


def _guard_manual_approval():
    # A software flag is NOT physical proof of a disconnected motor rail.
    # Both must be verified separately by the operator before any launch.
    if os.environ.get("M23AQ_CAMERA_ONLY_OPERATOR_APPROVAL") != "REVIEWED_AND_APPROVED":
        raise RuntimeError("M23AQ_BLOCKED=EXPLICIT_CAMERA_ONLY_REVIEW_REQUIRED")
    if os.environ.get("M23AQ_MOTOR_POWER_ISOLATION_ATTESTED") != "YES":
        raise RuntimeError("M23AQ_BLOCKED=MOTOR_POWER_NOT_INDEPENDENTLY_ATTESTED")
    video = os.environ.get("M23AQ_VIDEO_DEVICE", "")
    if re.fullmatch(r"/dev/video[0-9]+", video) is None:
        raise RuntimeError("M23AQ_BLOCKED=EXPLICIT_VIDEO_DEVICE_REQUIRED")
    if os.environ.get("M23AQ_CALIBRATION_VERIFIED") != "YES":
        raise RuntimeError("M23AQ_BLOCKED=CAMERA_CALIBRATION_IDENTITY_UNVERIFIED")
    if os.environ.get("M23AQ_CALIBRATION_PATH") != CALIBRATION:
        raise RuntimeError("M23AQ_BLOCKED=PINNED_CALIBRATION_PATH_REQUIRED")
    if os.environ.get("LD_PRELOAD"):
        raise RuntimeError("M23AQ_BLOCKED=NO_PRELOAD_ALLOWED")
    if os.environ.get("LD_LIBRARY_PATH", "").split(":")[0] != OPENCV_PREFIX:
        raise RuntimeError("M23AQ_BLOCKED=NO_PTHREADS_OPENCV_LOADER_PRECEDENCE")
    # This only constructs a launch description. Not device access by itself.
    return video


def generate_launch_description():
    video = _guard_manual_approval()
    camera = Node(
        package="usb_cam",
        executable="usb_cam_node_exe",
        namespace="camera",
        name="usb_cam",
        output="screen",
        parameters=[{
            "video_device": video,
            "image_width": 640,
            "image_height": 480,
            "framerate": 30.0,
            "io_method": "mmap",
            "pixel_format": "mjpeg2rgb",
            "camera_name": "front_camera",
            "frame_id": "camera_optical_frame",
            "camera_info_url": "file://" + CALIBRATION,
        }],
    )
    rectifier = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="m23aq_camera_only_container",
        namespace="",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify_node",
                namespace="/camera",
                remappings=[("image", "image_raw"),
                            ("camera_info", "camera_info")],
            ),
        ],
    )
    return LaunchDescription([camera, rectifier])
