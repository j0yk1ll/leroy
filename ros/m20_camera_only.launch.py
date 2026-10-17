"""M20: physical front USB camera + rectifier ONLY, no DepthART/motors/VIO."""
import os
import stat

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


def _camera_only(_context, *args, **kwargs):
    for name, required in EXPECTED_ENV.items():
        if os.environ.get(name) != required:
            raise RuntimeError("M20 refuses nonisolated camera launch")
    if not stat.S_ISCHR(os.stat("/dev/video2").st_mode):
        raise RuntimeError("M20 requires mapped camera character device")
    if not os.path.isfile("/config/camera/front_camera.yaml"):
        raise RuntimeError("M20 requires existing real camera calibration")
    camera = Node(
        package="usb_cam", executable="usb_cam_node_exe",
        namespace="camera", name="usb_cam", output="screen",
        parameters=[{
            "video_device": "/dev/video2",
            "image_width": 640, "image_height": 480,
            "framerate": 30.0, "io_method": "mmap",
            "pixel_format": "mjpeg2rgb",
            "camera_name": "front_camera",
            "frame_id": "camera_optical_frame",
            "camera_info_url": "file:///config/camera/front_camera.yaml",
        }],
    )
    rectifier = ComposableNodeContainer(
        package="rclcpp_components",
        executable="component_container",
        name="m20_image_proc_container",
        namespace="", output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="image_proc", plugin="image_proc::RectifyNode",
                name="rectify_node", namespace="/camera",
                remappings=[
                    ("image", "image_raw"),
                    ("camera_info", "camera_info"),
                ],
            ),
        ],
    )
    return [camera, rectifier]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_camera_only)])
