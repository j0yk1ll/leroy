"""M23BV V3 isolated explicit-RELIABLE rectifier CANDIDATE, not production.

Uses an independently compiled BSD-derived rectify node *process*, not M20's
component_container. Only the rectifier process gets the alternative
OpenCV pthreads path. This file never starts IMU, motion or Nav2.
"""
import os
import stat
from launch import LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node

EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
ALT_OPENCV = "/opt/m23aj-opencv/lib"


def _camera_only(_context, *args, **kwargs):
    for name, required in EXPECTED_ENV.items():
        if os.environ.get(name) != required:
            raise RuntimeError("M23BV refuses nonisolated camera launch")
    if not stat.S_ISCHR(os.stat("/dev/video2").st_mode):
        raise RuntimeError("M23BV requires mapped camera device")
    if not os.path.isfile("/config/camera/front_camera.yaml"):
        raise RuntimeError("M23BV requires original real camera calibration")
    if os.environ.get("M23BV_RELIABLE_RECTIFIER_PROTO_V3") != "1":
        raise RuntimeError("M23BV reliable rectifier v3 candidate not explicitly selected")
    if os.environ.get("M23BV_CAMERA_READY_CANDIDATE") != "1":
        raise RuntimeError("M23BV readiness candidate explicit selection missing")
    if not os.path.isfile("/opt/m23bv/m23bv_camera_ready_gate.py"):
        raise RuntimeError("M23BV missing readiness candidate checker")
    if not os.path.isdir(ALT_OPENCV):
        raise RuntimeError("M23BV pinned pthreads OpenCV path is absent")

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
    # This distinct BSD-derived component explicitly requests RELIABLE
    # for raw-image inputs; actual graph and image payload MUST still be
    # independently verified, never inferred from source parameters.
    rectifier = Node(
        package="m23bv_reliable_rectifier", executable="m23bv_rectify_reliable_node",
        namespace="camera", name="rectify_node", output="screen",
        remappings=[("image", "image_raw"),
                    ("camera_info", "camera_info")],
        parameters=[{"qos_overrides": {
            "/camera/image_raw": {"subscription": {
                "reliability": "reliable", "history": "keep_last", "depth": 100}},
            "/camera/image_rect": {"publisher": {
                "reliability": "reliable", "history": "keep_last", "depth": 100}},
        }}],
        additional_env={
            "LD_LIBRARY_PATH": ALT_OPENCV + ":" + os.environ.get("LD_LIBRARY_PATH", "")
        },
    )
    ready = ExecuteProcess(
        cmd=["python3", "-u", "/opt/m23bv/m23bv_camera_ready_gate.py"],
        name="m23bv_camera_ready", output="screen",
    )

    def ready_exited(event, _ctx):
        # No timer: the rectifier is returned only after a reliable camera
        # publisher and matched image/info packets passed the DDS gate.
        if event.returncode == 0:
            return [rectifier]
        return [EmitEvent(event=Shutdown(reason="BLOCKED_M23BV_CAMERA_READY"))]

    def camera_exited(_event, _ctx):
        # Reject camera loss even if a previous readiness check succeeded.
        return [EmitEvent(event=Shutdown(reason="BLOCKED_M23BV_CAMERA_EXIT"))]

    return [
        RegisterEventHandler(OnProcessExit(target_action=ready,
                                           on_exit=ready_exited)),
        RegisterEventHandler(OnProcessExit(target_action=camera,
                                           on_exit=camera_exited)),
        camera, ready,
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=_camera_only)])
