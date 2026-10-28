#!/usr/bin/env python3
"""Source-only Stage 3 camera/IMU timestamp and extrinsic provenance inventory.

No ROS nodes, containers, camera/serial I/O, recording, or motor control.
Only reads pinned local Git sources. It does not assert any calibration
has been validated from camera/IMU sample counts or a clock-rate estimate.
"""
import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

ROS = Path(__file__).resolve().parent
ROOT = ROS.parent
EXPECTED_BRANCH = "feature/m23bv-v3-stage3-fullstack-diagnostic"
PINS = {
    "ros/m23bv_reliable_rectifier_v3_candidate.launch.py":
        "a0ae75a62da39235b113e3d2c324efda5fcfc6e1",
    "ros/m23bv_reliable_rectifier_proto/src/rectify.cpp":
        "7e5d199e633f7cf0dac266a656ae70849df07f71",
    "ros/config/camera/front_camera.yaml":
        "a2536ccdc02979087bc18f61789686bd52e23371",
    "ros/src/leroy_description/urdf/leroy.urdf.xacro":
        "353e79ac7bd349c0291f7e5c07e5473fde6f9ff6",
    "ros/src/leroy_bridge/leroy_bridge/clock_sync.py":
        "ed13ea895bc9e879d5300efbe83806e520cf96b6",
    "ros/m18_imu_stream_source.py":
        "3c5fb37ca291533e2702a9159f3e8db11b076d22",
    "ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp":
        "6e4b84f0afbafcdc79eb67e43d86ac33549d8137",
    "ros/m23bv_v3_stage3_xrslam_vio_diag.launch.py":
        "5ba365b635133a930153aec4cd3d2d995bb0cd59",
}
CAL_SHA256 = "f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138"


def git(*args):
    return subprocess.run(
        ["git", "-C", str(ROOT), *args], check=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
    ).stdout.strip()


def assert_git_clean_and_pinned():
    if git("branch", "--show-current") != EXPECTED_BRANCH:
        raise ValueError("BRANCH_NOT_STAGE3")
    if git("status", "--porcelain"):
        raise ValueError("WORKTREE_NOT_CLEAN")
    for path, expected in PINS.items():
        file = ROOT / path
        if not file.is_file() or file.is_symlink():
            raise ValueError("SOURCE_MISSING_OR_SYMLINK:" + path)
        actual = git("hash-object", "--", str(file))
        committed = git("rev-parse", "HEAD:" + path)
        if actual != expected or committed != expected:
            raise ValueError("ACCEPTED_SOURCE_HASH_CHANGED:" + path)
    if hashlib.sha256((ROOT / "ros/config/camera/front_camera.yaml").read_bytes()).hexdigest() != CAL_SHA256:
        raise ValueError("CAMERA_CALIBRATION_BYTES_CHANGED")


def dictionary_literals(filename):
    result = {}
    source = (ROOT / filename).read_text()
    for node in ast.walk(ast.parse(source, filename=filename)):
        if isinstance(node, ast.Dict):
            for key, value in zip(node.keys, node.values):
                if (isinstance(key, ast.Constant) and isinstance(key.value, str)
                        and isinstance(value, ast.Constant)):
                    result[key.value] = value.value
    return result


def inspect_camera():
    source = (ROOT / "ros/m23bv_reliable_rectifier_v3_candidate.launch.py").read_text()
    params = dictionary_literals("ros/m23bv_reliable_rectifier_v3_candidate.launch.py")
    expected = {
        "video_device": "/dev/video2",
        "image_width": 640, "image_height": 480,
        "framerate": 30.0, "io_method": "mmap",
        "pixel_format": "mjpeg2rgb", "frame_id": "camera_optical_frame",
        "camera_info_url": "file:///config/camera/front_camera.yaml",
    }
    if any(params.get(key) != value for key, value in expected.items()):
        raise ValueError("CAMERA_LAUNCH_PARAMETERS_CHANGED")
    if 'package="usb_cam", executable="usb_cam_node_exe"' not in source:
        raise ValueError("CAMERA_DRIVER_CHANGED")
    rect = (ROOT / "ros/m23bv_reliable_rectifier_proto/src/rectify.cpp").read_text()
    if ("cv_bridge::CvImage(image_msg->header, image_msg->encoding, rect).toImageMsg()"
            not in rect or "pub_rect_.publish(image_msg)" not in rect):
        raise ValueError("RECTIFIER_HEADER_LINEAGE_CHANGED")
    print("STAGE3_CAL_CAMERA_DRIVER=usb_cam_node_exe")
    print("STAGE3_CAL_CAMERA_MODE=640x480_30fps_MJPEG2RGB_MMAP_CONFIGURED")
    print("STAGE3_CAL_CAMERA_OPTICAL_FRAME=camera_optical_frame")
    print("STAGE3_CAL_IMAGE_HEADER_RECTIFIER=PRESERVED")
    print("STAGE3_CAL_CAMERA_INFO_STAMP=PREVIOUSLY_OBSERVED_MATCHING")
    print("STAGE3_CAL_CAMERA_EXPOSURE_STAMP=NOT_MEASURED")
    print("STAGE3_CAL_V4L2_BUFFER_TIME_DOMAIN=NOT_VERIFIED_FOR_PINNED_BINARY")


def inspect_mcu():
    mapper = (ROOT / "ros/src/leroy_bridge/leroy_bridge/clock_sync.py").read_text()
    sender = (ROOT / "ros/m18_imu_stream_source.py").read_text()
    tokens = (
        "_INITIAL_TRANSPORT_GUARD_NS = 5_000_000",
        "self._last_stamp_ns = received_ros_ns - self._INITIAL_TRANSPORT_GUARD_NS",
        "mapped = self._last_stamp_ns + round(delta_us * 1000 * self._rate)",
    )
    if not all(token in mapper for token in tokens):
        raise ValueError("MCU_MAPPER_CONTRACT_CHANGED")
    if not all(token in sender for token in (
        "stamp_ns = self._mapper.observe(",
        'msg.header.frame_id = "imu_link"',
        "msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(",
    )):
        raise ValueError("M18_STAMP_OR_FRAME_CHANGED")
    print("STAGE3_CAL_MCU_TIME=MCU_TICKS_RATE_MAPPED_TO_ROS_RECEIPT_EPOCH")
    print("STAGE3_CAL_MCU_FIRST_STAMP_GUARD_MS=5")
    print("STAGE3_CAL_MCU_TRUE_TRANSPORT_LATENCY=UNKNOWN")
    print("STAGE3_CAL_CAMERA_IMU_ABSOLUTE_PHASE=UNMEASURED")


def inspect_geometry_and_slam_gate():
    urdf = ET.parse(ROOT / "ros/src/leroy_description/urdf/leroy.urdf.xacro")
    joints = {}
    for joint in urdf.findall(".//joint"):
        origin = joint.find("origin")
        if origin is not None:
            joints[joint.attrib["name"]] = origin.attrib
    required = ("base_to_imu", "base_to_front_camera",
                "front_camera_to_optical",
                "front_optical_to_legacy_camera_optical")
    if any(x not in joints for x in required):
        raise ValueError("CANONICAL_CAMERA_IMU_TF_CHAIN_CHANGED")
    imu = joints["base_to_imu"]
    cam = joints["base_to_front_camera"]
    if (imu.get("xyz") != "0.095 -0.095 0.0"
            or imu.get("rpy") != "0 0 0"
            or cam.get("xyz") != "0 0 0"
            or cam.get("rpy") != "0 0.5235987755982988 0"):
        raise ValueError("URDF_PROVISIONAL_GEOMETRY_CHANGED")
    params = dictionary_literals("ros/m23bv_v3_stage3_xrslam_vio_diag.launch.py")
    if (params.get("camera_imu_offset_sec") != 0.0
            or params.get("pose_convention_validated") is not False
            or params.get("enable_map_tf") is not False):
        raise ValueError("XRSLAM_DIAGNOSTIC_POSE_SAFETY_GATE_CHANGED")
    print("STAGE3_CAL_BASE_TO_IMU_XYZ_M=" + json.dumps([0.095, -0.095, 0.0]))
    print("STAGE3_CAL_BASE_TO_IMU_RPY_RAD=" + json.dumps([0.0, 0.0, 0.0]))
    print("STAGE3_CAL_BASE_TO_FRONT_CAMERA_XYZ_M=" + json.dumps([0.0, 0.0, 0.0]))
    print("STAGE3_CAL_BASE_TO_FRONT_CAMERA_RPY_RAD=" +
          json.dumps([0.0, 0.5235987755982988, 0.0]))
    print("STAGE3_CAL_URDF_EXTRINSICS=PROVISIONAL_NOT_PHYSICALLY_VERIFIED")
    print("STAGE3_CAL_XRSLAM_OFFSET_SEC=0.0_PROVISIONAL_ONLY")
    print("STAGE3_CAL_XRSLAM_POSE_AND_MAP_TF=INHIBITED")


def main():
    assert_git_clean_and_pinned()
    print("STAGE3_CAL_PINNED_SOURCE_BYTES=PASS")
    print("STAGE3_CAL_FRONT_CAMERA_INTRINSICS_SHA256=" + CAL_SHA256)
    inspect_camera()
    inspect_mcu()
    inspect_geometry_and_slam_gate()
    print("STAGE3_CAL_PROVENANCE=PASS_SOURCE_ONLY_UNCALIBRATED")
    print("STAGE3_CAL_SOURCE_CHECK_HARDWARE_ACCESS=NONE")
    print("STAGE3_CAL_MOTION_AND_RECORDING=NOT_AUTHORIZED")


if __name__ == "__main__":
    try:
        main()
    except (OSError, ValueError, subprocess.CalledProcessError, ET.ParseError,
            SyntaxError, KeyError) as exc:
        print("STAGE3_CAL_PROVENANCE=BLOCKED_" + type(exc).__name__ +
              ":" + str(exc), file=sys.stderr)
        sys.exit(2)
