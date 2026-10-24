#!/usr/bin/env python3
"""Read-only M23BV V3 Stage 2 source inventory. NEVER launches ROS or hardware.

Stage 2 physical MCU serial and bag recording are NOT authorized.
This gate checks pinned existing source contracts only, not runtime behavior.
"""
import ast
import hashlib
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
PINS = {
    "m23bv_v3_stage1_camera_run.sh": "266ff7ff71ed3e1b77ce8d881b798a9f3df70196",
    "m23bv_v3_stage1_camera_observer.py": "93320b985c5dda3fdf0d1f8def001ba06480e7b6",
    "m23bv_reliable_rectifier_v3_candidate.launch.py": "a0ae75a62da39235b113e3d2c324efda5fcfc6e1",
    "m23bv_reliable_rectifier_proto/src/rectify.cpp": "7e5d199e633f7cf0dac266a656ae70849df07f71",
    "m20_sensor_only_run.sh": "88c300b174f6e53bd536467f2e445d22e5ed1f8c",
    "m20_combined_observer.py": "5cf2f88cb2f79be80821888401b57a6393ee5e3b",
    "m20_combined_contract.py": "66111d8da486dc1e0c78057c1948f5e1aad9a06e",
    "m18_imu_stream_source.py": "3c5fb37ca291533e2702a9159f3e8db11b076d22",
    "m23bv_sensor_only_run.sh": "8e0db7a1f223fc992ba7704039e9ceb5a48465a4",
    "m23bv_qos_guard.py": "0e744818d819be6f8601d9be617777cfb3d4b7f3",
    "m23bv_bounded_capture.py": "81ae4eb4d31f5453e2a6f573da36385d38d7f659",
}


def require(condition, reason):
    if not condition:
        raise ValueError("BLOCKED_" + reason)


def git_blob(payload):
    return hashlib.sha1(b"blob " + str(len(payload)).encode() +
                        b"\0" + payload).hexdigest()


def inspect(root=ROOT):
    sources = {}
    for path, expected in PINS.items():
        candidate = root / path
        require(candidate.is_file() and not candidate.is_symlink(),
                "SOURCE_MISSING_OR_LINK:" + path)
        payload = candidate.read_bytes()
        require(git_blob(payload) == expected, "SOURCE_PIN_DRIFT:" + path)
        sources[path] = payload.decode("utf-8")

    def has(path, *tokens):
        for token in tokens:
            require(token in sources[path], "SOURCE_CONTRACT:" + path + ":" + token)

    stage1 = "m23bv_v3_stage1_camera_run.sh"
    has(stage1, '"$camera_dev:/dev/video2:rw"',
        "M23BV_STAGE1_CAMERA_ONLY_APPROVED",
        "M23BV_STAGE1_MOTOR_RAILS_ISOLATED",
        "CALIBRATION VERIFIED FOR CAMERA")
    require("/dev/ttyACM0" not in sources[stage1], "STAGE1_SERIAL_REFERENCE")
    has("m23bv_v3_stage1_camera_observer.py",
        "MIN_RECT_FRAMES = 90", "M23BV_V3_STAGE1_QOS=PASS",
        "M23BV_V3_STAGE1_FRAMES=PASS",
        "M23BV_V3_STAGE1_LAUNCH_REAPED=")
    v3 = "m23bv_reliable_rectifier_v3_candidate.launch.py"
    has(v3, "m23bv_rectify_reliable_node", "M23BV_CAMERA_READY_CANDIDATE",
        "M23BV_RELIABLE_RECTIFIER_PROTO_V3",
        "BLOCKED_M23BV_CAMERA_EXIT")

    selected = "m23bv_sensor_only_run.sh"
    has(selected, "ros2 launch /opt/m23bv/m23bv_camera_only.launch.py",
        "m20_combined_observer.py", "m23bv_qos_guard.py")
    require("m23bv_reliable_rectifier_v3_candidate.launch.py" not in
            sources[selected], "LEGACY_SELECTOR_ALREADY_CHANGED")

    m20 = "m20_combined_observer.py"
    has(m20, "with serial.Serial(", "check_isolated_graph(src)",
        "send_only_whitelisted_serial_command(device, START)",
        "send_only_whitelisted_serial_command(device, STOP)",
        "M20_IMU_STREAM_START_ACK=PASS",
        "M20_IMU_STREAM_STOP_ACK=")
    # M18 owns START/STOP byte definitions via imported immutable contract;
    # check the whitelist function itself without guessing the literal bytes.
    has("m18_imu_stream_source.py", "if payload not in (START, STOP):",
        "device.write(payload)")
    has("m20_sensor_only_run.sh",
        "ros2 launch /opt/m20/m20_camera_only.launch.py",
        "M20_SHUTDOWN=SENSOR_PROCESSES_EXITED")
    has("m20_combined_contract.py",
        "DURATION_S = 120", "MIN_IMU_MESSAGES = 5700",
        "MIN_IMAGE_MESSAGES = 400", "MIN_INFO_MESSAGES = 400",
        "MIN_OVERLAP_S = 90.0")

    # The currently selected camera/recording graph guard is NOT reusable
    # unchanged for Stage 2 without recording.
    has("m23bv_qos_guard.py", "rosbag2_recorder",
        "BLOCKED_RECORDER_IMAGE_QOS")
    has("m23bv_bounded_capture.py", '"ros2", "bag", "record"',
        "M23BV_OPERATOR_CAPTURE_APPROVED")

    for path, content in sources.items():
        if path.endswith(".py"):
            ast.parse(content, filename=path)
    print("M23BV_V3_STAGE2_SOURCE_PINS=PASS")
    print("M23BV_V3_STAGE2_LEGACY_SELECTOR=UNCHANGED_NOT_V3")
    print("M23BV_V3_STAGE2_RECORDER_DEPENDENT_QOS_GUARD=IDENTIFIED")
    print("M23BV_V3_STAGE2_MCU_OR_RECORDING=NOT_AUTHORIZED")
    print("M23BV_V3_STAGE2_PHYSICAL_DEVICE_ACCESS=NONE")
    print("M23BV_V3_STAGE2_SOURCE_REVIEW=PASS")


if __name__ == "__main__":
    try:
        inspect()
    except (OSError, UnicodeError, SyntaxError, ValueError) as exc:
        print("M23BV_V3_STAGE2_SOURCE_REVIEW=BLOCKED:" + str(exc),
              file=sys.stderr)
        sys.exit(2)
