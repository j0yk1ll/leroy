#!/usr/bin/env python3
"""M23C real XRSLAM C++ VIO-path test using SYNTHETIC CDR inputs only.

No physical robot, USB/serial/video device, production bridge, Nav2 or
motor authority. The EuRoC YAML, identity static TF, gyro/accel samples
and camera pixels are expressly NOT calibrated Leroy measurements.
"""
from collections import deque
import json
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from m23a_xrslam_rosbag_preflight import inspect_bag
from m23b_real_xrslam_monocular_replay_selftest import (
    ReplayNode, _bounded_log_reader, pump,
)
from m23c_synthetic_vio_bag import (
    DROPOUT_NS, FRAME_COUNT, IMU_COUNT, LAST_IMU_NS, IMU_TOPIC,
    synthetic_vio_records, write_synthetic_vio_bag,
)

ENVIRONMENT = {
    "LEROY_NAV2_DRY_RUN": "1",
    "M23C_SYNTHETIC_VIO_ONLY": "1",
    "ROS_DOMAIN_ID": "173",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
EXECUTABLE = Path(
    "/leroy_ws/install/leroy_xrslam/lib/leroy_xrslam/leroy_xrslam_node")
SLAM_SOURCE = Path("/opt/xrslam/configs/euroc_slam.yaml")
TEMPLATE_SOURCE = Path("/opt/xrslam/configs/euroc_sensor.yaml")
DISCOVERY_LIMIT_S = 12.0


def require(value, reason):
    if not value:
        raise AssertionError("M23C: " + reason)


class VioReplayNode(ReplayNode):
    def __init__(self):
        super().__init__()
        self.imu_pub = self.create_publisher(
            Imu, IMU_TOPIC, qos_profile_sensor_data)

    def publish_synthetic_tf(self):
        transforms = []
        for child in ("camera_optical_frame", "imu_link"):
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_link"
            tf.child_frame_id = child
            tf.transform.rotation.w = 1.0
            transforms.append(tf)
        # One /tf_static sample containing both entirely FAKE identities.
        self.static_tf.sendTransform(transforms)


def _synthetic_example(path, output, *, mode=None):
    content = path.read_text(encoding="utf-8")
    if content.startswith("%YAML:1.0"):
        content = content.split("\n", 1)[1]
    if mode == "monocular_imu":
        require("sensor:" not in content,
                "unexpected upstream sensor section; refuse overwrite")
        content += "\nsensor:\n  mode: monocular_imu\n"
    require(content.strip(), "upstream example YAML is empty")
    output.write_text(
        "# TEST-ONLY UPSTREAM EXAMPLE; NOT LEROY CALIBRATION\n" + content,
        encoding="utf-8")
    return output


def _late_synthetic_imu(stamp_ns):
    msg = Imu()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = "imu_link"
    msg.angular_velocity.z = 0.01
    msg.linear_acceleration.z = 9.81
    return msg


def run():
    for name, expected in ENVIRONMENT.items():
        require(os.environ.get(name) == expected,
                "isolation environment " + name)
    for dev in ("/dev/ttyACM0", "/dev/ttyACM1",
                "/dev/video2", "/dev/video8"):
        require(not Path(dev).exists(), "unexpected device " + dev)
    for source in (EXECUTABLE, SLAM_SOURCE, TEMPLATE_SOURCE):
        require(source.is_file(), "missing base-image dependency " +
                str(source))
    require(os.access(EXECUTABLE, os.X_OK), "XRSLAM executable not runnable")

    with tempfile.TemporaryDirectory(prefix="m23c-synthetic-vio-") as temp:
        work = Path(temp)
        slam = _synthetic_example(
            SLAM_SOURCE, work / "uncalibrated_synthetic_slam.yaml",
            mode="monocular_imu")
        example = _synthetic_example(
            TEMPLATE_SOURCE, work / "uncalibrated_synthetic_sensor.yaml")
        require("cam0:" in example.read_text() and
                "imu:" in example.read_text(),
                "upstream sensor example missing expected keys")
        bag = write_synthetic_vio_bag(work / "synthetic_vio_frames")
        preflight = inspect_bag(
            bag, storage_id="sqlite3", mode="monocular_imu",
            provenance="synthetic", camera_frame="camera_optical_frame",
            imu_frame="imu_link")
        require(preflight["verdict"] == "REPLAY_INPUT_CONTRACT_PASS",
                "M23A input contract rejected fabricated bag: " +
                str(preflight["faults"]))
        require(preflight["counts"] == {
            "image": FRAME_COUNT, "info": FRAME_COUNT, "imu": IMU_COUNT},
            "fixture CDR counts do not match")
        print("M23C_SYNTHETIC_VIO_BAG_PREFLIGHT=PASS", flush=True)

        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))
        rclpy.init()
        node = None
        proc = None
        log_thread = None
        logs = deque(maxlen=500)
        lock = threading.Lock()
        delivered = {"image": 0, "info": 0, "imu": 0}

        def recent():
            with lock:
                return list(logs)

        try:
            node = VioReplayNode()
            node.assert_no_motion_authority()
            node.publish_synthetic_tf()
            # The offset 0.0 is only a FIXTURE PARAMETER.
            # It is NOT a measured camera exposure/MCU sampling offset.
            cmd = [
                str(EXECUTABLE), "--ros-args",
                "-p", "slam_yaml:=" + str(slam),
                "-p", "device_template_yaml:=" + str(example),
                "-p", "image_topic:=/camera/image_rect",
                "-p", "camera_info_topic:=/camera/camera_info",
                "-p", "imu_topic:=/imu/data_raw",
                "-p", "camera_optical_frame:=camera_optical_frame",
                "-p", "imu_frame:=imu_link",
                "-p", "camera_imu_offset_sec:=0.0",
                "-p", "pose_convention_validated:=false",
                "-p", "enable_map_tf:=false",
            ]
            proc = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, bufsize=1)
            log_thread = threading.Thread(
                target=_bounded_log_reader, args=(proc, logs, lock),
                daemon=True)
            log_thread.start()

            deadline = time.monotonic() + DISCOVERY_LIMIT_S
            while time.monotonic() < deadline:
                pump(node, 0.1, proc)
                if (node.count_subscribers("/camera/camera_info") == 1
                        and node.count_subscribers("/camera/image_rect") == 1
                        and node.count_subscribers(IMU_TOPIC) == 1):
                    break
            else:
                raise AssertionError("VIO ROS subscriptions not discovered")
            print("M23C_REAL_XRSLAM_VIO_SUBSCRIPTIONS=PASS", flush=True)

            first_info = next(
                message for at, topic, message in synthetic_vio_records()
                if topic == "/camera/camera_info")
            deadline = time.monotonic() + DISCOVERY_LIMIT_S
            while time.monotonic() < deadline:
                node.publish_synthetic_tf()
                node.info_pub.publish(first_info)
                pump(node, 0.12, proc)
                if "initializing" in node.status:
                    break
            else:
                raise AssertionError(
                    "VIO initialization not observed: " + str(recent()[-10:]))
            print("M23C_REAL_XRSLAM_VIO_INITIALIZING=PASS", flush=True)

            # Use only serialized messages from the preflighted bag.
            while reader.has_next():
                topic, payload, _receipt_ns = reader.read_next()
                if topic == "/camera/camera_info":
                    node.info_pub.publish(
                        deserialize_message(payload, CameraInfo))
                    delivered["info"] += 1
                elif topic == "/camera/image_rect":
                    node.image_pub.publish(
                        deserialize_message(payload, Image))
                    delivered["image"] += 1
                elif topic == IMU_TOPIC:
                    node.imu_pub.publish(
                        deserialize_message(payload, Imu))
                    delivered["imu"] += 1
                else:
                    raise AssertionError("forbidden bag topic: " + topic)
                pump(node, 0.025, proc)
            pump(node, 0.8, proc)
            require(delivered == {
                "image": FRAME_COUNT, "info": FRAME_COUNT, "imu": IMU_COUNT},
                "CDR replay incomplete")
            # C++ logs a frame only after the camera can queue;
            # with VIO enabled that requires a real forwarded IMU pair.
            require(any(
                "Submitted 1 camera frames to XRSLAM (monocular_imu)"
                in line for line in recent()),
                "no VIO camera submission receipt: " +
                str(recent()[-14:]))
            print("M23C_REAL_XRSLAM_SYNTHETIC_VIO_SUBMITTED=PASS",
                  flush=True)

            # This gap deliberately violates normal IMU constraints.
            # It is injected OUTSIDE the healthy bag to exercise the
            # actual C++ adapter's terminal imu_gap status.
            node.imu_pub.publish(
                _late_synthetic_imu(LAST_IMU_NS + DROPOUT_NS))
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                pump(node, 0.05, proc)
                if "imu_gap" in node.status:
                    break
            require("imu_gap" in node.status,
                    "synthetic IMU dropout did not latch imu_gap: " +
                    str(node.status[-8:]) + " " + str(recent()[-10:]))
            print("M23C_REAL_XRSLAM_IMU_GAP_LATCH=PASS", flush=True)

            # Recovery-shaped IMU data does not clear the fault latch.
            node.imu_pub.publish(
                _late_synthetic_imu(LAST_IMU_NS + DROPOUT_NS +
                                    20_000_000))
            pump(node, 0.5, proc)
            require(node.status[-1] == "imu_gap",
                    "IMU gap fault state was automatically rearmed")
            node.assert_no_motion_authority()
            require(node.validity and not any(node.validity),
                    "unvalidated XRSLAM ever published true localization")
            print("M23C_VIO_FAULT_STAYS_LATCHED_NO_ODOM=PASS",
                  flush=True)
            print("M23C_REPORT_JSON=" + json.dumps({
                "milestone": "M23C",
                "evidence_type": "SYNTHETIC_CAMERA_IMU_TO_REAL_XRSLAM",
                "cdr_images_replayed": delivered["image"],
                "cdr_camera_info_replayed": delivered["info"],
                "cdr_imu_replayed": delivered["imu"],
                "synthetic_imu_rate_hz": 50,
                "synthetic_camera_period_ms": 125,
                "injected_imu_gap_ms": DROPOUT_NS // 1_000_000,
                "real_vio_camera_submission_observed": True,
                "imu_gap_latched": True,
                "odom_messages": node.odom_receipts,
                "dynamic_tf_messages": node.dynamic_tf_receipts,
                "cmd_vel_publishers": 0,
                "pose_convention_validated": False,
                "enable_map_tf": False,
                "camera_imu_time_offset_calibrated": False,
                "extrinsics_validated": False,
                "recording_is_real": False,
                "xrslam_tracking_accuracy_validated": False,
                "physical_motion_authorized": False,
                "physical_device_access": False,
            }, sort_keys=True), flush=True)
            print("M23C_REAL_XRSLAM_SYNTHETIC_VIO_SELFTEST=PASS",
                  flush=True)
            print("M23C_VERDICT=REAL_XRSLAM_SYNTHETIC_VIO_FAULT_ONLY",
                  flush=True)
            print("M23C_DEVICE_ACCESS=NONE", flush=True)
            return 0
        finally:
            if proc is not None:
                if proc.poll() is None:
                    proc.terminate()
                    try:
                        proc.wait(timeout=3)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait(timeout=3)
                if log_thread is not None:
                    log_thread.join(timeout=2)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M23C_REAL_XRSLAM_SYNTHETIC_VIO_SELFTEST=FAIL", flush=True)
        print("M23C_ERROR=" + type(exc).__name__ + ": " + str(exc),
              flush=True)
        raise
