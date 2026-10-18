#!/usr/bin/env python3
"""M23B: actual XRSLAM ROS node + synthetic monocular rosbag, NO hardware.

Runs in isolated no-device Podman ONLY. Camera pixels, CameraInfo, and
base->camera TF are synthetic and NOT calibrated. The real C++ XRSLAM
adapter must ingest a frame while it remains unable to publish /odom.
No MCU, bridge, Nav2, motors, physical camera, /cmd_vel, or live data.
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
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String
from tf2_ros import StaticTransformBroadcaster

from m23a_xrslam_rosbag_preflight import inspect_bag
from m23b_synthetic_monocular_bag import (
    FRAME_COUNT, write_synthetic_bag,
)

EXPECTED_ENV = {
    "LEROY_NAV2_DRY_RUN": "1",
    "M23B_SYNTHETIC_XRSLAM_ONLY": "1",
    "ROS_DOMAIN_ID": "172",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
EXECUTABLE = Path(
    "/leroy_ws/install/leroy_xrslam/lib/leroy_xrslam/leroy_xrslam_node")
SLAM = Path("/opt/xrslam/configs/visual_only_slam.yaml")
TEMPLATE = Path("/opt/xrslam/configs/euroc_sensor.yaml")
TIMEOUT_S = 12.0


def require(condition, message):
    if not condition:
        raise AssertionError("M23B: " + message)


class ReplayNode(Node):
    def __init__(self):
        super().__init__("m23b_synthetic_monocular_replay")
        self.info_pub = self.create_publisher(
            CameraInfo, "/camera/camera_info", qos_profile_sensor_data)
        self.image_pub = self.create_publisher(
            Image, "/camera/image_rect", qos_profile_sensor_data)
        self.static_tf = StaticTransformBroadcaster(self)
        self.status = []
        self.validity = []
        self.odom_receipts = 0
        status_qos = QoSProfile(
            depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)
        self.status_sub = self.create_subscription(
            String, "/xrslam/tracking_status",
            lambda msg: self.status.append(msg.data), status_qos)
        self.valid_sub = self.create_subscription(
            Bool, "/xrslam/localization_valid",
            lambda msg: self.validity.append(msg.data), status_qos)
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._on_odom, 10)

    def _on_odom(self, _msg):
        self.odom_receipts += 1

    def publish_synthetic_tf(self):
        # Identity only in the synthetic isolated test. NEVER robot calibration.
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "base_link"
        tf.child_frame_id = "camera_optical_frame"
        tf.transform.rotation.w = 1.0
        self.static_tf.sendTransform(tf)

    def assert_no_motion_authority(self):
        require(not self.get_publishers_info_by_topic("/cmd_vel"),
                "unexpected raw cmd_vel publisher")
        require(self.odom_receipts == 0,
                "XRSLAM published odom despite pose_convention_validated=false")
        require(not any(self.validity),
                "XRSLAM published a true localization flag")
        # The adapter advertises /odom but intentionally does not emit any
        # Odometry while pose convention validation is FALSE.


def _bounded_log_reader(proc, out, lock):
    try:
        for line in proc.stdout:
            with lock:
                out.append(line.rstrip("\n")[:500])
    finally:
        proc.stdout.close()


def pump(node, seconds, proc=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        if proc is not None and proc.poll() is not None:
            raise RuntimeError("XRSLAM process exited early: " +
                               str(proc.returncode))
        rclpy.spin_once(node, timeout_sec=0.025)
        node.assert_no_motion_authority()


def run():
    for key, expected in EXPECTED_ENV.items():
        require(os.environ.get(key) == expected,
                "isolation environment " + key)
    for device in ("/dev/ttyACM0", "/dev/ttyACM1",
                   "/dev/video2", "/dev/video8"):
        require(not Path(device).exists(),
                "forbidden physical device present " + device)
    for path in (EXECUTABLE, SLAM, TEMPLATE):
        require(path.is_file(), "missing test dependency " + str(path))
    require(os.access(EXECUTABLE, os.X_OK), "XRSLAM binary not executable")

    with tempfile.TemporaryDirectory(prefix="m23b-synthetic-monocular-") as temp:
        # The upstream EuRoC example begins with an OpenCV-specific
        # %YAML:1.0 directive, which standard yaml-cpp may reject.
        # Make a separate *test-only* copy; never alter the input asset
        # or present this sample as Leroy-calibrated sensor values.
        example = TEMPLATE.read_text(encoding="utf-8")
        if example.startswith("%YAML:1.0"):
            example = example.split("\n", 1)[1]
        require("cam0:" in example and "imu:" in example,
                "unexpected upstream synthetic fixture format")
        fixture_template = Path(temp) / "uncalibrated_euroc_example.yaml"
        fixture_template.write_text(
            "# TEST-ONLY EXAMPLE, NOT LEROY CALIBRATION\n" + example,
            encoding="utf-8")
        bag = write_synthetic_bag(Path(temp) / "frames")
        report = inspect_bag(
            bag, storage_id="sqlite3", mode="monocular",
            provenance="synthetic", camera_frame="camera_optical_frame",
            imu_frame="imu_link")
        require(report["verdict"] == "REPLAY_INPUT_CONTRACT_PASS",
                "preflight blocked synthetic monocular bag: " +
                str(report["faults"]))
        require(report["counts"] == {
            "image": FRAME_COUNT, "info": FRAME_COUNT, "imu": 0},
            "fixture topic count mismatch")
        print("M23B_SYNTHETIC_MONOCULAR_BAG_PREFLIGHT=PASS", flush=True)

        import rosbag2_py
        from rclpy.serialization import deserialize_message
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("cdr", "cdr"))

        rclpy.init()
        node = None
        proc = None
        reader_thread = None
        logs = deque(maxlen=500)
        log_lock = threading.Lock()

        def recent_log_lines():
            with log_lock:
                return list(logs)

        delivered = {"image": 0, "info": 0}
        try:
            node = ReplayNode()
            require(not node.get_publishers_info_by_topic("/cmd_vel"),
                    "raw /cmd_vel exists before startup")
            node.publish_synthetic_tf()
            cmd = [
                str(EXECUTABLE), "--ros-args",
                "-p", "slam_yaml:=" + str(SLAM),
                "-p", "device_template_yaml:=" + str(fixture_template),
                "-p", "image_topic:=/camera/image_rect",
                "-p", "camera_info_topic:=/camera/camera_info",
                "-p", "camera_optical_frame:=camera_optical_frame",
                "-p", "pose_convention_validated:=false",
                "-p", "enable_map_tf:=false",
            ]
            proc = subprocess.Popen(
                cmd, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT, text=True, bufsize=1)
            reader_thread = threading.Thread(
                target=_bounded_log_reader, args=(proc, logs, log_lock), daemon=True)
            reader_thread.start()

            # Graph discovery is asynchronous; wait for actual XRSLAM
            # subscriptions, not merely for a subprocess PID to exist.
            deadline = time.monotonic() + TIMEOUT_S
            while time.monotonic() < deadline:
                pump(node, 0.1, proc)
                if (node.count_subscribers("/camera/camera_info") == 1
                        and node.count_subscribers("/camera/image_rect") == 1):
                    break
            else:
                raise AssertionError("XRSLAM sensor subscriptions undiscovered")
            print("M23B_REAL_XRSLAM_ROS_SUBSCRIPTIONS=PASS", flush=True)

            # The camera-info callback and synthetic static TF must arrive
            # before the adapter can call XRSLAMCreate. The repeated
            # first CameraInfo is test-only, not a live sensor retry.
            from m23b_synthetic_monocular_bag import synthetic_records
            first_info = next(
                msg for _at, topic, msg in synthetic_records()
                if topic == "/camera/camera_info")
            init_end = time.monotonic() + TIMEOUT_S
            while time.monotonic() < init_end:
                node.info_pub.publish(first_info)
                node.publish_synthetic_tf()
                pump(node, 0.12, proc)
                if "initializing" in node.status:
                    break
            else:
                raise AssertionError(
                    "real XRSLAM adapter did not reach initializing: " +
                    str(recent_log_lines()[-8:]))
            print("M23B_REAL_XRSLAM_SYNTHETIC_INITIALIZING=PASS", flush=True)

            # Replay ONLY already-generated bag CDR sensor messages.
            # The node has no service, action, command or motor publisher.
            while reader.has_next():
                topic, blob, _bag_time = reader.read_next()
                require(topic in (
                    "/camera/camera_info", "/camera/image_rect"),
                    "unapproved rosbag topic")
                if topic == "/camera/camera_info":
                    node.info_pub.publish(
                        deserialize_message(blob, CameraInfo))
                    delivered["info"] += 1
                else:
                    node.image_pub.publish(
                        deserialize_message(blob, Image))
                    delivered["image"] += 1
                pump(node, 0.08, proc)
            pump(node, 1.5, proc)
            require(delivered["image"] == FRAME_COUNT
                    and delivered["info"] == FRAME_COUNT,
                    "synthetic CDR replay incomplete")

            # Console receipt originates in the exact C++ adapter's
            # XRSLAMPushSensorData -> XRSLAMRunOneFrame drain path.
            submitted = any("Submitted 1 camera frames to XRSLAM (monocular)"
                            in line for line in recent_log_lines())
            require(submitted,
                    "no actual XRSLAM first-camera submission receipt: " +
                    str(recent_log_lines()[-12:]))
            print("M23B_REAL_XRSLAM_MONOCULAR_FRAME_SUBMITTED=PASS",
                  flush=True)
            node.assert_no_motion_authority()
            require(node.validity and not any(node.validity),
                    "missing false localization status")
            print("M23B_ODOM_AND_CMD_OUTPUT_INHIBITED=PASS", flush=True)
            print("M23B_REPORT_JSON=" + json.dumps({
                "milestone": "M23B",
                "evidence_type": "SYNTHETIC_MONOCULAR_BAG_TO_REAL_XRSLAM",
                "cdr_images_replayed": delivered["image"],
                "cdr_camera_info_replayed": delivered["info"],
                "xrslam_camera_submission_observed": True,
                "status_receipts": len(node.status),
                "odom_messages": node.odom_receipts,
                "cmd_vel_publishers": 0,
                "imu_used": False,
                "pose_convention_validated": False,
                "enable_map_tf": False,
                "real_recording_validated": False,
                "xrslam_tracking_validated": False,
                "extrinsics_validated": False,
                "physical_device_access": False,
                "physical_motion_authorized": False,
            }, sort_keys=True), flush=True)
            print("M23B_REAL_XRSLAM_SYNTHETIC_REPLAY_SELFTEST=PASS",
                  flush=True)
            print("M23B_VERDICT=REAL_XRSLAM_MONOCULAR_SYNTHETIC_ONLY",
                  flush=True)
            print("M23B_DEVICE_ACCESS=NONE", flush=True)
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
                if reader_thread is not None:
                    reader_thread.join(timeout=2)
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M23B_REAL_XRSLAM_SYNTHETIC_REPLAY_SELFTEST=FAIL",
              flush=True)
        print("M23B_ERROR=" + type(exc).__name__ + ": " + str(exc),
              flush=True)
        raise
