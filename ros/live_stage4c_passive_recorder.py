#!/usr/bin/env python3
"""Stage 4C passive ROS bag capture candidate — NOT AUTHORIZED FOR LIVE USE.

Reader only: subscribes to images, CameraInfo, IMU and source MCU metadata.
It never opens any device or publishes ROS topics. Its live entry point is
deliberately hard-disabled pending independent source and physical review.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time

from live_stage4c_capture_contract import (
    CaptureContract, CaptureError, METADATA_TOPIC, TOPICS,
)
from live_stage4d_capture_ingress import CaptureIngress

# Deliberately hard-disabled in this source candidate. No environment
# variable or command line parameter can bypass this block.
LIVE_CAPTURE_COMMISSIONED = False
REQUIRED_ENV = {
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "LEROY_NAV2_DRY_RUN": "1",
    "LEROY_STAGE4C_CAPTURE_ONLY": "1",
}
REQUIRED_NODES = {
    "usb_cam", "rectify_node", "depthart", "depth_obstacles",
    "costmap", "leroy_xrslam", "m18_imu_source",
}
FORBIDDEN_NODES = {
    "leroy_bridge", "leroy_bridge_node", "controller_server",
    "planner_server", "bt_navigator", "behavior_server",
    "velocity_smoother", "collision_monitor", "motor_bridge",
}
FORBIDDEN_TOPICS = {
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed",
    "/motor/command", "/motor/enable", "/joint_commands",
}


def safe_bag_path(name):
    """Allow a new immediate /capture/stage4c-* directory only."""
    if not re.fullmatch(r"stage4c-[a-zA-Z0-9_-]{6,50}", name):
        raise CaptureError("unsafe_capture_name")
    root = Path("/capture")
    if not root.is_dir() or root.is_symlink():
        raise CaptureError("capture_root_missing_or_symlink")
    output = root / name
    if output.exists() or output.is_symlink():
        raise CaptureError("capture_target_already_exists")
    return output


def capture_hashes(bag_dir):
    """SHA256 ordinary rosbag2 files, never following symlinks."""
    result = {}
    files = list(bag_dir.iterdir())
    if len(files) > 16:
        raise CaptureError("unexpected_output_file_count")
    for path in sorted(files):
        if path.is_symlink() or not path.is_file():
            raise CaptureError("unsafe_output_file")
        if path.name not in ("metadata.yaml",) and not path.name.endswith(".db3"):
            raise CaptureError("unexpected_output_filename")
        digest = hashlib.sha256()
        with path.open("rb") as fp:
            for block in iter(lambda: fp.read(1024 * 1024), b""):
                digest.update(block)
        result[path.name] = digest.hexdigest()
    if not result or "metadata.yaml" not in result or not any(k.endswith(".db3") for k in result):
        raise CaptureError("incomplete_rosbag_storage")
    return result


def _run_live(name):
    """Unreachable unless a future reviewed change explicitly commissions it."""
    for key, value in REQUIRED_ENV.items():
        if os.environ.get(key) != value:
            raise CaptureError("unsafe_environment_" + key)
    if any(Path(p).exists() for p in ("/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0")):
        raise CaptureError("device_visible_to_recorder")
    path = safe_bag_path(name)
    # One-run synchronization with the existing M18 sidecar. The recorder
    # creates this marker only after the SQLite writer and subscriptions are
    # armed; it is NOT a physical calibration or movement authorization.
    armed_path = Path("/capture") / (name + "-armed")
    if armed_path.exists() or armed_path.is_symlink():
        raise CaptureError("stale_capture_armed_marker")

    # Delayed ROS imports: --contract-smoke works without ROS/hardware.
    import rclpy
    import rosbag2_py
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import (
        qos_profile_sensor_data, QoSProfile, ReliabilityPolicy,
        DurabilityPolicy,
    )
    from rclpy.serialization import serialize_message
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from std_msgs.msg import Bool, String
    from tf2_msgs.msg import TFMessage

    class PassiveRecorder(Node):
        def __init__(self):
            super().__init__(
                "leroy_stage4c_passive_capture",
                enable_rosout=False, start_parameter_services=False)
            self.policy = CaptureContract()
            self.writer = None
            self.ingress = CaptureIngress(
                contract=self.policy, serialize=serialize_message,
                write=lambda topic, payload, stamp: self.writer.write(
                    topic, payload, stamp))
            self.failures = []
            self.running = False
            self.valid = None
            self.map_valid = None
            message_types = {
                "/camera/image_rect": Image,
                "/camera/camera_info": CameraInfo,
                "/imu/data_raw": Imu,
                METADATA_TOPIC: String,
            }
            self.subs = [
                self.create_subscription(
                    typ, topic, lambda msg, t=topic: self.receive(t, msg),
                    qos_profile_sensor_data)
                for topic, typ in message_types.items()
            ]
            reliable = QoSProfile(
                depth=5, reliability=ReliabilityPolicy.RELIABLE)
            transient = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.subs.extend([
                self.create_subscription(
                    Odometry, "/odom", self.reject_odom, reliable),
                self.create_subscription(
                    TFMessage, "/tf", self.reject_tf, reliable),
                self.create_subscription(
                    Bool, "/xrslam/localization_valid",
                    self.set_valid, transient),
                self.create_subscription(
                    Bool, "/xrslam/map_localization_valid",
                    self.set_map_valid, transient),
            ])

        def reject_odom(self, _msg):
            self.failures.append("unexpected_odom")

        def reject_tf(self, _msg):
            self.failures.append("unexpected_dynamic_tf")

        def set_valid(self, msg):
            self.valid = bool(msg.data)
            if self.valid:
                self.failures.append("unexpected_localization_valid")

        def set_map_valid(self, msg):
            self.map_valid = bool(msg.data)
            if self.map_valid:
                self.failures.append("unexpected_map_localization_valid")

        def graph_ready(self):
            names = {name for name, _ in self.get_node_names_and_namespaces()}
            topics = {name for name, _ in self.get_topic_names_and_types()}
            if names & FORBIDDEN_NODES or topics & FORBIDDEN_TOPICS:
                raise CaptureError("forbidden_motor_or_navigation_graph")
            if not REQUIRED_NODES <= names:
                return False
            for topic, (typ, _) in TOPICS.items():
                publishers = self.get_publishers_info_by_topic(topic)
                if len(publishers) != 1:
                    return False
                pub = publishers[0]
                expected = "m18_imu_source" if topic in (
                    "/imu/data_raw", METADATA_TOPIC) else None
                if pub.topic_type != typ or (
                    expected and (pub.node_name != expected
                                  or pub.node_namespace != "/")
                ):
                    raise CaptureError("unexpected_publisher_" + topic)
            return self.valid is False and self.map_valid is False

        def receive(self, topic, msg):
            if not self.running or self.failures:
                return
            try:
                # Same fail-closed payload validation and disk-write path
                # exercised by Stage 4D no-device synthetic-load tests.
                self.ingress.accept(topic, msg)
            except Exception as exc:
                self.failures.append(type(exc).__name__ + ":" + str(exc))

    rclpy.init(args=[])
    node = PassiveRecorder()
    running = [True]
    previous = {}
    def interrupt(_sig, _frame):
        running[0] = False
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, interrupt)

    bag_open = False
    armed_created = False
    failure = None
    result = None
    try:
        deadline = time.monotonic() + 25
        while running[0] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.failures:
                raise CaptureError(node.failures[0])
            if node.graph_ready():
                break
        else:
            raise CaptureError("stage4c_graph_or_false_validity_not_ready")

        writer = rosbag2_py.SequentialWriter()
        writer.open(
            rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"))
        for topic, (typ, _) in TOPICS.items():
            writer.create_topic(rosbag2_py.TopicMetadata(
                id=0, name=topic, type=typ, serialization_format="cdr"))
        node.writer = writer
        bag_open = True
        node.running = True
        # The MCU source must never start first: make a one-run, exclusive
        # readiness marker visible to its READ-ONLY /capture mount only
        # after the recorder is actually receiving and able to write.
        with armed_path.open("x", encoding="ascii") as marker:
            marker.write("STAGE4C_RECORDER_ARMED=1\n")
        armed_created = True
        print("STAGE4C_RECORDER_ARMED=PASS", flush=True)
        end = time.monotonic() + 75
        last_graph_check = 0.0
        while running[0] and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            if node.failures:
                raise CaptureError(node.failures[0])
            now = time.monotonic()
            if now - last_graph_check >= 0.5:
                if not node.graph_ready():
                    raise CaptureError("capture_sensor_graph_changed")
                last_graph_check = now
        if not running[0]:
            raise CaptureError("capture_interrupted")
        result = node.policy.report()
        if result["readiness"] != "INPUT_FORMAT_COMPLETE_UNVERIFIED":
            raise CaptureError("incomplete_sensor_or_raw_tick_capture")
    except Exception as exc:
        failure = type(exc).__name__ + ":" + str(exc)
    finally:
        node.running = False
        # The marker is a startup handshake, not a retained acceptance
        # artifact. Remove it regardless of writer success/failure.
        if armed_created:
            try:
                armed_path.unlink()
            except OSError as exc:
                if failure is None:
                    failure = "capture_armed_marker_cleanup_failed:" + str(exc)
        # Disarm callbacks and release any injected writer reference before
        # finalizing the SQLite bag or recording a failure manifest.
        node.ingress.close()
        if node.writer is not None:
            del node.writer
            node.writer = None
        if bag_open:
            # Close SQLite/metadata files before hashing them.
            try:
                del writer
                import gc
                gc.collect()
            except Exception:
                if failure is None:
                    failure = "bag_close_failed"
        for sig, prev in previous.items():
            signal.signal(sig, prev)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if result is None:
            result = node.policy.report()
        manifest = {
            **result,
            "error": failure,
            "bag_path": str(path) if bag_open else None,
            "capture_data_physically_calibrated": False,
        }
        if bag_open:
            try:
                manifest["sha256_files"] = capture_hashes(path)
            except (CaptureError, OSError) as exc:
                failure = "hash_audit_failed:" + str(exc)
                manifest["error"] = failure
        # The calibration authority is always BLOCKED, even with valid inputs.
        if bag_open:
            out = Path("/capture") / (name + "-capture-summary.json")
            if out.exists() or out.is_symlink():
                failure = "summary_already_exists"
            else:
                with out.open("x", encoding="utf8") as fp:
                    json.dump(manifest, fp, sort_keys=True)
                    fp.write("\n")
        print("STAGE4C_CAPTURE_JSON=" + json.dumps(manifest, sort_keys=True),
              flush=True)
        print("STAGE4C_CALIBRATION=BLOCKED_NOT_VALIDATED", flush=True)
    # Never return from finally: that would suppress an unexpected exception
    # and could incorrectly report capture success on a cleanup failure.
    return 0 if failure is None else 2


def _contract_smoke():
    from live_stage4c_capture_contract import CaptureContract
    c = CaptureContract()
    writes = []
    c.accept("/imu/data_raw", 1_000_000_000, "imu_link", b"cdr", 2_000_000_000,
             lambda *a: writes.append(a))
    r = c.report()
    if (len(writes) != 1 or r["counts"]["/imu/data_raw"] != 1
            or r["physical_camera_imu_offset_validated"] is not False
            or r["readiness"] != "BLOCKED_INCOMPLETE_CAPTURE"):
        raise CaptureError("contract_smoke_failed")
    print("STAGE4C_OFFLINE_CAPTURE_CONTRACT=PASS_NO_DEVICE", flush=True)
    print("STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--contract-smoke", action="store_true")
    parser.add_argument("--output-name")
    args = parser.parse_args(argv)
    if args.contract_smoke:
        _contract_smoke()
        return 0
    if not LIVE_CAPTURE_COMMISSIONED:
        print("STAGE4C=BLOCKED_LIVE_CAPTURE_NOT_COMMISSIONED",
              file=sys.stderr, flush=True)
        return 2
    if args.output_name is None:
        parser.error("--output-name required")
    return _run_live(args.output_name)


if __name__ == "__main__":
    sys.exit(main())
