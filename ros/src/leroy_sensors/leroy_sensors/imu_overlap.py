#!/usr/bin/env python3
"""Stage 4A: supervised real MCU IMU alongside existing live camera graph.

This is NOT XRSLAM VIO, absolute camera/IMU synchronization, odometry,
motor commissioning or navigation. Serial writes are exclusively the unchanged
M18 START/STOP commands. If STOP ACK is missing, state is UNVERIFIED.
"""
import json
import math
import os
from pathlib import Path
import re
import signal
import sys
import time

import rclpy
import serial
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, Imu
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from leroy_imu_observer.imu_protocol import START, STOP, PORT
from leroy_sensors.delivery_trace import DeliveryTrace
from leroy_sensors.spin_budget import service_callbacks
from leroy_imu_observer.imu_source import (
    ImuStreamSource, send_only_whitelisted_serial_command, wait_ack,
)

DURATION_S = 75.0
# Capture-mode source ends before the recorder's 75-second upper bound,
# allowing a validated MCU STOP ACK and bounded DDS callback drain.
CAPTURE_SOURCE_DURATION_S = 65.0
EXPECTED = {
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "LEROY_STAGE4_IMU_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
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


class PassiveOverlap(Node):
    def __init__(self, trace=None):
        super().__init__("leroy_stage4_passive_overlap", enable_rosout=False,
                         start_parameter_services=False)
        self.counts = {"image": 0, "info": 0, "imu": 0, "odom": 0, "tf": 0}
        self.spans = {"image": [None, None], "imu": [None, None]}
        self.first_stamp = {"image": None, "imu": None}
        self.last_stamp = {"image": None, "info": None, "imu": None}
        self.failures = []
        self.trace = trace
        self.valid_status = None
        self.map_valid_status = None
        self._subs = [
            self.create_subscription(
                Image, "/camera/image_rect",
                lambda m: self._take("image", m, "camera_optical_frame"),
                qos_profile_sensor_data),
            self.create_subscription(
                CameraInfo, "/camera/camera_info",
                lambda m: self._take("info", m, "camera_optical_frame"),
                qos_profile_sensor_data),
            self.create_subscription(
                Imu, "/imu/data_raw",
                lambda m: self._take("imu", m, "imu_link"),
                qos_profile_sensor_data),
            self.create_subscription(
                Odometry, "/odom", self._odom,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)),
            self.create_subscription(
                TFMessage, "/tf", self._tf,
                QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)),
            self.create_subscription(
                Bool, "/xrslam/localization_valid", self._valid,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(
                Bool, "/xrslam/map_localization_valid", self._map_valid,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)),
        ]

    def _take(self, stream, msg, expected_frame):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        if (msg.header.frame_id != expected_frame or not math.isfinite(stamp)
                or stamp <= 0 or not -0.1 <= now - stamp <= 2.5):
            self.failures.append("invalid_" + stream + "_frame_or_age")
            return
        prev = self.last_stamp[stream]
        if prev is not None and stamp <= prev:
            self.failures.append("nonmonotonic_" + stream + "_stamp")
            return
        if stream == "imu":
            xyz = (
                msg.linear_acceleration.x, msg.linear_acceleration.y,
                msg.linear_acceleration.z, msg.angular_velocity.x,
                msg.angular_velocity.y, msg.angular_velocity.z,
            )
            if not all(math.isfinite(x) for x in xyz):
                self.failures.append("nonfinite_imu")
                return
        if stream == "image" and (not msg.data or msg.width == 0 or msg.height == 0):
            self.failures.append("empty_image")
            return
        self.counts[stream] += 1
        if stream == "imu" and self.trace is not None:
            # Exact header-stamp matching against the POST-PUBLISH M18 hook.
            # Callback time is host monotonic, NOT physical sensor acquisition.
            self.trace.subscriber(
                int(msg.header.stamp.sec) * 1_000_000_000
                + int(msg.header.stamp.nanosec), time.monotonic_ns())
        if stream in self.spans:
            if self.spans[stream][0] is None:
                self.spans[stream][0] = time.monotonic()
                self.first_stamp[stream] = stamp
            self.spans[stream][1] = time.monotonic()
        self.last_stamp[stream] = stamp

    def _odom(self, _msg):
        self.counts["odom"] += 1
        self.failures.append("unexpected_odom_message")

    def _tf(self, _msg):
        self.counts["tf"] += 1
        self.failures.append("unexpected_dynamic_tf_message")

    def _valid(self, msg):
        self.valid_status = bool(msg.data)
        if msg.data:
            self.failures.append("unexpected_localization_authority")

    def _map_valid(self, msg):
        self.map_valid_status = bool(msg.data)
        if msg.data:
            self.failures.append("unexpected_map_localization_authority")

    def overlap_s(self):
        a, b = self.spans["image"], self.spans["imu"]
        if None in a + b:
            return 0.0
        return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))

    def graph_ready(self, *, require_source_meta=False):
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        topics = {topic for topic, _ in self.get_topic_names_and_types()}
        if names & FORBIDDEN_NODES or topics & FORBIDDEN_TOPICS:
            raise RuntimeError("forbidden_motor_or_nav_graph")
        if self.count_publishers("/imu/data_raw") > 1:
            raise RuntimeError("duplicate_imu_publisher")
        if not REQUIRED_NODES <= names:
            return False
        pub = self.get_publishers_info_by_topic("/imu/data_raw")
        imu_ready = (len(pub) == 1
                     and pub[0].node_name == "m18_imu_source"
                     and pub[0].node_namespace == "/"
                     and pub[0].topic_type == "sensor_msgs/msg/Imu")
        if not require_source_meta:
            return imu_ready
        metadata_publishers = self.get_publishers_info_by_topic(
            "/stage4c/imu_source_meta")
        return (imu_ready and len(metadata_publishers) == 1
                and metadata_publishers[0].node_name == "m18_imu_source"
                and metadata_publishers[0].node_namespace == "/"
                and metadata_publishers[0].topic_type == "std_msgs/msg/String")


def source_metadata_enabled():
    """Explicit opt-in only; the accepted Stage 4A/B path stays unchanged."""
    requested = os.environ.get("LEROY_STAGE4C_SOURCE_META", "0")
    if requested not in ("0", "1"):
        raise RuntimeError("invalid_source_metadata_mode")
    if requested == "1" and os.environ.get("LEROY_STAGE4C_CAPTURE_ONLY") != "1":
        raise RuntimeError("source_metadata_requires_capture_only_session")
    return requested == "1"


def run():
    metadata_enabled = source_metadata_enabled()
    capture_requested = os.environ.get("LEROY_STAGE4C_RECORD", "0")
    if capture_requested not in ("0", "1"):
        raise RuntimeError("invalid_real_capture_mode")
    capture_armed_path = None
    session_duration_s = (CAPTURE_SOURCE_DURATION_S
                          if capture_requested == "1" else DURATION_S)
    if capture_requested == "1":
        if not metadata_enabled:
            raise RuntimeError("real_capture_requires_m18_metadata")
        capture_name = os.environ.get("LEROY_STAGE4C_CAPTURE_NAME", "")
        if not re.fullmatch(r"stage4c-[a-zA-Z0-9_-]{6,50}", capture_name):
            raise RuntimeError("invalid_capture_armed_name")
        capture_armed_path = Path("/capture") / (capture_name + "-armed")
        if not capture_armed_path.parent.is_dir():
            raise RuntimeError("capture_read_only_mount_missing")
    for name, value in EXPECTED.items():
        if os.environ.get(name) != value:
            raise RuntimeError("blocked_environment_" + name)
    if not os.path.exists(PORT) or not os.path.exists("/dev/ttyACM0"):
        raise RuntimeError("MCU_device_not_mapped")
    if os.path.exists("/dev/video2"):
        raise RuntimeError("camera_must_remain_in_parent_only")
    if (START, STOP) != (
        b"set_imu_stream_hz 50\r\n", b"set_imu_stream_hz 0\r\n"
    ):
        raise RuntimeError("serial_command_contract_changed")

    rclpy.init(args=[])
    quitting = [False]
    def interrupt(_signal, _frame):
        quitting[0] = True
    previous = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, interrupt)

    source = observer = metadata_adapter = None
    trace = DeliveryTrace()
    spin_stats = {"serial_loop_iterations": 0, "spin_attempts": 0,
                  "max_callback_burst_s": 0.0}
    started = stopped = False
    complete = False
    error = None
    try:
        # The pre-existing M18 callback fires only after successful ROS
        # publication. It sends metadata, never another MCU command or image.
        def after_imu_publish(fields, stamp, steady):
            # Reuse M18 after-publish callback; never duplicate IMU/serial.
            trace.source(stamp, time.monotonic_ns())
            if metadata_adapter is not None:
                metadata_adapter.on_imu_published(fields, stamp, steady)

        source = ImuStreamSource(on_imu_published=after_imu_publish)
        if metadata_enabled:
            # Same physical M18 publisher node; constructed before serial.
            # Any callback failure latches the M18 source and attempts STOP.
            from leroy_capture.metadata_ros import SourceMetadataRosAdapter
            metadata_adapter = SourceMetadataRosAdapter(source)
        observer = PassiveOverlap(trace=trace)
        # Before opening the motor-capable serial port, prove the camera,
        # depth, XRSLAM and this source are present in the shared DDS graph.
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline and not quitting[0]:
            rclpy.spin_once(observer, timeout_sec=0.05)
            if observer.failures:
                raise RuntimeError(observer.failures[0])
            if (observer.graph_ready(require_source_meta=metadata_enabled)
                    and observer.valid_status is False
                    and observer.map_valid_status is False):
                break
        else:
            raise RuntimeError("required_stage3_graph_or_false_validity_not_ready")
        print("STAGE4A_PRE_SERIAL_DDS_GRAPH=PASS", flush=True)
        if capture_armed_path is not None:
            # The ROS graph alone does not prove the SQLite writer is open.
            # Wait for the existing passive recorder to arm its writer
            # BEFORE even opening the motor-capable STM32 CDC interface.
            deadline = time.monotonic() + 20.0
            while time.monotonic() < deadline and not quitting[0]:
                try:
                    marker = capture_armed_path.read_text(encoding="ascii")
                except (OSError, UnicodeError):
                    marker = ""
                if marker == "STAGE4C_RECORDER_ARMED=1\n":
                    break
                time.sleep(0.05)
            else:
                raise RuntimeError("capture_not_armed_before_serial")
            print("STAGE4C_RECORDER_ARMED_BEFORE_SERIAL=PASS", flush=True)

        with serial.Serial(port=PORT, baudrate=115200, timeout=0.08,
                           write_timeout=0.25, exclusive=True) as device:
            # CDC open may reset the MCU; independent motor-power isolation
            # is the only physical protection.
            try:
                time.sleep(0.3)
                device.reset_input_buffer()
                send_only_whitelisted_serial_command(device, START)
                started = wait_ack(device, timeout_s=2.0)
                if not started:
                    raise RuntimeError("MCU_start_ack_missing")
                print("STAGE4A_IMU_START_ACK=PASS", flush=True)
                end = time.monotonic() + session_duration_s
                while time.monotonic() < end and not quitting[0]:
                    payload = device.readline()
                    if payload:
                        source.consume(payload)
                    # One spin per 50 Hz serial line starved camera/info/IMU
                    # subscriptions in Stage 4B's first physical run.
                    # Service several NONBLOCKING callbacks with a strict
                    # time and count cap before returning to serial reads.
                    def check_subscriber_faults():
                        if observer.failures:
                            raise RuntimeError(observer.failures[0])
                    attempts, elapsed = service_callbacks(
                        lambda: rclpy.spin_once(observer, timeout_sec=0.0),
                        check_subscriber_faults)
                    spin_stats["serial_loop_iterations"] += 1
                    spin_stats["spin_attempts"] += attempts
                    spin_stats["max_callback_burst_s"] = max(
                        spin_stats["max_callback_burst_s"], elapsed)
                    if not observer.graph_ready(require_source_meta=metadata_enabled):
                        raise RuntimeError("sensor_graph_changed")
                complete = (not quitting[0] and time.monotonic() >= end
                            and source.terminal_fault is None)
            finally:
                # Never send motor commands. Always ATTEMPT STOP and verify
                # ACK, including errors and interrupted sessions.
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stopped = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("STAGE4A_IMU_STOP_ERROR=" + type(exc).__name__, flush=True)
                print("STAGE4A_IMU_STOP_ACK=" +
                      ("PASS" if stopped else "UNVERIFIED"), flush=True)
    except Exception as exc:
        error = type(exc).__name__ + ":" + str(exc)
        print("STAGE4A_FAULT=" + error, flush=True)
    finally:
        for sig, prev in previous.items():
            signal.signal(sig, prev)
        result = source.export() if source is not None else None
        overlap = observer.overlap_s() if observer is not None else 0.0
        counts = dict(observer.counts) if observer is not None else {}
        passed = bool(
            complete and started and stopped and error is None
            and result is not None
            and result["clock_rate_estimate_ready"] is True
            and result["terminal_fault"] is None
            and result["counts"]["published_imu_messages"] >= 1000
            and result["counts"]["sequence_faults"] == 0
            and result["counts"]["timestamp_faults"] == 0
            and result["counts"]["malformed_imu_packets"] == 0
            and result["counts"]["sequence_gap_samples"] == 0
            and result["counts"]["stale_or_future_withheld"] == 0
            and result["counts"]["calibration_withheld"] == 0
            and result["counts"]["invalid_ascii_lines"] == 0
            and result["counts"]["unexpected_serial_lines"] == 0
            and counts.get("image", 0) >= 100
            and counts.get("imu", 0) >= 500
            and counts.get("info", 0) >= 100
            and counts.get("odom", 0) == 0
            and counts.get("tf", 0) == 0
            and overlap >= 15.0
            and observer.valid_status is False
            and observer.map_valid_status is False
            and not observer.failures
            and (not metadata_enabled or (
                metadata_adapter is not None
                and metadata_adapter.tap.count ==
                result["counts"]["published_imu_messages"]))
        )
        print("STAGE4A_RESULT_JSON=" + json.dumps({
            "duration_s": session_duration_s, "start_ack": started, "stop_ack": stopped,
            "overlap_receipt_s": round(overlap, 3), "counts": counts,
            "source": result, "error": error,
            "stage4b_delivery_trace": trace.report(),
            "stage4b_executor_service": {
                "serial_loop_iterations": spin_stats["serial_loop_iterations"],
                "nonblocking_spin_attempts": spin_stats["spin_attempts"],
                "max_callback_burst_ms": round(
                    1000 * spin_stats["max_callback_burst_s"], 3),
                "spin_attempts_are_not_callbacks": True,
                "physical_calibration_validated": False,
            },
            "real_source_metadata_mode": metadata_enabled,
            "original_mcu_metadata_published": (
                metadata_adapter.tap.count if metadata_adapter is not None
                else 0),
            "camera_imu_absolute_offset_validated": False,
            "camera_imu_extrinsics_validated": False,
            "xrslam_vio_started": False,
            "metric_odometry_authorized": False,
        }, sort_keys=True), flush=True)
        print("STAGE4A_IMU_OVERLAP=" +
              ("PASS_TRANSPORT_ONLY" if passed else "BLOCKED_OR_INCOMPLETE"),
              flush=True)
        if not stopped:
            print("STAGE4A_MCU_STOP_STATE=UNVERIFIED_PHYSICALLY_ISOLATE",
                  flush=True)
        if observer is not None:
            observer.destroy_node()
        if source is not None:
            source.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    # Do not return from finally: cleanup failures must propagate to the
    # top-level BLOCKED handler rather than being silently suppressed.
    return 0 if passed else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except Exception as exc:
        print("STAGE4A=BLOCKED:" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
