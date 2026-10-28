#!/usr/bin/env python3
"""Stage 4A: supervised real MCU IMU alongside existing live camera graph.

This is NOT XRSLAM VIO, absolute camera/IMU synchronization, odometry,
motor commissioning or navigation. Serial writes are exclusively the unchanged
M18 START/STOP commands. If STOP ACK is missing, state is UNVERIFIED.
"""
import json
import math
import os
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

from m18_imu_stream_contract import START, STOP, PORT
from m18_imu_stream_source import (
    ImuStreamSource, send_only_whitelisted_serial_command, wait_ack,
)

DURATION_S = 75.0
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
    def __init__(self):
        super().__init__("leroy_stage4_passive_overlap", enable_rosout=False,
                         start_parameter_services=False)
        self.counts = {"image": 0, "info": 0, "imu": 0, "odom": 0}
        self.spans = {"image": [None, None], "imu": [None, None]}
        self.first_stamp = {"image": None, "imu": None}
        self.last_stamp = {"image": None, "imu": None}
        self.failures = []
        self.valid_status = None
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
                Bool, "/xrslam/localization_valid", self._valid,
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
        if stream in self.spans:
            if self.spans[stream][0] is None:
                self.spans[stream][0] = time.monotonic()
                self.first_stamp[stream] = stamp
            self.spans[stream][1] = time.monotonic()
        self.last_stamp[stream] = stamp

    def _odom(self, _msg):
        self.counts["odom"] += 1
        self.failures.append("unexpected_odom_message")

    def _valid(self, msg):
        self.valid_status = bool(msg.data)
        if msg.data:
            self.failures.append("unexpected_localization_authority")

    def overlap_s(self):
        a, b = self.spans["image"], self.spans["imu"]
        if None in a + b:
            return 0.0
        return max(0.0, min(a[1], b[1]) - max(a[0], b[0]))

    def graph_ready(self):
        names = {n for n, _ in self.get_node_names_and_namespaces()}
        topics = {topic for topic, _ in self.get_topic_names_and_types()}
        if names & FORBIDDEN_NODES or topics & FORBIDDEN_TOPICS:
            raise RuntimeError("forbidden_motor_or_nav_graph")
        if self.count_publishers("/imu/data_raw") > 1:
            raise RuntimeError("duplicate_imu_publisher")
        if not REQUIRED_NODES <= names:
            return False
        pub = self.get_publishers_info_by_topic("/imu/data_raw")
        return (len(pub) == 1
                and pub[0].node_name == "m18_imu_source"
                and pub[0].node_namespace == "/"
                and pub[0].topic_type == "sensor_msgs/msg/Imu")


def run():
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

    source = observer = None
    started = stopped = False
    complete = False
    error = None
    try:
        source = ImuStreamSource()
        observer = PassiveOverlap()
        # Before opening the motor-capable serial port, prove the camera,
        # depth, XRSLAM and this source are present in the shared DDS graph.
        deadline = time.monotonic() + 25.0
        while time.monotonic() < deadline and not quitting[0]:
            rclpy.spin_once(observer, timeout_sec=0.05)
            if observer.failures:
                raise RuntimeError(observer.failures[0])
            if observer.graph_ready() and observer.valid_status is False:
                break
        else:
            raise RuntimeError("required_stage3_graph_or_false_validity_not_ready")
        print("STAGE4A_PRE_SERIAL_DDS_GRAPH=PASS", flush=True)

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
                end = time.monotonic() + DURATION_S
                while time.monotonic() < end and not quitting[0]:
                    payload = device.readline()
                    if payload:
                        source.consume(payload)
                    rclpy.spin_once(observer, timeout_sec=0.002)
                    if observer.failures:
                        raise RuntimeError(observer.failures[0])
                    if not observer.graph_ready():
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
            and counts.get("image", 0) >= 100
            and counts.get("imu", 0) >= 500
            and counts.get("info", 0) >= 100
            and counts.get("odom", 0) == 0
            and overlap >= 15.0
            and observer.valid_status is False
            and not observer.failures
        )
        print("STAGE4A_RESULT_JSON=" + json.dumps({
            "duration_s": DURATION_S, "start_ack": started, "stop_ack": stopped,
            "overlap_receipt_s": round(overlap, 3), "counts": counts,
            "source": result, "error": error,
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
        return 0 if passed else 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except Exception as exc:
        print("STAGE4A=BLOCKED:" + type(exc).__name__ + ":" + str(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
