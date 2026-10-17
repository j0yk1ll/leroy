#!/usr/bin/env python3
"""M18 experimental MCU IMU-only ROS 2 sender + passive observer.

NOT a motor bridge. The only outbound serial packets are a fixed
set_imu_stream_hz 50 request and set_imu_stream_hz 0 cleanup request.
Opening this motor-capable MCU's CDC interface requires actual physical
inhibition of wheel/manipulator power and an isolated ROS network.
The mapped ROS timestamps retain UNKNOWN absolute MCU-to-camera phase.
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import serial

from leroy_bridge.clock_sync import ClockSyncError, McuClockSync
from leroy_nav2_integration.m16_passive_observer import PassiveSensorObserver
from m18_imu_stream_contract import (
    START, STOP, PORT, STREAM_HZ, PacketError, parse_packet, sequence_gap,
)


REQUIRED_ENV = {
    "LEROY_M18_IMU_STREAM": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "169",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
DURATION_S = 30
MAX_STAMP_AGE_S = 2.0


class ImuStreamSource(Node):
    def __init__(self, on_imu_published=None):
        # Optional M21F metadata-only tap. Existing M18/M20 callers pass None
        # and preserve the previously accepted physical publication path.
        if on_imu_published is not None and not callable(on_imu_published):
            raise TypeError("IMU metadata observer must be callable")
        super().__init__(
            "m18_imu_source", enable_rosout=False,
            start_parameter_services=False)
        self._on_imu_published = on_imu_published
        self._imu_pub = self.create_publisher(
            Imu, "/imu/data_raw", qos_profile_sensor_data)
        if self._imu_pub.topic_name != "/imu/data_raw":
            raise RuntimeError("M18 refuses topic remapping")
        self._mapper = McuClockSync()
        self.last_sequence = None
        self.counts = {
            "received_imu_packets": 0,
            "published_imu_messages": 0,
            "calibration_withheld": 0,
            "malformed_imu_packets": 0,
            "sequence_gap_samples": 0,
            "sequence_faults": 0,
            "timestamp_faults": 0,
            "stale_or_future_withheld": 0,
            "unexpected_serial_lines": 0,
            "invalid_ascii_lines": 0,
        }
        self.first_publish = None
        self.last_publish = None
        self.last_age_s = None
        self.max_age_s = None
        self.terminal_fault = None

    def consume(self, payload):
        if len(payload) > 512:
            self.counts["malformed_imu_packets"] += 1
            return
        try:
            line = payload.decode("ascii").strip()
        except UnicodeDecodeError:
            self.counts["invalid_ascii_lines"] += 1
            return
        if not line.startswith("telemetry imu "):
            if line:
                self.counts["unexpected_serial_lines"] += 1
            return
        self.counts["received_imu_packets"] += 1
        try:
            fields = parse_packet(line)
        except PacketError:
            self.counts["malformed_imu_packets"] += 1
            return
        try:
            skipped = sequence_gap(self.last_sequence, fields["seq"])
        except PacketError as exc:
            self.counts["sequence_faults"] += 1
            self.terminal_fault = str(exc)
            raise
        self.last_sequence = fields["seq"]
        self.counts["sequence_gap_samples"] += skipped
        if fields["cal"] != 1:
            self.counts["calibration_withheld"] += 1
            return

        # Directly reuse the previously tested bridge clock mapper, without
        # importing/constructing the motor-capable bridge node itself.
        steady_ns = time.monotonic_ns()
        ros_ns = self.get_clock().now().nanoseconds
        try:
            stamp_ns = self._mapper.observe(
                fields["t_us"], ros_ns, steady_ns)
        except ClockSyncError as exc:
            self.counts["timestamp_faults"] += 1
            self.terminal_fault = str(exc)
            raise
        age_s = (ros_ns - stamp_ns) * 1e-9
        if not -0.1 <= age_s <= MAX_STAMP_AGE_S:
            self.counts["stale_or_future_withheld"] += 1
            self.terminal_fault = "unacceptable_mapped_acquisition_age"
            raise RuntimeError(self.terminal_fault)

        msg = Imu()
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(
            stamp_ns, 1_000_000_000)
        msg.header.frame_id = "imu_link"
        msg.linear_acceleration.x = fields["af_mm_s2"] / 1000.0
        msg.linear_acceleration.y = fields["al_mm_s2"] / 1000.0
        msg.linear_acceleration.z = fields["au_mm_s2"] / 1000.0
        msg.angular_velocity.x = fields["gr_mrad_s"] / 1000.0
        msg.angular_velocity.y = fields["gp_mrad_s"] / 1000.0
        msg.angular_velocity.z = fields["gy_mrad_s"] / 1000.0
        # Raw accel and gyro present; fused orientation unavailable.
        # Covariance of accel/gyro is not independently calibrated.
        msg.orientation_covariance[0] = -1.0
        self._imu_pub.publish(msg)
        now = time.monotonic()
        if self.first_publish is None:
            self.first_publish = now
        self.last_publish = now
        self.last_age_s = round(age_s, 4)
        self.max_age_s = max(self.max_age_s or 0.0, age_s)
        self.counts["published_imu_messages"] += 1
        # This fires ONLY after a successfully published, calibrated packet.
        # Source-level MCU sequence and acquisition ticks cannot be recovered
        # from sensor_msgs/Imu; pass the original parsed fields directly.
        # NEVER invoke on a withheld or malformed packet.
        if self._on_imu_published is not None:
            try:
                self._on_imu_published(dict(fields), stamp_ns, steady_ns)
            except Exception as exc:
                # Fail closed: the owning serial runner's finally block must
                # attempt STOP rather than silently lose timing metadata.
                self.terminal_fault = "imu_metadata_callback_failed"
                raise RuntimeError(self.terminal_fault) from exc

    def export(self):
        count = self.counts["published_imu_messages"]
        span = (self.last_publish - self.first_publish
                if self.first_publish is not None else 0.0)
        return {
            "serial_stream_frequency_requested_hz": STREAM_HZ,
            "pub_topic": "/imu/data_raw",
            "publisher_node": "/m18_imu_source",
            "mcu_sequence_and_t_us_observed": count > 0,
            "absolute_mcu_camera_offset_validated": False,
            "spatial_imu_extrinsics_validated": False,
            "gyro_noise_validated": False,
            "xrslam_ready": False,
            "motor_commands_sent": 0,
            "counts": dict(self.counts),
            "measured_publication_rate_hz": (
                round((count - 1) / span, 2) if count > 1 and span > 0
                else None),
            "last_estimated_acquisition_age_s": self.last_age_s,
            "max_estimated_acquisition_age_s": (
                round(self.max_age_s, 4) if self.max_age_s is not None
                else None),
            "clock_rate_estimate_ready": self._mapper.ready,
            "clock_rate_estimate_ppm": round(self._mapper.rate_ppm, 2),
            "terminal_fault": self.terminal_fault,
        }


def send_only_whitelisted_serial_command(device, payload):
    if payload not in (START, STOP):
        raise ValueError("M18 forbids all unapproved MCU writes")
    device.write(payload)
    device.flush()


def wait_ack(device, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        data = device.readline()
        if data.strip() == b"ok set_imu_stream_hz":
            return True
        if data.strip().startswith(b"error "):
            raise RuntimeError("firmware rejected IMU stream configuration")
    return False


def enforce_environment():
    for key, expected in REQUIRED_ENV.items():
        if os.environ.get(key) != expected:
            raise RuntimeError(f"M18 requires {key}={expected}")


def node_smoke():
    rclpy.init(args=[])
    src = obs = None
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        assert len(obs._m16_subscriptions) == 9
        summary = obs.report(5.0)
        assert summary["observation_only"]
        assert summary["verdict"] == "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"
        assert src._imu_pub.topic_name == "/imu/data_raw"
        print("M18_ROS_NODES_SELFTEST=PASS", flush=True)
    finally:
        if obs is not None:
            obs.destroy_node()
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run():
    enforce_environment()
    rclpy.init(args=[])
    src = obs = None
    requested_stop = [False]
    old_handlers = {}

    def on_stop(_signum, _frame):
        requested_stop[0] = True

    for sig in (signal.SIGTERM, signal.SIGINT):
        old_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, on_stop)

    stream_acked = False
    stream_stop_ack = False
    passive_report = None
    result_code = 1
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        with serial.Serial(
            port=PORT, baudrate=115200, timeout=0.08,
            write_timeout=0.25, exclusive=True,
        ) as device:
            # Serial-open/DTR may reset the MCU. Do not assume the boot
            # state of the motors; operator MUST isolate motor power.
            time.sleep(0.3)
            device.reset_input_buffer()
            try:
                send_only_whitelisted_serial_command(device, START)
                if not wait_ack(device):
                    raise RuntimeError("MCU did not acknowledge IMU stream start")
                stream_acked = True
                print("M18_IMU_STREAM_START_ACK=PASS", flush=True)
                start = time.monotonic()
                deadline = start + DURATION_S
                while time.monotonic() < deadline and not requested_stop[0]:
                    data = device.readline()
                    if data:
                        src.consume(data)
                    rclpy.spin_once(obs, timeout_sec=0.001)
                if not requested_stop[0] and src.terminal_fault is None:
                    passive_report = obs.report(DURATION_S)
            finally:
                # Always try to disable IMU streaming, even after exceptions.
                # No write to wheel, actuator or watchdog registers.
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stream_stop_ack = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("M18_IMU_STREAM_STOP_ERROR=" +
                          type(exc).__name__, flush=True)
                print("M18_IMU_STREAM_STOP_ACK=" +
                      ("PASS" if stream_stop_ack else "UNVERIFIED"),
                      flush=True)

        if passive_report is not None:
            print("M16_PASSIVE_REPORT_JSON=" +
                  json.dumps(passive_report, sort_keys=True), flush=True)
            print("M16_VERDICT=" + passive_report["verdict"], flush=True)

        if src is not None:
            metrics = src.export()
            imu = (passive_report or {}).get("topics", {}).get("imu", {})
            publishers = imu.get("publishers", [])
            acceptable = bool(
                stream_acked and stream_stop_ack
                and metrics["counts"]["published_imu_messages"] >= 100
                and metrics["counts"]["sequence_faults"] == 0
                and metrics["counts"]["timestamp_faults"] == 0
                and metrics["counts"]["malformed_imu_packets"] == 0
                and metrics["counts"]["sequence_gap_samples"] == 0
                and metrics["counts"]["calibration_withheld"] == 0
                and imu.get("accepted", 0) >= 50
                and imu.get("invalid", 0) == 0
                and imu.get("fresh_valid_receipt") is True
                and len(publishers) == 1
                and publishers[0]["node"] == "m18_imu_source"
                and publishers[0]["namespace"] == "/"
                and publishers[0]["type"] == "sensor_msgs/msg/Imu"
            )
            verdict = (
                "OBSERVED_STAMPED_IMU_ROS_MESSAGE_CONTRACT"
                if acceptable else "INCOMPLETE_OR_UNVERIFIED_STAMPED_IMU")
            metrics.update({
                "milestone": "M18",
                "serial_commands_allowlist": [
                    "set_imu_stream_hz 50", "set_imu_stream_hz 0",
                ],
                "imu_stream_start_acknowledged": stream_acked,
                "imu_stream_stop_acknowledged": stream_stop_ack,
                "observation_duration_requested_s": DURATION_S,
                "verdict": verdict,
            })
            print("M18_IMU_STREAM_REPORT_JSON=" +
                  json.dumps(metrics, sort_keys=True), flush=True)
            print("M18_IMU_STREAM_VERDICT=" + verdict, flush=True)
            result_code = 0 if acceptable else 2
    except (serial.SerialException, OSError, RuntimeError,
            ClockSyncError, PacketError, ValueError) as exc:
        print("M18_IMU_STREAM_ERROR=" + type(exc).__name__ +
              ": " + str(exc), flush=True)
        result_code = 1
    finally:
        for sig, handler in old_handlers.items():
            signal.signal(sig, handler)
        if obs is not None:
            obs.destroy_node()
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result_code


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    enforce_environment()
    if args.self_test:
        node_smoke()
        return 0
    return run()


if __name__ == "__main__":
    sys.exit(main())
