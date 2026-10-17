#!/usr/bin/env python3
"""M19 stationary MCU IMU clock-rate audit, never motor command authority.

Reuses M18's experimentally validated IMU-only ROS source, exact fixed
serial START/STOP allowlist, and M16 passive observer. No camera, XRSLAM,
motor bridge, other ROS publisher or optional CLI override is started.
"""
import json
import math
import os
import signal
import sys
import time

import rclpy
import serial

from m18_imu_stream_source import (
    START, STOP, PORT, ImuStreamSource, PassiveSensorObserver,
    send_only_whitelisted_serial_command, wait_ack, enforce_environment,
)


DURATION_S = 120
MIN_PUBLISHED = 5700
MIN_OBSERVED = 5600


def audit_acceptance(*, complete, start_ack, stop_ack, metrics, imu):
    counts = metrics["counts"]
    publishers = imu.get("publishers", [])
    return bool(
        complete and start_ack and stop_ack
        and metrics["clock_rate_estimate_ready"] is True
        and math.isfinite(metrics["clock_rate_estimate_ppm"])
        and abs(metrics["clock_rate_estimate_ppm"]) <= 200.0
        and counts["published_imu_messages"] >= MIN_PUBLISHED
        and 48.0 <= metrics["measured_publication_rate_hz"] <= 52.0
        and all(counts[k] == 0 for k in (
            "calibration_withheld", "malformed_imu_packets",
            "sequence_gap_samples", "sequence_faults", "timestamp_faults",
            "stale_or_future_withheld", "invalid_ascii_lines",
            "unexpected_serial_lines",
        ))
        and metrics["terminal_fault"] is None
        and imu.get("accepted", 0) >= MIN_OBSERVED
        and imu.get("invalid", 0) == 0
        and imu.get("fresh_valid_receipt") is True
        and len(publishers) == 1
        and publishers[0] == {
            "node": "m18_imu_source",
            "namespace": "/",
            "type": "sensor_msgs/msg/Imu",
        }
    )


def selftest():
    enforce_environment()
    rclpy.init(args=[])
    src = obs = None
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        initial = src.export()
        report = obs.report(5.0)
        assert initial["clock_rate_estimate_ready"] is False
        assert report["verdict"] == "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"
        assert src._imu_pub.topic_name == "/imu/data_raw"
        assert len(obs._m16_subscriptions) == 9
        assert not audit_acceptance(
            complete=True, start_ack=True, stop_ack=True,
            metrics=initial, imu={"accepted": 6000, "invalid": 0,
                                  "fresh_valid_receipt": True,
                                  "publishers": [{"node": "m18_imu_source",
                                                  "namespace": "/",
                                                  "type": "sensor_msgs/msg/Imu"}]})
        print("M19_ROS_CLOCK_AUDIT_SELFTEST=PASS", flush=True)
    finally:
        if obs is not None:
            obs.destroy_node()
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run():
    enforce_environment()
    if os.environ.get("LEROY_M19_CLOCK_AUDIT") != "1":
        raise RuntimeError("M19 requires LEROY_M19_CLOCK_AUDIT=1")

    stopping = [False]
    previous_handlers = {}
    def request_stop(_signum, _frame):
        stopping[0] = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[sig] = signal.getsignal(sig)
        signal.signal(sig, request_stop)

    src = obs = None
    start_ack = stop_ack = complete = False
    passive = None
    fault = None
    rclpy.init(args=[])
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        with serial.Serial(
            port=PORT, baudrate=115200, timeout=0.08,
            write_timeout=0.25, exclusive=True,
        ) as device:
            # CDC ACM open can reset MCU; physical power isolation REQUIRED.
            time.sleep(0.3)
            device.reset_input_buffer()
            try:
                send_only_whitelisted_serial_command(device, START)
                start_ack = wait_ack(device, timeout_s=2.0)
                if not start_ack:
                    raise RuntimeError("MCU did not acknowledge IMU stream start")
                print("M19_IMU_STREAM_START_ACK=PASS", flush=True)
                deadline = time.monotonic() + DURATION_S
                while time.monotonic() < deadline and not stopping[0]:
                    data = device.readline()
                    if data:
                        src.consume(data)
                    rclpy.spin_once(obs, timeout_sec=0.001)
                complete = not stopping[0] and time.monotonic() >= deadline
                if complete and src.terminal_fault is None:
                    passive = obs.report(DURATION_S)
            finally:
                # This fixed non-actuating command is also sent on failures.
                # MCU state remains unverified on port loss or process kill.
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stop_ack = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("M19_IMU_STREAM_STOP_ERROR=" +
                          type(exc).__name__, flush=True)
                print("M19_IMU_STREAM_STOP_ACK=" +
                      ("PASS" if stop_ack else "UNVERIFIED"), flush=True)
    except (serial.SerialException, OSError, RuntimeError, ValueError) as exc:
        fault = type(exc).__name__ + ": " + str(exc)
        print("M19_IMU_CLOCK_ERROR=" + fault, flush=True)
    finally:
        for sig, handler in previous_handlers.items():
            signal.signal(sig, handler)
        try:
            if passive is not None:
                print("M16_PASSIVE_REPORT_JSON=" +
                      json.dumps(passive, sort_keys=True), flush=True)
                print("M16_VERDICT=" + passive["verdict"], flush=True)
            metrics = src.export() if src is not None else None
            if metrics is not None:
                imu = (passive or {}).get("topics", {}).get("imu", {})
                acceptable = audit_acceptance(
                    complete=complete, start_ack=start_ack, stop_ack=stop_ack,
                    metrics=metrics, imu=imu)
                verdict = (
                    "OBSERVED_STATIONARY_IMU_CLOCK_RATE_CONTRACT"
                    if acceptable else "INCOMPLETE_OR_UNVERIFIED_IMU_CLOCK_RATE")
                report = {
                    "milestone": "M19",
                    "observation_duration_requested_s": DURATION_S,
                    "observation_completed": complete,
                    "imu_stream_start_acknowledged": start_ack,
                    "imu_stream_stop_acknowledged": stop_ack,
                    "only_allowed_serial_commands": [
                        "set_imu_stream_hz 50", "set_imu_stream_hz 0"],
                    "absolute_mcu_camera_offset_validated": False,
                    "camera_imu_extrinsics_validated": False,
                    "xrslam_ready": False,
                    "terminal_error": fault,
                    "imu": metrics,
                    "passive_imu_accepted": imu.get("accepted", 0),
                    "passive_imu_invalid": imu.get("invalid", 0),
                    "verdict": verdict,
                }
                print("M19_IMU_CLOCK_AUDIT_JSON=" +
                      json.dumps(report, sort_keys=True), flush=True)
                print("M19_IMU_CLOCK_VERDICT=" + verdict, flush=True)
                return 0 if acceptable else 2
            return 1
        finally:
            if obs is not None:
                obs.destroy_node()
            if src is not None:
                src.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    if os.environ.get("LEROY_M19_SELFTEST") == "1":
        selftest()
        sys.exit(0)
    sys.exit(run())
