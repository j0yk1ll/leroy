#!/usr/bin/env python3
"""M20 passive joint front-camera/real-IMU ROS contract, no motion/VIO.

The camera launch is a separate sensor-only process managed by the M20
shell runner. This process publishes only MCU IMU from the validated M18
source, and spins the existing M16 observer concurrently in a ROS thread.
MCU serial is motor-capable: PHYSICAL power isolation is mandatory.
"""
import json
import math
import os
import signal
import sys
import threading
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
import serial

from m18_imu_stream_source import (
    ImuStreamSource, PassiveSensorObserver, START, STOP, PORT,
    send_only_whitelisted_serial_command, wait_ack,
)
from m20_combined_contract import DURATION_S, evaluate


EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
FORBIDDEN_TOPICS = (
    "/cmd_vel", "/odom", "/tf", "/depthart/depth",
    "/leroy_nav2/diagnostics/obstacle_points",
)
FORBIDDEN_NODES = {
    "leroy_bridge", "leroy_xrslam", "controller_server",
    "planner_server", "bt_navigator", "depthart", "depth_obstacles",
}


def enforce_environment():
    for key, expected in EXPECTED_ENV.items():
        if os.environ.get(key) != expected:
            raise RuntimeError(f"M20 requires {key}={expected}")


def check_isolated_graph(node):
    """Reject any actuation, odometry or excluded depth publishers."""
    bad_topics = [
        name for name in FORBIDDEN_TOPICS
        if node.count_publishers(name) != 0
    ]
    bad_nodes = [
        name for name, _namespace in node.get_node_names_and_namespaces()
        if name in FORBIDDEN_NODES
    ]
    if bad_topics or bad_nodes:
        raise RuntimeError(
            "forbidden_ros_graph:" + repr((bad_topics, bad_nodes)))


def receipt_overlap(observer):
    """Intersection of actual per-topic monotonic receipt intervals."""
    spans = []
    for key in ("camera_image", "camera_info", "imu"):
        stats = observer.stats[key]
        if stats.first_recv is None or stats.last_valid_recv is None:
            return None
        spans.append((stats.first_recv, stats.last_valid_recv))
    return max(0.0, min(end for _start, end in spans)
               - max(start for start, _end in spans))


def selftest():
    enforce_environment()
    rclpy.init(args=[])
    src = obs = None
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        assert len(obs._m16_subscriptions) == 9
        assert src._imu_pub.topic_name == "/imu/data_raw"
        assert obs.report(5.0)["verdict"] == "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS"
        assert receipt_overlap(obs) is None
        check_isolated_graph(src)
        print("M20_ROS_NODES_SELFTEST=PASS", flush=True)
    finally:
        if obs is not None:
            obs.destroy_node()
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run():
    enforce_environment()
    quitting = threading.Event()
    stop_observer = threading.Event()
    observer_faults = []
    previous = {}

    def request_stop(_signum, _frame):
        quitting.set()

    for sig in (signal.SIGINT, signal.SIGTERM):
        previous[sig] = signal.getsignal(sig)
        signal.signal(sig, request_stop)

    src = obs = executor = worker = None
    started = stopped = complete = False
    graph_fault = None
    error = None
    passive = None
    overlap = None

    rclpy.init(args=[])
    try:
        src = ImuStreamSource()
        obs = PassiveSensorObserver()
        executor = SingleThreadedExecutor()
        executor.add_node(obs)

        def spin_observer():
            while not stop_observer.is_set() and rclpy.ok():
                try:
                    executor.spin_once(timeout_sec=0.05)
                except Exception as exc:
                    observer_faults.append(type(exc).__name__ + ": " + str(exc))
                    stop_observer.set()
                    quitting.set()
                    return

        worker = threading.Thread(
            target=spin_observer, name="m20-passive-ros-observer", daemon=True)
        worker.start()
        check_isolated_graph(src)  # BEFORE first physical serial request

        with serial.Serial(
            port=PORT, baudrate=115200, timeout=0.08,
            write_timeout=0.25, exclusive=True,
        ) as device:
            # CDC may reset MCU; power to motors MUST be physically isolated.
            time.sleep(0.3)
            device.reset_input_buffer()
            try:
                send_only_whitelisted_serial_command(device, START)
                started = wait_ack(device, timeout_s=2.0)
                if not started:
                    raise RuntimeError("MCU did not acknowledge IMU stream start")
                print("M20_IMU_STREAM_START_ACK=PASS", flush=True)
                deadline = time.monotonic() + DURATION_S
                graph_check_at = time.monotonic()
                while time.monotonic() < deadline and not quitting.is_set():
                    payload = device.readline()
                    if payload:
                        src.consume(payload)
                    if time.monotonic() >= graph_check_at:
                        try:
                            check_isolated_graph(src)
                        except RuntimeError as exc:
                            graph_fault = str(exc)
                            raise
                        graph_check_at = time.monotonic() + 5.0
                complete = (time.monotonic() >= deadline
                            and not quitting.is_set()
                            and src.terminal_fault is None)
            finally:
                # The ONLY serial requests are the M18-proven IMU enable/disable.
                # Loss of port/forced termination can leave MCU state unknown.
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stopped = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("M20_IMU_STREAM_STOP_ERROR=" +
                          type(exc).__name__, flush=True)
                print("M20_IMU_STREAM_STOP_ACK=" +
                      ("PASS" if stopped else "UNVERIFIED"), flush=True)
    except Exception as exc:
        error = type(exc).__name__ + ": " + str(exc)
        print("M20_COMBINED_ERROR=" + error, flush=True)
    finally:
        # Join observer BEFORE reading its stats; no concurrent mutation.
        stop_observer.set()
        if worker is not None:
            worker.join(timeout=3.0)
        if worker is not None and worker.is_alive():
            observer_faults.append("observer_executor_did_not_stop")
        if executor is not None and (worker is None or not worker.is_alive()):
            executor.shutdown(timeout_sec=2.0)
        if obs is not None and (worker is None or not worker.is_alive()):
            passive = obs.report(DURATION_S)
            overlap = receipt_overlap(obs)

        if passive is not None:
            print("M16_PASSIVE_REPORT_JSON=" +
                  json.dumps(passive, sort_keys=True), flush=True)
            print("M16_VERDICT=" + passive["verdict"], flush=True)

        if src is not None:
            source_report = src.export()
            accepted = bool(
                passive is not None
                and evaluate(
                    completed=complete, start_ack=started, stop_ack=stopped,
                    source=source_report, observer=passive, overlap_s=overlap,
                    forbidden_graph_seen=graph_fault is not None,
                    observer_thread_error=bool(observer_faults))
                and error is None)
            verdict = (
                "OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT"
                if accepted else "INCOMPLETE_OR_UNVERIFIED_CONCURRENT_STREAMS")
            report = {
                "milestone": "M20",
                "duration_requested_s": DURATION_S,
                "observation_completed": complete,
                "imu_stream_start_acknowledged": started,
                "imu_stream_stop_acknowledged": stopped,
                "serial_command_allowlist": [
                    "set_imu_stream_hz 50", "set_imu_stream_hz 0"],
                "motor_commands_sent": 0,
                "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
                "streams_tested": [
                    "/camera/image_rect", "/camera/camera_info",
                    "/imu/data_raw"],
                "depthart_started": False,
                "xrslam_started": False,
                "odom_tf_started": False,
                "clock_rate_ready": source_report["clock_rate_estimate_ready"],
                "clock_rate_estimate_ppm": source_report["clock_rate_estimate_ppm"],
                "sensor_receipt_overlap_s": (
                    round(overlap, 3) if overlap is not None else None),
                "imu_source": source_report,
                "forbidden_graph_fault": graph_fault,
                "observer_thread_faults": observer_faults,
                "error": error,
                "absolute_camera_imu_offset_validated": False,
                "camera_imu_extrinsics_validated": False,
                "camera_calibration_accuracy_validated": False,
                "xrslam_ready": False,
                "verdict": verdict,
            }
            print("M20_COMBINED_REPORT_JSON=" +
                  json.dumps(report, sort_keys=True), flush=True)
            print("M20_COMBINED_VERDICT=" + verdict, flush=True)

        for sig, handler in previous.items():
            signal.signal(sig, handler)
        # The observer thread should always terminate before ROS shutdown.
        if executor is not None and (worker is None or not worker.is_alive()):
            try:
                executor.remove_node(obs)
            except Exception:
                pass
        if obs is not None and (worker is None or not worker.is_alive()):
            obs.destroy_node()
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if src is not None and passive is not None and accepted else 2


if __name__ == "__main__":
    if os.environ.get("LEROY_M20_SELFTEST") == "1":
        selftest()
        sys.exit(0)
    sys.exit(run())
