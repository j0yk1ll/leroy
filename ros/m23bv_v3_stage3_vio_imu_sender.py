#!/usr/bin/env python3
"""Stage 3 supervised MCU-IMU publisher for diagnostic VIO only.

One serial open; the only writes are unchanged M18 START/STOP.
CDC open/DTR can reset motor-capable firmware; physical rail isolation
and a NEW host-side operator approval are mandatory. Never launch alone.
"""
import json
import os
import signal
import time

import rclpy
import serial

from m18_imu_stream_source import (
    ImuStreamSource, START, STOP, PORT,
    send_only_whitelisted_serial_command, wait_ack,
)

DURATION_SEC = 70.0
REQUIRED = {
    "M23BV_STAGE3_VIO_IMU_ONLY": "YES",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
}
REQUIRED_NODES = {
    "/camera/usb_cam", "/camera/rectify_node", "/depthart",
    "/leroy_xrslam", "/robot_state_publisher", "/controller_server",
    "/planner_server", "/bt_navigator",
}
FORBIDDEN_NODES = {
    "leroy_bridge", "velocity_smoother", "diff_drive_controller",
    "controller_manager", "ros2_control_node", "m20_combined_observer",
}
FORBIDDEN_COMMANDS = (
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_out", "/diff_drive_controller/cmd_vel",
)
EXPECTED_BLOBS = {
    "/opt/m18/m18_imu_stream_source.py": "3c5fb37ca291533e2702a9159f3e8db11b076d22",
    "/opt/m18/m18_imu_stream_contract.py": "443aa82066e88d50cb88859304e9197e3f459ee0",
}


def guard_env():
    for key, value in REQUIRED.items():
        if os.environ.get(key) != value:
            raise RuntimeError("BLOCKED_ENV_" + key)
    if not os.path.exists(PORT) or os.path.exists("/dev/video2"):
        raise RuntimeError("BLOCKED_DEVICE_SCOPE")
    if START != b"set_imu_stream_hz 50\r\n" or STOP != b"set_imu_stream_hz 0\r\n":
        raise RuntimeError("BLOCKED_IMU_SERIAL_COMMAND_ALLOWLIST")
    import hashlib
    from pathlib import Path
    for path, expected in EXPECTED_BLOBS.items():
        data = Path(path).read_bytes()
        actual = hashlib.sha1(b"blob " + str(len(data)).encode()
                              + b"\0" + data).hexdigest()
        if actual != expected:
            raise RuntimeError("BLOCKED_PINNED_M18_SOURCE:" + path)


def graph_guard(node, require_ready):
    names = {
        ((namespace.rstrip("/") if namespace != "/" else "") + "/" + name)
        for name, namespace in node.get_node_names_and_namespaces()
    }
    if any(name.rsplit("/", 1)[-1] in FORBIDDEN_NODES for name in names):
        raise RuntimeError("BLOCKED_MOTOR_OR_OLD_SENSOR_NODE")
    for topic in FORBIDDEN_COMMANDS:
        if (node.get_publishers_info_by_topic(topic)
                or node.get_subscriptions_info_by_topic(topic)):
            raise RuntimeError("BLOCKED_MOTOR_COMMAND_TOPIC:" + topic)
    odom = node.get_publishers_info_by_topic("/odom")
    if any((ep.node_name, ep.node_namespace) != ("leroy_xrslam", "/")
           for ep in odom):
        raise RuntimeError("BLOCKED_UNEXPECTED_ODOM_PUBLISHER")
    dynamic_tf = node.get_publishers_info_by_topic("/tf")
    if any((ep.node_name, ep.node_namespace) not in
           (("leroy_xrslam", "/"), ("robot_state_publisher", "/"))
           for ep in dynamic_tf):
        raise RuntimeError("BLOCKED_UNEXPECTED_TF_AUTHORITY")
    imu_sub = node.get_subscriptions_info_by_topic("/imu/data_raw")
    xr_imu = any(ep.node_name == "leroy_xrslam"
                 and ep.node_namespace == "/" for ep in imu_sub)
    return REQUIRED_NODES.issubset(names) and xr_imu if require_ready else True


def run():
    guard_env()
    quitting = [False]
    original = {}
    for signo in (signal.SIGINT, signal.SIGTERM):
        original[signo] = signal.getsignal(signo)
        signal.signal(signo, lambda _sig, _frame: quitting.__setitem__(0, True))
    rclpy.init(args=[])
    src = None
    start_ack = False
    stop_ack = False
    finished = False
    error = None
    try:
        src = ImuStreamSource()
        # Never open the CDC interface until the intended XRSLAM
        # subscription and all diagnostic graph nodes are established.
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline and not quitting[0]:
            if graph_guard(src, True):
                break
            time.sleep(0.1)
        else:
            raise RuntimeError("BLOCKED_PRE_SERIAL_VIO_GRAPH_OR_INTERRUPT")
        print("STAGE3_VIO_PRE_SERIAL_GRAPH=PASS", flush=True)
        # Opening this port itself may reset motor-capable firmware.
        # Guarded by the host runner's physical rail disconnection.
        with serial.Serial(
            port=PORT, baudrate=115200, timeout=0.08,
            write_timeout=0.25, exclusive=True,
        ) as device:
            # STOP is attempted even if post-open settling/reset or START fails.
            try:
                time.sleep(0.3)
                device.reset_input_buffer()
                send_only_whitelisted_serial_command(device, START)
                start_ack = wait_ack(device, timeout_s=2.0)
                if not start_ack:
                    raise RuntimeError("BLOCKED_MCU_START_ACK")
                print("STAGE3_VIO_IMU_START_ACK=PASS", flush=True)
                deadline = time.monotonic() + DURATION_SEC
                check_graph_at = 0.0
                while time.monotonic() < deadline and not quitting[0]:
                    packet = device.readline()
                    if packet:
                        src.consume(packet)
                    if time.monotonic() >= check_graph_at:
                        if not graph_guard(src, True):
                            raise RuntimeError("BLOCKED_RUNTIME_VIO_GRAPH_CHANGED")
                        check_graph_at = time.monotonic() + 2.0
                finished = (not quitting[0] and time.monotonic() >= deadline
                            and src.terminal_fault is None)
            finally:
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stop_ack = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("STAGE3_VIO_IMU_STOP_ERROR=" +
                          type(exc).__name__, flush=True)
                print("STAGE3_VIO_IMU_STOP_ACK=" +
                      ("PASS" if stop_ack else "UNVERIFIED"), flush=True)
    except Exception as exc:
        error = type(exc).__name__ + ": " + str(exc)
        print("STAGE3_VIO_IMU_ERROR=" + error, flush=True)
    finally:
        if src is not None:
            metrics = src.export()
            print("STAGE3_VIO_IMU_METRICS=" + json.dumps(metrics, sort_keys=True),
                  flush=True)
        else:
            metrics = None
        if src is not None:
            src.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        for signo, old in original.items():
            signal.signal(signo, old)
    accepted = bool(
        start_ack and stop_ack and finished and error is None
        and metrics is not None
        and metrics["counts"]["published_imu_messages"] >= 2000
        and metrics["counts"]["sequence_faults"] == 0
        and metrics["counts"]["timestamp_faults"] == 0
        and metrics["counts"]["stale_or_future_withheld"] == 0
        and metrics["counts"]["malformed_imu_packets"] == 0
        and metrics["counts"]["sequence_gap_samples"] == 0
        and metrics["counts"]["calibration_withheld"] == 0
        and metrics["counts"]["invalid_ascii_lines"] == 0
        and metrics["counts"]["unexpected_serial_lines"] == 0
        and metrics["terminal_fault"] is None
    )
    print("STAGE3_VIO_IMU_VERDICT=" +
          ("PASS_DIAGNOSTIC_STREAM" if accepted
           else "BLOCKED_INCOMPLETE_OR_UNVERIFIED"), flush=True)
    if not stop_ack:
        print("STAGE3_VIO_MCU_STATE=UNKNOWN_OPERATOR_INTERVENTION_REQUIRED",
              flush=True)
    return 0 if accepted else 2


if __name__ == "__main__":
    raise SystemExit(run())
