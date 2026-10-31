#!/usr/bin/env python3
"""Persistent sensor-only M18 publisher for the existing isolated live graph.

Requires a separately verified PHYSICAL motor-power disconnect. Even though
only IMU stream START/STOP packets are emitted, the MCU CDC is motor-capable.
No bridge, motor control, ROS cmd_vel, Nav2 action, or key material.
Stops on signal, transport/MCU failure, unexpected graph owner or IMU loss.
No automatic restart after a power cut or process failure.
"""

# TODO(AUTONOMY) #002 [MCU sensor ownership]: In a future motor-capable runtime,
# migrate the single real STM32 serial writer to the commissioned bridge
# without starting this independent IMU-only CDC owner at the same time.
# Track firmware identity, stream sequence/timestamp health, USB reconnects,
# bounded queue latency and STOP ACK semantics as verified health evidence.
# Acceptance: unplug/replug, out-of-order telemetry and MCU reset invalidate
# readiness; restoring communication does not automatically rearm actuators.


import json
import os
import signal
import time

REQUIRED_ENV = {
    "LEROY_BOOT_PERSISTENT_IMU_ONLY": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
}
DEVICE = "/dev/ttyACM0"
MAX_SILENCE_S = 0.45
STARTUP_TIMEOUT_S = 8.0
STATUS_INTERVAL_S = 10.0
FORBIDDEN_NODES = frozenset((
    "leroy_bridge", "leroy_bridge_node", "controller_server",
    "planner_server", "bt_navigator", "behavior_server",
    "velocity_smoother", "collision_monitor", "motor_bridge",
))
FORBIDDEN_TOPICS = frozenset((
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed",
    "/motor/command", "/motor/enable", "/joint_commands",
))


def preflight(env=None, exists=None):
    env = os.environ if env is None else env
    exists = os.path.exists if exists is None else exists
    for key, value in REQUIRED_ENV.items():
        if env.get(key) != value:
            raise RuntimeError("BOOT_IMU_ENV_" + key)
    if not exists(DEVICE):
        raise RuntimeError("BOOT_IMU_MCU_DEVICE_MISSING")
    for forbidden in ("/dev/video2", "/dev/gpiochip0"):
        if exists(forbidden):
            raise RuntimeError("BOOT_IMU_UNEXPECTED_DEVICE")
    return True


def graph_ok(node):
    nodes = {name for name, _ in node.get_node_names_and_namespaces()}
    topics = {topic for topic, _ in node.get_topic_names_and_types()}
    if nodes & FORBIDDEN_NODES or topics & FORBIDDEN_TOPICS:
        return False
    pubs = node.get_publishers_info_by_topic("/imu/data_raw")
    return (
        len(pubs) == 1
        and pubs[0].node_name == "m18_imu_source"
        and pubs[0].node_namespace == "/"
        and pubs[0].topic_type == "sensor_msgs/msg/Imu"
        and "leroy_xrslam" in nodes
        and "rectify_node" in nodes
    )


def run():
    # Reuse the existing operator-tested parser, time mapper, ROS publisher
    # and *exact* MCU START and STOP implementations.
    preflight()
    import serial
    import rclpy
    from leroy_sensors.imu_protocol import START, STOP, PORT
    from leroy_sensors.imu_source import (
        ImuStreamSource, send_only_whitelisted_serial_command, wait_ack,
    )

    if PORT != DEVICE or (START, STOP) != (
            b"set_imu_stream_hz 50\r\n",
            b"set_imu_stream_hz 0\r\n"):
        raise RuntimeError("BOOT_IMU_COMMAND_ALLOWLIST_CHANGED")

    rclpy.init(args=[])
    node = None
    stopping = [False]
    previous = {}
    for signum in (signal.SIGINT, signal.SIGTERM):
        previous[signum] = signal.getsignal(signum)
        signal.signal(signum, lambda _s, _f: stopping.__setitem__(0, True))

    started = False
    stopped = False
    fault = None
    try:
        node = ImuStreamSource()
        deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while not stopping[0] and time.monotonic() < deadline:
            if graph_ok(node):
                break
            time.sleep(0.05)
        else:
            raise RuntimeError("BOOT_IMU_GRAPH_NOT_READY")
        print("BOOT_IMU_SOURCE_GRAPH=PASS_SENSOR_ONLY", flush=True)

        with serial.Serial(
            port=DEVICE, baudrate=115200, timeout=0.08,
            write_timeout=0.25, exclusive=True,
        ) as device:
            try:
                # Serial open can reboot firmware; never interpret its
                # resulting output as motor-rail disconnection evidence.
                time.sleep(0.3)
                device.reset_input_buffer()
                send_only_whitelisted_serial_command(device, START)
                started = wait_ack(device, timeout_s=2.0)
                if not started:
                    raise RuntimeError("BOOT_IMU_START_ACK_MISSING")
                print("BOOT_IMU_SOURCE_START_ACK=PASS", flush=True)

                last_sample = time.monotonic()
                last_status = last_sample
                while not stopping[0]:
                    if not graph_ok(node):
                        raise RuntimeError("BOOT_IMU_GRAPH_CHANGED")
                    packet = device.readline()
                    if packet:
                        before = node.counts["published_imu_messages"]
                        node.consume(packet)
                        if node.terminal_fault is not None:
                            raise RuntimeError("BOOT_IMU_SOURCE_TERMINAL_FAULT")
                        if node.counts["published_imu_messages"] > before:
                            last_sample = time.monotonic()
                    now = time.monotonic()
                    if now - last_sample > MAX_SILENCE_S:
                        raise RuntimeError("BOOT_IMU_SAMPLE_TIMEOUT")
                    c = node.counts
                    if (c["sequence_gap_samples"] or c["sequence_faults"]
                            or c["timestamp_faults"]
                            or c["malformed_imu_packets"]
                            or c["calibration_withheld"]
                            or c["stale_or_future_withheld"]
                            or c["unexpected_serial_lines"]
                            or c["invalid_ascii_lines"]):
                        raise RuntimeError("BOOT_IMU_SOURCE_INTEGRITY_FAULT")
                    if now - last_status >= STATUS_INTERVAL_S:
                        print("BOOT_IMU_SOURCE_STATUS_JSON=" + json.dumps({
                            "imu_published": c["published_imu_messages"],
                            "sensor_only": True,
                            "motor_commands_sent": 0,
                            "source": "/m18_imu_source",
                        }, sort_keys=True), flush=True)
                        last_status = now
            finally:
                # STOP is mandatory even after a failure or SIGTERM.
                try:
                    send_only_whitelisted_serial_command(device, STOP)
                    stopped = wait_ack(device, timeout_s=2.0)
                except (serial.SerialException, OSError, RuntimeError) as exc:
                    print("BOOT_IMU_SOURCE_STOP_ERROR=" +
                          type(exc).__name__, flush=True)
                print("BOOT_IMU_SOURCE_STOP_ACK=" +
                      ("PASS" if stopped else "UNVERIFIED"), flush=True)
    except Exception as exc:
        fault = type(exc).__name__ + ":" + str(exc)
        print("BOOT_IMU_SOURCE_FAULT=" + fault, flush=True)
    finally:
        for signum, handler in previous.items():
            signal.signal(signum, handler)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    good = started and stopped and fault is None and stopping[0]
    print("BOOT_IMU_SOURCE_RESULT=" + (
        "PASS_OPERATOR_STOPPED" if good else "BLOCKED"), flush=True)
    if not stopped:
        print("BOOT_IMU_SOURCE_STOP_STATE=UNVERIFIED_DISCONNECT_POWER",
              flush=True)
    return 0 if good else 2


if __name__ == "__main__":
    raise SystemExit(run())
