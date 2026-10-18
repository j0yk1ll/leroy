#!/usr/bin/env python3
"""M22F actual ROS bridge lifecycle, in-memory CDC ONLY.

Monkeypatch pyserial before LeroyBridgeNode construction in isolated Podman.
The source bridge still has a zero-wheel-only serial writer and no /cmd_vel
subscriber. Do not run outside the M22F no-device image.
"""
import json
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

import leroy_bridge.node as module
from leroy_bridge.command_ingress import command_to_bytes

ENV = {"LEROY_NAV2_DRY_RUN": "1", "ROS_DOMAIN_ID": "170",
       "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
       "M22F_FAKE_SERIAL_ONLY": "1"}
URI = "m22f://memory"
STOP = b"set_wheels 0 0 0 0\n"


def check(ok, reason):
    if not ok:
        raise AssertionError(reason)


def preflight():
    for name, value in ENV.items():
        check(os.environ.get(name) == value,
              "isolation env differs: " + name)
    for path in ("/dev/ttyACM0", "/dev/ttyACM1",
                 "/dev/video2", "/dev/video8"):
        check(not os.path.exists(path),
              "physical device present: " + path)


class MemoryCDC:
    """Strict fake USB/CDC receiver. Never touches the filesystem."""

    def __init__(self, *, port, baudrate, timeout, write_timeout):
        check(port == URI, "real or unexpected serial URI")
        check(baudrate == 115200, "unexpected baud")
        self.is_open = True
        self.events = []
        self.lock = threading.Lock()
        self.fail_next_write = False

    def reset_input_buffer(self):
        return None

    def readline(self):
        time.sleep(0.008)
        return b""

    def write(self, data):
        check(type(data) is bytes and data.endswith(b"\n"),
              "bad fake serial framing")
        with self.lock:
            if self.fail_next_write:
                self.fail_next_write = False
                raise module.serial.SerialException("fake CDC write error")
            command = data[:-1].decode("ascii", errors="strict")
            check(command_to_bytes(command) == data,
                  "serial allowlist bypassed")
            self.events.append(data)
        return len(data)

    def flush(self):
        return None

    def close(self):
        self.is_open = False

    def snapshot(self):
        with self.lock:
            return list(self.events)


def run():
    preflight()
    real_serial_constructor = module.serial.Serial
    instances = []
    bridge = probe = executor = fresh_bridge = None
    initialized = False

    def fake_factory(*args, **kwargs):
        check(not args, "positional serial args forbidden")
        fake = MemoryCDC(**kwargs)
        instances.append(fake)
        return fake

    module.serial.Serial = fake_factory
    try:
        rclpy.init(args=[
            "--ros-args", "-p", "serial_port:=" + URI,
            "-p", "imu_stream_hz:=0", "-p", "encoder_stream_hz:=0",
        ])
        initialized = True
        bridge = module.LeroyBridgeNode()
        check(len(instances) == 1, "expected exactly one fake CDC")
        fake = instances[0]
        check(bridge._serial is fake, "bridge CDC object replaced")
        check(bridge.get_parameter("serial_port").value == URI,
              "fake serial parameter was not applied")
        startup = fake.snapshot()
        check(startup[:3] == [
            b"set_watchdog_ms 500\n",
            b"set_imu_stream_hz 0\n",
            b"set_encoder_stream_hz 0\n",
        ], "unexpected startup CDC packet sequence")
        check(bridge.get_subscriptions_info_by_topic("/cmd_vel") == [],
              "raw velocity subscriber unexpectedly exists")
        print("M22F_FAKE_BRIDGE_STARTUP=PASS", flush=True)

        executor = SingleThreadedExecutor()
        executor.add_node(bridge)
        probe = Node("m22f_untrusted_injector", enable_rosout=False,
                     start_parameter_services=False)
        executor.add_node(probe)
        publisher = probe.create_publisher(Twist, "/cmd_vel", 10)
        until = time.monotonic() + 0.75
        sent = 0
        while time.monotonic() < until:
            command = Twist()
            command.linear.x = 0.9
            command.linear.y = -0.8
            command.angular.z = 1.3
            publisher.publish(command)
            sent += 1
            executor.spin_once(timeout_sec=0.01)
        check(sent > 10, "insufficient adversarial ROS messages")
        check(bridge.get_subscriptions_info_by_topic("/cmd_vel") == [],
              "bridge subscribed to adversarial /cmd_vel")
        check(b"keepalive\n" in fake.snapshot(),
              "actual bridge keepalive timer never ran")
        check(all(not packet.startswith(b"set_wheels ")
                  or packet == STOP for packet in fake.snapshot()),
              "nonzero wheel packet reached fake CDC")
        print("M22F_RAW_ROS_BYPASS_INHIBIT=PASS", flush=True)

        original_count = len(fake.snapshot())
        check(bridge._send_command("set_wheels 64 -64 64 -64") is False,
              "nonzero wheels accepted")
        check(len(fake.snapshot()) == original_count,
              "nonzero wheels reached fake CDC")
        check(bridge._send_command("set_wheels 0 0 0 0") is True,
              "zero wheels refused")
        check(fake.snapshot()[-1] == STOP, "wrong zero-wheel bytes")
        print("M22F_CDC_ZERO_WHEELS_ONLY=PASS", flush=True)

        bridge._clock_fault = True
        earlier = len(fake.snapshot())
        bridge._send_keepalive()
        check(len(fake.snapshot()) == earlier,
              "keepalive continued through clock fault")
        bridge._clock_fault = False
        print("M22F_CLOCK_FAULT_KEEPALIVE_INHIBIT=PASS", flush=True)

        fake.fail_next_write = True
        earlier = len(fake.snapshot())
        check(bridge._send_command("set_wheels 0 0 0 0") is False,
              "fake write failure not surfaced")
        check(len(fake.snapshot()) == earlier,
              "failed fake write was counted")
        check(bridge._transport_fault.is_set(),
              "CDC fault was not terminally latched")
        check(bridge._stop_event.is_set(),
              "CDC fault did not quiesce bridge reader")
        check(bridge._send_command("keepalive") is False,
              "post-fault keepalive unexpectedly accepted")
        check(bridge._send_command("set_wheels 0 0 0 0") is False,
              "post-fault STOP write unexpectedly attempted")
        bridge._send_keepalive()
        check(len(fake.snapshot()) == earlier,
              "post-fault CDC log changed")
        print("M22F_FAKE_WRITE_ERROR_DETECTED=PASS", flush=True)
        print("M22F_TERMINAL_TRANSPORT_FAULT=PASS", flush=True)

        executor.remove_node(probe)
        probe.destroy_node()
        probe = None
        executor.remove_node(bridge)
        bridge.destroy_node()
        bridge = None
        check(fake.is_open is False, "fake CDC not closed on shutdown")
        packets = fake.snapshot()
        check(len(packets) == earlier,
              "CDC wrote after terminal fault during shutdown")
        check(all(not packet.startswith(b"set_wheels ")
                  or packet == STOP for packet in packets),
              "nonzero wheel output after shutdown")
        print("M22G_SHUTDOWN_NO_CDC_AFTER_FAULT=PASS", flush=True)

        # Fresh node construction is the only permitted transport-fault
        # recovery in this milestone; this is not operator rearming.
        fresh_bridge = module.LeroyBridgeNode()
        check(len(instances) == 2, "fresh bridge did not create new fake CDC")
        fresh_fake = instances[-1]
        check(fresh_bridge._serial is fresh_fake,
              "fresh bridge inherited old fake CDC")
        check(not fresh_bridge._transport_fault.is_set(),
              "fresh bridge incorrectly inherited terminal CDC latch")
        fresh_bridge.destroy_node()
        fresh_bridge = None
        check(fresh_fake.is_open is False,
              "fresh fake CDC not closed")
        fresh_log = fresh_fake.snapshot()
        check(fresh_log[-3:] == [
            STOP, b"set_imu_stream_hz 0\n",
            b"set_encoder_stream_hz 0\n",
        ], "fault-free shutdown did not send stop and streams-off")
        check(all(not item.startswith(b"set_wheels ")
                  or item == STOP for item in fresh_log),
              "fresh instance emitted nonzero fake wheel duty")
        print("M22G_FRESH_BRIDGE_SHUTDOWN_ZERO=PASS", flush=True)

        print("M22F_BRIDGE_REPORT_JSON=" + json.dumps({
            "milestone": "M22F", "evidence_type": "FAKE_ROS_LIFECYCLE",
            "bridge_source": "LeroyBridgeNode",
            "fake_cdc_writes": len(packets),
            "untrusted_cmd_vel_samples": sent,
            "nonzero_fake_wheel_writes": 0,
            "serial_port": URI, "physical_device_access": False,
            "motion_ingress_enabled": False,
            "transport_failure_terminal_latch_validated": True,
            "fresh_bridge_shutdown_stop_validated": True,
            "physical_stop_acknowledgement_validated": False,
        }, sort_keys=True), flush=True)
        print("M22F_ROS_BRIDGE_SELFTEST=PASS", flush=True)
        print("M22F_VERDICT=REAL_ROS_NODE_FAKE_CDC_ZERO_ONLY", flush=True)
        print("M22F_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        if executor is not None:
            if probe is not None:
                executor.remove_node(probe)
            if bridge is not None:
                executor.remove_node(bridge)
            executor.shutdown(timeout_sec=1.0)
        if probe is not None:
            probe.destroy_node()
        if bridge is not None:
            bridge.destroy_node()
        if fresh_bridge is not None:
            fresh_bridge.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        module.serial.Serial = real_serial_constructor


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M22F_ROS_BRIDGE_SELFTEST=FAIL", flush=True)
        print("M22F_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
