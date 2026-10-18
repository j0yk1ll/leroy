#!/usr/bin/env python3
"""M22H: signed ROS envelopes + real LeroyBridgeNode, FAKE CDC ONLY.

The sidecar checks HMAC/session/rearm through M22E and publishes to
unconsumed DIAGNOSTIC topics. It is NOT a deployable bridge ingress:
keys are synthetic constants, node names are unauthenticated, sensor
health is invented, and the production M22D serial writer remains
zero-wheel-only. Never run on a physical robot or outside isolated Podman.
"""
from dataclasses import asdict, fields
import json
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

import leroy_bridge.node as bridge_module
from leroy_bridge.authorized_session import (
    MotionEnvelope, RearmEnvelope, SessionVerifier, OWNER,
    sign_test_motion, sign_test_rearm,
)
from m22a_diagnostic_authority import Inputs
from m22c_fake_serial_boundary import FakeSerialTransport, ZERO_PWM
from m22e_fake_signed_ingress import FakeSignedIngress
from m22f_fake_bridge_ros_selftest import MemoryCDC, preflight, URI, STOP

COMMAND = "/leroy_nav2/diagnostics/m22h/signed_motion"
REARM = "/leroy_nav2/diagnostics/m22h/signed_rearm"
HEALTH = "/leroy_nav2/diagnostics/m22h/synthetic_health"
OUTPUT = "/leroy_nav2/diagnostics/m22h/accepted_twist"
STATUS = "/leroy_nav2/diagnostics/m22h/gate_status"
KEY_MOTION = b"M" * 32
KEY_OPERATOR = b"O" * 32
HEALTH_FIELDS = tuple(Inputs.__dataclass_fields__)
ENV = {"LEROY_NAV2_DRY_RUN": "1", "ROS_DOMAIN_ID": "170",
       "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
       "M22F_FAKE_SERIAL_ONLY": "1",
       "M22H_FAKE_SIGNED_ROS_ONLY": "1"}


def require(condition, detail):
    if not condition:
        raise AssertionError("M22H: " + detail)


def message(obj):
    msg = String()
    msg.data = json.dumps(asdict(obj), sort_keys=True,
                          allow_nan=False)
    return msg


def decode(cls, data):
    try:
        source = json.loads(data)
        if type(source) is not dict:
            return None
        allowed = {f.name for f in fields(cls)}
        if set(source) != allowed:
            return None
        return cls(**source)
    except (TypeError, ValueError, KeyError):
        return None


class SignedDiagnosticGate(Node):
    def __init__(self, session):
        super().__init__("m22h_signed_diagnostic_gate",
                         enable_rosout=False, start_parameter_services=False)
        self.session = session
        self.fake_sink = FakeSerialTransport()
        self.authority = FakeSignedIngress(
            transport=self.fake_sink, session=session)
        self.pending_command = None
        self.pending_rearm = None
        self.invalid_command_seen = False
        self.invalid_rearm_seen = False
        self.synthetic_health = None
        self.last_health_ns = None
        self.invalid_health_seen = False
        self.last_result = None
        self.cycles = 0
        self.command_sub = self.create_subscription(
            String, COMMAND, self.on_command, 10)
        self.rearm_sub = self.create_subscription(
            String, REARM, self.on_rearm, 10)
        self.health_sub = self.create_subscription(
            String, HEALTH, self.on_health, 10)
        self.twist_pub = self.create_publisher(Twist, OUTPUT, 10)
        self.status_pub = self.create_publisher(String, STATUS, 10)
        for expected, actual in (
            (COMMAND, self.command_sub.topic_name),
            (REARM, self.rearm_sub.topic_name),
            (HEALTH, self.health_sub.topic_name),
            (OUTPUT, self.twist_pub.topic_name),
            (STATUS, self.status_pub.topic_name),
        ):
            require(expected == actual, "diagnostic topic remap refused")
        self.create_timer(0.1, self.tick)

    def on_command(self, msg):
        obj = decode(MotionEnvelope, msg.data)
        if obj is None:
            self.invalid_command_seen = True
        else:
            self.pending_command = obj

    def on_rearm(self, msg):
        obj = decode(RearmEnvelope, msg.data)
        if obj is None:
            self.invalid_rearm_seen = True
        else:
            self.pending_rearm = obj

    def on_health(self, msg):
        self.synthetic_health = None
        self.last_health_ns = None
        try:
            state = json.loads(msg.data)
            if (type(state) is not dict
                    or set(state) != set(HEALTH_FIELDS)
                    or any(type(state[name]) is not bool
                           for name in HEALTH_FIELDS)):
                raise ValueError("malformed synthetic health")
            self.synthetic_health = Inputs(**state)
            self.last_health_ns = time.monotonic_ns()
        except (TypeError, KeyError, ValueError):
            self.invalid_health_seen = True

    def sources_ok(self):
        def unique(topic, name):
            pubs = self.get_publishers_info_by_topic(topic)
            return (len(pubs) == 1
                    and pubs[0].node_name == name
                    and pubs[0].node_namespace == "/"
                    and pubs[0].topic_type == "std_msgs/msg/String")
        return (unique(COMMAND, OWNER)
                and unique(REARM, "m22h_fake_operator")
                and unique(HEALTH, "m22h_fake_health"))

    def tick(self):
        now_ns = time.monotonic_ns()
        graph_ok = self.sources_ok()
        good_health = (
            not self.invalid_health_seen
            and self.synthetic_health is not None
            and self.last_health_ns is not None
            and 0 <= now_ns - self.last_health_ns <= 250_000_000
        )
        health = (self.synthetic_health if good_health
                  else Inputs(False, False, False, False, False))
        cmd = (None if self.invalid_command_seen
               else self.pending_command)
        rearm = (False if self.invalid_rearm_seen
                 else self.pending_rearm)
        self.pending_command = None
        self.pending_rearm = None
        self.invalid_command_seen = False
        self.invalid_rearm_seen = False
        self.invalid_health_seen = False

        report = self.authority.cycle(
            now_ns=now_ns, health=health,
            publishers=(OWNER,) if graph_ok else (),
            command=cmd, rearm=rearm)
        self.last_result = report
        self.cycles += 1

        out = Twist()
        if (report.state == "ARMED" and report.pwm != ZERO_PWM
                and isinstance(cmd, MotionEnvelope)):
            out.linear.x = cmd.vx
            out.linear.y = cmd.vy
            out.angular.z = cmd.wz
        self.twist_pub.publish(out)
        stat = String()
        stat.data = json.dumps({
            "diagnostic_only": True, "physical_device_access": False,
            "state": report.state, "reason": report.reason,
            "fake_pwm": list(report.pwm),
            "session_id_logged": False,
        }, sort_keys=True)
        self.status_pub.publish(stat)


class ROSAudit(Node):
    def __init__(self):
        super().__init__("m22h_diagnostic_audit", enable_rosout=False,
                         start_parameter_services=False)
        self.twists = []
        self.statuses = []
        self.create_subscription(Twist, OUTPUT, self.record_twist, 10)
        self.create_subscription(String, STATUS, self.record_status, 10)

    def record_twist(self, data):
        self.twists.append((
            data.linear.x, data.linear.y, data.angular.z))

    def record_status(self, data):
        self.statuses.append(json.loads(data.data))


class Driver:
    def __init__(self, executor, source, health, operator, gate):
        self.executor = executor
        self.gate = gate
        self.source = source.create_publisher(String, COMMAND, 10)
        self.health = health.create_publisher(String, HEALTH, 10)
        self.operator = operator.create_publisher(String, REARM, 10)
        self.seq = 0
        self.rearm_counter = 0

    def rearm(self):
        now = time.monotonic_ns()
        self.operator.publish(message(sign_test_rearm(
            key=KEY_OPERATOR, session_id=self.gate.session.session_id,
            counter=self.rearm_counter, generated_ns=now - 10_000_000,
            expires_ns=now + 150_000_000)))
        self.rearm_counter += 1

    def send_command(self, *, key=KEY_MOTION):
        now = time.monotonic_ns()
        self.source.publish(message(sign_test_motion(
            key=key, session_id=self.gate.session.session_id,
            sequence=self.seq, generated_ns=now - 10_000_000,
            expires_ns=now + 150_000_000)))
        self.seq += 1

    def run_for(self, duration_s, *, send_commands=True,
                send_health=True):
        until = time.monotonic() + duration_s
        next_send = 0.0
        while time.monotonic() < until:
            now = time.monotonic()
            if now >= next_send:
                if send_commands:
                    self.send_command()
                if send_health:
                    payload = String()
                    payload.data = json.dumps(
                        {name: True for name in HEALTH_FIELDS})
                    self.health.publish(payload)
                next_send = now + 0.04
            self.executor.spin_once(timeout_sec=0.005)


def run():
    preflight()
    for name, value in ENV.items():
        require(os.environ.get(name) == value, "isolation env mismatch " + name)
    true_serial = bridge_module.serial.Serial
    instances = []
    nodes = []
    executor = None
    initialized = False

    def fake_factory(*args, **kwargs):
        require(not args, "serial positional arguments rejected")
        fake = MemoryCDC(**kwargs)
        instances.append(fake)
        return fake

    bridge_module.serial.Serial = fake_factory
    try:
        rclpy.init(args=[
            "--ros-args", "-p", "serial_port:=" + URI,
            "-p", "imu_stream_hz:=0", "-p", "encoder_stream_hz:=0"])
        initialized = True
        bridge = bridge_module.LeroyBridgeNode()
        session = SessionVerifier(
            motion_key=KEY_MOTION, operator_key=KEY_OPERATOR)
        gate = SignedDiagnosticGate(session)
        source = Node(OWNER, enable_rosout=False,
                      start_parameter_services=False)
        health = Node("m22h_fake_health", enable_rosout=False,
                      start_parameter_services=False)
        operator = Node("m22h_fake_operator", enable_rosout=False,
                        start_parameter_services=False)
        audit = ROSAudit()
        nodes.extend((bridge, gate, source, health, operator, audit))
        executor = SingleThreadedExecutor()
        for node in nodes:
            executor.add_node(node)
        driver = Driver(executor, source, health, operator, gate)
        for _ in range(30):
            if gate.sources_ok():
                break
            driver.run_for(0.1)
        require(gate.sources_ok(), "expected fake publishers not discovered")
        require(len(instances) == 1, "real bridge did not use only fake CDC")
        require(bridge.get_subscriptions_info_by_topic("/cmd_vel") == [],
                "bridge raw velocity bypass reintroduced")
        driver.run_for(0.35)
        require(gate.authority.state == "LATCHED",
                "diagnostic gate automatically armed at startup")
        require(audit.twists and all(
            twist == (0.0, 0.0, 0.0) for twist in audit.twists),
            "startup diagnostic Twist not zero")
        print("M22H_SIGNED_ROS_STARTUP_INHIBIT=PASS", flush=True)

        driver.rearm()
        driver.run_for(1.5)
        require(gate.authority.state == "ARMED",
                "signed operator grant plus dwell did not arm")
        require(any(any(abs(x) > 0 for x in twist)
                    for twist in audit.twists),
                "no nonzero authorized diagnostic Twist")
        require(any(event.startswith(b"set_wheels ")
                    and event != STOP
                    for event in gate.fake_sink.writes),
                "signed gate never emitted fake-only PWM")
        require(not any(event.startswith(b"set_wheels ")
                        and event != STOP
                        for event in instances[0].snapshot()),
                "nonzero PWM escaped to actual bridge fake CDC")
        require(bridge._send_command("set_wheels 10 0 0 0") is False,
                "production bridge writer accepted nonzero PWM")
        print("M22H_SIGNED_REARM_DIAGNOSTIC_FORWARD=PASS", flush=True)
        print("M22H_PRODUCTION_CDC_LOCKOUT=PASS", flush=True)

        start = len(audit.statuses)
        driver.send_command(key=b"x" * 32)
        driver.run_for(0.30, send_commands=False)
        require(any(item["reason"] == "BLOCKED_MOTION_AUTH"
                    for item in audit.statuses[start:]),
                "forged ROS envelope did not fault")
        require(gate.authority.state == "LATCHED",
                "forged envelope did not latch")
        require(all(t == (0.0, 0.0, 0.0)
                    for t in audit.twists[-2:]),
                "forged envelope caused nonzero diagnostic output")
        driver.run_for(0.32)
        require(gate.authority.state == "LATCHED",
                "healthy proposals silently rearmed after forgery")
        print("M22H_FORGED_COMMAND_INHIBIT=PASS", flush=True)
        print("M22H_NO_AUTOMATIC_REARM=PASS", flush=True)

        prior = sign_test_motion(
            key=KEY_MOTION, session_id=session.session_id,
            sequence=driver.seq, generated_ns=time.monotonic_ns() - 10_000_000,
            expires_ns=time.monotonic_ns() + 100_000_000)
        newer_session = SessionVerifier(
            motion_key=KEY_MOTION, operator_key=KEY_OPERATOR)
        valid, reason = newer_session.verify_motion(
            prior, now_ns=time.monotonic_ns())
        require(not valid and reason == "BLOCKED_SESSION_OR_OWNER",
                "old signed packet accepted in fresh verifier epoch")
        print("M22H_SESSION_RESTART_REPLAY_REJECT=PASS", flush=True)

        fake = instances[0]
        previous = len(fake.snapshot())
        fake.fail_next_write = True
        require(bridge._send_command("set_wheels 0 0 0 0") is False,
                "fake bridge CDC failure not detected")
        require(bridge._transport_fault.is_set(),
                "terminal bridge CDC fault not latched")
        require(bridge._send_command("keepalive") is False,
                "bridge accepted keepalive after transport fault")
        require(len(fake.snapshot()) == previous,
                "CDC bytes written after terminal fault")
        print("M22H_BRIDGE_TERMINAL_CDC_INHIBIT=PASS", flush=True)

        require(all(item["diagnostic_only"] is True
                    and item["physical_device_access"] is False
                    and item["session_id_logged"] is False
                    for item in audit.statuses),
                "diagnostic provenance or secret logging invalid")
        print("M22H_ROS_REPORT_JSON=" + json.dumps({
            "milestone": "M22H",
            "evidence_type": "SYNTHETIC_SIGNED_ROS_WITH_REAL_BRIDGE",
            "diagnostic_only": True,
            "ros_diagnostic_status_receipts": len(audit.statuses),
            "ros_diagnostic_twist_receipts": len(audit.twists),
            "gate_ticks": gate.cycles,
            "production_bridge_nonzero_writes": 0,
            "fake_only_pwm_writes": len(gate.fake_sink.writes),
            "bridge_transport_terminal_latch": True,
            "production_motion_enabled": False,
            "physical_device_access": False,
        }, sort_keys=True), flush=True)
        print("M22H_ROS_SIGNED_BRIDGE_SELFTEST=PASS", flush=True)
        print("M22H_VERDICT=SIMULATED_SIGNED_ROS_DIAGNOSTIC_ONLY", flush=True)
        print("M22H_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        if executor is not None:
            for n in nodes:
                executor.remove_node(n)
            executor.shutdown(timeout_sec=1)
        for n in reversed(nodes):
            n.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
        bridge_module.serial.Serial = true_serial


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M22H_ROS_SIGNED_BRIDGE_SELFTEST=FAIL", flush=True)
        print("M22H_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
