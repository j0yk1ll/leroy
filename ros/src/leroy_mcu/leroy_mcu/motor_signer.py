"""Signed Nav2 command producer and explicit local operator grant.

These processes hold separate protected key files. ROS graph checks are
conservative conflict checks, NOT cryptographic publisher identity.
Hardware release and sensor readiness remain bridge-owned.
"""
import argparse
from dataclasses import asdict
import json
import math
import re
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from leroy_mcu.authorized_session import (
    sign_motion, sign_rearm,
)
from leroy_mcu.motor_signed_ingress import read_protected_key

_SESSION_RE = re.compile(r"[a-f0-9]{32}\Z")


def _wire(envelope):
    return String(data=json.dumps(asdict(envelope), separators=(",", ":"),
                                  allow_nan=False))


class SignedCommandProducer(Node):
    def __init__(self):
        super().__init__("leroy_motor_controller_signer")
        self.declare_parameter("motion_key_file", "")
        path = self.get_parameter("motion_key_file").value
        self._key = read_protected_key(path)
        self._session = None
        self._sequence = 0
        self._last_input_ns = 0
        self._pub = self.create_publisher(
            String, "/leroy/motor/signed_motion", 10)
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._session_sub = self.create_subscription(
            String, "/leroy/motor/session_id", self._on_session, qos)
        self._cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, 10)

    def _on_session(self, msg):
        if _SESSION_RE.fullmatch(msg.data or "") is None:
            self._session = None
            return
        if msg.data != self._session:
            self._session = msg.data
            self._sequence = 0
            self._last_input_ns = 0

    def _on_cmd_vel(self, msg):
        # Exactly one ROS publisher is a conflict check, not authority
        # authentication; bridge must independently verify controller.
        if (self._session is None or
                self._cmd_sub.get_publisher_count() != 1):
            return
        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        discarded = (msg.linear.z, msg.angular.x, msg.angular.y)
        if (not all(math.isfinite(v) for v in cmd + discarded)
                or any(abs(v) > 1e-9 for v in discarded)
                or abs(cmd[0]) > 0.15
                or abs(cmd[1]) > 0.15 or abs(cmd[2]) > 0.30):
            self.get_logger().error("Unsafe Nav2 motor command rejected")
            return
        now = time.monotonic_ns()
        if now <= self._last_input_ns:
            return
        self._sequence += 1
        packet = sign_motion(
            key=self._key, session_id=self._session,
            sequence=self._sequence, generated_ns=now,
            expires_ns=now + 100_000_000,
            vx=float(cmd[0]), vy=float(cmd[1]), wz=float(cmd[2]))
        self._pub.publish(_wire(packet))
        self._last_input_ns = now


class LocalOperatorGrant(Node):
    def __init__(self, key_file):
        super().__init__("leroy_motor_operator_grant")
        self._key = read_protected_key(key_file)
        self.session = None
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self._sub = self.create_subscription(
            String, "/leroy/motor/session_id", self._receive, qos)
        self.pub = self.create_publisher(
            String, "/leroy/motor/signed_rearm", 10)

    def _receive(self, message):
        if _SESSION_RE.fullmatch(message.data or ""):
            self.session = message.data


def main_controller(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SignedCommandProducer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_operator(args=None):
    parser = argparse.ArgumentParser(
        description="One deliberate local motor rearm request")
    parser.add_argument("--operator-key-file", required=True)
    opts = parser.parse_args(args)
    # Require an actual local console; never accept automated stdin input
    # as evidence of the present operator.
    try:
        with open("/dev/tty", "r", encoding="ascii") as tty_in, \
             open("/dev/tty", "w", encoding="ascii") as tty_out:
            tty_out.write(
                "Confirm independent motor cutoff and clear wheel paths.\n"
                "Type ARM_MOTORS to request this bridge session's grant: "
            )
            tty_out.flush()
            if tty_in.readline().strip() != "ARM_MOTORS":
                raise SystemExit("Operator declined motor arming")
    except OSError as exc:
        raise SystemExit(
            f"Interactive operator terminal unavailable: {exc}"
        ) from exc
    # The explicit --operator-key-file argument is handled by argparse,
    # not by the ROS argument parser.
    rclpy.init(args=[])
    node = None
    try:
        node = LocalOperatorGrant(opts.operator_key_file)
        end = time.monotonic() + 10.0
        while rclpy.ok() and node.session is None and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.session is None or node.pub.get_subscription_count() != 1:
            raise RuntimeError("Unique active motor bridge session unavailable")
        now = time.monotonic_ns()
        grant = sign_rearm(
            key=node._key, session_id=node.session,
            counter=1, generated_ns=now, expires_ns=now + 100_000_000)
        node.pub.publish(_wire(grant))
        rclpy.spin_once(node, timeout_sec=0.05)
        print("Motor arming REQUEST sent; not proof that MCU is armed")
    finally:
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
