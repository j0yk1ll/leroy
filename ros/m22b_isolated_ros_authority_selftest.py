#!/usr/bin/env python3
"""M22B: isolated ROS 2 command authority with SYNTHETIC fake publishers.

Only DIAGNOSTIC topics. No /cmd_vel publisher, bridge, MCU, camera,
serial, physical devices, or Nav2. No operator-rearm ROS endpoint:
the test harness triggers rearm only through a direct test-only method.

ROS graph node names are advisory and are NOT authentication. The JSON
proposal generation clock is a single-process fake sender, NOT a secure
cross-process timestamp or deployable authenticated command protocol.
"""
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from m22a_diagnostic_authority import (
    DiagnosticAuthority, Inputs, Proposal, ZERO, EXPECTED_OWNER,
)

PROPOSAL_TOPIC = "/leroy_nav2/diagnostics/m22b/proposed_envelope"
HEALTH_TOPIC = "/leroy_nav2/diagnostics/m22b/synthetic_health"
OUTPUT_TOPIC = "/leroy_nav2/diagnostics/m22b/gated_cmd_vel"
STATUS_TOPIC = "/leroy_nav2/diagnostics/m22b/authority_status"

REQUIRED_ENV = {
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}
HEALTH_NAMES = (
    "odom_fresh", "depth_fresh", "tf_fresh",
    "costmap_valid", "localization_valid",
)
PROPOSAL_NAMES = (
    "owner", "sequence", "generated_mono_ns",
    "vx", "vy", "wz", "vz", "wx", "wy",
)


def check_environment():
    for name, expected in REQUIRED_ENV.items():
        if os.environ.get(name) != expected:
            raise RuntimeError("M22B requires " + name + "=" + expected)
    for path in ("/dev/ttyACM0", "/dev/video2", "/dev/video8"):
        if os.path.exists(path):
            raise RuntimeError("M22B refuses physical device " + path)


def one_source(endpoints, *, node, msg_type):
    """Fail closed when source is undiscovered, duplicated, or mismatched."""
    return (
        len(endpoints) == 1
        and getattr(endpoints[0], "node_name", None) == node
        and getattr(endpoints[0], "node_namespace", None) == "/"
        and getattr(endpoints[0], "topic_type", None) == msg_type
    )


class SyntheticAuthorityNode(Node):
    """M22A applied to incoming fake ROS messages, DIAGNOSTIC output only."""

    def __init__(self):
        super().__init__("m22b_authority_gate", enable_rosout=False,
                         start_parameter_services=False)
        self.core = DiagnosticAuthority()
        self.pending_proposal = None
        self.invalid_proposal_seen = False
        self.inputs = None
        self.last_health_ns = None
        self.invalid_health_seen = False
        self.pending_rearm = False
        self.proposal_subscription = self.create_subscription(
            String, PROPOSAL_TOPIC, self.on_proposal, 10)
        self.health_subscription = self.create_subscription(
            String, HEALTH_TOPIC, self.on_health, 10)
        self.command_output = self.create_publisher(Twist, OUTPUT_TOPIC, 10)
        self.status_output = self.create_publisher(String, STATUS_TOPIC, 10)
        endpoints = (
            (self.proposal_subscription.topic_name, PROPOSAL_TOPIC),
            (self.health_subscription.topic_name, HEALTH_TOPIC),
            (self.command_output.topic_name, OUTPUT_TOPIC),
            (self.status_output.topic_name, STATUS_TOPIC),
        )
        if any(actual != expected for actual, expected in endpoints):
            raise RuntimeError("M22B denies ROS topic remapping")
        self.last_decision = None
        self.tick_count = 0
        self.create_timer(0.1, self.tick)

    def on_proposal(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or set(data) != set(PROPOSAL_NAMES):
                raise ValueError("unexpected proposal envelope shape")
            self.pending_proposal = Proposal(**data)
        except (TypeError, ValueError, KeyError):
            self.pending_proposal = None
            self.invalid_proposal_seen = True

    def on_health(self, msg):
        self.inputs = None
        self.last_health_ns = None
        try:
            data = json.loads(msg.data)
            if not isinstance(data, dict) or set(data) != set(HEALTH_NAMES):
                self.invalid_health_seen = True
                return
            if not all(type(data[name]) is bool for name in HEALTH_NAMES):
                self.invalid_health_seen = True
                return
            self.inputs = Inputs(**data)
            self.last_health_ns = time.monotonic_ns()
        except (TypeError, ValueError, KeyError):
            self.invalid_health_seen = True
            return

    def request_test_only_rearm(self):
        # Not a ROS subscriber, service, parameter or operator mechanism.
        self.pending_rearm = True

    def graph_ok(self):
        if self.count_publishers("/cmd_vel") != 0:
            return False
        if any(name == "leroy_bridge"
               for name, _namespace in self.get_node_names_and_namespaces()):
            return False
        return (
            one_source(self.get_publishers_info_by_topic(PROPOSAL_TOPIC),
                       node=EXPECTED_OWNER, msg_type="std_msgs/msg/String")
            and one_source(self.get_publishers_info_by_topic(HEALTH_TOPIC),
                           node="m22b_fake_health",
                           msg_type="std_msgs/msg/String")
        )

    def tick(self):
        now = time.monotonic_ns()
        graph_ok = self.graph_ok()
        fresh_health = (
            self.inputs is not None
            and self.last_health_ns is not None
            and 0 <= now - self.last_health_ns <= 250_000_000
        )
        health = (self.inputs if fresh_health and not self.invalid_health_seen
                  else Inputs(False, False, False, False, False))
        proposal = (None if self.invalid_proposal_seen
                    else self.pending_proposal)
        self.pending_proposal = None  # Never reuse stale ROS callback state.
        self.invalid_proposal_seen = False
        self.invalid_health_seen = False
        rearm = self.pending_rearm
        self.pending_rearm = False
        result = self.core.evaluate(
            now_ns=now, inputs=health, proposal=proposal,
            publishers=(EXPECTED_OWNER,) if graph_ok else (),
            operator_rearm=rearm,
        )
        command = Twist()
        command.linear.x, command.linear.y, command.angular.z = result.output
        self.command_output.publish(command)
        state = String()
        state.data = json.dumps({
            "evidence_type": "SYNTHETIC", "diagnostic_only": True,
            "reason": result.reason, "state": result.state,
            "output_xyz": list(result.output),
            "physical_device_access": False,
        }, sort_keys=True)
        self.status_output.publish(state)
        self.last_decision = result
        self.tick_count += 1


class Audit(Node):
    def __init__(self):
        super().__init__("m22b_audit", enable_rosout=False,
                         start_parameter_services=False)
        self.outputs = []
        self.statuses = []
        self.create_subscription(
            Twist, OUTPUT_TOPIC, self.on_command, 10)
        self.create_subscription(
            String, STATUS_TOPIC, self.on_status, 10)

    def on_command(self, msg):
        self.outputs.append((
            msg.linear.x, msg.linear.y, msg.angular.z))
        if len(self.outputs) > 150:
            raise RuntimeError("bounded output audit exceeded")

    def on_status(self, msg):
        self.statuses.append(json.loads(msg.data))
        if len(self.statuses) > 150:
            raise RuntimeError("bounded status audit exceeded")


def payload(data):
    msg = String()
    msg.data = json.dumps(data, sort_keys=True)
    return msg


class TestDriver:
    def __init__(self, executor, proposal_publisher, health_publisher):
        self.executor = executor
        self.proposal_publisher = proposal_publisher
        self.health_publisher = health_publisher
        self.sequence = 0

    def propose_once(self, *, age_ns=10_000_000, values=None):
        self.sequence += 1
        data = {
            "owner": EXPECTED_OWNER, "sequence": self.sequence,
            "generated_mono_ns": time.monotonic_ns() - age_ns,
            "vx": 0.04, "vy": 0.0, "wz": 0.07,
            "vz": 0.0, "wx": 0.0, "wy": 0.0,
        }
        if values:
            data.update(values)
        self.proposal_publisher.publish(payload(data))

    def run_for(self, duration_s, *, send_proposals=True, send_health=True):
        end = time.monotonic() + duration_s
        next_pub = 0.0
        while time.monotonic() < end:
            if time.monotonic() >= next_pub:
                if send_health:
                    self.health_publisher.publish(
                        payload({name: True for name in HEALTH_NAMES}))
                if send_proposals:
                    self.propose_once()
                next_pub = time.monotonic() + 0.04
            self.executor.spin_once(timeout_sec=0.005)


def require(value, what):
    if not value:
        raise AssertionError("M22B " + what)


def run():
    check_environment()
    rclpy.init(args=[])
    nodes = []
    executor = SingleThreadedExecutor()
    rogue = None
    try:
        gate = SyntheticAuthorityNode()
        source = Node(EXPECTED_OWNER, enable_rosout=False,
                      start_parameter_services=False)
        health = Node("m22b_fake_health", enable_rosout=False,
                      start_parameter_services=False)
        audit = Audit()
        nodes.extend((gate, source, health, audit))
        for node in nodes:
            executor.add_node(node)
        proposer = source.create_publisher(String, PROPOSAL_TOPIC, 10)
        health_pub = health.create_publisher(String, HEALTH_TOPIC, 10)
        driver = TestDriver(executor, proposer, health_pub)

        # DDS discovery is asynchronous. Wait for the isolated two
        # expected publishers before asserting the authority contract.
        for _ in range(20):
            if gate.graph_ok():
                break
            driver.run_for(0.10)
        require(gate.graph_ok(), "expected synthetic sources undiscovered")

        # Startup must withhold commands without explicit test rearm.
        driver.run_for(0.42)
        require(len(audit.outputs) >= 2, "no ROS Twist diagnostics received")
        require(all(output == ZERO for output in audit.outputs),
                "startup emitted nonzero diagnostic command")
        require(any(item["reason"] == "BLOCKED_MANUAL_REARM_REQUIRED"
                    for item in audit.statuses),
                "startup did not remain latched")
        print("M22B_STARTUP_INHIBIT=PASS", flush=True)

        # Test-harness-only explicit rearm; generation and receipt clocks
        # are deliberately the same host monotonic source.
        gate.request_test_only_rearm()
        driver.run_for(1.52)
        require(gate.core.state == "ARMED", "healthy dwell did not arm")
        require(any(any(abs(value) > 0 for value in output)
                    for output in audit.outputs),
                "no real diagnostic ROS nonzero message after rearm")
        require(any(item["reason"] == "FORWARD_SYNTHETIC_DIAGNOSTIC_ONLY"
                    for item in audit.statuses),
                "no valid ROS diagnostic forward status")
        print("M22B_REARM_AND_DIAGNOSTIC_FORWARD=PASS", flush=True)

        # A second publisher must cause a latched fault and exact ZERO.
        rogue = Node("m22b_rogue_producer", enable_rosout=False,
                     start_parameter_services=False)
        executor.add_node(rogue)
        rogue.create_publisher(String, PROPOSAL_TOPIC, 10)
        for _ in range(20):
            if len(gate.get_publishers_info_by_topic(PROPOSAL_TOPIC)) >= 2:
                break
            driver.run_for(0.10)
        require(len(gate.get_publishers_info_by_topic(PROPOSAL_TOPIC)) == 2,
                "competing ROS publisher not discovered")
        start_status = len(audit.statuses)
        driver.run_for(0.45)
        require(any(item["reason"] == "BLOCKED_COMMAND_OWNERSHIP"
                    for item in audit.statuses[start_status:]),
                "duplicate ROS publisher was accepted")
        require(all(output == ZERO for output in audit.outputs[-2:]),
                "duplicate publisher did not suppress ROS Twist output")
        print("M22B_DUPLICATE_PUBLISHER_INHIBIT=PASS", flush=True)

        executor.remove_node(rogue)
        rogue.destroy_node()
        rogue = None
        for _ in range(20):
            if gate.graph_ok():
                break
            driver.run_for(0.10)
        require(gate.graph_ok(), "ROS publisher removal was not discovered")
        start_status = len(audit.statuses)
        driver.run_for(0.40)
        require(any(item["reason"] == "BLOCKED_MANUAL_REARM_REQUIRED"
                    for item in audit.statuses[start_status:]),
                "publisher recovery silently rearmed")
        require(all(output == ZERO for output in audit.outputs[-2:]),
                "post-fault output not held at zero")
        print("M22B_NO_AUTOMATIC_REARM=PASS", flush=True)

        # Re-arm in isolation to then validate a stale generation stamp.
        gate.request_test_only_rearm()
        driver.run_for(1.52)
        require(gate.core.state == "ARMED", "second dwell did not arm")

        # A malformed JSON message cannot be hidden by a later valid
        # proposal callback before the next 10Hz safety evaluation.
        malformed = String()
        malformed.data = "{not-valid-json"
        proposer.publish(malformed)
        start_status = len(audit.statuses)
        driver.run_for(0.35)  # deliberately send subsequent valid envelopes
        require(any(item["reason"] == "BLOCKED_COMMAND_FORMAT"
                    for item in audit.statuses[start_status:]),
                "malformed ROS envelope did not latch")
        require(all(output == ZERO for output in audit.outputs[-2:]),
                "malformed envelope did not inhibit diagnostic output")
        print("M22B_MALFORMED_MESSAGE_INHIBIT=PASS", flush=True)

        # Recover deliberately once more to verify stale generation age.
        gate.request_test_only_rearm()
        driver.run_for(1.52)
        require(gate.core.state == "ARMED", "third dwell did not arm")
        driver.propose_once(age_ns=500_000_000)
        start_status = len(audit.statuses)
        driver.run_for(0.35, send_proposals=False)
        require(any(item["reason"] == "BLOCKED_STALE_OR_FUTURE_COMMAND"
                    for item in audit.statuses[start_status:]),
                "stale ROS command did not latch")
        require(gate.core.state == "LATCHED", "stale packet did not latch")
        require(all(output == ZERO for output in audit.outputs[-2:]),
                "stale command did not suppress ROS output")
        print("M22B_STALE_COMMAND_INHIBIT=PASS", flush=True)

        # A lost producer must not replay an old successful command.
        start_status = len(audit.statuses)
        driver.run_for(0.25, send_proposals=False)
        require(any(item["reason"] == "BLOCKED_COMMAND_FORMAT"
                    for item in audit.statuses[start_status:]),
                "no-message interval did not fail closed")
        require(all(output == ZERO for output in audit.outputs[-2:]),
                "absent commands reused a prior ROS Twist")
        require(gate.count_publishers("/cmd_vel") == 0,
                "unexpected physical command topic producer")
        require(one_source(gate.get_publishers_info_by_topic(OUTPUT_TOPIC),
                           node="m22b_authority_gate",
                           msg_type="geometry_msgs/msg/Twist"),
                "diagnostic output owner invalid")
        require(all(item["evidence_type"] == "SYNTHETIC"
                    and item["diagnostic_only"] is True
                    and item["physical_device_access"] is False
                    for item in audit.statuses),
                "invalid simulated status provenance")
        print("M22B_MISSING_MESSAGE_NO_REPLAY=PASS", flush=True)
        print("M22B_ROS_REPORT_JSON=" + json.dumps({
            "milestone": "M22B", "evidence_type": "SYNTHETIC",
            "diagnostic_only": True, "ros_gate_ticks": gate.tick_count,
            "ros_twist_receipts": len(audit.outputs),
            "ros_status_receipts": len(audit.statuses),
            "lifecycle_final_state": gate.core.state,
            "bridge_started": False,
            "cmd_vel_publisher_count": gate.count_publishers("/cmd_vel"),
            "physical_device_access": False,
            "verdict": "M22B_SIMULATED_ROS_COMMAND_AUTHORITY",
        }, sort_keys=True), flush=True)
        print("M22B_ROS_AUTHORITY_SELFTEST=PASS", flush=True)
        print("M22B_VERDICT=SIMULATED_ROS_DIAGNOSTIC_GATE_ONLY", flush=True)
        print("M22B_DEVICE_ACCESS=NONE", flush=True)
        return 0
    finally:
        if rogue is not None:
            executor.remove_node(rogue)
            rogue.destroy_node()
        executor.shutdown(timeout_sec=1)
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except Exception as exc:
        print("M22B_ROS_AUTHORITY_SELFTEST=FAIL", flush=True)
        print("M22B_ERROR=" + type(exc).__name__ + ": " + str(exc), flush=True)
        raise
