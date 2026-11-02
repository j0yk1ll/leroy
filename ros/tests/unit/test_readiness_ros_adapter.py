"""Fail-closed distributed health contract and ROS aggregation tests."""
import json
import sys
import types
import unittest
from unittest.mock import patch

from leroy_bringup.health_contract import decode, encode, TOPICS, OWNERS
from leroy_bringup.readiness_node import ReadinessNode


class String:
    def __init__(self):
        self.data = ""


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(json.loads(msg.data))


class Node:
    def __init__(self):
        self.publisher = Publisher()
        self.owners = {
            topic: [types.SimpleNamespace(
                node_name=OWNERS[c], node_namespace="/",
                topic_type="std_msgs/msg/String")]
            for c, topic in TOPICS.items() if c in OWNERS
        }

    def create_publisher(self, *_args):
        return self.publisher

    def create_subscription(self, *_args):
        return object()

    def create_timer(self, *_args):
        return object()

    def get_publishers_info_by_topic(self, topic):
        return self.owners.get(topic, [])


class HealthIntegrationTests(unittest.TestCase):
    def setUp(self):
        module = types.ModuleType("std_msgs")
        messages = types.ModuleType("std_msgs.msg")
        messages.String = String
        module.msg = messages
        self.patcher = patch.dict(sys.modules, {
            "std_msgs": module, "std_msgs.msg": messages})
        self.patcher.start()
        self.addCleanup(self.patcher.stop)
        self.node = Node()
        self.adapter = ReadinessNode(self.node)

    def last(self):
        self.adapter.tick()
        return self.node.publisher.messages[-1]

    def test_missing_health_never_releases(self):
        state = self.last()
        self.assertFalse(state["navigation_prerequisites_met"])
        self.assertFalse(state["motion_authorized"])
        self.assertIn("stop:missing", state["failure_reasons"])

    def test_observed_reports_are_not_independent_verification(self):
        for component in OWNERS:
            self.adapter.on_health(component, types.SimpleNamespace(
                data=encode(component, observed=True, reason="stream_observed")))
        state = self.last()
        self.assertFalse(state["navigation_prerequisites_met"])
        self.assertIn("localization:unverified", state["failure_reasons"])
        self.assertIn("mcu:unverified", state["failure_reasons"])
        self.assertIn("stop:missing", state["failure_reasons"])

    def test_claimed_verification_and_fake_publisher_are_rejected(self):
        raw = json.loads(encode("sensors", observed=True, reason="stream"))
        raw["verified"] = True
        self.assertIsNone(decode(json.dumps(raw), "sensors"))
        self.adapter.on_health("sensors", types.SimpleNamespace(data=json.dumps(raw)))
        self.assertIn("sensors:missing", self.last()["failure_reasons"])
        self.node.owners[TOPICS["sensors"]] = [
            types.SimpleNamespace(node_name="impersonator",
                                  node_namespace="/",
                                  topic_type="std_msgs/msg/String")]
        self.adapter.on_health("sensors", types.SimpleNamespace(
            data=encode("sensors", observed=True, reason="stream")))
        self.assertIn("sensors:missing", self.last()["failure_reasons"])

    def test_replay_latches_source_fault(self):
        value = encode("sensors", observed=True, reason="stream")
        self.adapter.on_health("sensors", types.SimpleNamespace(data=value))
        self.adapter.on_health("sensors", types.SimpleNamespace(data=value))
        state = self.last()
        self.assertIn("sensors:unverified", state["failure_reasons"])
        self.assertFalse(state["motion_authorized"])


if __name__ == "__main__":
    unittest.main()
