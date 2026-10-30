"""No-device tests for isolated Stage 4C ROS graph policy."""
from types import SimpleNamespace
import unittest

from live_stage4c_capture_contract import CaptureError, TOPICS
from live_stage4c_graph_contract import REQUIRED_NODES, graph_ready


def publishers():
    return {
        topic: [SimpleNamespace(
            topic_type=typ,
            node_name="m18_imu_source" if topic.startswith(("/imu/", "/stage4c/")) else "camera",
            node_namespace="/")]
        for topic, (typ, _) in TOPICS.items()
    }


class GraphPolicyTest(unittest.TestCase):
    def check(self, *, nodes=None, topics=None, endpoints=None,
              valid=False, map_valid=False):
        return graph_ready(
            REQUIRED_NODES if nodes is None else nodes,
            set(TOPICS) if topics is None else topics,
            publishers() if endpoints is None else endpoints,
            valid, map_valid)

    def test_healthy_graph_never_implies_calibration(self):
        self.assertTrue(self.check())
        self.assertFalse(self.check(valid=None))
        self.assertFalse(self.check(map_valid=True))

    def test_missing_nodes_or_publishers_return_not_ready(self):
        self.assertFalse(self.check(nodes=REQUIRED_NODES - {"m18_imu_source"}))
        p = publishers()
        p["/imu/data_raw"] = []
        self.assertFalse(self.check(endpoints=p))
        p["/imu/data_raw"] = publishers()["/imu/data_raw"] * 2
        self.assertFalse(self.check(endpoints=p))

    def test_forbidden_nodes_and_topics_fail_closed(self):
        with self.assertRaisesRegex(CaptureError, "forbidden_motor_or_navigation_graph"):
            self.check(nodes=REQUIRED_NODES | {"controller_server"})
        with self.assertRaisesRegex(CaptureError, "forbidden_motor_or_navigation_graph"):
            self.check(topics=set(TOPICS) | {"/cmd_vel"})

    def test_wrong_types_or_mcu_identity_fail_closed(self):
        p = publishers()
        p["/imu/data_raw"][0].topic_type = "std_msgs/msg/String"
        with self.assertRaisesRegex(CaptureError, "unexpected_publisher_/imu/data_raw"):
            self.check(endpoints=p)
        p = publishers()
        p["/stage4c/imu_source_meta"][0].node_namespace = "/other"
        with self.assertRaisesRegex(CaptureError, "unexpected_publisher_/stage4c/imu_source_meta"):
            self.check(endpoints=p)


if __name__ == "__main__":
    unittest.main()
