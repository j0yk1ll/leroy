"""No-device regression tests for live costmap graph discovery and inhibition."""
import unittest
from types import SimpleNamespace

from leroy_nav2_perception.live_costmap_probe import LiveCostmapProbe


RAW = "/leroy_nav2/diagnostics/raw_depth_points"
GRID = "/costmap"


def source(name, typ, namespace="/"):
    return SimpleNamespace(node_namespace=namespace,
                           node_name=name, topic_type=typ)


class FakeGraph:
    def __init__(self):
        self.types = {}
        self.nodes = []
        self.endpoints = {}

    def get_topic_names_and_types(self):
        return list(self.types.items())

    def get_node_names_and_namespaces(self):
        return list(self.nodes)

    def get_publishers_info_by_topic(self, topic):
        return list(self.endpoints.get(topic, []))

    def status(self):
        return LiveCostmapProbe.graph_status(self)

    def ready_sources(self):
        self.endpoints = {
            RAW: [source("depth_obstacles", "sensor_msgs/msg/PointCloud2")],
            GRID: [source("costmap", "nav_msgs/msg/OccupancyGrid")],
        }


class GraphTest(unittest.TestCase):
    def test_empty_initial_graph_waits_for_dds_discovery(self):
        g = FakeGraph()
        state, details = g.status()
        self.assertEqual(state, "WAITING")
        self.assertIn(RAW, details)
        self.assertIn(GRID, details)

    def test_partial_graph_waits_then_transitions_to_ready(self):
        g = FakeGraph()
        g.endpoints[RAW] = [
            source("depth_obstacles", "sensor_msgs/msg/PointCloud2")]
        self.assertEqual(g.status()[0], "WAITING")
        g.ready_sources()
        self.assertEqual(g.status(), ("READY", "required_publishers_verified"))

    def test_anonymous_dds_publisher_is_unverified_never_ready(self):
        g = FakeGraph()
        g.ready_sources()
        g.endpoints[RAW] = [source(
            "_NODE_NAME_UNKNOWN_", "sensor_msgs/msg/PointCloud2",
            "_NODE_NAMESPACE_UNKNOWN_")]
        state, details = g.status()
        self.assertEqual(state, "WAITING")
        self.assertIn("publisher_identity_unverified", details)
        g.ready_sources()
        self.assertEqual(g.status()[0], "READY")

    def test_anonymous_source_does_not_mask_identifiable_rogue(self):
        g = FakeGraph()
        g.ready_sources()
        g.endpoints[RAW] = [source(
            "_NODE_NAME_UNKNOWN_", "sensor_msgs/msg/PointCloud2",
            "_NODE_NAMESPACE_UNKNOWN_")]
        g.endpoints[GRID] = [source(
            "unexpected_grid_owner", "nav_msgs/msg/OccupancyGrid")]
        self.assertEqual(g.status()[0], "UNSAFE")

    def test_unnamed_duplicate_or_wrong_type_fails_immediately(self):
        g = FakeGraph()
        g.ready_sources()
        g.endpoints[RAW].append(source(
            "_NODE_NAME_UNKNOWN_", "sensor_msgs/msg/PointCloud2",
            "_NODE_NAMESPACE_UNKNOWN_"))
        self.assertEqual(g.status()[0], "UNSAFE")
        g.ready_sources()
        g.endpoints[RAW] = [source(
            "_NODE_NAME_UNKNOWN_", "sensor_msgs/msg/Image",
            "_NODE_NAMESPACE_UNKNOWN_")]
        self.assertEqual(g.status()[0], "UNSAFE")

    def test_motor_topics_fail_immediately_even_during_discovery(self):
        g = FakeGraph()
        g.types["/cmd_vel"] = ["geometry_msgs/msg/Twist"]
        self.assertEqual(g.status()[0], "UNSAFE")

    def test_motor_bridge_node_fails_even_with_missing_publishers(self):
        g = FakeGraph()
        g.nodes = [("leroy_bridge_node", "/")]
        self.assertIn("forbidden_nodes", g.status()[1])
        self.assertEqual(g.status()[0], "UNSAFE")

    def test_unexpected_duplicated_or_wrong_typed_sources_fail(self):
        g = FakeGraph()
        g.ready_sources()
        g.endpoints[GRID].append(source("costmap", "nav_msgs/msg/OccupancyGrid"))
        self.assertEqual(g.status()[0], "UNSAFE")
        g.ready_sources()
        g.endpoints[GRID] = [source(
            "costmap", "nav_msgs/msg/OccupancyGrid", "/unexpected")]
        self.assertEqual(g.status()[0], "UNSAFE")
        g.ready_sources()
        g.endpoints[RAW] = [source(
            "depth_obstacles", "sensor_msgs/msg/Image")]
        self.assertEqual(g.status()[0], "UNSAFE")


if __name__ == "__main__":
    unittest.main()
