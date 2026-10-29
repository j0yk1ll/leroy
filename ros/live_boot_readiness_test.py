"""Minimal no-device guard checks for the real ROS boot observation launcher."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import live_boot_readiness as live

ROOT = Path(__file__).resolve().parent


class LiveBootReadOnlyContract(unittest.TestCase):
    def environment(self):
        return {
            "ROS_DOMAIN_ID": "170",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
            "LEROY_LIVE_BOOT_READ_ONLY": "1",
        }

    def test_isolation_environment(self):
        env = self.environment()
        self.assertTrue(live.preflight(env, lambda path: False))
        for key in env:
            with self.subTest(key=key):
                bad = dict(env)
                bad.pop(key)
                with self.assertRaisesRegex(
                    RuntimeError, "BOOT_LIVE_ENV_INVALID_"):
                    live.preflight(bad, lambda path: False)

    def test_denies_physical_device_paths(self):
        for path in live.DEVICE_PATHS:
            with self.subTest(path=path):
                with self.assertRaisesRegex(
                    RuntimeError, "BOOT_LIVE_DEVICE_VISIBLE"):
                    live.preflight(
                        self.environment(), lambda p: p == path)

    def test_exact_real_stage3_provenance(self):
        class Graph:
            def get_publishers_info_by_topic(self, topic):
                name, namespace = live.EXPECTED[topic]
                return [SimpleNamespace(
                    node_name=name, node_namespace=namespace)]

        self.assertTrue(all(live.graph_report(Graph()).values()))

        class WrongGraph(Graph):
            def get_publishers_info_by_topic(self, topic):
                if topic == live.IMAGE:
                    return []
                return super().get_publishers_info_by_topic(topic)

        self.assertFalse(live.graph_report(WrongGraph())[live.IMAGE])

    def test_live_script_has_no_ros_publisher_or_actuation_import(self):
        source = (ROOT / "live_boot_readiness.py").read_text()
        parsed = ast.parse(source)
        attrs = [node.func.attr for node in ast.walk(parsed)
                 if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Attribute)]
        self.assertNotIn("create_publisher", attrs)
        self.assertNotIn("send_goal_async", attrs)
        self.assertNotIn("Serial", attrs)
        self.assertIn("model.tick(Observation(", source)
        self.assertIn("physical_motion_authorized", source)
        self.assertIn("LIVE_BOOT_MOTION=BLOCKED_UNCOMMISSIONED", source)

    def test_does_not_override_rclpy_node_subscriptions_property(self):
        source = (ROOT / "live_boot_readiness.py").read_text()
        self.assertIn("self._boot_subscriptions = [", source)
        self.assertIn("for sub in self._boot_subscriptions", source)
        tree = ast.parse(source)
        for item in ast.walk(tree):
            if not isinstance(item, (ast.Assign, ast.AnnAssign)):
                continue
            targets = (item.targets if isinstance(item, ast.Assign)
                       else [item.target])
            for target in targets:
                self.assertFalse(
                    isinstance(target, ast.Attribute)
                    and isinstance(target.value, ast.Name)
                    and target.value.id == "self"
                    and target.attr == "subscriptions",
                    "rclpy.Node.subscriptions is a read-only property")

    def test_launcher_maps_only_source_files_not_hardware(self):
        source = (ROOT / "live_boot_readiness.sh").read_text()
        self.assertIn('parent=leroy-live-costmap', source)
        self.assertIn('--network "container:$parent"', source)
        self.assertIn('--ipc "container:$parent"', source)
        self.assertIn('--pid "container:$parent"', source)
        self.assertIn('--read-only', source)
        self.assertIn('--cap-drop=all', source)
        self.assertIn('readonly"', source)
        self.assertNotIn('--device', source)
        self.assertNotIn('--privileged', source)
        self.assertNotIn('podman stop', source)


if __name__ == "__main__":
    unittest.main()
