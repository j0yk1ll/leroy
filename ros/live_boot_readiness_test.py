"""Minimal no-device guard checks for the real ROS boot observation launcher."""
import ast
import json
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

    def test_live_supervisor_consumes_relative_imu_yaw(self):
        source = (ROOT / "live_boot_readiness.py").read_text()
        self.assertIn('YAW = "/leroy_boot/relative_yaw_status"', source)
        self.assertIn("String, YAW, self.on_yaw", source)
        self.assertIn("yaw_at=node.boot_yaw_at", source)
        self.assertIn("yaw_rad=node.boot_yaw", source)
        self.assertIn("boot_relative_yaw_samples", source)
        self.assertIn("d.get(\"motor_motion_authorized\") is not False", source)
        self.assertIn("now - node.boot_yaw_at > 0.35", source)

    def test_auto_calibration_candidate_is_real_fresh_and_not_a_grant(self):
        t = 20_000_000_000
        status = {
            "schema": "leroy_auto_imu_calibration_v1",
            "source": "live_ros_m18_imu",
            "state": "VERIFIED_STATIONARY_CANDIDATE",
            "owner_verified": True,
            "motor_motion_authorized": False,
            "physical_motion_authorized": False,
            "physical_heading_accuracy_validated": False,
            "absolute_heading_validated": False,
            "camera_imu_extrinsics_validated": False,
            "samples": 401,
            "window_elapsed_s": 8.0,
            "independent_bias_rad_s": -0.0015,
            "heading_bias_rad_s": -0.0015,
            "bias_disagreement_rad_s": 0.0,
            "split_bias_delta_rad_s": 0.0,
            "gyro_z_noise_std_rad_s": 0.001,
            "yaw_drift_deg": 0.02,
            "monotonic_status_ns": t - 100_000_000,
        }
        result = live.parse_auto_imu_candidate(json.dumps(status), t)
        self.assertEqual(result["samples"], 401)
        self.assertEqual(result["yaw_drift_deg"], 0.02)
        for key, invalid in (
                ("source", "untrusted_imu"),
                ("state", "COLLECTING"),
                ("owner_verified", False),
                ("motor_motion_authorized", True),
                ("physical_motion_authorized", True),
                ("absolute_heading_validated", True),
                ("samples", 1),
                ("samples", "401"),
                ("window_elapsed_s", float("nan")),
                ("monotonic_status_ns", t - 500_000_000),
                ("gyro_z_noise_std_rad_s", float("nan"))):
            with self.subTest(key=key, invalid=invalid):
                modified = dict(status)
                modified[key] = invalid
                self.assertIsNone(
                    live.parse_auto_imu_candidate(
                        json.dumps(modified), t))
        self.assertIsNone(live.parse_auto_imu_candidate(
            "malformed", t))

    def test_boot_readiness_observes_auto_calibration_without_arm(self):
        source = (ROOT / "live_boot_readiness.py").read_text()
        self.assertIn(
            'AUTO_IMU = "/leroy_boot/auto_imu_calibration_status"',
            source)
        self.assertIn("String, AUTO_IMU, self.on_auto_calibration", source)
        self.assertIn('"auto_imu_stationary_candidate_observed"', source)
        self.assertIn("source_calibration_validated\": False", source)
        self.assertIn("LIVE_BOOT_MOTION=BLOCKED_UNCOMMISSIONED", source)

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
