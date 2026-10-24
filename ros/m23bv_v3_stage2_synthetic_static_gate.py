#!/usr/bin/env python3
"""No-device Stage 2A synthetic composition source-contract tests (stdlib)."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class Stage2ASyntheticSource(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.probe = (ROOT / "m23bv_v3_stage2_synthetic_probe.py").read_text()
        cls.runner = (ROOT / "m23bv_v3_stage2_synthetic_preflight.sh").read_text()
        cls.tree = ast.parse(cls.probe)
        assigns = {
            name.id: node.value
            for node in cls.tree.body if isinstance(node, ast.Assign)
            for name in node.targets if isinstance(name, ast.Name)
        }
        cls.fake_camera = ast.literal_eval(assigns["FAKE_CAMERA"])
        cls.fake_imu = ast.literal_eval(assigns["FAKE_IMU"])
        ast.parse(cls.fake_camera)
        ast.parse(cls.fake_imu)

    def test_no_physical_devices_or_recording(self):
        self.assertNotIn("--device", self.runner)
        self.assertNotIn("--mount", self.runner)
        self.assertIn("--network none", self.runner)
        self.assertIn("--read-only", self.runner)
        self.assertIn("--cap-drop all", self.runner)
        self.assertIn("m23bv_reliable_rectifier_proto/ws/install/setup.bash",
                      self.runner)
        for source in (self.probe, self.fake_camera, self.fake_imu, self.runner):
            self.assertNotIn("ros2 bag record", source)
            self.assertNotIn("serial.Serial(", source)
            self.assertNotIn("m20_combined_observer.py", source)
            self.assertNotIn("/cmd_vel", source)
            self.assertNotIn("leroy-m20-sensor-run", source)

    def test_pinned_v3_candidate_exact_ast(self):
        self.assertIn("a0ae75a62da39235b113e3d2c324efda5fcfc6e1",
                      self.probe)
        self.assertIn("7e5d199e633f7cf0dac266a656ae70849df07f71",
                      self.probe)
        self.assertIn("777db7cbeb0ce6c3669a837f1530f7b1499dd9dd",
                      self.probe)
        self.assertIn("set(funcs) == {\"ready_exited\", \"camera_exited\"}",
                      self.probe)
        self.assertIn('"target_action=ready" in original', self.probe)
        self.assertIn('"target_action=camera" in original', self.probe)
        self.assertIn('"return [rectifier]" in ast.unparse', self.probe)

    def test_exact_pinned_image_and_isolated_run(self):
        self.assertIn("f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c", self.runner)
        self.assertIn("podman image inspect", self.runner)
        self.assertIn("podman ps --format", self.runner)
        self.assertIn("timeout --foreground", self.runner)
        self.assertIn("60s", self.runner)
        self.assertIn('exec python3 -u -', self.runner)
        self.assertIn("sha256sum \"$log\"", self.runner)

    def test_actual_graph_endpoint_qos(self):
        for expected in (
            "get_publishers_info_by_topic", "get_subscriptions_info_by_topic",
            "ReliabilityPolicy.RELIABLE", "ReliabilityPolicy.BEST_EFFORT",
            "DurabilityPolicy.VOLATILE", "get_topic_names_and_types()",
            "get_node_names_and_namespaces()", "UNKNOWN_TOPICS", "UNKNOWN_NODES",
            '"BLOCKED_IMU_OFFERED_QOS"', '"PASS"',
        ):
            self.assertIn(expected, self.probe)
        self.assertNotIn("startswith(", self.probe.split("def evaluate_graph(node):")[1].split("def stamp(message):")[0])

    def test_fake_camera_is_not_physical(self):
        self.assertIn('create_node("usb_cam", namespace="/camera"', self.fake_camera)
        self.assertIn('create_publisher(Image, "/camera/image_raw"', self.fake_camera)
        self.assertIn('create_publisher(CameraInfo, "/camera/camera_info"', self.fake_camera)
        self.assertIn("time.time_ns()", self.fake_camera)
        self.assertIn("msg.width, msg.height = 16, 16", self.fake_camera)
        self.assertNotIn("usb_cam_node_exe", self.fake_camera)

    def test_synthetic_imu_has_negative_cases_and_no_mcu(self):
        self.assertIn('case == "imu_qos_reliable"', self.fake_imu)
        self.assertIn('case == "missing_imu"', self.fake_imu)
        self.assertIn('create_publisher(Imu, "/imu/data_raw"', self.fake_imu)
        self.assertIn('time.sleep(1 / 50)', self.fake_imu)
        self.assertNotIn("serial", self.fake_imu.lower())
        self.assertNotIn("START", self.fake_imu)
        self.assertNotIn("STOP_ACK", self.fake_imu)

    def test_health_is_distinct_from_negative_results(self):
        self.assertIn('CASE == "healthy"', self.probe)
        self.assertIn("len(joint) >= 100 and len(imus) >= 300", self.probe)
        self.assertIn("t1 - t0 >= 3.0", self.probe)
        self.assertIn("NEGATIVE_IMU_QOS_NOT_REJECTED", self.probe)
        self.assertIn("NEGATIVE_MISSING_IMU_NOT_REJECTED", self.probe)
        self.assertIn("HEALTHY_NO_QOS_FRAME_OR_OVERLAP", self.probe)
        self.assertIn("CAMERA_AND_IMU=SYNTHETIC_ONLY", self.probe)

    def test_two_additional_negative_cases_are_fail_closed(self):
        self.assertIn('"bad_imu_stamp"', self.probe)
        self.assertIn('"unexpected_topic"', self.probe)
        self.assertIn("bad_imu_stamp|unexpected_topic", self.runner)
        self.assertIn('if case == "bad_imu_stamp":', self.fake_imu)
        self.assertIn("msg.header.stamp.sec = 0", self.fake_imu)
        self.assertIn('"/stage2a/unexpected_imu"', self.fake_imu)
        self.assertNotIn('"/stage2a/unexpected_imu"', self.probe.split(
            "ALLOWED_TOPICS = {")[1].split("ALLOWED_NODES = {")[0])
        self.assertIn('blocked_unknown_topic = True', self.probe)
        self.assertIn('blocked_bad_imu_stamp = True', self.probe)
        self.assertIn("str(exc) == \"BLOCKED_UNKNOWN_TOPICS:['/stage2a/unexpected_imu']\"",
                      self.probe)
        self.assertIn('NEGATIVE_BAD_IMU_STAMP_NOT_REJECTED', self.probe)
        self.assertIn('NEGATIVE_UNEXPECTED_TOPIC_NOT_REJECTED', self.probe)
        self.assertIn('M23BV_V3_STAGE2A_NEGATIVE_BAD_IMU_STAMP=PASS', self.probe)
        self.assertIn('M23BV_V3_STAGE2A_NEGATIVE_UNEXPECTED_TOPIC=PASS', self.probe)

    def test_child_shutdown_must_be_clean(self):
        self.assertIn("rc == 0 and not forced", self.probe)
        self.assertIn('"process has died" not in output', self.probe)
        self.assertIn("len(child_started) == 4 and child_clean == child_started",
                      self.probe)
        self.assertIn('Path("/proc/" + pid).exists()', self.probe)
        self.assertIn("FAKE_CAMERA_STOP=PASS", self.probe)
        self.assertIn("FAKE_IMU_STOP=PASS", self.probe)

    def test_no_banned_import_roots(self):
        sources = (self.probe, self.fake_camera, self.fake_imu)
        banned = {"serial", "rosbag2_py", "cv2", "socket", "motor", "xrslam"}
        for source in sources:
            tree = ast.parse(source)
            roots = {
                alias.name.split(".")[0]
                for node in ast.walk(tree) if isinstance(node, ast.Import)
                for alias in node.names
            } | {
                node.module.split(".")[0]
                for node in ast.walk(tree)
                if isinstance(node, ast.ImportFrom) and node.module
            }
            self.assertFalse(roots & banned, roots & banned)


if __name__ == "__main__":
    unittest.main(verbosity=2)
