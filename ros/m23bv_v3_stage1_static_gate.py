#!/usr/bin/env python3
"""No-device Stage 1 regression gate: safety-critical source integration only."""
import ast
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parent


class Stage1SourceContract(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.sh = (ROOT / "m23bv_v3_stage1_camera_run.sh").read_text()
        cls.py = (ROOT / "m23bv_v3_stage1_camera_observer.py").read_text()
        cls.tree = ast.parse(cls.py)

    def test_no_serial_or_motion_or_recorder(self):
        for text in (self.sh, self.py):
            self.assertNotIn("/dev/ttyACM0", text)
            self.assertNotIn("m20_combined_observer.py", text)
            self.assertNotIn("m23bv_sensor_only_run.sh &", text)
            self.assertNotIn("ros2 bag record", text)
            self.assertNotIn("/cmd_vel\"", text.split("ALLOWED_TOPICS = {")[-1].split("}")[0])
        self.assertIn("--device \"$camera_dev:/dev/video2:rw\"", self.sh)
        self.assertEqual(self.sh.count("--device "), 1)
        self.assertIn('"$camera_dev" == /dev/video8', self.sh)
        self.assertIn("usb-Wed_Camera_Wed_Camera_20240420112206-video-index0", self.sh)
        self.assertIn("udevadm info --query=property --name=", self.sh)
        self.assertIn('"$udev_serial" == "$CAMERA_SERIAL"', self.sh)
        self.assertIn("CALIBRATION VERIFIED FOR CAMERA", self.sh)
        self.assertIn("CALIBRATION_SERIAL_ATTESTATION", self.sh)
        self.assertIn('M23BV_V3_STAGE1_CAMERA_HOST_DEVICE=$camera_dev', self.sh)
        self.assertIn('M23BV_V3_STAGE1_CAMERA_CONTAINER_DEVICE=/dev/video2', self.sh)

    def test_all_protected_blob_pins_are_full_length(self):
        block = self.sh.split("done <<'PINS'", 1)[1].split("\nPINS\n", 1)[0]
        rows = [line for line in block.splitlines() if line.startswith("ros/")]
        self.assertEqual(len(rows), 5)
        for row in rows:
            path, sha = row.split()
            self.assertEqual(len(sha), 40, path)
            self.assertTrue(all(char in "0123456789abcdef" for char in sha))
        self.assertIn("8e0db7a1f223fc992ba7704039e9ceb5a48465a4", self.sh)

    def test_explicit_human_and_motor_safety_before_mapping(self):
        for marker in (
            "M23BV_STAGE1_CAMERA_ONLY_APPROVED",
            "M23BV_STAGE1_MOTOR_RAILS_ISOLATED",
            "M23BV_STAGE1_ESTOP_VERIFIED",
            "M23BV_STAGE1_CHASSIS_SECURED",
            "AUTHORIZE V3 CAMERA ONLY",
            "CALIBRATION_HASH_MISMATCH",
            "SOURCE_PIN_CHANGED",
            "WRONG_BRANCH",
        ):
            self.assertIn(marker, self.sh)
            self.assertLess(self.sh.index(marker), self.sh.index("podman run"))

    def test_immutable_pin_and_v3_launch_only(self):
        self.assertIn(
            "f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c",
            self.sh,
        )
        self.assertIn("m23bv_reliable_rectifier_v3_candidate.launch.py", self.py)
        self.assertIn("feature/m23bv-v3-stage1-camera-only-optin", self.sh)
        self.assertNotIn('["ros2", "launch", "/opt/m23bv/m23bv_camera_only.launch.py"]', self.py)
        self.assertIn("/opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash", self.sh)

    def test_bounded_and_fail_closed(self):
        self.assertIn("timeout --foreground", self.sh)
        self.assertIn("75s", self.sh)
        self.assertIn("MAX_OBSERVE_SECONDS = 48", self.py)
        self.assertIn("os.killpg(proc.pid, signal.SIGINT)", self.py)
        self.assertIn("M23BV_V3_STAGE1_LAUNCH_REAPED=", self.py)
        self.assertIn("proc.returncode == 0", self.py)
        self.assertIn("not relay_thread.is_alive()", self.py)
        self.assertIn("M23BV_V3_STAGE1_RESULT=PASS", self.py)
        self.assertIn("sha256sum \"$log\"", self.sh)

    def test_ros_launch_marker_parsing_survives_prefixed_output(self):
        self.assertIn('re.search(r"M23BV_CANDIDATE_MATCHED_CAMERA_FRAMES=', self.py)
        self.assertIn('"M23BV_CANDIDATE_CAMERA_READINESS=PASS" in line', self.py)

    def test_calibration_cross_check(self):
        self.assertIn("yaml.safe_load", self.py)
        self.assertIn("info_matches_calibration(msg, expected_cal)", self.py)
        self.assertIn("CAMERAINFO_CALIBRATION_MISMATCH", self.py)

    def test_qos_is_live_endpoints_and_frames_not_config_only(self):
        for text in (
            "get_publishers_info_by_topic", "get_subscriptions_info_by_topic",
            "ReliabilityPolicy.RELIABLE", "DurabilityPolicy.VOLATILE",
            '"/camera/image_rect"', "MIN_RECT_FRAMES = 90",
            'marker["ready"]', 'marker["matched"]',
        ):
            self.assertIn(text, self.py)
        self.assertIn("topic for topic, _types in node.get_topic_names_and_types()", self.py)
        self.assertIn("node.get_node_names_and_namespaces()", self.py)

    def test_exact_image_transport_topics_and_graph_fail_closed(self):
        # Source-only guard: accept only four real usb_cam transport names,
        # not a prefix, wildcard, motion topic, recorder or unrelated graph.
        topic_assignments = [
            node.value for node in self.tree.body
            if isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "ALLOWED_TOPICS"
                    for target in node.targets)
        ]
        self.assertEqual(len(topic_assignments), 1)
        self.assertEqual(ast.literal_eval(topic_assignments[0]), {
            "/camera/image_raw", "/camera/camera_info", "/camera/image_rect",
            "/camera/image_raw/compressed", "/camera/image_raw/compressedDepth",
            "/camera/image_raw/theora", "/camera/image_raw/zstd",
            "/parameter_events", "/rosout",
        })
        self.assertIn("if topics - ALLOWED_TOPICS:", self.py)
        self.assertIn('fail("UNEXPECTED_TOPICS:"', self.py)
        self.assertIn("if nodes - ALLOWED_NODES:", self.py)
        self.assertIn('fail("UNEXPECTED_NODES:"', self.py)

    def test_unexpected_camera_child_exit_blocks_cleanup_acceptance(self):
        self.assertIn('"process has died" in line', self.py)
        self.assertIn('"[usb_cam_node_exe-" in line', self.py)
        self.assertIn('"[m23bv_rectify_reliable_node-" in line', self.py)
        self.assertIn("abnormal_children.append(line.rstrip())", self.py)
        self.assertIn("launch_reaped = launch_reaped and not abnormal_children", self.py)
        self.assertIn("M23BV_V3_STAGE1_CHILD_EXIT=BLOCKED_ABNORMAL", self.py)
        self.assertIn("BLOCKED_LAUNCH_OR_CHILD_EXIT", self.py)

    def test_observer_has_no_non_ros_device_imports(self):
        imported_roots = {
            alias.name.split(".")[0]
            for node in ast.walk(self.tree) if isinstance(node, ast.Import)
            for alias in node.names
        } | {
            node.module.split(".")[0]
            for node in ast.walk(self.tree)
            if isinstance(node, ast.ImportFrom) and node.module
        }
        self.assertFalse(imported_roots & {
            "serial", "cv2", "motor", "nav2", "xrslam", "rosbag2_py", "usb", "v4l2",
        })
        calls = [node for node in ast.walk(self.tree)
                 if isinstance(node, ast.Call) and isinstance(node.func, ast.Name)]
        self.assertFalse(any(node.func.id in {"exec", "eval"} for node in calls))
        file_opens = [node for node in calls if node.func.id == "open"]
        self.assertEqual(len(file_opens), 1)
        self.assertEqual(ast.unparse(file_opens[0].args[0]), "path")
        self.assertIn('with open(path, encoding="utf-8")', self.py)


if __name__ == "__main__":
    unittest.main(verbosity=2)
