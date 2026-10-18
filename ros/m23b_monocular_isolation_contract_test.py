"""M23B no-device pure-Python guard tests for synthetic XRSLAM replay.

These inspect the exact code to prevent accidental live sensor/motor
activation. A passing contract test does NOT prove the ROS runtime.
"""
import ast
from pathlib import Path
import unittest

from m23b_synthetic_monocular_bag import (
    FRAME_COUNT, WIDTH, HEIGHT, PERIOD_NS, synthetic_pixels,
)

HERE = Path(__file__).resolve().parent
REPLAY = (HERE / "m23b_real_xrslam_monocular_replay_selftest.py").read_text()
FIXTURE = (HERE / "m23b_synthetic_monocular_bag.py").read_text()
RUNNER = (HERE / "m23b_no_device_run.sh").read_text()
DOCKER = (HERE / "Dockerfile.m23b-selftest").read_text()


class M23BMonocularIsolationContract(unittest.TestCase):
    def test_fixture_size_is_bounded(self):
        self.assertEqual((WIDTH, HEIGHT), (640, 480))
        self.assertEqual(FRAME_COUNT, 16)
        self.assertLessEqual(WIDTH * HEIGHT * FRAME_COUNT, 5_000_000)

    def test_fixture_pixel_format_and_determinism(self):
        one = synthetic_pixels(0)
        self.assertEqual(len(one), WIDTH * HEIGHT)
        self.assertEqual(one, synthetic_pixels(0))
        self.assertNotEqual(one, synthetic_pixels(1))
        self.assertEqual(set(one), {0, 255})

    def test_fixture_rejects_out_of_range_index(self):
        for index in (-1, FRAME_COUNT, FRAME_COUNT + 1):
            with self.subTest(index=index):
                with self.assertRaises(ValueError):
                    synthetic_pixels(index)

    def test_replay_cadence_is_bounded(self):
        self.assertGreater(PERIOD_NS, 0)
        self.assertLessEqual(PERIOD_NS, 250_000_000)

    def test_only_synthetic_camera_topics_created(self):
        self.assertIn('"/camera/camera_info"', FIXTURE)
        self.assertIn('"/camera/image_rect"', FIXTURE)
        for topic in ('"/cmd_vel"', '"/odom"', '"/imu/data_raw"'):
            self.assertNotIn("writer.create_topic(" + topic, FIXTURE)
        self.assertIn("rosbag2_py.SequentialWriter()", FIXTURE)

    def test_actual_adapter_started_with_inhibition(self):
        self.assertIn('"-p", "pose_convention_validated:=false"', REPLAY)
        self.assertIn('"-p", "enable_map_tf:=false"', REPLAY)
        self.assertIn("/leroy_ws/install/leroy_xrslam/", REPLAY)
        self.assertIn('"/opt/xrslam/configs/visual_only_slam.yaml"', REPLAY)
        self.assertIn("XRSLAMRunOneFrame", REPLAY)

    def test_synthetic_tf_is_explicit_and_never_robot_calibration(self):
        self.assertIn('tf.header.frame_id = "base_link"', REPLAY)
        self.assertIn('tf.child_frame_id = "camera_optical_frame"', REPLAY)
        self.assertIn("tf.transform.rotation.w = 1.0", REPLAY)
        self.assertIn("extrinsics_validated", REPLAY)

    def test_odom_messages_and_command_publishers_denied(self):
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")', REPLAY)
        self.assertIn("self.odom_receipts == 0", REPLAY)
        self.assertIn("not any(self.validity)", REPLAY)
        self.assertIn('"xrslam_tracking_validated": False', REPLAY)

    def test_runner_fails_closed_on_physical_devices(self):
        self.assertIn("/dev/ttyACM0", RUNNER)
        self.assertIn("/dev/video2", RUNNER)
        self.assertIn("M23B_SYNTHETIC_XRSLAM_ONLY", RUNNER)
        self.assertIn("ROS_AUTOMATIC_DISCOVERY_RANGE", RUNNER)

    def test_no_production_bridge_or_robot_launch_used(self):
        self.assertNotIn("leroy_bridge.node", REPLAY)
        self.assertNotIn("robot.launch.py", REPLAY)
        self.assertNotIn("serial.Serial(", REPLAY)
        self.assertNotIn("create_publisher(Twist", REPLAY)
        self.assertNotIn('"/cmd_vel",', REPLAY)
        self.assertNotIn("--privileged", DOCKER)
        self.assertNotIn("chmod 666", DOCKER)

    def test_synthetic_origin_and_no_device_report_explicit(self):
        self.assertIn('"evidence_type": "SYNTHETIC_MONOCULAR_BAG_TO_REAL_XRSLAM"', REPLAY)
        self.assertIn('"physical_motion_authorized": False', REPLAY)
        self.assertIn('"physical_device_access": False', REPLAY)
        self.assertIn("M23B_DEVICE_ACCESS=NONE", REPLAY)

    def test_no_hidden_subprocess_from_fixture(self):
        tree = ast.parse(FIXTURE)
        top_imports = []
        for node in tree.body:
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                top_imports.append(ast.unparse(node))
        self.assertNotIn("subprocess", " ".join(top_imports))
        self.assertNotIn("serial", " ".join(top_imports))
        self.assertNotIn("rclpy", " ".join(top_imports))


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23BMonocularIsolationContract))
    if result.wasSuccessful():
        print("M23B_MONOCULAR_ISOLATION_CONTRACT=PASS", flush=True)
        print("M23B_VERDICT=STATIC_SYNTHETIC_REPLAY_CONTRACT_ONLY",
              flush=True)
        print("M23B_REAL_XRSLAM_RUNTIME_VALIDATED=FALSE", flush=True)
        print("M23B_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23B_MONOCULAR_ISOLATION_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
