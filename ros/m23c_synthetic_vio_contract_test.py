"""M23C pure-Python, no-device synthetic VIO fault contract tests.

These run on Arch without ROS, XRSLAM or devices. They check fixture
timing and fail-closed source invariants; runtime evidence comes later.
"""
import ast
from pathlib import Path
import unittest

from m23c_synthetic_vio_bag import (
    DROPOUT_NS, FRAME_COUNT, IMU_COUNT, IMU_PERIOD_NS,
    IMU_START_NS, LAST_IMU_NS, PERIOD_NS, START_NS,
    camera_records_import_free, imu_stamps, schedule_metadata,
)

ROOT = Path(__file__).resolve().parent
FIXTURE = (ROOT / "m23c_synthetic_vio_bag.py").read_text()
REPLAY = (ROOT / "m23c_real_xrslam_synthetic_vio_selftest.py").read_text()
RUNNER = (ROOT / "m23c_no_device_run.sh").read_text()
DOCKER = (ROOT / "Dockerfile.m23c-selftest").read_text()


class M23CSyntheticVioContract(unittest.TestCase):
    def test_camera_imagerate_bounded(self):
        self.assertEqual(FRAME_COUNT, 16)
        self.assertEqual(PERIOD_NS, 125_000_000)

    def test_imu_is_50_hz_and_bounded(self):
        self.assertEqual(IMU_PERIOD_NS, 20_000_000)
        self.assertEqual(IMU_COUNT, 104)
        self.assertEqual(LAST_IMU_NS,
                         IMU_START_NS + (IMU_COUNT - 1) * IMU_PERIOD_NS)

    def test_imu_header_times_monotonic(self):
        times = imu_stamps()
        self.assertEqual(len(times), IMU_COUNT)
        self.assertTrue(all(a < b for a, b in zip(times, times[1:])))

    def test_synthetic_imu_covers_both_ends_of_camera(self):
        self.assertLess(imu_stamps()[0], START_NS)
        self.assertGreater(imu_stamps()[-1],
                           START_NS + (FRAME_COUNT - 1) * PERIOD_NS)

    def test_metadata_topics_exact_and_sorted(self):
        events = schedule_metadata()
        self.assertEqual(len(events), 2 * FRAME_COUNT + IMU_COUNT)
        self.assertEqual(
            {topic for _at, topic in events},
            {"/camera/image_rect", "/camera/camera_info", "/imu/data_raw"})
        self.assertEqual(
            [at for at, _topic in events],
            sorted(at for at, _topic in events))

    def test_camera_stamps_and_topics_match(self):
        events = list(camera_records_import_free())
        self.assertEqual(len(events), 2 * FRAME_COUNT)
        for i in range(FRAME_COUNT):
            self.assertEqual(events[i * 2][1], "/camera/camera_info")
            self.assertEqual(events[i * 2 + 1][1], "/camera/image_rect")
            self.assertEqual(events[i * 2 + 1][0],
                             events[i * 2][0] + 1)

    def test_fault_injection_exceeds_adapter_imu_limit(self):
        self.assertEqual(DROPOUT_NS, 500_000_000)
        self.assertGreater(DROPOUT_NS, 300_000_000)

    def test_only_source_topics_written_to_bag(self):
        self.assertIn('"sensor_msgs/msg/Imu"', FIXTURE)
        self.assertIn('"sensor_msgs/msg/CameraInfo"', FIXTURE)
        self.assertIn('"sensor_msgs/msg/Image"', FIXTURE)
        self.assertNotIn('"sensor_msgs/msg/Twist"', FIXTURE)
        self.assertNotIn("serial.Serial(", FIXTURE)

    def test_actual_vio_adapter_subprocess_and_mode(self):
        self.assertIn('str(EXECUTABLE), "--ros-args"', REPLAY)
        self.assertIn('mode="monocular_imu"', REPLAY)
        self.assertIn('"camera_imu_offset_sec:=0.0"', REPLAY)
        self.assertIn('mode="monocular_imu"', REPLAY)
        self.assertIn('"-p", "pose_convention_validated:=false"', REPLAY)
        self.assertIn('"-p", "enable_map_tf:=false"', REPLAY)

    def test_vio_real_forwarding_guard_and_fault_latch(self):
        self.assertIn(
            "Submitted 1 camera frames to XRSLAM (monocular_imu)", REPLAY)
        self.assertIn('if "imu_gap" in node.status:', REPLAY)
        self.assertIn('node.status[-1] == "imu_gap"', REPLAY)
        self.assertIn("M23C_REAL_XRSLAM_IMU_GAP_LATCH=PASS", REPLAY)

    def test_guarded_outputs_and_synthetic_status(self):
        self.assertIn("node.assert_no_motion_authority()", REPLAY)
        self.assertIn('"odom_messages": node.odom_receipts', REPLAY)
        self.assertIn('"dynamic_tf_messages": node.dynamic_tf_receipts', REPLAY)
        self.assertIn('"physical_device_access": False', REPLAY)
        self.assertIn('"physical_motion_authorized": False', REPLAY)
        self.assertIn('"camera_imu_time_offset_calibrated": False', REPLAY)

    def test_runner_stays_strictly_no_device(self):
        for term in (
                "M23C_SYNTHETIC_VIO_ONLY", "ROS_DOMAIN_ID",
                "/dev/ttyACM0", "/dev/video2", "PYTHONDONTWRITEBYTECODE"):
            self.assertIn(term, RUNNER)

    def test_docker_copies_test_dependencies_at_expected_paths(self):
        for term in (
                "COPY m23c_no_device_run.sh /opt/m23c/m23c_no_device_run.sh",
                "COPY Dockerfile.m23c-selftest /opt/m23c/Dockerfile.m23c-selftest",
                "COPY m23c_no_device_run.sh /usr/local/bin/leroy-m23c-test-run",
                "COPY m23c_synthetic_vio_contract_test.py /opt/m23c/"):
            self.assertIn(term, DOCKER)

    def test_docker_has_no_network_install_or_devices(self):
        for bad in ("apt-get", "curl ", "--privileged", "--device",
                    "pip install", "chmod 666"):
            self.assertNotIn(bad, DOCKER)

    def test_no_production_launch_from_runtime(self):
        for bad in ("serial.Serial(", "robot.launch.py",
                    "leroy_bridge.node", "create_publisher(Twist"):
            self.assertNotIn(bad, REPLAY)

    def test_fixture_imports_do_not_start_ros(self):
        parsed = ast.parse(FIXTURE)
        imported = " ".join(ast.unparse(t) for t in parsed.body
                            if isinstance(t, (ast.Import, ast.ImportFrom)))
        self.assertNotIn("import rclpy", imported)
        self.assertNotIn("import serial", imported)
        self.assertNotIn("import subprocess", imported)


if __name__ == "__main__":
    outcome = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23CSyntheticVioContract))
    if outcome.wasSuccessful():
        print("M23C_SYNTHETIC_VIO_STATIC_CONTRACT=PASS", flush=True)
        print("M23C_VERDICT=STATIC_SYNTHETIC_VIO_FAULT_CONTRACT_ONLY",
              flush=True)
        print("M23C_REAL_XRSLAM_VIO_RUNTIME_VALIDATED=FALSE", flush=True)
        print("M23C_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23C_SYNTHETIC_VIO_STATIC_CONTRACT=FAIL", flush=True)
    raise SystemExit(2)
