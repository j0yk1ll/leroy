"""M23G strictly offline preparation contract — never opens the STM32.

Only temporary ordinary directories, mocked disk-space metadata and
declarative rosbag CLI/QoS are inspected. No ROS, serial or camera.
"""
from pathlib import Path
from types import SimpleNamespace as S
from unittest import mock
import tempfile
import unittest

import m23g_live_recording_preflight as sut


class M23GRecordingSafetyContract(unittest.TestCase):
    def test_topics_are_exact_three_sensor_messages(self):
        self.assertEqual(sut.TOPICS, (
            "/camera/image_rect", "/camera/camera_info", "/imu/data_raw"))

    def test_motor_topics_never_requested(self):
        a = " ".join(sut.recorder_argv(Path("/tmp/data/fake")))
        for topic in ("/cmd_vel", "/odom", "/tf", "/wheel/encoder_counts"):
            self.assertNotIn(topic, a)
        self.assertNotIn("--all", a)
        self.assertNotIn(" -a ", " " + a + " ")

    def test_recorder_is_explicit_sqlite3_and_three_topics(self):
        a = sut.recorder_argv(Path("/tmp/dummy"))
        self.assertEqual(a[:3], ["ros2", "bag", "record"])
        self.assertEqual(a[a.index("--storage") + 1], "sqlite3")
        self.assertEqual(a[a.index("--topics") + 1:], list(sut.TOPICS))
        self.assertIn("--qos-profile-overrides-path", a)

    def test_nominal_120_second_budget_includes_safety_margin(self):
        raw = 30 * 640 * 480 * 3 * 120
        self.assertEqual(sut.required_bytes(120),
                         raw * 2 + 2 * 1024**3)

    def test_refuses_unbounded_recording_duration(self):
        for seconds in (-1, 0, 29, 181, 3600):
            with self.subTest(seconds=seconds):
                with self.assertRaises(ValueError):
                    sut.required_bytes(seconds)

    def test_valid_short_bounded_duration(self):
        self.assertGreater(sut.required_bytes(30), 2 * 1024**3)

    def test_invalid_duration_type_blocked(self):
        for x in (120.0, "120", True):
            with self.subTest(value=x):
                with self.assertRaises(TypeError):
                    sut.required_bytes(x)

    def test_nonexistent_output_parent_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            with self.assertRaises(ValueError):
                sut.preflight(Path(td) / "missing", "capture1")

    def test_system_device_roots_are_denied(self):
        for path in (Path("/"), Path("/dev"), Path("/proc"), Path("/sys")):
            if path.exists():
                with self.subTest(path=str(path)):
                    with self.assertRaises(ValueError):
                        sut.validate_root(path)

    def test_symlinked_output_parent_is_rejected(self):
        with tempfile.TemporaryDirectory() as td:
            base = Path(td)
            actual = base / "actual"
            actual.mkdir()
            link = base / "alias"
            link.symlink_to(actual, target_is_directory=True)
            with self.assertRaises(ValueError):
                sut.preflight(link, "session1")

    def test_session_id_rejects_path_traversal(self):
        with tempfile.TemporaryDirectory() as td:
            for identifier in ("../outside", "/tmp/a", ".", "foo/bar",
                               "", "UPPER", "hello world"):
                with self.subTest(identifier=identifier):
                    with self.assertRaises(ValueError):
                        sut.preflight(Path(td), identifier)

    def test_existing_target_never_overwritten(self):
        with tempfile.TemporaryDirectory() as td:
            target = Path(td) / "m23g_live_existing"
            target.mkdir()
            with self.assertRaises(ValueError):
                sut.preflight(Path(td), "existing")

    def test_symlink_target_never_overwritten(self):
        with tempfile.TemporaryDirectory() as td:
            target = Path(td) / "m23g_live_existing"
            target.symlink_to(Path(td) / "absent-target")
            with self.assertRaises(ValueError):
                sut.preflight(Path(td), "existing")

    def test_successful_disk_check_still_denies_physical_capture(self):
        with tempfile.TemporaryDirectory() as td:
            fake = S(f_bavail=20 * 1024**3, f_frsize=1)
            with mock.patch.object(sut.os, "statvfs", return_value=fake):
                report = sut.preflight(Path(td), "capture1")
        self.assertEqual(report["software_preflight_verdict"],
                         "DISK_BUDGET_PASS_HARDWARE_NOT_AUTHORIZED")
        self.assertIs(report["disk_capacity_sufficient"], True)
        for k in (
            "physical_safety_signoff_received",
            "mcu_serial_access_authorized", "camera_device_access_authorized",
            "rosbag_recording_started", "complete_real_bag_recorded",
            "camera_imu_phase_calibrated",
            "wheel_manipulator_power_inhibited_verified",
            "physical_estop_verified", "mcu_device_identity_revalidated",
            "camera_device_identity_revalidated", "stop_ack_observed",
            "physical_motion_authorized", "device_access",
        ):
            self.assertIs(report[k], False, k)

    def test_insufficient_disk_blocks(self):
        with tempfile.TemporaryDirectory() as td:
            fake = S(f_bavail=10, f_frsize=1024)
            with mock.patch.object(sut.os, "statvfs", return_value=fake):
                report = sut.preflight(Path(td), "capture1")
        self.assertEqual(report["software_preflight_verdict"],
                         "BLOCKED_INSUFFICIENT_CAPTURE_SPACE")
        self.assertIs(report["disk_capacity_sufficient"], False)

    def test_preflight_does_not_create_a_target(self):
        with tempfile.TemporaryDirectory() as td:
            root = Path(td)
            original = list(root.iterdir())
            fake = S(f_bavail=20 * 1024**3, f_frsize=1)
            with mock.patch.object(sut.os, "statvfs", return_value=fake):
                sut.preflight(root, "candidate")
            self.assertEqual(list(root.iterdir()), original)

    def test_qos_yaml_matches_sensor_topics_and_best_effort(self):
        yaml = (Path(__file__).resolve().parent /
                "m23g_sensor_qos.yaml").read_text()
        for topic in sut.TOPICS:
            self.assertIn(topic + ":", yaml)
        self.assertEqual(yaml.count("reliability: best_effort"), 3)
        self.assertNotIn("/cmd_vel:", yaml)

    def test_source_has_no_ros_hardware_or_process_starter(self):
        source = (Path(__file__).resolve().parent /
                  "m23g_live_recording_preflight.py").read_text()
        for bad in ("import serial", "import rclpy", "rclpy.init(",
                    "subprocess.run(", "subprocess.Popen(",
                    "create_publisher(", "ros2 launch", "robot.launch.py"):
            self.assertNotIn(bad, source)

    def test_cmd_is_only_a_plan_never_a_runtime_launch(self):
        with tempfile.TemporaryDirectory() as td:
            fake = S(f_bavail=20 * 1024**3, f_frsize=1)
            with mock.patch.object(sut.os, "statvfs", return_value=fake):
                report = sut.preflight(Path(td), "trial001")
        self.assertEqual(report["planned_argv_NOT_EXECUTED"],
                         sut.recorder_argv(
                            Path(td) / "m23g_live_trial001"))
        self.assertIs(report["rosbag_recording_started"], False)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23GRecordingSafetyContract))
    if result.wasSuccessful():
        print("M23G_OFFLINE_CAPTURE_PREPARATION=PASS", flush=True)
        print("M23G_VERDICT=CAPACITY_AND_COMMAND_PLAN_ONLY", flush=True)
        print("M23G_PHYSICAL_CAPTURE_AUTHORIZED=FALSE", flush=True)
        print("M23G_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M23G_OFFLINE_CAPTURE_PREPARATION=FAIL", flush=True)
    raise SystemExit(2)
