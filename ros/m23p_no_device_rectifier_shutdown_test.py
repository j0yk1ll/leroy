"""M23P: ten offline contracts, without loading ROS or opening any device."""
from pathlib import Path
from unittest.mock import patch
import unittest

from m23p_no_device_rectifier_shutdown import (
    PROFILES, REQUIRED_ENV, assess_container_exit, selected_profile,
    prohibit_devices,
)


def offline_env(profile="idle"):
    result = dict(REQUIRED_ENV)
    result["M23P_PROFILE"] = profile
    return result


class M23PNoDeviceRectifierContracts(unittest.TestCase):
    def test_only_two_device_free_profiles(self):
        self.assertEqual(PROFILES, ("idle", "synthetic"))
        for name in PROFILES:
            self.assertEqual(selected_profile(offline_env(name)), name)

    def test_missing_or_unknown_profile_fails_closed(self):
        for value in ("", "camera", "real", "idle,publisher", None):
            env = offline_env()
            if value is None:
                del env["M23P_PROFILE"]
            else:
                env["M23P_PROFILE"] = value
            with self.assertRaises(ValueError):
                selected_profile(env)

    def test_all_isolation_markers_are_required(self):
        for key in REQUIRED_ENV:
            env = offline_env()
            env.pop(key)
            with self.assertRaises(ValueError):
                selected_profile(env)

    def test_no_device_mapping_is_refused(self):
        with patch("m23p_no_device_rectifier_shutdown.glob.glob",
                   side_effect=lambda query:
                       ["/dev/video8"] if query == "/dev/video*" else []):
            with self.assertRaises(ValueError):
                prohibit_devices()

    def test_clean_component_shutdown_is_positive_only_for_expected_exit(self):
        clean = "[INFO] [component_container-1]: process has finished cleanly [pid 123]"
        for code in (0, 130, -2):
            self.assertEqual(assess_container_exit(clean, code),
                             "CLEAN_COMPONENT_SHUTDOWN")

    def test_sigsegv_blocks_even_when_launcher_exits_zero(self):
        crash = ("[ERROR] [component_container-1]: process has died "
                 "[pid 123, exit code -11]")
        self.assertEqual(assess_container_exit(crash, 0),
                         "BLOCKED_RECTIFIER_CHILD_CRASH")

    def test_absent_clean_marker_or_bad_code_blocks(self):
        self.assertEqual(
            assess_container_exit("[INFO] Loaded node", 0),
            "BLOCKED_RECTIFIER_SHUTDOWN_NOT_CONFIRMED")
        clean = "[component_container-1]: process has finished cleanly"
        self.assertEqual(assess_container_exit(clean, 1),
                         "BLOCKED_RECTIFIER_LAUNCH_EXIT_1")

    def test_launch_contains_only_rectifier_component(self):
        root = Path(__file__).resolve().parent
        source = (root / "m23p_rectifier_only.launch.py").read_text()
        for token in ("ComposableNodeContainer", "component_container",
                      "image_proc::RectifyNode", "namespace=\"/camera\"",
                      '("image", "image_raw")',
                      '("camera_info", "camera_info")'):
            self.assertIn(token, source)
        for forbidden in ("usb_cam", "serial.Serial(", "Nav2", "XRSLAM",
                          "motor_bridge", "robot_controller"):
            self.assertNotIn(forbidden, source)

    def test_runner_reuses_local_synthetic_cpp_not_real_capture(self):
        source = (Path(__file__).resolve().parent /
                  "m23p_no_device_rectifier_shutdown.py").read_text()
        for token in (
            "m23l_synthetic_image_publisher",
            "m23k_camera_header_counter",
            'env["M23M_SUB_QOS"] = "reliable_100"',
            'env["M23K_RAW_ONLY"] = "0"',
            "parse_publisher_log(",
            "parse_counter_log(observer_log, require_rect=True)",
            "camera_crash_context(launch_text)",
            "M23P_LIFECYCLE_JSON=",
            "M23P_DEVICE_ACCESS=NONE",
        ):
            self.assertIn(token, source)
        for forbidden in ("create_publisher(", "serial.Serial(",
                          "ros2 bag record", "--device "):
            self.assertNotIn(forbidden, source)

    def test_container_contract_is_completely_device_free(self):
        root = Path(__file__).resolve().parent
        docker = (root / "Dockerfile.m23p-rectifier-shutdown").read_text()
        runbook = (root / "M23P_NO_DEVICE_RECTIFIER_SHUTDOWN.md").read_text()
        self.assertIn(
            "FROM localhost/leroy-sensors:m23o-rect-camera-qos", docker)
        self.assertIn("M23P_JAZZY_NO_DEVICE_IMPORT=PASS", docker)
        self.assertIn("--network none", runbook)
        self.assertIn("--cap-drop all", runbook)
        self.assertIn("--read-only", runbook)
        for forbidden in ("--device ", "--privileged", "--network host",
                          "apt-get ", "wget ", "curl "):
            self.assertNotIn(forbidden, docker)
            self.assertNotIn(forbidden, runbook)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23PNoDeviceRectifierContracts))
    if result.wasSuccessful():
        print("M23P_NO_DEVICE_SHUTDOWN_SOURCE_CONTRACT=PASS", flush=True)
        print("M23P_PHYSICAL_CAMERA_RUN=NONE", flush=True)
        print("M23P_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
