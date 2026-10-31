"""No-device contracts for the unified Leroy sensor runtime CLI."""
from pathlib import Path
import subprocess
import unittest

ROOT = Path(__file__).resolve().parents[2]
RUNNER = ROOT / "leroy.sh"


class UnifiedLeroyRuntimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = RUNNER.read_text()

    def test_help_is_safe_without_podman_devices_or_consent(self):
        cp = subprocess.run(
            ["bash", str(RUNNER), "help"], cwd=ROOT, check=True,
            capture_output=True, text=True, timeout=5,
        )
        for token in (
            "LEROY_MOTOR_RAILS_DISCONNECTED=YES",
            "LEROY_MCU_BY_ID=",
            "BOOT_IMU_ONLY",
            "run",
            "status",
            "build",
            "check",
        ):
            self.assertIn(token, cp.stdout)
        self.assertNotIn("LEROY_RUNTIME=STARTING", cp.stdout)

    def test_invalid_command_does_not_launch_any_service(self):
        cp = subprocess.run(
            ["bash", str(RUNNER), "invalid"], cwd=ROOT,
            capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(cp.returncode, 2)
        self.assertIn("BLOCKED_UNKNOWN_COMMAND", cp.stderr)

    def test_only_one_verified_camera_and_imu_workflow(self):
        s = self.source
        self.assertIn("LEROY_MOTOR_POWER_DISCONNECTED=1 bash ./src/leroy_bringup/scripts/sensor_graph.sh", s)
        self.assertIn("bash ./src/leroy_bringup/scripts/sensor_session.sh", s)
        self.assertLess(s.index("bash ./src/leroy_bringup/scripts/sensor_graph.sh"),
                        s.index("bash ./src/leroy_bringup/scripts/sensor_session.sh"))
        self.assertIn('LEROY_STAGE4_MCU_BY_ID="$mcu"', s)
        self.assertIn("LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES", s)
        self.assertIn("LEROY_RUNTIME=CAMERA_DEPTH_XRSLAM_READY", s)

    def test_physical_and_serial_guardrails_retained(self):
        s = self.source
        for token in (
            "rootless", "LEROY_MOTOR_RAILS_DISCONNECTED",
            "STABLE_STM32_DEVICE_REQUIRED", "/dev/serial/by-id/",
            "/dev/ttyACM0", "leroy-boot-imu-source",
            "leroy-signed-bridge", "leroy-actuated-yaw-calibration",
            "podman container exists", "MISSING_CURRENT_IMAGE",
        ):
            self.assertIn(token, s)
        for forbidden in (
            "--privileged", "--device=", "--network=host",
            "set_wheels", "signed_motion", "/cmd_vel",
            "ros2 launch nav2_bringup",
        ):
            self.assertNotIn(forbidden, s)

    def test_cleanup_dependency_order_and_fault_propagation(self):
        s = self.source
        self.assertIn("trap cleanup EXIT", s)
        self.assertIn("UNEXPECTED_IMU_SOURCE_STILL_PRESENT", s)
        self.assertIn("rc=2", s)
        self.assertIn("leroy-live-xrslam leroy-live-costmap", s)
        self.assertIn("LEROY_RUNTIME=STOPPED_SENSOR_ONLY_NO_MOTION_AUTHORITY", s)
        self.assertIn("LEROY_RUNTIME=FAILED_OR_INTERRUPTED_EXIT_", s)
        self.assertIn("trap ':' INT", s)
        self.assertIn("trap 'exit 143' TERM", s)
        self.assertNotIn("podman rm -f leroy-live-costmap", s)
        self.assertNotIn("podman rm -f leroy-live-xrslam", s)

    def test_retained_launchers_are_valid_bash(self):
        cp = subprocess.run(
            ["bash", "-n", str(RUNNER)], cwd=ROOT,
            capture_output=True, text=True, timeout=5,
        )
        self.assertEqual(cp.returncode, 0, cp.stderr)


if __name__ == "__main__":
    unittest.main()
