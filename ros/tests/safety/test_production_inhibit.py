"""Production startup is explicitly fail-closed until physical commissioning.

No ROS, container, serial, camera or motor device is opened by these tests.
"""
import subprocess
from pathlib import Path
import unittest

ROS = Path(__file__).resolve().parents[2]


class ProductionInhibit(unittest.TestCase):
    def test_operator_run_is_denied_before_any_hardware_access(self):
        result = subprocess.run(
            ["bash", str(ROS / "leroy.sh"), "run"],
            cwd=ROS, text=True, capture_output=True, timeout=5)
        self.assertEqual(result.returncode, 2)
        self.assertIn("BLOCKED_PRODUCTION_MOTION_NOT_COMMISSIONED", result.stderr)

    def test_hardware_manifest_remains_disabled(self):
        manifest = (ROS / "src/leroy_navigation/config/hardware.disabled.yaml").read_text()
        self.assertIn("enabled: false", manifest)
        self.assertIn("bridge_command_output_enabled: false", manifest)
        self.assertIn("odometry_validated: false", manifest)

    def test_no_diagnostic_build_or_launch_entry_points(self):
        self.assertFalse(list(ROS.glob("Dockerfile.*")))
        self.assertFalse((ROS / "src/leroy_capture").exists())
        self.assertFalse((ROS / "src/leroy_bringup/scripts/build_diagnostic_images.sh").exists())
        self.assertFalse((ROS / "src/leroy_navigation/launch/dry_run.launch.py").exists())

    def test_motor_ingress_rejects_nonzero_commands(self):
        from leroy_bridge.command_ingress import CommandRejected, command_to_bytes
        with self.assertRaises(CommandRejected):
            command_to_bytes("set_wheels 1 0 0 0")
        with self.assertRaises(CommandRejected):
            command_to_bytes("set_wheels_tx seq=1 1 0 0 0")
        self.assertEqual(command_to_bytes("set_wheels 0 0 0 0"),
                         b"set_wheels 0 0 0 0\n")


if __name__ == "__main__":
    unittest.main()
