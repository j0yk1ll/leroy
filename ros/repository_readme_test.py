"""Contract: one tracked README is the fresh-clone executable runbook.

Source-only test: no images or physical devices.
"""
from pathlib import Path
import subprocess
import unittest

ROS = Path(__file__).resolve().parent
REPO = ROS.parent


class SingleRunbookTest(unittest.TestCase):
    def test_only_one_tracked_readme(self):
        output = subprocess.run(
            ["git", "-C", str(REPO), "ls-files", "-z"],
            check=True, capture_output=True,
        ).stdout.decode("utf-8")
        readmes = sorted(
            p for p in output.split("\0")
            if p == "README.md" or p.endswith("/README.md")
        )
        self.assertEqual(readmes, ["README.md"])

    def test_every_public_command_has_source(self):
        guide = (REPO / "README.md").read_text()
        for command in ("check", "build", "status", "run"):
            self.assertIn("bash ./leroy.sh " + command, guide)
        for file in (
            "ros/leroy.sh", "ros/build_current_images.sh",
            "ros/check_current_checkout.sh",
            "ros/live_stage3_start.sh", "ros/live_boot_session.sh",
            "firmware/Makefile",
            "ros/src/leroy_nav2_perception/launch/live_depth_costmap.launch.py",
        ):
            with self.subTest(path=file):
                self.assertTrue((REPO / file).is_file())
                self.assertIn(file, guide)
        self.assertIn("make build", guide)
        self.assertIn("plain `make` or `make all` also flashes", guide)

    def test_perception_package_does_not_install_deleted_readme(self):
        setup = (REPO / "ros/src/leroy_nav2_perception/setup.py").read_text()
        self.assertNotIn("['README.md']", setup)

    def test_operator_safety_and_uncommissioned_components_are_explicit(self):
        guide = (REPO / "README.md").read_text()
        for term in (
            "LEROY_MOTOR_RAILS_DISCONNECTED=YES",
            "LEROY_MCU_BY_ID=",
            "BOOT_IMU_ONLY",
            "BOOT_IMU_SOURCE_STOP_ACK=PASS",
            "motor and manipulator",
            "metric localization",
            "hardware.disabled.yaml",
            "archive/before-single-readme-2026-10-29",
        ):
            self.assertIn(term, guide)

    def test_autonomy_todos_are_in_the_owner_files(self):
        required = (
            "ros/leroy.sh",
            "ros/boot_localization_core.py",
            "ros/live_boot_readiness.py",
            "ros/boot_yaw_core.py",
            "ros/boot_auto_calibration_core.py",
            "ros/src/leroy_bridge/leroy_bridge/node.py",
            "ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py",
            "ros/src/leroy_nav2_perception/leroy_nav2_perception/depth_obstacles.py",
            "ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp",
            "ros/src/leroy_nav2/config/hardware.disabled.yaml",
        )
        for file in required:
            with self.subTest(file=file):
                self.assertIn("TODO(AUTONOMY)", (REPO / file).read_text())


if __name__ == "__main__":
    unittest.main()
