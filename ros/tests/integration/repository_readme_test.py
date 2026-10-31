"""Contract: one tracked README is the fresh-clone executable runbook.

Source-only test: no images or physical devices.
"""
from pathlib import Path
import re
import subprocess
import unittest

ROS = Path(__file__).resolve().parents[2]
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
            "ros/src/leroy_bringup/scripts/live_stage3_start.sh", "ros/src/leroy_bringup/scripts/live_boot_session.sh",
            "firmware/Makefile",
            "ros/src/leroy_perception/launch/live_depth_costmap.launch.py",
        ):
            with self.subTest(path=file):
                self.assertTrue((REPO / file).is_file())
                self.assertIn(file, guide)
        self.assertIn("make build", guide)
        self.assertIn("plain `make` or `make all` also flashes", guide)

    def test_perception_package_does_not_install_deleted_readme(self):
        cmake = (REPO / "ros/src/leroy_perception/CMakeLists.txt").read_text()
        self.assertNotIn("README.md", cmake)
        self.assertIn("ament_python_install_package", cmake)

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

    def test_ordered_nonsecurity_todo_index_matches_code(self):
        """Each task has one numbered source owner and one ordered README row."""
        output = subprocess.run(
            ["git", "-C", str(REPO), "ls-files", "-z"],
            check=True, capture_output=True,
        ).stdout.decode("utf-8")
        source_pattern = re.compile(
            r"TODO\(AUTONOMY\) #([0-9]{3}) \[([^\]]+)\]"
        )
        entries = {}
        for relative in filter(None, output.split("\0")):
            if not relative.endswith((".py", ".sh", ".yaml", ".c", ".cpp")):
                continue
            source = REPO / relative
            # A tracked, locally deleted unrelated file is not a TODO owner.
            # The final numbered-owner/index comparison still detects a
            # deleted source file containing a required autonomy TODO.
            if not source.is_file():
                continue
            content = source.read_text(errors="replace")
            for line in content.splitlines():
                if not line.lstrip().startswith(("# TODO(AUTONOMY)", "// TODO(AUTONOMY)", "/* TODO(AUTONOMY)", "* TODO(AUTONOMY)")):
                    continue
                match = source_pattern.search(line)
                self.assertIsNotNone(match, (relative, line))
                number = int(match.group(1))
                self.assertNotIn(number, entries, (number, relative))
                entries[number] = relative
                for security_specific in (
                    "cryptographic", "signing key", "protected key",
                    "credential", "secret management", "hmac",
                ):
                    self.assertNotIn(security_specific, line.lower())
        self.assertEqual(sorted(entries), list(range(1, 43)))

        guide = (REPO / "README.md").read_text()
        index_start = guide.index("## 11. Fully autonomous operation")
        index_end = guide.index("## 12. Preserve external state", index_start)
        index = guide[index_start:index_end]
        readme_entries = re.findall(
            r"^\|\s*(\d{3})\s*\|[^|]*\|\s*\[`([^`]+)`\]"
            r"\(([^)]+)\)\s*\|$", index, flags=re.MULTILINE
        )
        self.assertEqual(len(readme_entries), 42)
        for expected, (number, label, path) in enumerate(readme_entries, 1):
            self.assertEqual(int(number), expected)
            self.assertEqual(label, path)
            self.assertEqual(path, entries[expected])

    def test_current_python_and_container_dependency_inventory(self):
        """Ensure deprecated milestone modules do not re-enter the build."""
        ros = REPO / "ros"
        active_milestones = sorted(p.name for p in ros.glob("m[0-9]*.py"))
        self.assertEqual(active_milestones, [])
        integration = (
            ros / "src/leroy_sensors/leroy_sensors"
        )
        self.assertEqual(sorted(p.name for p in integration.glob("*.py")), [
            "__init__.py", "delivery_trace.py", "fault_contract.py", "imu_overlap.py",
            "imu_protocol.py", "imu_service.py", "imu_source.py",
            "observer_contract.py", "passive_observer.py", "sensor_contract.py",
            "spin_budget.py",
        ])
        source = (integration / "imu_source.py").read_text()
        self.assertIn(
            "leroy_sensors.passive_observer", source,
        )
        self.assertIn("leroy_sensors.imu_protocol", source)
        setup = (
            ros / "src/leroy_sensors/setup.py"
        ).read_text()
        self.assertIn("passive_observer:main", setup)
        self.assertNotIn("m14_software_gate", setup)
        self.assertEqual(sorted(p.name for p in ros.glob("Dockerfile*")), [
            "Dockerfile", "Dockerfile.live-costmap",
            "Dockerfile.live-perception", "Dockerfile.live-stage4-imu",
            "Dockerfile.live-stage4c-capture", "Dockerfile.xrslam",
        ])
        for legacy in ["m23bv_sensor_qos.yaml","m23bv_v3_stage3_smoke_device.yaml","m23bv_v3_stage3_smoke_monocular.yaml","m23bv_v3_stage3_xrslam_vio_diag.yaml","m23g_sensor_qos.yaml"]:
            self.assertFalse((ros / legacy).exists(), legacy)
        self.assertFalse((ros / "m16_sensor_only_run.sh").exists())
        self.assertFalse((ros / "src/leroy_imu_observer").exists())
        self.assertFalse((ros / "src/leroy_sensors/test").exists())

    def test_autonomy_todos_are_in_the_owner_files(self):
        required = (
            "ros/leroy.sh",
            "ros/src/leroy_localization/leroy_localization/localization_core.py",
            "ros/src/leroy_bringup/leroy_bringup/readiness.py",
            "ros/src/leroy_localization/leroy_localization/yaw_core.py",
            "ros/src/leroy_localization/leroy_localization/auto_calibration_core.py",
            "ros/src/leroy_bridge/leroy_bridge/node.py",
            "ros/src/leroy_bridge/leroy_bridge/motor_cdc_protocol.py",
            "ros/src/leroy_perception/leroy_perception/depth_obstacles.py",
            "ros/src/leroy_xrslam/src/leroy_xrslam_node.cpp",
            "ros/src/leroy_navigation/config/hardware.disabled.yaml",
        )
        for file in required:
            with self.subTest(file=file):
                self.assertIn("TODO(AUTONOMY)", (REPO / file).read_text())


if __name__ == "__main__":
    unittest.main()
