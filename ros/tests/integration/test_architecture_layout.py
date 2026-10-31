"""Architectural boundaries for the production ROS 2 source tree.

Source-only: never opens ROS, Podman, serial, camera or motor devices.
"""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROS = Path(__file__).resolve().parents[2]
PACKAGES = {
    "leroy_bringup", "leroy_bridge", "leroy_sensors", "leroy_perception",
    "leroy_localization", "leroy_navigation", "leroy_capture",
    "leroy_description", "leroy_msgs",
}
SCRIPTS = {
    "container_entrypoint.sh", "readiness.sh", "sensor_session.sh",
    "heading.sh", "heading_snapshot.sh", "build_diagnostic_images.sh",
    "xrslam_diagnostic.sh", "sensor_graph.sh", "imu_diagnostic.sh",
}

class ArchitectureLayoutTest(unittest.TestCase):
    def test_exactly_nine_ros_packages(self):
        src = ROS / "src"
        actual = {p.name for p in src.iterdir() if p.is_dir()}
        self.assertEqual(actual, PACKAGES)
        for pkg in PACKAGES:
            manifest = src / pkg / "package.xml"
            self.assertTrue(manifest.is_file(), manifest)
            self.assertEqual(ET.parse(manifest).getroot().findtext("name"), pkg)

    def test_root_has_only_three_shell_interfaces_and_no_python_modules(self):
        self.assertEqual({p.name for p in ROS.glob("*.sh")},
                         {"leroy.sh", "build_current_images.sh", "check_current_checkout.sh"})
        self.assertEqual(list(ROS.glob("*.py")), [])
        self.assertEqual(list(ROS.glob("*.yaml")), [])

    def test_subordinate_bringup_scripts_are_owned_by_package(self):
        directory = ROS / "src/leroy_bringup/scripts"
        self.assertEqual({p.name for p in directory.glob("*.sh")}, SCRIPTS)
        operator = (ROS / "leroy.sh").read_text()
        self.assertIn("src/leroy_bringup/scripts/sensor_graph.sh", operator)
        self.assertIn("src/leroy_bringup/scripts/sensor_session.sh", operator)
        self.assertNotIn("live_stage3_start.sh", operator)

    def test_uncommissioned_xrslam_config_is_not_a_production_default(self):
        config = ROS / "config"
        self.assertTrue((config / "xrslam_monocular_diagnostic.yaml").is_file())
        self.assertTrue((config / "xrslam_device_template_UNCALIBRATED.yaml").is_file())
        launcher = (ROS / "src/leroy_bringup/scripts/xrslam_diagnostic.sh").read_text()
        self.assertIn("config/xrslam_device_template_UNCALIBRATED.yaml", launcher)
        self.assertIn("--network", launcher)

if __name__ == "__main__":
    unittest.main()
