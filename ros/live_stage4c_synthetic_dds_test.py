"""Pure-Python preflight and static guards for disconnected synthetic DDS test."""
import ast
from pathlib import Path
import unittest

from live_stage4c_capture_contract import CaptureError
from live_stage4c_synthetic_dds_bag import (
    check_isolation, DEVICE_PATHS, SYNTHETIC_COUNT, FORBIDDEN_TOPICS,
)

SCRIPT = Path(__file__).with_name("live_stage4c_synthetic_dds_bag.py")


class SyntheticDDSNoDevice(unittest.TestCase):
    @staticmethod
    def valid_environment():
        return {
            "LEROY_STAGE4C_SYNTHETIC_ONLY": "1",
            "ROS_DOMAIN_ID": "171",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        }

    def test_requires_explicit_isolated_synthetic_env(self):
        self.assertTrue(check_isolation(
            self.valid_environment(), exists=lambda _: False))
        for key in self.valid_environment():
            wrong = self.valid_environment()
            wrong.pop(key)
            with self.subTest(key=key), self.assertRaisesRegex(
                CaptureError, "synthetic_preflight_environment_" + key):
                check_isolation(wrong, exists=lambda _: False)

    def test_rejects_visible_serial_camera_or_gpio_device(self):
        for device in DEVICE_PATHS:
            with self.subTest(device=device), self.assertRaisesRegex(
                CaptureError, "synthetic_preflight_device_present"):
                check_isolation(
                    self.valid_environment(),
                    exists=lambda path: str(path) == device)

    def test_synthetic_script_cannot_start_real_m18_or_motor(self):
        source = SCRIPT.read_text()
        tree = ast.parse(source)
        imports = [
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.Import)
            for alias in node.names
        ]
        self.assertNotIn("serial", imports)
        self.assertNotIn("subprocess", imports)
        for text in (
            "ImuStreamSource(", "serial.Serial(", "device.write(",
            "create_publisher(Twist", "robot.launch.py",
            "LIVE_CAPTURE_COMMISSIONED = True", "STAGE4_IMU_ONLY",
        ):
            self.assertNotIn(text, source)
        self.assertIn("SourceMetadataRosAdapter(source)", source)
        self.assertIn("CaptureContract(duration_s=45)", source)
        self.assertIn("TemporaryDirectory(", source)

    def test_does_not_call_live_recorder_entrypoint(self):
        source = SCRIPT.read_text()
        self.assertNotIn("from live_stage4c_passive_recorder import", source)
        self.assertNotIn("live_stage4c_passive_recorder.py", source)
        self.assertIn("check_isolation()", source)
        self.assertIn("rclpy.init(args=[])", source)
        self.assertEqual(SYNTHETIC_COUNT, 12)
        self.assertTrue({"/cmd_vel", "/odom", "/tf"} <= FORBIDDEN_TOPICS)

    def test_sensor_provenance_and_calibration_are_never_claimed(self):
        source = SCRIPT.read_text()
        self.assertIn('outcome["physical_source_authenticated"] is not False', source)
        self.assertIn('outcome["physical_camera_imu_offset_validated"] is not False', source)
        self.assertIn("STAGE4C_PHYSICAL_CALIBRATION_VALIDATED=FALSE", source)
        self.assertIn("STAGE4C_LIVE_CAPTURE_AUTHORIZED=FALSE", source)
        self.assertIn("STAGE4C_SYNTHETIC_DDS_BAG_INTEGRATION=PASS_NO_DEVICE", source)


if __name__ == "__main__":
    unittest.main()
