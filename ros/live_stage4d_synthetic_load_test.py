"""Offline static safety and acceptance tests for Stage 4D paced DDS load."""
import ast
from pathlib import Path
import unittest

from live_stage4c_capture_contract import CaptureError
from live_stage4c_synthetic_dds_bag import check_isolation
from live_stage4d_synthetic_load import IMU_COUNT, CAMERA_COUNT, MAX_DATABASE_BYTES

ROOT = Path(__file__).resolve().parent


class Stage4DGraphGuards(unittest.TestCase):
    def test_exact_paced_sensor_counts_and_storage_cap(self):
        self.assertEqual(IMU_COUNT, 250)
        self.assertEqual(CAMERA_COUNT, 150)
        self.assertLessEqual(MAX_DATABASE_BYTES, 96_000_000)
        self.assertEqual(sum(i % 5 in (0, 2, 4) for i in range(IMU_COUNT)),
                         CAMERA_COUNT)

    def test_synthetic_no_device_gate_inherits_stage4c_contract(self):
        env = {
            "LEROY_STAGE4C_SYNTHETIC_ONLY": "1",
            "ROS_DOMAIN_ID": "171",
            "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
            "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        }
        self.assertTrue(check_isolation(env, exists=lambda _: False))
        with self.assertRaises(CaptureError):
            check_isolation(env, exists=lambda _: True)

    def test_no_robot_or_live_recorder_imports(self):
        s = (ROOT / "live_stage4d_synthetic_load.py").read_text()
        tree = ast.parse(s)
        imports = [alias.name for node in ast.walk(tree)
                   if isinstance(node, ast.Import) for alias in node.names]
        for forbidden in ("serial", "subprocess", "cv2"):
            self.assertNotIn(forbidden, imports)
        for token in (
            "serial.Serial(", "device.write(", "/dev/serial",
            "create_publisher(Twist", "ImuStreamSource(", "ros2 launch",
            "live_stage4c_passive_recorder", "LEROY_STAGE4_MOTOR",
        ):
            self.assertNotIn(token, s)
        self.assertIn("CaptureIngress(", s)
        self.assertIn("SourceMetadataRosAdapter(source)", s)
        self.assertIn("TemporaryDirectory(", s)
        self.assertIn("STAGE4D_LIVE_CAPTURE_AUTHORIZED=FALSE", s)
        self.assertIn("STAGE4D_PHYSICAL_CALIBRATION_VALIDATED=FALSE", s)

    def test_hard_live_gate_unchanged(self):
        r = (ROOT / "live_stage4c_passive_recorder.py").read_text()
        self.assertIn("LIVE_CAPTURE_COMMISSIONED = False", r)
        self.assertIn("STAGE4C=BLOCKED_LIVE_CAPTURE_NOT_COMMISSIONED", r)
        self.assertNotIn("serial.Serial(", r)
        self.assertNotIn("create_publisher(", r)


if __name__ == "__main__":
    unittest.main()
