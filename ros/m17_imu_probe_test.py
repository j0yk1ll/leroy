"""M17 offline test for exact non-actuating MCU request and parser."""
import ast
from pathlib import Path
import unittest

import m17_imu_query_probe as probe


class M17QueryContract(unittest.TestCase):
    def test_known_firmware_response(self):
        line = (
            "imu_robot=ok calibrated=1 accel_forward_mps2=0.000 "
            "accel_left_mps2=0.000 accel_up_mps2=9.807 "
            "gyro_roll_radps=0.001 gyro_pitch_radps=0.002 "
            "gyro_yaw_radps=-0.003")
        calibrated, values = probe.parse_imu_response(line)
        self.assertTrue(calibrated)
        self.assertEqual(len(values), 6)
        self.assertEqual(values[-1], -0.003)

    def test_reject_nonfinite_or_extra_fields(self):
        valid = (
            "imu_robot=ok calibrated=1 accel_forward_mps2=0.000 "
            "accel_left_mps2=0.000 accel_up_mps2=9.807 "
            "gyro_roll_radps=0.001 gyro_pitch_radps=0.002 "
            "gyro_yaw_radps=0.003")
        for corrupted in (
            valid.replace("9.807", "inf"),
            valid.replace("gyro_yaw_radps=0.003", ""),
            valid + " set_wheels=50",
            valid.replace("calibrated=1", "calibrated=false"),
            "imu_robot=unavailable",
            "telemetry imu seq=1",
        ):
            with self.subTest(corrupted=corrupted):
                with self.assertRaises(ValueError):
                    probe.parse_imu_response(corrupted)

    def test_exactly_one_serial_write_call_and_fixed_payload(self):
        source = Path(probe.__file__).read_text()
        tree = ast.parse(source)
        writes = [
            node for node in ast.walk(tree)
            if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "write"
        ]
        self.assertEqual(len(writes), 1)
        self.assertIsInstance(writes[0].args[0], ast.Name)
        self.assertEqual(writes[0].args[0].id, "QUERY")
        self.assertEqual(probe.QUERY, b"get_imu_robot\r\n")
        self.assertEqual(probe.PORT, "/dev/ttyACM0")
        self.assertEqual(probe.POLL_PERIOD_S, 0.5)
        self.assertNotIn("rclpy", source)
        self.assertNotIn("cmd_vel", source)
        self.assertNotIn("serial.Serial(port=", source)  # fixed named input
        self.assertNotIn("subprocess", source)


if __name__ == "__main__":
    unittest.main()
