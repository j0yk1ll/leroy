"""M18 ROS-free serial contract/command allowlist tests."""
import ast
from pathlib import Path
import unittest

from leroy_imu_observer.imu_protocol import (
    START, STOP, PORT, STREAM_HZ, PacketError, parse_packet, sequence_gap,
)


GOOD = (
    "telemetry imu seq=42 t_us=2400000 cal=1 "
    "af_mm_s2=10 al_mm_s2=-20 au_mm_s2=9807 "
    "gr_mrad_s=1 gp_mrad_s=-2 gy_mrad_s=3 "
    "mag_valid=0 mx_raw=0 my_raw=0 mz_raw=0"
)


class M18OfflineSafety(unittest.TestCase):
    def test_real_firmware_layout(self):
        sample = parse_packet(GOOD)
        self.assertEqual(sample["seq"], 42)
        self.assertEqual(sample["t_us"], 2400000)
        self.assertEqual(sample["au_mm_s2"], 9807)
        self.assertEqual(sample["gy_mrad_s"], 3)
        self.assertIsNone(parse_packet("ok set_imu_stream_hz"))

    def test_malformed_or_nonfinite_like_inputs_rejected(self):
        for bad in (
            GOOD.replace("cal=1", "cal=2"),
            GOOD.replace("au_mm_s2=9807", "au_mm_s2=nan"),
            GOOD.replace("seq=42", "seq=-1"),
            GOOD.replace("t_us=2400000", "t_us=4294967296"),
            GOOD.replace("gr_mrad_s=1", "gr_mrad_s=1.0"),
            GOOD + " set_wheels=100",
            GOOD.replace("mag_valid=0", "mag_valid=0 mag_valid=1"),
            GOOD.replace(" mx_raw=0", ""),
            GOOD.replace("seq=42", "seq=42.4"),
        ):
            with self.subTest(packet=bad):
                with self.assertRaises(PacketError):
                    parse_packet(bad)

    def test_sequence_rollover_missing_or_reordered(self):
        self.assertEqual(sequence_gap(None, 0), 0)
        self.assertEqual(sequence_gap(0xffffffff, 0), 0)
        self.assertEqual(sequence_gap(10, 13), 2)
        with self.assertRaises(PacketError):
            sequence_gap(10, 10)
        with self.assertRaises(PacketError):
            sequence_gap(10, 9)

    def test_exact_two_nonactuating_command_bytes_and_one_write_site(self):
        self.assertEqual(START, b"set_imu_stream_hz 50\r\n")
        self.assertEqual(STOP, b"set_imu_stream_hz 0\r\n")
        self.assertEqual(PORT, "/dev/ttyACM0")
        self.assertEqual(STREAM_HZ, 50)
        source = (Path(__file__).resolve().parents[2] / "src/leroy_imu_observer/leroy_imu_observer/imu_source.py").read_text()
        tree = ast.parse(source)
        writes = [node for node in ast.walk(tree)
                  if isinstance(node, ast.Call)
                  and isinstance(node.func, ast.Attribute)
                  and node.func.attr == "write"]
        self.assertEqual(len(writes), 1)
        self.assertIsInstance(writes[0].args[0], ast.Name)
        self.assertEqual(writes[0].args[0].id, "payload")
        self.assertIn("if payload not in (START, STOP)", source)
        self.assertIn("self._imu_pub = self.create_publisher(", source)
        self.assertNotIn("create_subscription(", source)
        self.assertNotIn("create_client(", source)
        self.assertNotIn("set_wheels", source)
        self.assertNotIn("keepalive", source)
        self.assertNotIn("leroy_bridge.node", source)
        self.assertNotIn("enable_map_tf", source)
        self.assertNotIn("cmd_vel", source)


if __name__ == "__main__":
    unittest.main()
