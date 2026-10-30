"""No-device MCU source metadata adapter regression tests; NEVER start ROS."""
import json
from pathlib import Path
import unittest

from live_stage4c_capture_contract import CaptureError, source_metadata
from live_stage4c_source_meta import SourceMetadataTap

HERE = Path(__file__).resolve().parents[2]


class MetadataTapContract(unittest.TestCase):
    def setUp(self):
        self.sent = []
        self.now = [10_000_000]
        def clock():
            self.now[0] += 1_000
            return self.now[0]
        self.tap = SourceMetadataTap(self.sent.append, monotonic_ns=clock)

    def publish(self, seq, ticks, stamp, mapper_steady=5):
        return self.tap.on_imu_published(
            {"seq": seq, "t_us": ticks, "cal": 1}, stamp, mapper_steady)

    def test_exact_original_mcu_fields_and_callback_clock(self):
        raw = self.publish(42, 1_000_000, 1_790_000_000_000_000_000)
        obj = json.loads(raw)
        self.assertEqual(obj["seq"], 42)
        self.assertEqual(obj["t_us"], 1_000_000)
        self.assertEqual(obj["imu_stamp_ns"], 1_790_000_000_000_000_000)
        self.assertEqual(obj["publish_monotonic_ns"], 10_001_000)
        self.assertNotEqual(obj["publish_monotonic_ns"], 5)
        self.assertEqual(obj["publisher_node"], "/m18_imu_source")
        self.assertEqual(source_metadata(self.sent[0]), obj)
        self.assertEqual(self.tap.count, 1)

    def test_sequence_and_mcu_tick_wraps(self):
        self.publish(0xFFFFFFFF, 0xFFFFFF00, 1_000_000_000)
        self.publish(0, 19744, 1_020_000_000)
        self.assertEqual(self.tap.count, 2)
        self.assertEqual(len(self.sent), 2)

    def test_missing_source_fields_and_uncalibrated_packet_are_rejected(self):
        for fields in (
            {}, {"seq": 5, "t_us": 20},
            {"seq": 5, "t_us": 20, "cal": False},
        ):
            tap = SourceMetadataTap(self.sent.append)
            with self.subTest(fields=fields), self.assertRaises(CaptureError):
                tap.on_imu_published(fields, 1, 2)
            self.assertIsNotNone(tap.terminal_fault)
            with self.assertRaisesRegex(CaptureError, "fault_latched"):
                tap.on_imu_published({"seq": 5, "t_us": 20, "cal": 1}, 3, 2)

    def test_sequence_gap_is_fatal_and_not_published(self):
        self.publish(100, 1_000_000, 1_000_000_000)
        with self.assertRaisesRegex(CaptureError, "source_sequence_gap"):
            self.publish(102, 1_040_000, 1_040_000_000)
        self.assertEqual(self.tap.count, 1)
        self.assertEqual(len(self.sent), 1)

    def test_mcu_tick_gap_is_fatal_and_not_published(self):
        self.publish(100, 1_000_000, 1_000_000_000)
        with self.assertRaisesRegex(CaptureError, "source_mcu_tick_discontinuity"):
            self.publish(101, 3_000_001, 1_020_000_000)
        self.assertEqual(len(self.sent), 1)

    def test_ros_stamp_regression_and_clock_regression_are_fatal(self):
        self.publish(100, 1_000_000, 1_000_000_000)
        with self.assertRaisesRegex(CaptureError, "nonmonotonic_source_timestamp"):
            self.publish(101, 1_020_000, 1_000_000_000)
        now = [1000, 999]
        tap = SourceMetadataTap(
            self.sent.append, monotonic_ns=lambda: now.pop(0))
        tap.on_imu_published({"seq": 200, "t_us": 100, "cal": 1}, 100, 50)
        with self.assertRaisesRegex(CaptureError, "nonmonotonic_source_timestamp"):
            tap.on_imu_published({"seq": 201, "t_us": 20_100, "cal": 1}, 200, 50)

    def test_sender_failure_latches_without_committing(self):
        def fail(_data):
            raise OSError("metadata_publish_failed")
        tap = SourceMetadataTap(fail)
        with self.assertRaisesRegex(OSError, "metadata_publish_failed"):
            tap.on_imu_published(
                {"seq": 1, "t_us": 20_000, "cal": 1}, 100, 42)
        self.assertEqual(tap.count, 0)
        self.assertIsNotNone(tap.terminal_fault)
        with self.assertRaisesRegex(CaptureError, "fault_latched"):
            tap.on_imu_published(
                {"seq": 2, "t_us": 40_000, "cal": 1}, 200, 42)

    def test_metadata_tap_is_opt_in(self):
        adapter = (HERE / "live_stage4c_source_meta.py").read_text()
        runner = (HERE / "live_stage4_imu_overlap.py").read_text()
        m18 = (HERE / "src/leroy_imu_observer/leroy_imu_observer/imu_source.py").read_text()
        self.assertIn("live_stage4c_source_meta_ros", runner)
        self.assertIn("metadata_adapter.on_imu_published(fields, stamp, steady)", runner)
        self.assertIn("source_metadata_requires_capture_only_session", runner)
        self.assertNotIn("live_stage4c_source_meta", m18)
        self.assertNotIn("serial.Serial(", adapter)
        self.assertNotIn("device.write(", adapter)
        self.assertNotIn("create_publisher(", adapter)
        self.assertNotIn("rclpy", adapter)
        self.assertNotIn("from std_msgs", adapter)
        self.assertNotIn("import std_msgs", adapter)
        self.assertIn("def on_imu_published(", adapter)
        self.assertIn('self._send(serialized)', adapter)


if __name__ == "__main__":
    unittest.main()
