"""M21D pure Python tests; never start ROS, Podman, camera or STM32 serial."""
import ast
import json
from pathlib import Path
import tempfile
import unittest

from m21d_metadata_capture import CaptureError, MetadataCapture
from m21_record_contract import load


def example(capture, count=11, gap_ns=100000000, skip_info=False):
    for k in range(count):
        stamp = 1_900_000_000_000_000_000 + k * gap_ns
        receipt = 2_000_000_000 + k * gap_ns
        capture.image(frame_index=k, stamp_ns=stamp,
                      receipt_mono_ns=receipt,
                      frame_id="camera_optical_frame", width=640, height=480)
        if not skip_info:
            capture.camera_info(stamp_ns=stamp + 1000,
                                receipt_mono_ns=receipt + 1000,
                                frame_id="camera_optical_frame", width=640, height=480)
        capture.imu_source(mcu_seq=k, mcu_t_us=k * 100000,
                           stamp_ns=stamp + 2000,
                           receipt_mono_ns=receipt + 2000,
                           gyro_xyz_rad_s=[0.1, 0.2, -0.4])


class RecorderTests(unittest.TestCase):
    def test_roundtrip_exact_metadata_without_pixels(self):
        capture = MetadataCapture()
        example(capture)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "new.jsonl"
            report = capture.finish(out)
            self.assertEqual(report["verdict"], "M21C_OFFLINE_METADATA_FORMAT_VALID")
            self.assertEqual(report["recorder_event_count"], 33)
            self.assertEqual(len(load(out)), 34)
            self.assertFalse(report["physical_camera_imu_offset_validated"])
            self.assertFalse(report["recorder_device_access"])
            self.assertNotIn("pixels", out.read_text())
            self.assertNotIn("data", json.dumps(load(out)))

    def test_do_not_overwrite(self):
        capture = MetadataCapture()
        example(capture)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "existing.jsonl"
            out.write_text("KEEP")
            with self.assertRaises(FileExistsError):
                capture.finish(out)
            self.assertEqual(out.read_text(), "KEEP")

    def test_capture_capacity_fail_closed(self):
        capture = MetadataCapture(max_events=6)
        example(capture, count=2)
        with self.assertRaises(CaptureError):
            capture.image(frame_index=2, stamp_ns=19999999999,
                          receipt_mono_ns=9999999999,
                          frame_id="camera_optical_frame", width=640, height=480)

    def test_missing_stream_does_not_emit_file(self):
        capture = MetadataCapture()
        example(capture, skip_info=True)
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "should_not_exist.jsonl"
            with self.assertRaises(ValueError):
                capture.finish(out)
            self.assertFalse(out.exists())

    def test_wrong_publisher_rejected(self):
        capture = MetadataCapture()
        with self.assertRaises(CaptureError):
            capture.image(frame_index=0, stamp_ns=2000000000, receipt_mono_ns=2000000000,
                          frame_id="camera_optical_frame", width=640,
                          height=480, publisher="/leroy_bridge")

    def test_wrong_frame_rejected(self):
        capture = MetadataCapture()
        with self.assertRaises(CaptureError):
            capture.camera_info(stamp_ns=2000000000, receipt_mono_ns=2000000000,
                                frame_id="base_link", width=640, height=480)

    def test_invalid_ros_stamp_rejected(self):
        with self.assertRaises(ValueError):
            MetadataCapture.stamp_ns(-1, 0)
        with self.assertRaises(ValueError):
            MetadataCapture.stamp_ns(100, 1000000000)
        with self.assertRaises(ValueError):
            MetadataCapture.stamp_ns(True, 0)

    def test_duplicate_mcu_sequence_rejected_at_finish(self):
        capture = MetadataCapture()
        example(capture)
        capture.rows[-1]["mcu_seq"] = capture.rows[-4]["mcu_seq"]
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp) / "reject.jsonl"
            with self.assertRaises(ValueError): capture.finish(out)
            self.assertFalse(out.exists())

    def test_uint32_rollover_preserved(self):
        capture = MetadataCapture()
        for k in range(11):
            stamp = 1900000000000000000 + k * 100000000
            receipt = 2000000000 + k * 100000000
            capture.image(frame_index=k, stamp_ns=stamp,
                          receipt_mono_ns=receipt,
                          frame_id="camera_optical_frame", width=640, height=480)
            capture.camera_info(stamp_ns=stamp + 1000,
                                receipt_mono_ns=receipt + 1000,
                                frame_id="camera_optical_frame", width=640, height=480)
            capture.imu_source(mcu_seq=((1 << 32) - 5 + k) % (1 << 32),
                               mcu_t_us=((1 << 32) - 300000 + k*100000) % (1 << 32),
                               stamp_ns=stamp + 2000, receipt_mono_ns=receipt + 2000,
                               gyro_xyz_rad_s=[0.0, 0.0, 0.1])
        with tempfile.TemporaryDirectory() as tmp:
            report=capture.finish(Path(tmp)/"wrap.jsonl")
            self.assertEqual(report["stream_diagnostics"]["imu"]["sequence_missing_samples"],0)

    def test_rejects_nan_gyro_at_finish(self):
        capture = MetadataCapture()
        example(capture)
        capture.rows[-1]["gyro_xyz_rad_s"][0] = float("nan")
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(ValueError): capture.finish(Path(tmp)/"nan.jsonl")

    def test_closed_capture_rejects_more_samples_and_finish(self):
        capture = MetadataCapture()
        example(capture)
        with tempfile.TemporaryDirectory() as tmp:
            capture.finish(Path(tmp)/"close.jsonl")
            with self.assertRaises(CaptureError): capture.finish(Path(tmp)/"twice.jsonl")
            with self.assertRaises(CaptureError):
                capture.camera_info(stamp_ns=5000000000, receipt_mono_ns=5000000000,
                                    frame_id="camera_optical_frame", width=640, height=480)

    def test_adapter_has_no_automatic_runtime_entrypoint(self):
        code=(Path(__file__).parent/"m21d_passive_ros_adapter.py").read_text()
        parsed=ast.parse(code)
        self.assertFalse(any(isinstance(n, ast.If) and
                             isinstance(n.test, ast.Compare) and
                             any(isinstance(x, ast.Constant) and x.value == "__main__"
                                 for x in n.test.comparators)
                             for n in ast.walk(parsed)))
        self.assertFalse("serial.Serial(" in code or "rclpy.init(" in code)
        self.assertIn("class PassiveCameraMetadataNode", code)


if __name__ == "__main__":
    unittest.main()
