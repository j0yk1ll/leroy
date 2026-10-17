"""M21I offline 120s workload tests; synthetic metadata only, no devices."""
from pathlib import Path
import tempfile
import unittest

from m21_record_contract import load
from m21d_metadata_capture import CaptureError, MetadataCapture
from m21i_full_window_synthetic_replay import (
    TOTAL, CAM_IMAGES, CAM_INFO, IMU_SAMPLES, assess, fill, camera_times,
)


class FullWindowTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.path = Path(cls.temp.name) / "m21i_synthetic.jsonl"
        capture = MetadataCapture("SYNTHETIC", max_events=12000)
        fill(capture)
        cls.report = assess(capture.finish(cls.path))

    @classmethod
    def tearDownClass(cls):
        cls.temp.cleanup()

    def test_expected_m20_message_volume(self):
        self.assertEqual(self.report["recorder_event_count"], 11661)
        self.assertEqual(self.report["recorder_event_count"], TOTAL)
        streams = self.report["stream_diagnostics"]
        self.assertEqual(streams["camera_image"]["messages"], CAM_IMAGES)
        self.assertEqual(streams["camera_info"]["messages"], CAM_INFO)
        self.assertEqual(streams["imu"]["messages"], IMU_SAMPLES)

    def test_expected_large_frame_gap_and_receipt_overlap(self):
        self.assertEqual(self.report["receipt_overlap_s"], 119.886)
        self.assertAlmostEqual(self.report["stream_diagnostics"]["camera_image"]
                               ["max_receive_gap_s"], 1.351, places=8)
        self.assertTrue(self.report["large_camera_gap_over_250ms"])
        times = list(camera_times())
        self.assertEqual(len(times), CAM_IMAGES)
        self.assertEqual(times[-1] - times[0], 119_886_000_000)
        self.assertEqual(max(b - a for a, b in zip(times, times[1:])),
                         1_351_000_000)

    def test_uint32_mcu_wrap_without_sequence_loss(self):
        self.assertEqual(self.report["stream_diagnostics"]["imu"]
                         ["sequence_missing_samples"], 0)
        rows = load(self.path)
        imu = [row for row in rows if row["kind"] == "imu"]
        self.assertEqual(len(imu), IMU_SAMPLES)
        self.assertTrue(any(b["mcu_t_us"] < a["mcu_t_us"]
                            for a, b in zip(imu, imu[1:])))

    def test_capture_file_is_bounded_and_contains_no_pixels(self):
        self.assertLess(self.path.stat().st_size, 8 * 1024 * 1024)
        self.assertEqual(self.report["record_file_bytes"],
                         self.path.stat().st_size)
        content = self.path.read_text()
        self.assertNotIn('"data":', content)
        self.assertNotIn('"pixels":', content)

    def test_synthetic_provenance_never_calibrated(self):
        self.assertEqual(self.report["evidence_type"], "SYNTHETIC")
        self.assertFalse(self.report["physical_camera_imu_offset_validated"])
        self.assertIsNone(self.report["physical_camera_imu_offset_s"])
        self.assertFalse(self.report["camera_imu_extrinsics_validated"])
        self.assertFalse(self.report["recorder_device_access"])

    def test_existing_capture_cannot_be_overwritten(self):
        # Existing paths must stay intact even for a COMPLETE capture.
        capture = MetadataCapture("SYNTHETIC", max_events=12000)
        fill(capture)
        initial_size = self.path.stat().st_size
        with self.assertRaises(FileExistsError):
            capture.finish(self.path)
        self.assertEqual(self.path.stat().st_size, initial_size)

    def test_capacity_exhaustion_fails_closed(self):
        capture = MetadataCapture("SYNTHETIC", max_events=TOTAL - 1)
        with self.assertRaises(CaptureError):
            fill(capture)
        self.assertEqual(len(capture.rows), TOTAL)  # manifest plus bounded events
        self.assertFalse(capture.closed)


if __name__ == "__main__":
    unittest.main()
