"""M21J pure-Python tests: no ROS, camera, serial or Podman dependencies."""
import math
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from m21d_metadata_capture import MetadataCapture
from m21j_offline_capacity_stop import admission, finalize_after_stop


def fixture():
    capture = MetadataCapture("SYNTHETIC", max_events=100)
    for i in range(3):
        stamp = 1_800_000_000_000_000_000 + 100_000_000 * i
        receipt = 5_000_000_000 + 100_000_000 * i
        capture.image(frame_index=i, stamp_ns=stamp,
                      receipt_mono_ns=receipt, frame_id="camera_optical_frame",
                      width=640, height=480)
        capture.camera_info(stamp_ns=stamp + 1000,
                            receipt_mono_ns=receipt + 1000,
                            frame_id="camera_optical_frame", width=640, height=480)
        capture.imu_source(mcu_seq=i, mcu_t_us=100_000 * i,
                           stamp_ns=stamp + 2000, receipt_mono_ns=receipt + 2000,
                           gyro_xyz_rad_s=[0.0, 0.0, 0.1])
    return capture


class CapacityAndShutdownTests(unittest.TestCase):
    def test_120s_30_30_50_hz_budget_admitted_at_20k(self):
        r = admission()
        self.assertEqual(r["projected_event_count"], 13203)
        self.assertEqual(r["required_event_capacity"], 15844)
        self.assertEqual(r["remaining_reserved_events"], 4156)
        self.assertTrue(r["capacity_ready"])
        self.assertFalse(r["physical_camera_imu_offset_validated"])

    def test_prior_12k_cap_rejected(self):
        r = admission(capacity=12000)
        self.assertFalse(r["capacity_ready"])
        self.assertEqual(r["verdict"], "CAPACITY_ADMISSION_REJECTED")
        self.assertLess(r["remaining_reserved_events"], 0)

    def test_excessive_frequencies_rejected(self):
        r = admission(image_hz=50, camera_info_hz=50, imu_hz=50)
        self.assertFalse(r["capacity_ready"])

    def test_invalid_budget_inputs_rejected(self):
        bad = [
            {"duration_s": 0},
            {"duration_s": 121},
            {"image_hz": float("nan")},
            {"imu_hz": False},
            {"camera_info_hz": float("inf")},
            {"reserve_pct": 0},
            {"reserve_pct": 101},
            {"capacity": 20001},
            {"capacity": True},
        ]
        for kwargs in bad:
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                admission(**kwargs)

    def test_stop_precedes_file_serialization(self):
        capture = fixture()
        order = []
        real_finish = capture.finish
        def traced_finish(path):
            order.append("finish")
            return real_finish(path)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "synthetic.jsonl"
            with mock.patch.object(capture, "finish", side_effect=traced_finish):
                report = finalize_after_stop(
                    capture, path, lambda: (order.append("stop"), True)[1])
            self.assertEqual(order, ["stop", "finish"])
            self.assertEqual(report["verdict"],
                             "SIMULATED_POST_STOP_FINALIZATION_READY")
            self.assertEqual(report["saved_event_count"], 9)
            self.assertTrue(path.exists())

    def test_missing_stop_ack_preserves_evidence_but_rejects_acceptance(self):
        capture = fixture()
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "evidence.jsonl"
            report = finalize_after_stop(capture, path, lambda: False)
            self.assertTrue(path.exists())
            self.assertTrue(report["metadata_saved"])
            self.assertFalse(report["stop_acknowledged"])
            self.assertEqual(report["verdict"], "UNVERIFIED_STOP_OR_METADATA_CAPTURE")

    def test_stop_exception_preserves_evidence_but_rejects_acceptance(self):
        capture = fixture()
        def failed_stop():
            raise IOError("synthetic STOP transport failure")
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "evidence.jsonl"
            report = finalize_after_stop(capture, path, failed_stop)
            self.assertFalse(report["stop_acknowledged"])
            self.assertIn("synthetic STOP", report["stop_error"])
            self.assertTrue(report["metadata_saved"])
            self.assertEqual(report["verdict"], "UNVERIFIED_STOP_OR_METADATA_CAPTURE")

    def test_nonboolean_ack_fails_closed(self):
        capture = fixture()
        with tempfile.TemporaryDirectory() as folder:
            report = finalize_after_stop(capture, Path(folder) / "evidence.jsonl",
                                         lambda: "PASS")
            self.assertFalse(report["stop_acknowledged"])
            self.assertIn("ValueError", report["stop_error"])
            self.assertEqual(report["verdict"], "UNVERIFIED_STOP_OR_METADATA_CAPTURE")

    def test_capture_persistence_failure_does_not_skip_stop(self):
        capture = fixture()
        calls = []
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "existing.jsonl"
            path.write_text("DO NOT OVERWRITE")
            report = finalize_after_stop(capture, path,
                                         lambda: (calls.append("stop"), True)[1])
            self.assertEqual(calls, ["stop"])
            self.assertEqual(path.read_text(), "DO NOT OVERWRITE")
            self.assertFalse(report["metadata_saved"])
            self.assertIn("FileExistsError", report["capture_error"])
            self.assertEqual(report["verdict"], "UNVERIFIED_STOP_OR_METADATA_CAPTURE")

    def test_invalid_recorder_or_stop_callback_rejected_preflight(self):
        capture = fixture()
        with self.assertRaises(TypeError):
            finalize_after_stop(None, "not-created", lambda: True)
        with self.assertRaises(TypeError):
            finalize_after_stop(capture, "not-created", None)


if __name__ == "__main__":
    unittest.main()
