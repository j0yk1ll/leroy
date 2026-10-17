"""Offline-only M21C metadata contract regression tests."""
from contextlib import redirect_stdout
from io import StringIO
import json
from pathlib import Path
import tempfile
import unittest

from m21_record_contract import load, summarize, main


def fixture():
    rows = [{"kind": "session", "schema": "leroy_m21c_v1", "evidence_type": "SYNTHETIC"}]
    for i in range(11):
        t = 1_000_000_000 + i * 100_000_000
        rows.extend([
            {"kind": "camera_image", "frame_index": i, "width": 640, "height": 480,
             "receipt_mono_ns": t, "header_ros_ns": t + 40_000_000,
             "frame_id": "camera_optical_frame", "publisher": "/camera/rectify_node"},
            {"kind": "camera_info", "width": 640, "height": 480,
             "receipt_mono_ns": t + 1000, "header_ros_ns": t + 41_000_000,
             "frame_id": "camera_optical_frame", "publisher": "/camera/usb_cam"},
            {"kind": "imu", "mcu_seq": i, "mcu_t_us": i * 100_000,
             "gyro_xyz_rad_s": [0.1, 0.0, -0.1],
             "receipt_mono_ns": t + 2000, "header_ros_ns": t + 42_000_000,
             "frame_id": "imu_link", "publisher": "/m18_imu_source"},
        ])
    return rows


class ContractTests(unittest.TestCase):
    def test_valid_synthetic_format_is_never_physical_calibration(self):
        report = summarize(fixture())
        self.assertEqual(report["verdict"], "M21C_OFFLINE_METADATA_FORMAT_VALID")
        self.assertFalse(report["physical_camera_imu_offset_validated"])
        self.assertIsNone(report["physical_camera_imu_offset_s"])
        self.assertEqual(report["stream_diagnostics"]["imu"]["messages"], 11)

    def test_unexpected_pixel_field_rejected(self):
        rows = fixture()
        rows[1]["pixels"] = [1, 2, 3]
        with self.assertRaises(ValueError): summarize(rows)

    def test_bad_provenance_rejected(self):
        rows = fixture()
        rows[1]["publisher"] = "/leroy_bridge"
        with self.assertRaises(ValueError): summarize(rows)

    def test_nonmonotonic_imu_clock_rejected(self):
        rows = fixture()
        rows[-1]["mcu_t_us"] = 0
        with self.assertRaises(ValueError): summarize(rows)

    def test_sequence_gap_recorded(self):
        rows = fixture()
        for row in rows:
            if row.get("kind") == "imu" and row["mcu_seq"] >= 5:
                row["mcu_seq"] += 1
        report = summarize(rows)
        self.assertEqual(report["stream_diagnostics"]["imu"]["sequence_missing_samples"], 1)

    def test_image_receipt_gap_summarized(self):
        rows = fixture()
        for row in rows:
            if row.get("kind") == "camera_image" and row["frame_index"] >= 6:
                row["receipt_mono_ns"] += 1_000_000_000
                row["header_ros_ns"] += 1_000_000_000
        report = summarize(rows)
        self.assertTrue(report["large_camera_gap_over_250ms"])
        self.assertGreater(report["stream_diagnostics"]["camera_image"]["max_receive_gap_s"], 1)

    def test_bad_frame_dimensions_rejected(self):
        rows = fixture()
        rows[1]["width"] = 1920
        with self.assertRaises(ValueError): summarize(rows)

    def test_nonfinite_gyro_rejected(self):
        rows = fixture()
        rows[3]["gyro_xyz_rad_s"][0] = float("nan")
        with self.assertRaises(ValueError): summarize(rows)

    def test_manifest_cannot_claim_calibrated(self):
        rows = fixture()
        rows[0]["evidence_type"] = "CALIBRATED"
        with self.assertRaises(ValueError): summarize(rows)

    def test_mcu_sequence_and_time_uint32_wrap(self):
        rows = fixture()
        for row in rows:
            if row.get("kind") == "imu":
                row["mcu_seq"] = (row["mcu_seq"] + (1 << 32) - 5) % (1 << 32)
                row["mcu_t_us"] = (row["mcu_t_us"] + (1 << 32) - 300_000) % (1 << 32)
        self.assertEqual(summarize(rows)["stream_diagnostics"]["imu"]["sequence_missing_samples"], 0)

    def test_duplicate_frame_index_rejected(self):
        rows = fixture()
        rows[4]["frame_index"] = 0
        with self.assertRaises(ValueError): summarize(rows)

    def test_cli_read_only_round_trip(self):
        rows = fixture()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "synthetic.jsonl"
            path.write_text("".join(json.dumps(x) + "\n" for x in rows))
            initial = path.read_bytes()
            self.assertEqual(load(path), rows)
            captured = StringIO()
            with redirect_stdout(captured):
                self.assertEqual(main([str(path)]), 0)
            self.assertEqual(captured.getvalue().count("M21C_FORMAT_REPORT_JSON="), 1)
            self.assertEqual(path.read_bytes(), initial)


if __name__ == "__main__": unittest.main()
