"""M23J pure-Python timing regression fixtures; no ROS or devices.

The REAL first bag is NEVER read by unit tests. Every example is an
in-memory nanosecond sequence deliberately unrelated to physical data.
"""
from pathlib import Path
import unittest
from unittest import mock

from m23j_real_bag_timeline import (
    _buckets, _internal_count, _quantiles_ns, MAX_MESSAGES_PER_STREAM,
    MAX_TOP_GAPS, Timeline,
)

N = 1_800_000_000_000_000_000
T = 20_000_000


def healthy():
    t = Timeline()
    for i in range(20):
        at = N + 20_000_000 + 40_000_000 * i
        t.ingest("image", at, at + 1_000_000)
        t.ingest("info", at, at + 2_000_000)
    for i in range(43):
        at = N + i * T
        t.ingest("imu", at, at + 1_000_000)
    return t


class M23JTimelineContract(unittest.TestCase):
    def test_healthy_input_is_timeline_only_not_vio(self):
        report = healthy().report()
        self.assertEqual(report["verdict"], "TIMELINE_DIAGNOSTIC_ONLY")
        self.assertTrue(report["imu_covers_entire_image_header_window"])
        self.assertEqual(report["image_header_gaps_over_250ms"], 0)
        for flag in ("physical_exposure_to_imu_phase_calibrated",
                     "extrinsics_validated", "xrslam_tracking_validated",
                     "device_access", "physical_motion_authorized"):
            self.assertIs(report[flag], False)

    def test_late_first_imu_is_boundary_blocker(self):
        t = healthy()
        t.headers["imu"] = [x + 40_000_000 for x in t.headers["imu"]]
        report = t.report()
        self.assertEqual(report["first_imu_minus_first_image_ms"], 20.0)
        self.assertIn("BLOCKED_NO_IMU_STAMP_COVERAGE", report["faults"])
        self.assertEqual(report["verdict"], "BLOCKED_IMAGE_TIMING_OR_IMU_COVERAGE")

    def test_early_last_imu_is_boundary_blocker(self):
        t = healthy()
        t.headers["imu"] = t.headers["imu"][:38]
        report = t.report()
        self.assertEqual(report["last_image_minus_last_imu_ms"], 40.0)
        self.assertIn("BLOCKED_NO_IMU_STAMP_COVERAGE", report["faults"])

    def test_three_second_gap_still_blocks(self):
        t = healthy()
        t.headers["image"][12] += 3_000_000_000
        # After this image, headers become nonmonotonic, also blocked.
        report = t.report()
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", report["faults"])
        self.assertGreaterEqual(report["image_header_intervals_ms"]["max_ms"],
                                3000)
        self.assertFalse(report["device_access"])
        self.assertLessEqual(len(report["largest_image_header_gaps"]),
                             MAX_TOP_GAPS)

    def test_gap_reports_simultaneous_info_and_imu_stamps(self):
        t = Timeline()
        for stamp in (N + 100_000_000, N + 3_100_000_000):
            t.ingest("image", stamp, stamp + 1_000_000)
        for i in range(160):
            at = N + i * T
            t.ingest("info", at, at + 2_000_000)
            t.ingest("imu", at, at + 3_000_000)
        r = t.report()
        gap = r["largest_image_header_gaps"][0]
        self.assertEqual(gap["image_header_gap_ms"], 3000.0)
        self.assertEqual(gap["camera_info_stamps_inside_gap"], 149)
        self.assertEqual(gap["imu_stamps_inside_gap"], 149)
        self.assertIn("BLOCKED_IMAGE_STAMP_GAP", r["faults"])

    def test_receipt_gap_may_differ_from_header_gap(self):
        t = Timeline()
        for i, delta in enumerate((0, 40_000_000, 1_040_000_000)):
            t.ingest("image", N + delta, N + 10_000_000 * i)
            t.ingest("info", N + delta, N + 10_000_000 * i)
        for i in range(55):
            t.ingest("imu", N + i * T, N + i * T)
        result = t.report()
        gap = result["largest_image_header_gaps"][0]
        self.assertEqual(gap["image_header_gap_ms"], 1000.0)
        self.assertEqual(gap["image_bag_receipt_gap_ms"], 10.0)
        self.assertEqual(result["image_header_gap_buckets"]["500_to_1000ms"], 1)

    def test_more_than_250ms_counts_exactly(self):
        data = [50_000_000, 100_000_000, 250_000_000,
                250_000_001, 500_000_001, 1_000_000_001]
        b = _buckets(data)
        self.assertEqual(b, {
            "up_to_50ms": 1, "50_to_100ms": 1,
            "100_to_250ms": 1, "250_to_500ms": 1,
            "500_to_1000ms": 1, "over_1000ms": 1,
        })

    def test_quantiles_of_small_distribution_deterministic(self):
        q = _quantiles_ns([3_000_000, 1_000_000, 2_000_000])
        self.assertEqual(q["p50_ms"], 2.0)
        self.assertEqual(q["max_ms"], 3.0)
        self.assertIsNone(_quantiles_ns([]))

    def test_internal_gaps_exclude_boundary_equal_stamps(self):
        events = [100, 150, 199, 200, 201, 400]
        self.assertEqual(_internal_count(events, 100, 200), 2)

    def test_missing_stream_is_blocker(self):
        t = Timeline()
        t.ingest("image", N + T, N + T)
        t.ingest("info", N + T, N + T)
        r = t.report()
        self.assertIn("MISSING_REQUIRED_SENSOR_STREAM", r["faults"])
        self.assertIn("BLOCKED_NO_IMU_STAMP_COVERAGE", r["faults"])

    def test_nonmonotonic_image_receipt_is_diagnostic_fault(self):
        t = Timeline()
        t.ingest("image", N + T, N + T)
        t.ingest("image", N + 2*T, N + T)
        t.ingest("info", N + T, N + T)
        t.ingest("imu", N + T, N + T)
        r = t.report()
        self.assertIn("NONMONOTONIC_IMAGE_BAG_RECEIPT", r["faults"])

    def test_reject_unbounded_samples_and_bad_timestamps(self):
        t = Timeline()
        for a, b in ((0, 1), (1, -1), (True, 1), (3.0, 2)):
            with self.subTest(a=a, b=b):
                with self.assertRaises(ValueError):
                    t.ingest("image", a, b)
        with mock.patch("m23j_real_bag_timeline.MAX_MESSAGES_PER_STREAM", 2):
            t.ingest("image", 1, 2)
            t.ingest("image", 2, 3)
            with self.assertRaises(ValueError):
                t.ingest("image", 3, 4)

    def test_no_hardware_or_ros_publisher_in_source(self):
        source = (Path(__file__).resolve().parent /
                  "m23j_real_bag_timeline.py").read_text()
        for forbidden in ("import serial", "rclpy.init(",
                          "serial.Serial(", "create_publisher(",
                          "subprocess.Popen(", "set_wheels "):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M23JTimelineContract))
    if result.wasSuccessful():
        print("M23J_READ_ONLY_BAG_TIMELINE_CONTRACT=PASS", flush=True)
        print("M23J_250MS_CAMERA_GATE_UNCHANGED=PASS", flush=True)
        print("M23J_REAL_BAG_REPLAY_VALIDATED=FALSE", flush=True)
        print("M23J_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
