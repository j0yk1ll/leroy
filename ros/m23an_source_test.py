"""M23AN synthetic fixture offline receipt-analysis tests. No ROS or Podman."""
import unittest
from unittest.mock import patch

import m23an_synthetic_timing as timing


def fixture(
        frames=900, nanos_per_frame=33_333_333,
        raw_to_rect_delta_ns=12_000_000,
        with_publisher=True):
    lines = []
    for i in range(frames):
        stamp = (i+1)*nanos_per_frame
        recv = 20_000_000_000 + i*nanos_per_frame
        lines += [
            f"M23K_SAMPLE raw {stamp} {recv}",
            f"M23K_SAMPLE rect {stamp} {recv+raw_to_rect_delta_ns}",
            f"M23K_SAMPLE info {stamp} {recv+2_000_000}",
        ]
    pub = "M23P_PUBLISHER_PAYLOAD_BYTES=921600\n" if with_publisher else ""
    return "\n".join(lines), pub


class TimingTests(unittest.TestCase):
    def test_30fps_ninety_hundred_complete(self):
        obs, pub = fixture()
        result = timing.analyze(obs, pub)
        self.assertEqual(result["counts"], {"raw":900,"rect":900,"info":900})
        self.assertEqual(result["paired_header_stamps"], 900)
        self.assertAlmostEqual(result["observed_rectified_receipt_fps"], 30, places=2)
        self.assertEqual(result["rect_receipt_gaps_over_250ms"], 0)
        self.assertEqual(result["raw_to_rect_receipt_skew_p99_ms"], 12)
        self.assertFalse(result["actual_rectify_processing_latency_measured"])

    def test_under_900_rejected(self):
        obs, pub = fixture(frames=899)
        with self.assertRaisesRegex(timing.Blocked, "not exactly 900"):
            timing.analyze(obs, pub)

    def test_duplicate_header_rejected(self):
        obs, pub = fixture()
        obs += "\n" + obs.splitlines()[0]
        with self.assertRaisesRegex(timing.Blocked, "duplicate"):
            timing.analyze(obs, pub)

    def test_missing_info_stamp_rejected(self):
        obs, pub = fixture()
        obs = obs.replace("M23K_SAMPLE info 33333333", "M23K_SAMPLE info 33333334", 1)
        with self.assertRaisesRegex(timing.Blocked, "header stamps differ"):
            timing.analyze(obs, pub)

    def test_wrong_dimensions_rejected(self):
        obs, pub = fixture()
        with self.assertRaisesRegex(timing.Blocked, "640x480"):
            timing.analyze(obs, "M23P_PUBLISHER_PAYLOAD_BYTES=20000\n")

    def test_malformed_sample_rejected(self):
        obs, pub = fixture()
        with self.assertRaisesRegex(timing.Blocked, "malformed"):
            timing.analyze(obs + "\nM23K_SAMPLE rect abc 100\n", pub)

    def test_negative_observer_skew_reported(self):
        obs, pub = fixture(raw_to_rect_delta_ns=-1_000_000)
        result = timing.analyze(obs, pub)
        self.assertEqual(result["raw_to_rect_negative_skew_count"], 900)
        self.assertEqual(result["raw_to_rect_receipt_skew_p50_ms"], -1)

    def test_callback_gap_detection(self):
        obs, pub = fixture()
        modified = []
        for line in obs.splitlines():
            if line.startswith("M23K_SAMPLE rect "):
                tag, stream, stamp, recv = line.split()
                if int(stamp) >= 500 * 33_333_333:
                    line = f"{tag} {stream} {stamp} {int(recv) + 400_000_000}"
            modified.append(line)
        result = timing.analyze("\n".join(modified), pub)
        self.assertGreaterEqual(result["rect_receipt_gaps_over_250ms"], 1)

    def test_default_has_no_runtime(self):
        with patch.object(timing, "run", side_effect=AssertionError("ROS invoked")):
            self.assertEqual(timing.main([]), 0)

    def test_quantile_exact(self):
        self.assertEqual(timing.quantile([0,100,200],50),100)
        self.assertEqual(timing.quantile([0,100,200],95),190)

    def test_bounded_original_frame_size(self):
        self.assertEqual(timing.PAYLOAD_BYTES, 921600)
        self.assertEqual(timing.COUNT, 900)


if __name__ == "__main__":
    unittest.main()
