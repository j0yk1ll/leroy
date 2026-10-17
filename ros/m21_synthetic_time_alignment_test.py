"""Offline-only synthetic regression tests for lag estimation; no ROS, devices or motors."""
from contextlib import redirect_stdout
from io import StringIO
import json
import math
from pathlib import Path
import tempfile
import unittest

from m21_synthetic_time_alignment import estimate, main, synthetic


class SyntheticAlignmentTests(unittest.TestCase):
    def test_known_positive_offset_with_image_gap(self):
        report = estimate(*synthetic(offset_s=0.047))
        self.assertEqual(report['verdict'], 'OFFLINE_FEATURE_ALIGNMENT_CANDIDATE')
        self.assertLess(abs(report['effective_feature_lag_s'] - 0.047), 0.010)
        self.assertFalse(report['physical_camera_imu_offset_validated'])

    def test_known_negative_offset(self):
        report = estimate(*synthetic(offset_s=-0.056))
        self.assertEqual(report['verdict'], 'OFFLINE_FEATURE_ALIGNMENT_CANDIDATE')
        self.assertLess(abs(report['effective_feature_lag_s'] + 0.056), 0.010)

    def test_missing_frame_gap_is_not_interpolated(self):
        report = estimate(*synthetic(offset_s=0.032, drop_window=(3.5, 4.85)))
        self.assertLess(abs(report['candidate_lag_s'] - 0.032), 0.012)
        self.assertLess(report['matched_camera_samples'], 306)

    def test_outliers_do_not_destabilize_lag(self):
        report = estimate(*synthetic(offset_s=0.047, outlier_fraction=0.025))
        self.assertLess(abs(report['candidate_lag_s'] - 0.047), 0.015)

    def test_repetitive_ambiguous_motion_rejected(self):
        camera = [[0.1 + k / 17.36, 1.2 + math.sin(2 * math.pi * 8 * (0.1 + k / 17.36))]
                  for k in range(300)]
        imu = [[k / 50, 1.2 + math.sin(2 * math.pi * 8 * (k / 50 - 0.037))]
               for k in range(920)]
        report = estimate(camera, imu)
        self.assertEqual(report['verdict'], 'OFFLINE_FEATURE_ALIGNMENT_REJECTED')
        self.assertIsNone(report['effective_feature_lag_s'])

    def test_stationary_fails_closed(self):
        with self.assertRaises(ValueError):
            estimate(*synthetic(stationary=True))

    def test_insufficient_samples_fails_closed(self):
        camera, imu = synthetic()
        with self.assertRaises(ValueError):
            estimate(camera[:20], imu)

    def test_nonmonotonic_stamp_fails_closed(self):
        camera, imu = synthetic()
        imu[25][0] = imu[24][0]
        with self.assertRaises(ValueError):
            estimate(camera, imu)

    def test_nan_fails_closed(self):
        camera, imu = synthetic()
        camera[20][1] = math.nan
        with self.assertRaises(ValueError):
            estimate(camera, imu)

    def test_short_duration_fails_closed(self):
        camera, imu = synthetic()
        with self.assertRaises(ValueError):
            estimate(camera[:60], imu)

    def test_offline_feature_input_not_labeled_physical_or_synthetic(self):
        camera, imu = synthetic(offset_s=0.021)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'features.json'
            path.write_text(json.dumps({'camera_motion': camera,
                                        'imu_motion': imu}))
            captured = StringIO()
            with redirect_stdout(captured):
                self.assertEqual(main(['--input', str(path)]), 0)
            self.assertIn('"verdict": "OFFLINE_FEATURE_ALIGNMENT_CANDIDATE_UNCALIBRATED"',
                          captured.getvalue())
            self.assertIn('"physical_camera_imu_offset_validated": false',
                          captured.getvalue())
            self.assertNotIn('"synthetic_injected_offset_s"', captured.getvalue())

    def test_imu_outage_not_bridge_interpolation(self):
        camera, imu = synthetic(offset_s=0.041)
        imu = [p for p in imu if not 8.0 < p[0] < 10.0]
        result = estimate(camera, imu)
        self.assertLess(result['matched_camera_samples'], len(camera) - 20)

    def test_synthetic_cli_labels_injection_not_physical(self):
        stdout = StringIO()
        with redirect_stdout(stdout):
            self.assertEqual(main(['--demo']), 0)
        self.assertIn('"evidence_type": "SYNTHETIC_INJECTION"', stdout.getvalue())
        self.assertIn('"physical_camera_imu_offset_validated": false', stdout.getvalue())


if __name__ == '__main__':
    unittest.main()
