"""ROS-free regression tests for read-only ChArUco orientation lag scan."""
import math
from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from orientation_timing import orientation_lag_scan, visual_yaw_angles


class OrientationTimingTests(unittest.TestCase):
    @staticmethod
    def sequence(offset_ms, noise=0.012, gyro_bias=0.005):
        base = 1_790_000_000_000_000_000
        components = [(0.11, 0.39), (0.045, 0.82), (0.031, 1.23)]

        def yaw(t):
            return sum(a * math.sin(2 * math.pi * f * t)
                       for a, f in components)

        def gyro(t):
            return sum(2 * math.pi * f * a * math.cos(2 * math.pi * f * t)
                       for a, f in components)

        imu = [(base + round(k * 10_000_000),
                gyro(k / 100.0) + gyro_bias)
               for k in range(2401)]
        poses = []
        rng = np.random.default_rng(199)
        for k in range(720):
            t = k / 30.0
            a = -yaw(t) + float(rng.normal(0, noise))
            c, s = math.cos(a), math.sin(a)
            pose = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            poses.append((base + round(t * 1e9)
                          + offset_ms * 1_000_000, pose))
        return poses, imu

    def test_noisy_positive_lag(self):
        poses, imu = self.sequence(34)
        result = orientation_lag_scan(poses, imu, np.eye(3))
        self.assertIsNone(result.get('reason'), result)
        self.assertFalse(result['valid'])  # diagnostic can never certify
        self.assertLessEqual(abs(result['all']['lag_ms'] - 34), 10)
        self.assertLessEqual(abs(result['first_half']['lag_ms'] -
                                 result['second_half']['lag_ms']), 24)
        self.assertGreater(result['imu_yaw_std_rad_s'], 0.035)

    def test_noisy_negative_lag(self):
        poses, imu = self.sequence(-56)
        result = orientation_lag_scan(poses, imu, np.eye(3))
        self.assertIsNone(result.get('reason'), result)
        self.assertLessEqual(abs(result['all']['lag_ms'] + 56), 10)

    def test_pose_angle_has_correct_sign_and_units(self):
        poses, _ = self.sequence(34, noise=0.0, gyro_bias=0.0)
        angles = visual_yaw_angles(poses, np.eye(3))
        self.assertAlmostEqual(angles[0][1], 0.0, places=10)
        t = 1.0 / 30.0
        expected = sum(a * math.sin(2 * math.pi * f * t)
                       for a, f in [(0.11, 0.39), (0.045, 0.82),
                                    (0.031, 1.23)])
        self.assertAlmostEqual(angles[1][1], expected, places=8)

    def test_weak_real_imu_yaw_is_not_reported_as_lag(self):
        poses, imu = self.sequence(34)
        weak = [(t, 0.002 * value) for t, value in imu]
        result = orientation_lag_scan(poses, weak, np.eye(3))
        self.assertIn('Insufficient real IMU yaw', result['reason'])

    def test_insufficient_samples_gives_no_estimate(self):
        poses, imu = self.sequence(34)
        result = orientation_lag_scan(poses[:20], imu, np.eye(3))
        self.assertIn('Insufficient', result['reason'])


if __name__ == '__main__':
    unittest.main()
