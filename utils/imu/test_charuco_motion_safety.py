"""ROS-free regression tests for ChArUco motor motion safety gates."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from calibrate_time_offset import charuco_pose_violation


def pose(cx=0.48, cy=0.50, distance=1.17, displacement=0.0):
    return {
        'cx': cx, 'cy': cy, 'range_m': distance,
        'location_b': np.array([displacement, 0.0, 0.0]),
    }


class PoseSafetyTests(unittest.TestCase):
    def check(self, p, initial=None):
        return charuco_pose_violation(
            p, min_range=0.65, max_range=2.5,
            max_displacement=0.18, initial_camera_in_board=initial)

    def test_recorded_centered_pose_and_small_jitter_are_safe(self):
        self.assertIsNone(self.check(pose(), np.zeros(3)))
        # Replay log: all observed displacements were <= .055 m.
        self.assertIsNone(self.check(pose(displacement=0.055), np.zeros(3)))

    def test_single_missing_pose_is_not_a_geometry_error(self):
        self.assertEqual(self.check(None), 'ChArUco pose unavailable')

    def test_edge_violation_identifies_exact_cause(self):
        self.assertIn('image safety margins', self.check(pose(cx=0.80)))

    def test_range_violation_identifies_exact_cause(self):
        self.assertIn('range outside limits', self.check(pose(distance=2.9)))

    def test_displacement_violation_identifies_exact_cause(self):
        self.assertIn('displacement exceeds limit',
                      self.check(pose(displacement=0.19), np.zeros(3)))

    def test_nonfinite_estimate_is_rejected(self):
        self.assertIn('image safety margins', self.check(pose(cx=math.nan)))
        self.assertIn('displacement exceeds limit',
                      self.check(pose(displacement=math.nan), np.zeros(3)))

    def test_missing_pose_holds_zero_and_tracking_timeout_still_stops(self):
        source = (HERE / 'calibrate_time_offset.py').read_text()
        # Source contract: an isolated missed detection sends a stop command,
        # rather than being interpreted as a hard displacement-limit violation.
        hold = source.index('if self.last_pose is None:', source.index('def control(self):'))
        hard_lost = source.index("self.stop('Lost ChArUco tracking')", source.index('def control(self):'))
        bounds = source.index('violation = self.pose_violation(', source.index('def control(self):'))
        self.assertLess(hard_lost, hold)
        self.assertLess(hold, bounds)
        hold_block = source[hold:bounds]
        self.assertIn('self.send_stop()', hold_block)
        self.assertIn('return', hold_block)
        self.assertIn('now-self.good_since<0.20', source)
        self.assertIn('now-self.last_pose_at>0.30', source)


if __name__ == '__main__':
    unittest.main()
