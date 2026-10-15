"""ROS-free regression tests for ChArUco motor motion safety gates."""
import ast
import math
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from calibrate_time_offset import charuco_pose_violation, implausible_pose_jump


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


    def test_recorded_planar_pnp_flip_is_rejected(self):
        good = {'location_b': np.array([0.0, 0.0, 0.0]),
                'rot': np.eye(3)}
        theta = math.radians(21.9)
        jumped = {'location_b': np.array([0.425, 0.0, 0.0]),
                  'rot': np.array([
                      [math.cos(theta), -math.sin(theta), 0.0],
                      [math.sin(theta), math.cos(theta), 0.0],
                      [0.0, 0.0, 1.0],
                  ])}
        reason = implausible_pose_jump(jumped, good, 0.036)
        self.assertIsNotNone(reason)
        self.assertIn('0.425m', reason)
        self.assertIn('21.9deg', reason)
        # The following good frame is compared to the most recent accepted
        # pose (not the rejected jump), so tracking can safely recover.
        recovered = {'location_b': np.array([0.024, 0.0, 0.0]),
                     'rot': np.eye(3)}
        self.assertIsNone(implausible_pose_jump(recovered, good, 0.068))

    def test_normal_pose_jitter_and_real_slow_movement_are_accepted(self):
        previous = {'location_b': np.zeros(3), 'rot': np.eye(3)}
        theta = math.radians(2.0)
        nearby = {'location_b': np.array([0.055, 0.0, 0.0]),
                  'rot': np.array([
                      [math.cos(theta), -math.sin(theta), 0.0],
                      [math.sin(theta), math.cos(theta), 0.0],
                      [0.0, 0.0, 1.0],
                  ])}
        self.assertIsNone(implausible_pose_jump(nearby, previous, 0.033))
        # Truly moving 0.19m over 0.5s can be dynamically plausible;
        # the independent 0.18m displacement safety guard must still stop it.
        far = {'location_b': np.array([0.19, 0.0, 0.0]),
               'rot': np.eye(3)}
        self.assertIsNone(implausible_pose_jump(far, previous, 0.5))
        self.assertIn('displacement exceeds limit',
                      self.check({**pose(), 'location_b': far['location_b']},
                                 np.zeros(3)))

    def test_rotation_flip_and_nan_pose_are_rejected(self):
        previous = {'location_b': np.zeros(3), 'rot': np.eye(3)}
        theta = math.radians(21.9)
        flip = {'location_b': np.zeros(3), 'rot': np.array([
            [math.cos(theta), -math.sin(theta), 0.0],
            [math.sin(theta), math.cos(theta), 0.0],
            [0.0, 0.0, 1.0],
        ])}
        self.assertIn('21.9deg', implausible_pose_jump(flip, previous, 0.036))
        invalid = {'location_b': np.array([float('nan'), 0.0, 0.0]),
                   'rot': np.eye(3)}
        self.assertIn('Nonfinite', implausible_pose_jump(invalid, previous, 0.036))

    def test_jump_rejection_commands_stop_before_any_motion(self):
        source = (HERE / 'calibrate_time_offset.py').read_text()
        callback = source[source.index('def on_image(self,msg):'):
                          source.index('def write(self,topic,msg,ts):')]
        self.assertIn('reason=implausible_pose_jump(', callback)
        self.assertIn("self.counts['rejected_pose_jumps']+=1", callback)
        self.assertIn('self.last_pose=None', callback)
        self.assertIn('self.send_stop()', callback)
        self.assertIn("self.stop('Lost ChArUco tracking')", callback)
        self.assertLess(callback.index('violation=charuco_pose_violation('),
                        callback.index('reason=implausible_pose_jump('))
        self.assertIn('now-self.last_pose_at>0.30', callback)


if __name__ == '__main__':
    unittest.main()
