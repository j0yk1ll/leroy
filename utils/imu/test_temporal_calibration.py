"""ROS-free ChArUco temporal-offset tests."""
import math
import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from calibrate_time_offset import (
    search_offset, visual_gyro_z, windowed_visual_gyro_z, quaternion_matrix,
    choose_motion_segment, yaw_command_rad_s,
)

class TimingTests(unittest.TestCase):
    def sequence(self, offset_ms):
        base=1_790_000_000_000_000_000
        terms=[(0.12,0.43),(0.065,1.1),(0.038,2.2)]
        def yaw(t):
            return sum(a*math.sin(2*math.pi*f*t) for a,f in terms)
        def gyro(t):
            return sum(a*2*math.pi*f*math.cos(2*math.pi*f*t) for a,f in terms)
        imu=[(base+round(j*1e7),gyro(j/100)) for j in range(5001)]
        poses=[]
        for j in range(1501):
            t=j/30
            a=-yaw(t)
            c,s=math.cos(a),math.sin(a)
            r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
            poses.append((base+round(t*1e9)+offset_ms*1_000_000,r))
        return poses,imu

    def test_positive_offset(self):
        poses,imu=self.sequence(34)
        r=search_offset(poses,imu,np.eye(3))
        self.assertTrue(r['valid'],r)
        self.assertLessEqual(abs(r['lag_ms']-34),4)

    def test_negative_offset(self):
        poses,imu=self.sequence(-56)
        r=search_offset(poses,imu,np.eye(3))
        self.assertLessEqual(abs(r['lag_ms']+56),4)

    def test_stationary_rejected(self):
        poses,imu=self.sequence(34)
        result=search_offset([(t,np.eye(3)) for t,_ in poses],imu,np.eye(3))
        self.assertFalse(result['valid'])

    def test_insufficient_imu_yaw_diagnostics(self):
        poses, imu = self.sequence(34)
        weak_imu = [(t, 0.03 * omega) for t, omega in imu]
        result = search_offset(poses, weak_imu, np.eye(3))
        self.assertFalse(result['valid'])
        self.assertEqual(result['reason'], 'Insufficient varying yaw signal')
        self.assertLess(result['imu_yaw_std_rad_s'], 0.035)
        self.assertGreater(result['visual_yaw_std_rad_s'], 0.035)
        self.assertGreater(result['imu_peak_abs_yaw_rad_s'], 0)

    def test_opt_in_yaw_pattern_alternates_without_translation(self):
        class RandomStub:
            def choice(self, choices):
                self.choices = choices
                return 'front'
        rng = RandomStub()
        self.assertEqual(
            [choose_motion_segment(i, True, rng) for i in range(6)],
            ['yaw_left', 'yaw_right', 'yaw_left',
             'yaw_right', 'yaw_left', 'yaw_right'])
        self.assertEqual(choose_motion_segment(0, False, rng), 'front')
        self.assertAlmostEqual(yaw_command_rad_s(True), 0.21)
        self.assertAlmostEqual(yaw_command_rad_s(False), 0.16)
        # Existing wheel geometry and PWM gain: still below the +/-64 cap.
        duty = round((0.180 + 0.275) * 0.21 / 0.020 * 12.75)
        self.assertEqual(duty, 61)

    def test_longer_baseline_reduces_pnp_orientation_derivative_noise(self):
        poses, imu = self.sequence(34)
        rng = np.random.default_rng(123)
        noisy = []
        for ts, rot in poses:
            # Independent per-frame PnP yaw jitter: a short baseline amplifies
            # it, despite having precisely the correct camera timestamps.
            jitter = float(rng.normal(0.0, 0.018))
            c, si = math.cos(jitter), math.sin(jitter)
            perturb = np.array([[c, -si, 0], [si, c, 0], [0, 0, 1]])
            noisy.append((ts, perturb @ rot))
        original = search_offset(noisy, imu, np.eye(3))
        windowed = search_offset(noisy, imu, np.eye(3),
                                 visual_baseline_s=0.25)
        self.assertIn('correlation', original)
        self.assertIn('correlation', windowed)
        self.assertGreater(windowed['correlation'], original['correlation'] + 0.15)
        self.assertLessEqual(abs(windowed['lag_ms'] - 34), 10)

    def test_windowed_rate_has_requested_intervals_and_no_tf_edit(self):
        poses, _ = self.sequence(34)
        pairs = windowed_visual_gyro_z(poses[:50], np.eye(3), 0.25)
        self.assertGreater(len(pairs), 15)
        for ta, tb, omega in pairs:
            self.assertGreaterEqual((tb-ta)/1e9, 0.25)
            self.assertLess((tb-ta)/1e9, 0.37)
            self.assertTrue(math.isfinite(omega))
        with self.assertRaises(ValueError):
            windowed_visual_gyro_z(poses, np.eye(3), 0.03)

    def test_quaternion(self):
        q=quaternion_matrix((0,0,math.sin(math.pi/4),math.cos(math.pi/4)))
        np.testing.assert_allclose(q@np.array([1.,0,0]),[0,1,0],atol=1e-12)

if __name__=='__main__':
    unittest.main()
