"""ROS-free regression tests for independent raw ChArUco bearing lag scan."""
import math
import unittest
import numpy as np
from corner_bearing_timing import bearing_lag_scan, pixels_to_imu_bearings


class BearingTests(unittest.TestCase):
    @staticmethod
    def sequence(offset_ms, gain=-1.08):
        base = 1_790_000_000_000_000_000
        components = [(0.11, 0.39), (0.045, 0.82), (0.031, 1.23)]
        yaw = lambda t: sum(a*math.sin(2*math.pi*f*t) for a,f in components)
        gyro = lambda t: sum(2*math.pi*f*a*math.cos(2*math.pi*f*t) for a,f in components)
        imu = [(base+round(i*1e7), gyro(i/100) + 0.004) for i in range(2401)]
        rng = np.random.default_rng(44)
        track = [(base+round(i*1e9/30) + offset_ms*1_000_000,
                  0.24 + gain*yaw(i/30) + .0002*(i/30) + rng.normal(0,0.0011))
                 for i in range(720)]
        return track,imu

    def test_positive_lag(self):
        result = bearing_lag_scan(*self.sequence(34))
        self.assertFalse(result['valid'])
        self.assertLessEqual(abs(result['all']['lag_ms']-34), 6)
        self.assertAlmostEqual(result['all']['gain'], -1.08, delta=0.06)
        self.assertLessEqual(abs(result['first_half']['lag_ms']-result['second_half']['lag_ms']), 16)

    def test_negative_lag(self):
        result = bearing_lag_scan(*self.sequence(-56))
        self.assertLessEqual(abs(result['all']['lag_ms']+56), 6)

    def test_image_ray_azimuth_uses_existing_camera_to_imu_axes(self):
        r_imu_cam = np.array([[0,0,1], [1,0,0], [0,1,0]],dtype=float)
        k = np.array([[800,0,320],[0,800,240],[0,0,1]],dtype=float)
        bearing, forward, _ = pixels_to_imu_bearings(
            [[320+800*math.tan(0.17),240]], k, [0,0,0,0,0], r_imu_cam)
        self.assertAlmostEqual(float(bearing[0]), 0.17, places=10)
        self.assertGreater(float(forward[0]), 0)

    def test_insufficient_observations(self):
        track,imu = self.sequence(34)
        result = bearing_lag_scan(track[:20], imu)
        self.assertIn('Insufficient',result['reason'])

    def test_insufficient_excitation(self):
        track,imu = self.sequence(34)
        result = bearing_lag_scan(track, [(t,0.002*z) for t,z in imu])
        self.assertIn('excitation',result['reason'])


if __name__=='__main__':
    unittest.main()
