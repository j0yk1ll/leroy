"""No hardware required; rejects odometry gaps, jumps and invalid values."""
import unittest

from leroy_localization.odometry_quality import OdometryQualityGate, Sample


class QualityTest(unittest.TestCase):
    def setUp(self):
        self.gate = OdometryQualityGate(warmup_samples=2)
        self.t = 1_000_000_000

    def sample(self, *, t=None, x=0.0, yaw=0.0, frame="odom",
               variance=0.01):
        return Sample(self.t if t is None else t, x, 0.0, yaw,
                      0.0, 0.0, 0.0, variance, 0.01, frame=frame)

    def test_warmup_then_plausible(self):
        self.assertEqual(self.gate.observe(self.sample(), now_ns=self.t).reason,
                         "warming_up")
        self.assertTrue(self.gate.observe(self.sample(t=self.t + 100_000_000),
                                          now_ns=self.t + 100_000_000).valid)

    def test_jump_invalidates_warmup(self):
        self.gate.observe(self.sample(), now_ns=self.t)
        d = self.gate.observe(self.sample(t=self.t + 100_000_000, x=2.0),
                              now_ns=self.t + 100_000_000)
        self.assertEqual(d.reason, "pose_discontinuity")
        self.assertEqual(d.consecutive, 0)

    def test_time_regression_and_staleness(self):
        self.gate.observe(self.sample(), now_ns=self.t)
        self.assertEqual(self.gate.observe(self.sample(), now_ns=self.t + 1).reason,
                         "timestamp_discontinuity")
        self.assertEqual(self.gate.observe(self.sample(),
                         now_ns=self.t + 400_000_000).reason, "stale_or_future")

    def test_covariance_and_frame(self):
        self.assertEqual(self.gate.observe(self.sample(variance=100.0),
                         now_ns=self.t).reason, "covariance_unqualified")
        self.assertEqual(self.gate.observe(self.sample(frame="map"),
                         now_ns=self.t).reason, "frame_mismatch")

    def test_no_calibration_claim(self):
        self.gate.observe(self.sample(), now_ns=self.t)
        state = self.gate.observe(self.sample(t=self.t + 100_000_000),
                                  now_ns=self.t + 100_000_000)
        self.assertEqual(state.reason, "plausible_not_calibrated")
        self.assertFalse(hasattr(state, "motion_authorized"))


if __name__ == "__main__":
    unittest.main()
