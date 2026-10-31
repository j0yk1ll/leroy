"""Contracts for the real read-only manual startup-yaw witness.

These tests use synthetic timestamps to exercise mathematical boundaries;
they never qualify real motor safety, simulated obstacle clearance or STOP.
"""
import math
from pathlib import Path
import unittest

from leroy_localization.yaw_tracking_core import (
    Heading, YawTracking, TrackingDecision, WAYPOINTS_RAD,
    DIAGNOSTIC_RATE_CAP_RAD_S,
)

ROOT = Path(__file__).resolve().parents[2]
BASE = 10_000_000_000


def sample(t=BASE, yaw=0.0, **changes):
    args = dict(stamp_ns=t, receipt_ns=t, yaw_rad=yaw,
                source_active=True, owner_verified=True)
    args.update(changes)
    return Heading(**args)


class LiveYawTrackingTests(unittest.TestCase):
    def test_no_implicit_start_or_motor_authority(self):
        m = YawTracking()
        d = m.tick(sample(), BASE)
        self.assertEqual((d.state, d.reason),
                         ("INHIBITED", "EXPLICIT_LOCAL_START_REQUIRED"))
        self.assertFalse(d.motor_motion_authorized)
        self.assertEqual(d.diagnostic_wz_rad_s, 0.0)

    def test_initial_target_clockwise_negative(self):
        m = YawTracking()
        self.assertTrue(m.start())
        d = m.tick(sample(yaw=0.1), BASE)
        self.assertEqual(d.state, "TRACKING")
        self.assertAlmostEqual(d.target_rad, 0.1 - math.pi/2)
        self.assertLess(d.diagnostic_wz_rad_s, 0)
        self.assertLessEqual(abs(d.diagnostic_wz_rad_s),
                             DIAGNOSTIC_RATE_CAP_RAD_S)
        self.assertFalse(d.motor_motion_authorized)

    def test_full_manual_sweep_reaches_done_without_motor_release(self):
        m = YawTracking()
        m.start()
        points = [
            (0, 0.0, "TRACKING"),
            (200, -math.pi/2, "TRACKING"),
            (400, -math.pi/2, "SETTLING"),
            (1000, -math.pi/2, "TRACKING"),
            (1200, +math.pi/2, "TRACKING"),
            (1400, +math.pi/2, "SETTLING"),
            (2000, +math.pi/2, "TRACKING"),
            (2200, 0.0, "TRACKING"),
            (2400, 0.0, "SETTLING"),
            (3000, 0.0, "DONE"),
        ]
        for ms, yaw, state in points:
            t = BASE + ms*1_000_000
            d = m.tick(sample(t, yaw), t)
            self.assertEqual(d.state, state, (ms, d.reason))
            self.assertFalse(d.motor_motion_authorized)
        self.assertEqual(d.reason, "MANUAL_SWEEP_OBSERVED_NO_MOTOR_RELEASE")
        self.assertIsNone(d.target_rad)
        self.assertEqual(d.diagnostic_wz_rad_s, 0)
        self.assertEqual(m.tick(sample(BASE+3200_000_000, 0), BASE+3200_000_000).state,
                         "DONE")

    def test_one_arrival_does_not_count_as_stationary_settle(self):
        m = YawTracking()
        m.start()
        m.tick(sample(), BASE)
        d = m.tick(sample(BASE+200_000_000, -math.pi/2),
                   BASE+200_000_000)
        self.assertEqual(d.state, "TRACKING")
        self.assertEqual(d.reason, "TARGET_MOVING_NOT_SETTLED")

    def test_envelope_overrun_faults_irreversibly(self):
        m = YawTracking()
        m.start()
        m.tick(sample(), BASE)
        t = BASE+200_000_000
        d = m.tick(sample(t, -math.radians(105)), t)
        self.assertEqual((d.state,d.reason),
                         ("FAULT_LATCHED","YAW_ENVELOPE_EXCEEDED"))
        self.assertEqual(d.diagnostic_wz_rad_s, 0)
        self.assertFalse(m.start())

    def test_stale_duplicate_and_regressed_source_fault(self):
        for bad, now, expected in (
            (sample(BASE+200_000_000, 0, owner_verified=False),
             BASE+200_000_000, "HEADING_SOURCE_OR_OWNER_INVALID"),
            (sample(BASE, 0), BASE+100_000_000,
             "HEADING_STAMP_REGRESSION"),
            (sample(BASE+200_000_000, 0), BASE+700_000_000,
             "HEADING_STALE"),
            (sample(BASE+700_000_000, 0), BASE+700_000_000,
             "HEADING_UPDATE_GAP"),
            (sample(BASE+200_000_000, 0, yaw_rad=float("nan")),
             BASE+200_000_000, "INVALID_HEADING_OR_CLOCK"),
        ):
            with self.subTest(expected=expected):
                m = YawTracking()
                m.start()
                m.tick(sample(), BASE)
                d = m.tick(bad, now)
                self.assertEqual((d.state, d.reason),
                                 ("FAULT_LATCHED", expected))
                self.assertEqual(d.diagnostic_wz_rad_s, 0)

    def test_source_disappearance_latches_without_wheel_output(self):
        m = YawTracking()
        m.start()
        m.tick(sample(), BASE)
        d = m.on_timeout(BASE+400_000_000)
        self.assertEqual((d.state,d.reason), ("FAULT_LATCHED","HEADING_LOST"))
        self.assertEqual(d.diagnostic_wz_rad_s, 0)

    def test_operator_stop_latches_inhibited_diagnostic(self):
        m = YawTracking()
        m.start()
        m.tick(sample(), BASE)
        self.assertEqual(m.stop().state, "STOPPED")
        self.assertEqual(m.tick(sample(BASE+200_000_000), BASE+200_000_000).state,
                         "STOPPED")
        self.assertFalse(m.start())



if __name__ == "__main__":
    unittest.main()
