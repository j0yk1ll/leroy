"""Standard-library no-device regression tests for the Leroy boot model."""
from dataclasses import replace
import math
import unittest
from boot_localization_core import (
    BootLocalizationModel, Observation, SWEEP_RELATIVE_YAWS_RAD,
)


def synthetic(now=0.0, yaw=0.0, **updates):
    """Fictional fully qualified system; never live physical evidence."""
    o = Observation(
        now=now, image_at=now, obstacle_at=now, yaw_at=now,
        localization_at=now, yaw_rad=yaw,
        localization_status="initializing",
        independent_stop_proven=True,
        verified_stop_distance=True,
        exclusive_command_authority=True,
        qualified_yaw_odometry=True,
        validated_tf_geometry=True,
        measured_swept_footprint=True,
        calibrated_obstacle_classifier=True,
        full_swept_envelope_observed=True,
        full_swept_envelope_clear=True,
        physical_motion_release=True,
    )
    return replace(o, **updates)


class BootLocalizationTests(unittest.TestCase):
    def running(self):
        m = BootLocalizationModel()
        self.assertTrue(m.request_start())
        self.assertEqual(m.tick(synthetic()).state, "PREFLIGHT")
        s = m.tick(synthetic(1.01))
        self.assertEqual(s.state, "SWEEP")
        return m

    def test_default_is_inhibited_and_no_setpoint(self):
        model = BootLocalizationModel()
        self.assertEqual(model.tick(synthetic()).state, "INHIBITED")
        self.assertIsNone(model.decision().diagnostic_yaw_target_rad)

    def test_unqualified_real_defaults_fail_closed(self):
        m = BootLocalizationModel()
        m.request_start()
        d = m.tick(Observation(now=0.0))
        self.assertEqual((d.state, d.reason),
                         ("FAULT_LATCHED", "INDEPENDENT_STOP_UNPROVEN"))
        self.assertFalse(m.request_start())
        self.assertIsNone(d.diagnostic_yaw_target_rad)

    def test_preflight_dwell_cannot_skip(self):
        m = BootLocalizationModel()
        m.request_start()
        for t in (0, 0.25, 0.99):
            self.assertEqual(m.tick(synthetic(t)).state, "PREFLIGHT")
        self.assertEqual(m.tick(synthetic(1.0)).state, "SWEEP")

    def test_full_sweep_ends_without_translation(self):
        m = self.running()
        for target, arrive in zip(SWEEP_RELATIVE_YAWS_RAD, (1.7, 2.7, 3.7)):
            d = m.decision()
            self.assertEqual(d.state, "SWEEP")
            self.assertAlmostEqual(d.diagnostic_yaw_target_rad, target)
            self.assertEqual(m.tick(synthetic(arrive, target)).state, "SWEEP")
            m.tick(synthetic(arrive + 0.31, target))
        self.assertEqual(m.state, "NEEDS_PARALLAX")
        self.assertIsNone(m.decision().diagnostic_yaw_target_rad)
        self.assertEqual(m.tick(synthetic(4.2)).state, "NEEDS_PARALLAX")

    def test_turn_target_relative_to_boot_heading(self):
        m = BootLocalizationModel()
        m.request_start()
        m.tick(synthetic(0, yaw=1.25))
        d = m.tick(synthetic(1.01, yaw=1.25))
        self.assertAlmostEqual(d.diagnostic_yaw_target_rad, 1.25 - math.pi/2)

    def test_tracking_alone_not_metric_odom(self):
        m = self.running()
        d = m.tick(synthetic(1.2, localization_status="tracking"))
        self.assertEqual(d.state, "WAIT_NAVIGATION_RELEASE")
        self.assertIsNone(d.diagnostic_yaw_target_rad)
        d = m.tick(synthetic(1.3, localization_status="tracking",
                             metric_pose_validated=True,
                             navigation_release_validated=True))
        self.assertEqual(d.state, "READY")

    def test_tracking_early_skips_sweep_if_released(self):
        m = BootLocalizationModel()
        m.request_start()
        d = m.tick(synthetic(0, localization_status="tracking",
                             metric_pose_validated=True,
                             navigation_release_validated=True))
        self.assertEqual(d.state, "READY")
        self.assertIsNone(d.diagnostic_yaw_target_rad)

    def test_stale_frame_in_sweep_latches(self):
        m = self.running()
        d = m.tick(synthetic(1.45, image_at=1.01))
        self.assertEqual((d.state, d.reason),
                         ("FAULT_LATCHED", "STALE_OR_INVALID_IMAGE"))
        self.assertIsNone(d.diagnostic_yaw_target_rad)
        self.assertEqual(m.tick(synthetic(1.5)).state, "FAULT_LATCHED")

    def test_unknown_space_or_blind_side_latches(self):
        for key, reason in (
            ("full_swept_envelope_clear", "SWEPT_ENVELOPE_NOT_CLEAR"),
            ("full_swept_envelope_observed", "SWEPT_ENVELOPE_NOT_OBSERVED"),
            ("measured_swept_footprint", "SWEPT_FOOTPRINT_UNMEASURED"),
            ("calibrated_obstacle_classifier", "OBSTACLE_CLASSIFICATION_UNVERIFIED"),
        ):
            with self.subTest(key=key):
                m = self.running()
                d = m.tick(synthetic(1.2, **{key: False}))
                self.assertEqual((d.state, d.reason), ("FAULT_LATCHED", reason))
                self.assertIsNone(d.diagnostic_yaw_target_rad)

    def test_stop_command_owner_yaw_and_release_all_latch(self):
        for key in (
            "independent_stop_proven", "verified_stop_distance",
            "exclusive_command_authority", "qualified_yaw_odometry",
            "validated_tf_geometry", "physical_motion_release",
        ):
            with self.subTest(key=key):
                m = self.running()
                self.assertEqual(m.tick(synthetic(1.2, **{key: False})).state,
                                 "FAULT_LATCHED")

    def test_clock_regression_invalid_yaw_and_stale_data(self):
        m = self.running()
        self.assertEqual(m.tick(synthetic(1.01)).reason,
                         "CLOCK_INVALID_OR_REGRESSED")
        for updates in ({"yaw_rad": float("nan")},
                        {"yaw_at": -1},
                        {"obstacle_at": 0.0},
                        {"localization_status": "lost"}):
            m = self.running()
            self.assertEqual(m.tick(synthetic(1.4, **updates)).state,
                             "FAULT_LATCHED")

    def test_target_must_settle(self):
        m = self.running()
        self.assertEqual(m.tick(synthetic(1.3, -math.pi/2)).state, "SWEEP")
        self.assertEqual(m.tick(synthetic(1.5, -0.4)).state, "SWEEP")
        self.assertEqual(m.tick(synthetic(1.65, -math.pi/2)).state, "SWEEP")
        self.assertAlmostEqual(
            m.tick(synthetic(1.9, -math.pi/2)).diagnostic_yaw_target_rad,
            -math.pi/2)
        self.assertAlmostEqual(
            m.tick(synthetic(1.96, -math.pi/2)).diagnostic_yaw_target_rad,
            math.pi/2)

    def test_angle_overshoot_latches_without_timeout(self):
        m = self.running()
        d = m.tick(synthetic(1.2, -math.radians(108)))
        self.assertEqual((d.state, d.reason),
                         ("FAULT_LATCHED", "SWEEP_ANGLE_ENVELOPE_EXCEEDED"))
        self.assertIsNone(d.diagnostic_yaw_target_rad)

    def test_waypoint_timeout_latches(self):
        m = self.running()
        d = m.tick(synthetic(21.1, 0))
        self.assertEqual((d.state, d.reason),
                         ("FAULT_LATCHED", "WAYPOINT_DEADLINE_EXCEEDED"))

    def test_ready_remains_qualified_and_rejects_lost_health(self):
        for updates, reason in (
            ({"image_at": 0.0}, "STALE_OR_INVALID_IMAGE"),
            ({"full_swept_envelope_clear": False},
             "SWEPT_ENVELOPE_NOT_CLEAR"),
            ({"navigation_release_validated": False},
             "READY_LOCALIZATION_OR_RELEASE_LOST"),
            ({"localization_status": "lost"}, "LOCALIZATION_LOST"),
        ):
            with self.subTest(updates=updates):
                m = BootLocalizationModel()
                m.request_start()
                ready = m.tick(synthetic(
                    0.0, localization_status="tracking",
                    metric_pose_validated=True,
                    navigation_release_validated=True))
                self.assertEqual(ready.state, "READY")
                healthy = {"localization_status": "tracking",
                           "metric_pose_validated": True,
                           "navigation_release_validated": True}
                healthy.update(updates)
                d = m.tick(synthetic(0.5, **healthy))
                self.assertEqual((d.state, d.reason),
                                 ("FAULT_LATCHED", reason))
                self.assertIsNone(d.diagnostic_yaw_target_rad)

    def test_missing_stamps_and_future_stamps_block(self):
        for updates in ({"image_at": None}, {"obstacle_at": 5.0},
                        {"localization_at": float("nan")},
                        {"yaw_at": float("inf")}):
            m = self.running()
            self.assertEqual(m.tick(synthetic(1.2, **updates)).state,
                             "FAULT_LATCHED")

    def test_no_ros_device_or_actuation_in_pure_core(self):
        from pathlib import Path
        import ast
        path = Path(__file__).resolve().parents[2] / "boot_localization_core.py"
        s = path.read_text()
        parsed = ast.parse(s)
        names = set()
        for n in ast.walk(parsed):
            if isinstance(n, ast.Import):
                names.update(a.name.split('.')[0] for a in n.names)
            elif isinstance(n, ast.ImportFrom):
                names.add((n.module or '').split('.')[0])
        self.assertEqual(names, {"dataclasses", "math"})
        self.assertNotIn("create_publisher", s)
        self.assertNotIn("Serial(", s)
        self.assertNotIn("os.system", s)


if __name__ == "__main__":
    unittest.main()
