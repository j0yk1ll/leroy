"""Contracts for the real read-only manual startup-yaw witness.

These tests use synthetic timestamps to exercise mathematical boundaries;
they never qualify real motor safety, simulated obstacle clearance or STOP.
"""
import ast
import json
import math
from pathlib import Path
import subprocess
import unittest

from boot_yaw_tracking_core import (
    Heading, YawTracking, TrackingDecision, WAYPOINTS_RAD,
    DIAGNOSTIC_RATE_CAP_RAD_S,
)
from live_manual_yaw_witness import (
    read_status, preflight, REQUIRED_ENV, operator_action,
)

ROOT = Path(__file__).resolve().parent
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

    def test_real_schema_decoding_requires_fresh_owner_no_motor(self):
        now = 20_000_000_000
        status = dict(
            schema="leroy_boot_yaw_v1",
            source="live_ros_m18_imu",
            state="HEADING_ACTIVE",
            owner_verified=True,
            motor_motion_authorized=False,
            physical_heading_accuracy_validated=False,
            monotonic_status_ns=now-200_000_000,
            yaw_relative_rad=-math.pi/2)
        result = read_status(json.dumps(status), now)
        self.assertEqual(result.yaw_rad, -math.pi/2)
        self.assertEqual(result.source_active, True)
        for key, invalid in (
            ("motor_motion_authorized", True),
            ("owner_verified", False),
            ("state", "FAULT_LATCHED"),
            ("monotonic_status_ns", now-500_000_000),
            ("yaw_relative_rad", "invalid"),
        ):
            with self.subTest(key=key):
                bad = dict(status)
                bad[key] = invalid
                self.assertIsNone(read_status(json.dumps(bad), now))
        self.assertIsNone(read_status("not-json", now))

    def test_operator_prompts_are_human_only_and_signed_correctly(self):
        def result(state, err):
            return TrackingDecision(
                state=state, reason="TEST", waypoint_index=0,
                target_rad=-math.pi/2, error_rad=err,
                observed_rate_rad_s=0.0, diagnostic_wz_rad_s=0.0)
        for state, error, expected in (
            ("TRACKING", -math.radians(90), "TURN_CLOCKWISE_TO_TARGET"),
            ("TRACKING", math.radians(8), "TURN_COUNTERCLOCKWISE_TO_TARGET"),
            ("TRACKING", -math.radians(1), "STOP_TURNING_AND_HOLD_STILL"),
            ("TRACKING", math.radians(2), "STOP_TURNING_AND_HOLD_STILL"),
            ("SETTLING", 0.0, "HOLD_STILL_UNTIL_WAYPOINT_ADVANCES"),
            ("AWAITING_HEADING", None, "WAIT_FOR_LIVE_HEADING"),
            ("DONE", None, "SEQUENCE_COMPLETE"),
            ("FAULT_LATCHED", None, "STOP_NO_FURTHER_TURNS"),
        ):
            with self.subTest(state=state, error=error):
                self.assertEqual(operator_action(result(state, error)),
                                 expected)

    def test_progress_reports_original_heading_relative_offset(self):
        s = (ROOT/"live_manual_yaw_witness.py").read_text()
        self.assertIn('"current_relative_deg"', s)
        self.assertIn('"target_relative_deg"', s)
        self.assertIn('"operator_action": operator_action(d)', s)
        self.assertIn('now - last_log >= 2.0', s)
        self.assertIn("node.tracker.last_yaw - node.tracker.origin", s)
        self.assertIn('MANUAL_YAW_MOTOR=BLOCKED_NO_COMMAND_PATH', s)

    def test_contract_refuses_any_hardware_visible(self):
        self.assertTrue(preflight(REQUIRED_ENV, lambda _: False))
        for key in REQUIRED_ENV:
            bad = dict(REQUIRED_ENV)
            bad.pop(key)
            with self.assertRaisesRegex(RuntimeError, "MANUAL_YAW_ENV_"):
                preflight(bad, lambda _: False)
        for path in ("/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0"):
            with self.assertRaisesRegex(RuntimeError, "MANUAL_YAW_DEVICE_VISIBLE"):
                preflight(REQUIRED_ENV, lambda x: x == path)

    def test_ros_witness_never_publishes_or_writes_commands(self):
        script = (ROOT/"live_manual_yaw_witness.py").read_text()
        tree = ast.parse(script)
        publishers = [x for x in ast.walk(tree) if isinstance(x, ast.Call)
                      and isinstance(x.func, ast.Attribute)
                      and x.func.attr == "create_publisher"]
        self.assertEqual(len(publishers), 0)
        self.assertNotIn("serial.Serial", script)
        self.assertNotIn("sign_motion", script)
        self.assertNotIn("set_wheels", script)
        self.assertIn('EXPECTED_OWNER', script)
        self.assertIn('MANUAL_YAW_MOTOR=BLOCKED_NO_COMMAND_PATH', script)

    def test_launcher_is_device_free_and_syntactically_valid(self):
        path = ROOT/"live_manual_yaw_witness.sh"
        subprocess.run(["bash", "-n", str(path)], check=True)
        script = path.read_text()
        self.assertIn("source=leroy-boot-imu-source", script)
        self.assertIn("yaw=leroy-live-boot-yaw-persistent", script)
        self.assertIn('container:$parent', script)
        self.assertIn("--cap-drop=all", script)
        self.assertIn("--read-only", script)
        self.assertNotIn("--device=", script)
        self.assertNotIn("--privileged", script)


if __name__ == "__main__":
    unittest.main()
