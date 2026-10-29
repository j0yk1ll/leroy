"""No-device contract tests of the ACTUATING signed yaw-control software.

Fixtures never provision physical stop, 360-degree obstacle coverage, or
real motor power. They validate that production motor commands require the
independent hardware monitor's distinct authenticated lease.
"""
import ast
import json
import math
from pathlib import Path
import subprocess
import unittest

from autonomous_motor_yaw_core import (
    SafetyLeaseVerifier, MotorYawCalibration, LEASE_DOMAIN, LEASE_SCHEMA,
    _mac, MAX_MONITOR_LAG_NS,
)
from boot_yaw_tracking_core import Heading

ROOT = Path(__file__).resolve().parent
SESSION = "b" * 32
SAFETY_KEY = b"s" * 32
BASE = 10_000_000_000


def signed_lease(seq, now, **changes):
    data = {
        "schema": LEASE_SCHEMA, "session_id": SESSION,
        "sequence": seq, "issued_ns": now, "expires_ns": now + 120_000_000,
        "swept_footprint_id": "independently-measured-chassis-v1",
        "whole_envelope_measured": True,
        "whole_envelope_observed": True,
        "whole_envelope_clear": True,
        "independent_stop_proven": True,
        "stop_distance_measured": True,
        "command_owner_exclusive": True,
        "motor_output_interlock_proven": True,
        "stopped_verified": False,
        "max_yaw_rate_rad_s": 0.08,
        "measured_stop_distance_m": 0.03,
        "free_clearance_margin_m": 0.15,
        "safety_monitor_id": "bench-fixture-only",
    }
    data.update(changes)
    data["mac"] = _mac(SAFETY_KEY, {
        "domain": LEASE_DOMAIN, **data})
    return json.dumps(data, sort_keys=True)


def heading(t, yaw):
    return Heading(t, t, float(yaw), True, True)


class ActuatedCalibrationContracts(unittest.TestCase):
    def test_default_never_arms_without_verified_hardware(self):
        c = MotorYawCalibration(SESSION)
        self.assertEqual(c.step(heading(BASE, 0), BASE).wz_rad_s, 0)
        self.assertTrue(c.step(heading(BASE, 0), BASE)
                        .operator_or_motor_rearm_required)
        self.assertTrue(c.start())
        d = c.step(heading(BASE, 0), BASE)
        self.assertEqual(d.state, "LATCHED")
        self.assertEqual(d.reason, "SAFETY_LEASE_EXPIRED_OR_LOST")
        self.assertEqual(d.wz_rad_s, 0)

    def test_valid_hardware_lease_is_session_bound_and_one_use(self):
        v = SafetyLeaseVerifier(
            safety_key=SAFETY_KEY, bridge_session_id=SESSION)
        t = BASE
        item = signed_lease(1, t)
        grant = v.accept(item, t + 5_000_000)
        self.assertIsNotNone(grant)
        self.assertEqual(grant.session_id, SESSION)
        self.assertAlmostEqual(grant.max_yaw_rate_rad_s, 0.08)
        self.assertIsNone(v.accept(item, t + 6_000_000))
        self.assertIsNotNone(v.accept(
            signed_lease(2, t + 60_000_000), t + 61_000_000))

    def test_unsigned_stale_wrong_session_and_forged_clearance_rejected(self):
        for change, delay in (
            ({"whole_envelope_clear": False}, 0),
            ({"whole_envelope_observed": False}, 0),
            ({"independent_stop_proven": False}, 0),
            ({"stop_distance_measured": False}, 0),
            ({"command_owner_exclusive": False}, 0),
            ({"motor_output_interlock_proven": False}, 0),
            ({"session_id": "e"*32}, 0),
            ({"free_clearance_margin_m": 0.03}, 0),
            ({"max_yaw_rate_rad_s": 0.2}, 0),
            ({"stopped_verified": "yes"}, 0),
            ({}, 200_000_000),
        ):
            with self.subTest(change=change, delay=delay):
                v = SafetyLeaseVerifier(
                    safety_key=SAFETY_KEY, bridge_session_id=SESSION)
                self.assertIsNone(v.accept(
                    signed_lease(1, BASE, **change), BASE + delay))
        raw = json.loads(signed_lease(1, BASE))
        raw["whole_envelope_clear"] = False
        v = SafetyLeaseVerifier(safety_key=SAFETY_KEY,
                                bridge_session_id=SESSION)
        self.assertIsNone(v.accept(json.dumps(raw), BASE))
        raw = signed_lease(1, BASE).replace(
            '"sequence": 1', '"sequence": 1, "sequence": 2')
        self.assertIsNone(v.accept(raw, BASE))
        self.assertIsNone(v.accept('{"schema":NaN}', BASE))

    def test_bounded_nonzero_motor_output_only_under_renewed_lease(self):
        v = SafetyLeaseVerifier(
            safety_key=SAFETY_KEY, bridge_session_id=SESSION)
        c = MotorYawCalibration(SESSION)
        t = BASE
        self.assertTrue(c.fresh_lease(v.accept(signed_lease(1, t), t), t))
        self.assertTrue(c.start())
        first = c.step(heading(t, 0), t)
        self.assertEqual(first.wz_rad_s, 0.0)
        self.assertEqual(first.state, "HOLD_ZERO")
        t += 50_000_000
        self.assertTrue(c.fresh_lease(
            v.accept(signed_lease(2, t), t), t))
        second = c.step(heading(t, 0), t)
        self.assertLess(second.wz_rad_s, 0.0)
        self.assertGreaterEqual(second.wz_rad_s, -0.01)
        self.assertFalse(second.operator_or_motor_rearm_required)
        self.assertEqual(second.waypoint_index, 0)

    def test_monitor_loss_zeroes_and_latches_without_rearm(self):
        v = SafetyLeaseVerifier(
            safety_key=SAFETY_KEY, bridge_session_id=SESSION)
        c = MotorYawCalibration(SESSION)
        t = BASE
        self.assertTrue(c.fresh_lease(v.accept(signed_lease(1, t), t), t))
        self.assertTrue(c.start())
        c.step(heading(t, 0), t)
        t += 50_000_000
        self.assertTrue(c.fresh_lease(
            v.accept(signed_lease(2, t), t), t))
        self.assertLess(c.step(heading(t, 0), t).wz_rad_s, 0)
        lost = c.step(heading(t+200_000_000, 0),
                      t+200_000_000)
        self.assertEqual((lost.state, lost.wz_rad_s),
                         ("LATCHED", 0.0))
        self.assertFalse(c.start())

    def test_tampered_or_replayed_safety_lease_cannot_continue(self):
        v = SafetyLeaseVerifier(
            safety_key=SAFETY_KEY, bridge_session_id=SESSION)
        c = MotorYawCalibration(SESSION)
        t = BASE
        good = v.accept(signed_lease(1, t), t)
        self.assertTrue(c.fresh_lease(good, t))
        self.assertTrue(c.start())
        self.assertFalse(c.fresh_lease(good, t+20_000_000))
        self.assertEqual(c.step(heading(t+20_000_000, 0),
                                t+20_000_000).wz_rad_s, 0)
        self.assertEqual(c.fault, "SAFETY_LEASE_REPLAY")

    def test_signed_motor_ros_adapter_has_exclusive_imu_source(self):
        source = (ROOT / "live_actuated_yaw_calibration.py").read_text()
        ast.parse(source)
        self.assertIn('IMU_OWNER = ("leroy_bridge"', source)
        self.assertIn("sign_motion(", source)
        self.assertIn("sign_rearm(", source)
        self.assertIn("MAX_ACTUATION_WITHOUT_YAW_NS", source)
        self.assertIn("STOP_QUIET_RAD_S", source)
        self.assertIn("if self.motor.finish_with_stop_confirmation(", source)
        self.assertIn("self.publish_signed_motion(0.0)", source)
        self.assertNotIn("serial.Serial(", source)
        self.assertNotIn("set_wheels", source)
        self.assertNotIn("create_publisher(Twist", source)
        self.assertIn('SIGNED_MOTION = "/leroy/motor/signed_motion"', source)
        self.assertIn('LEASE_TOPIC = "/leroy/safety/rotation_lease"', source)

    def test_actuating_launcher_cannot_map_cdc_or_bypass_monitor(self):
        script = ROOT / "live_actuated_yaw_calibration.sh"
        subprocess.run(["bash", "-n", str(script)], check=True)
        s = script.read_text()
        self.assertIn("monitor=leroy-physical-safety-monitor", s)
        self.assertIn("bridge=leroy-signed-bridge", s)
        self.assertIn("ARM_PHYSICAL_AUTONOMOUS_YAW", s)
        self.assertIn("--userns=keep-id", s)
        self.assertIn("readonly", s)
        self.assertIn("--cap-drop=all", s)
        self.assertIn("LEROY_SAFETY_MONITOR_VERIFY_KEY_PATH", s)
        self.assertNotIn("--device=", s)
        self.assertNotIn("--privileged", s)


if __name__ == "__main__":
    unittest.main()
