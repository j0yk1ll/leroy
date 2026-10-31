"""Focused regression for the motor authority core (no ROS or serial)."""
import unittest

from leroy_mcu.authorized_session import (
    SessionVerifier, sign_test_motion, sign_test_rearm,
)
from leroy_mcu.motor_authority import (
    MotorAuthority, ZERO_WHEELS, HARD_PWM_CEILING,
)
from leroy_mcu.motor_signed_ingress import (
    MotorInputRejected, parse_signed_envelope,
)
from leroy_mcu.authorized_session import MotionEnvelope

GOOD_HEALTH = {
    "odom_fresh": True,
    "depth_fresh": True,
    "tf_fresh": True,
    "costmap_valid": True,
    "localization_valid": True,
}
INTERLOCKS = {
    "estop_released": True,
    "motor_rails_verified": True,
    "transport_healthy": True,
    "clock_healthy": True,
    "bridge_running": True,
    "single_controller": True,
}


class MotorAuthorityTest(unittest.TestCase):
    def setUp(self):
        self.key = b"a" * 32
        self.operator_key = b"b" * 32
        self.verifier = SessionVerifier(
            motion_key=self.key, operator_key=self.operator_key)
        self.authority = MotorAuthority(
            verifier=self.verifier, wheel_radius_m=0.020,
            half_wheelbase_m=0.180, half_track_m=0.275,
            pwm_per_rad_s=12.75, max_pwm=HARD_PWM_CEILING)
        self.t = 5_000_000_000

    def grant(self, counter=1):
        return sign_test_rearm(
            key=self.operator_key, session_id=self.verifier.session_id,
            counter=counter, generated_ns=self.t,
            expires_ns=self.t + 100_000_000)

    def motion(self, seq=1, vx=0.04, vy=0.0, wz=0.07):
        return sign_test_motion(
            key=self.key, session_id=self.verifier.session_id,
            sequence=seq, generated_ns=self.t,
            expires_ns=self.t + 100_000_000,
            vx=vx, vy=vy, wz=wz)

    def arm(self):
        d = self.authority.rearm(
            grant=self.grant(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertEqual((d.state, d.packet),
                         ("REARMING", ZERO_WHEELS))
        for _ in range(10):
            self.t += 100_000_000
            d = self.authority.step(
                motion=None, now_ns=self.t, health=GOOD_HEALTH,
                **INTERLOCKS)
            self.assertEqual(d.packet, ZERO_WHEELS)
        self.assertEqual(d.state, "ARMED")

    def test_valid_signed_motion_generates_bounded_wheel_packet(self):
        self.arm()
        self.t += 100_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertTrue(d.nonzero_candidate)
        self.assertEqual(d.reason, "FRESH_SIGNED_CONTROLLER_COMMAND")
        self.assertTrue(d.packet.startswith("set_wheels "))
        self.assertEqual(len(d.wheel_pwm), 4)
        self.assertTrue(all(abs(v) <= HARD_PWM_CEILING for v in d.wheel_pwm))

    def test_signed_zero_is_fresh_one_use_serial_stop_command(self):
        # A signed zero has no nonzero_candidate, but MUST be forwarded
        # to firmware's set_wheels_tx zero -> platform_disable path.
        self.arm()
        self.t += 100_000_000
        decision = self.authority.step(
            motion=self.motion(seq=1, vx=0.0, vy=0.0, wz=0.0),
            now_ns=self.t, health=GOOD_HEALTH, **INTERLOCKS)
        self.assertEqual(decision.state, "ARMED")
        self.assertEqual(
            decision.reason, "FRESH_SIGNED_CONTROLLER_COMMAND")
        self.assertFalse(decision.nonzero_candidate)
        self.assertEqual(decision.wheel_pwm, (0, 0, 0, 0))
        self.assertEqual(
            self.authority.consume_serial_payload(
                decision, now_ns=self.t + 1_000_000),
            b"set_wheels 0 0 0 0\n")
        self.assertIsNone(
            self.authority.consume_serial_payload(
                decision, now_ns=self.t + 2_000_000))

    def test_signed_ros_json_schema_and_duplicate_key_guard(self):
        import json
        from dataclasses import asdict

        raw = json.dumps(asdict(self.motion()))
        decoded = parse_signed_envelope(raw, MotionEnvelope)
        self.assertEqual(decoded.session_id, self.verifier.session_id)
        self.assertEqual(decoded.vx, 0.04)
        self.assertTrue(self.verifier.verify_motion(
            decoded, now_ns=self.t)[0])
        for invalid in (
            raw[:-1] + ', "vx": 0.04}',
            raw.replace('"vx": 0.04', '"vx": 1'),
            raw.replace('"vx": 0.04', '"vx": NaN'),
        ):
            with self.assertRaises(MotorInputRejected):
                parse_signed_envelope(invalid, MotionEnvelope)

    def test_single_use_serial_handoff_cannot_be_replayed(self):
        self.arm()
        self.t += 100_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertTrue(d.nonzero_candidate)
        first = self.authority.consume_serial_payload(
            d, now_ns=self.t + 10_000_000)
        self.assertEqual(first, (d.packet + "\n").encode("ascii"))
        self.assertIsNone(self.authority.consume_serial_payload(
            d, now_ns=self.t + 20_000_000))

    def test_stale_or_forged_serial_handoff_rejected(self):
        from dataclasses import replace
        self.arm()
        self.t += 100_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        forged = replace(d)
        self.assertIsNone(self.authority.consume_serial_payload(
            forged, now_ns=self.t + 1_000_000))
        self.assertIsNone(self.authority.consume_serial_payload(
            d, now_ns=self.t + 101_000_000))
        self.assertEqual(self.authority.state, "LATCHED")

    def test_stop_invalidates_pending_serial_handoff(self):
        self.arm()
        self.t += 100_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.authority.stop()
        self.assertIsNone(self.authority.consume_serial_payload(
            d, now_ns=self.t + 1_000_000))

    def test_without_physical_interlock_no_rearm(self):
        guards = dict(INTERLOCKS, estop_released=False)
        d = self.authority.rearm(
            grant=self.grant(), now_ns=self.t, health=GOOD_HEALTH,
            **guards)
        self.assertFalse(d.nonzero_candidate)
        self.assertEqual(d.packet, ZERO_WHEELS)
        self.assertEqual(d.state, "LATCHED")

    def test_replay_latches_and_requires_new_operator_grant(self):
        self.arm()
        self.t += 100_000_000
        signed = self.motion()
        valid = self.authority.step(
            motion=signed, now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertTrue(valid.nonzero_candidate)
        self.t += 100_000_000
        rejected = self.authority.step(
            motion=signed, now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertEqual((rejected.state, rejected.packet),
                         ("LATCHED", ZERO_WHEELS))

    def test_keepalive_expires_without_fresh_signed_motion(self):
        self.arm()
        self.t += 100_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t, health=GOOD_HEALTH,
            **INTERLOCKS)
        self.assertTrue(d.nonzero_candidate)
        self.assertTrue(self.authority.heartbeat_allowed(
            now_ns=self.t + 50_000_000))
        self.assertFalse(self.authority.heartbeat_allowed(
            now_ns=self.t + 250_000_000))
        self.t += 100_000_000
        self.authority.stop()
        self.assertFalse(self.authority.heartbeat_allowed(
            now_ns=self.t))

    def test_stalled_authority_loop_latches(self):
        self.arm()
        self.t += 300_000_000
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t,
            health=GOOD_HEALTH, **INTERLOCKS)
        self.assertEqual((d.state, d.packet), ("LATCHED", ZERO_WHEELS))

    def test_sensor_health_loss_latches(self):
        self.arm()
        self.t += 100_000_000
        bad_health = dict(GOOD_HEALTH, localization_valid=False)
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t,
            health=bad_health, **INTERLOCKS)
        self.assertEqual((d.state, d.packet), ("LATCHED", ZERO_WHEELS))

    def test_transport_fault_and_shutdown_are_terminal(self):
        self.arm()
        self.t += 100_000_000
        guards = dict(INTERLOCKS, transport_healthy=False)
        d = self.authority.step(
            motion=self.motion(), now_ns=self.t,
            health=GOOD_HEALTH, **guards)
        self.assertEqual((d.state, d.packet), ("TERMINAL", ZERO_WHEELS))
        self.t += 100_000_000
        recovered = self.authority.rearm(
            grant=self.grant(counter=2), now_ns=self.t,
            health=GOOD_HEALTH, **INTERLOCKS)
        self.assertEqual(recovered.state, "TERMINAL")
        self.assertEqual(self.authority.stop().packet, ZERO_WHEELS)


if __name__ == "__main__":
    unittest.main()
