"""M22I offline signer role isolation + fake authorization regressions.

All signing credentials are TEST-ONLY, ephemeral and held by an
unprivileged parent. Child process separation is NOT operator authentication.
No ROS, pyserial, physical devices or production bridge execution.
"""
from dataclasses import replace
from pathlib import Path
import os
import time
import unittest

from leroy_bridge.authorized_session import MotionEnvelope, RearmEnvelope
from leroy_bridge.command_ingress import CommandRejected, command_to_bytes
from m22a_diagnostic_authority import Inputs
from m22c_fake_serial_boundary import FakeSerialTransport, ZERO_PWM
from m22e_fake_signed_ingress import FakeSignedIngress
from m22i_isolated_role_signing import (
    FakeSessionSupervisor, SignerUnavailable, isolated_role_report,
)

HERE = Path(__file__).resolve().parent
HEALTH = Inputs(True, True, True, True, True)
OWNER = ("controller_server",)
NOW = 10_000_000_000
ZERO_BYTES = b"set_wheels 0 0 0 0\n"


class M22IRoleIsolationTests(unittest.TestCase):
    def make(self):
        supervisor = FakeSessionSupervisor()
        self.addCleanup(supervisor.close)
        return supervisor

    def gate(self, supervisor):
        fake = FakeSerialTransport()
        gate = FakeSignedIngress(
            transport=fake, session=supervisor.verifier)
        return gate, fake

    def cycle(self, gate, command, now, *, rearm=None):
        return gate.cycle(
            now_ns=now, health=HEALTH, publishers=OWNER,
            command=command, rearm=rearm)

    def test_signers_live_in_distinct_os_processes(self):
        s = self.make()
        self.assertNotEqual(s.motion.pid, s.operator.pid)
        self.assertNotEqual(s.motion.pid, os.getpid())
        self.assertNotEqual(s.operator.pid, os.getpid())
        self.assertTrue(s.motion.alive)
        self.assertTrue(s.operator.alive)

    def test_motion_role_signatures_validate_only_as_motion(self):
        s = self.make()
        item = s.issue_motion(0, NOW)
        self.assertIs(type(item), MotionEnvelope)
        self.assertTrue(s.verifier.verify_motion(item, now_ns=NOW)[0])
        self.assertFalse(s.verifier.verify_rearm(item, now_ns=NOW)[0])

    def test_operator_role_signatures_validate_only_as_rearm(self):
        s = self.make()
        grant = s.issue_rearm(0, NOW)
        self.assertIs(type(grant), RearmEnvelope)
        self.assertTrue(s.verifier.verify_rearm(grant, now_ns=NOW)[0])
        self.assertFalse(s.verifier.verify_motion(grant, now_ns=NOW)[0])

    def test_cross_role_signing_request_refused(self):
        s = self.make()
        with self.assertRaisesRegex(PermissionError, "CROSS_ROLE"):
            s.motion.sign(kind="rearm", counter=0, at_ns=NOW)
        with self.assertRaisesRegex(PermissionError, "CROSS_ROLE"):
            s.operator.sign(kind="motion", counter=0, at_ns=NOW)

    def test_malformed_signing_request_refused(self):
        s = self.make()
        with self.assertRaisesRegex(PermissionError, "BAD_SIGN_REQUEST"):
            s.motion.sign(kind="motion", counter=True, at_ns=NOW)
        with self.assertRaisesRegex(PermissionError, "BAD_SIGN_REQUEST"):
            s.operator.sign(kind="rearm", counter=-1, at_ns=NOW)
        with self.assertRaisesRegex(PermissionError, "BAD_SIGN_REQUEST"):
            s.motion.sign(kind="motion", counter=0, at_ns=0)

    def test_reply_is_signed_envelope_without_raw_key_material(self):
        s = self.make()
        motion = s.issue_motion(0, NOW)
        grant = s.issue_rearm(0, NOW)
        self.assertEqual(len(motion.mac), 64)
        self.assertEqual(len(grant.mac), 64)
        self.assertFalse(any(
            "key" in field for field in vars(motion)))
        self.assertFalse(any(
            "key" in field for field in vars(grant)))
        report = isolated_role_report()
        self.assertIs(report["credentials_production_ready"], False)
        self.assertIs(report["device_access"], False)

    def test_replayed_motion_sequence_is_rejected(self):
        s = self.make()
        command = s.issue_motion(0, NOW)
        self.assertTrue(s.verifier.verify_motion(command, now_ns=NOW)[0])
        ok, reason = s.verifier.verify_motion(command, now_ns=NOW + 1)
        self.assertFalse(ok)
        self.assertEqual(reason, "BLOCKED_MOTION_REPLAY")

    def test_replayed_operator_grant_is_rejected(self):
        s = self.make()
        grant = s.issue_rearm(0, NOW)
        self.assertTrue(s.verifier.verify_rearm(grant, now_ns=NOW)[0])
        ok, reason = s.verifier.verify_rearm(grant, now_ns=NOW + 1)
        self.assertFalse(ok)
        self.assertEqual(reason, "BLOCKED_REARM_REPLAY")

    def test_forged_signed_field_is_rejected(self):
        s = self.make()
        motion = s.issue_motion(0, NOW)
        ok, reason = s.verifier.verify_motion(
            replace(motion, vx=0.13), now_ns=NOW)
        self.assertFalse(ok)
        self.assertEqual(reason, "BLOCKED_MOTION_AUTH")

    def test_expired_and_future_commands_are_rejected(self):
        s = self.make()
        motion = s.issue_motion(0, NOW)
        ok, reason = s.verifier.verify_motion(
            motion, now_ns=NOW + 200_000_000)
        self.assertFalse(ok)
        self.assertEqual(reason, "BLOCKED_MOTION_FRESHNESS")
        ok, reason = s.verifier.verify_motion(
            motion, now_ns=NOW - 20_000_000)
        self.assertFalse(ok)
        self.assertEqual(reason, "BLOCKED_MOTION_FRESHNESS")

    def test_rotation_restarts_processes_keys_and_session(self):
        s = self.make()
        old_motion = s.issue_motion(0, NOW)
        old_grant = s.issue_rearm(0, NOW)
        old_id = s.session_id
        old_motion_pid = s.motion.pid
        old_operator_pid = s.operator.pid
        s.rotate_session()
        self.assertNotEqual(s.session_id, old_id)
        self.assertNotEqual(s.motion.pid, old_motion_pid)
        self.assertNotEqual(s.operator.pid, old_operator_pid)
        self.assertEqual(
            s.verifier.verify_motion(old_motion, now_ns=NOW)[1],
            "BLOCKED_SESSION_OR_OWNER")
        self.assertEqual(
            s.verifier.verify_rearm(old_grant, now_ns=NOW)[1],
            "BLOCKED_REARM_FORMAT")
        self.assertTrue(s.verifier.verify_motion(
            s.issue_motion(0, NOW), now_ns=NOW)[0])

    def test_close_or_crashed_signer_cannot_issue_commands(self):
        s = self.make()
        s.motion.close()
        self.assertFalse(s.motion.alive)
        with self.assertRaises(SignerUnavailable):
            s.issue_motion(0, NOW)
        s.operator._process.terminate()  # Deliberate test-only crash.
        s.operator._process.join(timeout=3)
        with self.assertRaises(SignerUnavailable):
            s.issue_rearm(0, NOW)

    def test_new_fake_session_requires_explicit_rearm_and_dwell(self):
        s = self.make()
        gate, fake = self.gate(s)
        initial = self.cycle(
            gate, s.issue_motion(0, NOW), NOW)
        self.assertEqual(initial.state, "LATCHED")
        self.assertEqual(initial.pwm, ZERO_PWM)
        rearm = self.cycle(
            gate, s.issue_motion(1, NOW + 100_000_000),
            NOW + 100_000_000,
            rearm=s.issue_rearm(0, NOW + 100_000_000))
        self.assertEqual(rearm.state, "REARMING")
        for i in range(2, 11):
            at = NOW + i * 100_000_000
            r = self.cycle(gate, s.issue_motion(i, at), at)
            self.assertEqual(r.pwm, ZERO_PWM)
        armed_at = NOW + 1_100_000_000
        active = self.cycle(
            gate, s.issue_motion(11, armed_at), armed_at)
        self.assertEqual(active.state, "ARMED")
        self.assertNotEqual(active.pwm, ZERO_PWM)
        self.assertTrue(any(
            p != ZERO_BYTES for p in fake.writes))
        self.assertEqual(fake.writes[-1][:11], b"set_wheels ")

    def test_missing_child_command_latches_no_replay(self):
        s = self.make()
        gate, fake = self.gate(s)
        first = self.cycle(gate, s.issue_motion(0, NOW), NOW,
                           rearm=s.issue_rearm(0, NOW))
        self.assertEqual(first.state, "REARMING")
        s.motion.close()
        with self.assertRaises(SignerUnavailable):
            s.issue_motion(1, NOW + 100_000_000)
        result = self.cycle(gate, None, NOW + 100_000_000)
        self.assertEqual(result.state, "LATCHED")
        self.assertEqual(result.pwm, ZERO_PWM)
        self.assertEqual(fake.writes[-1], ZERO_BYTES)

    def test_production_writer_retains_zero_only_lockout(self):
        with self.assertRaises(CommandRejected):
            command_to_bytes("set_wheels 1 0 0 0")
        self.assertEqual(
            command_to_bytes("set_wheels 0 0 0 0"), ZERO_BYTES)
        source = (HERE / "src/leroy_bridge/leroy_bridge/node.py").read_text()
        self.assertNotIn("self._cmd_sub =", source)
        self.assertIn("payload = command_to_bytes(command)", source)
        self.assertIn("self._transport_fault = threading.Event()", source)

    def test_source_does_not_use_ros_serial_or_device_access(self):
        source = (HERE / "m22i_isolated_role_signing.py").read_text()
        self.assertNotIn("import rclpy", source)
        self.assertNotIn("import serial", source)
        self.assertNotIn("/dev/", source)
        self.assertNotIn("subprocess", source)
        self.assertNotIn("os.environ", source)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M22IRoleIsolationTests))
    if result.wasSuccessful():
        print("M22I_INTERPROCESS_SIGNER_SELFTEST=PASS", flush=True)
        print("M22I_VERDICT=SYNTHETIC_EPHEMERAL_ROLE_SEPARATION_ONLY",
              flush=True)
        print("M22I_PRODUCTION_MOTION_ENABLED=FALSE", flush=True)
        print("M22I_OPERATOR_AUTHENTICATION_IMPLEMENTED=FALSE", flush=True)
        print("M22I_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22I_INTERPROCESS_SIGNER_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
