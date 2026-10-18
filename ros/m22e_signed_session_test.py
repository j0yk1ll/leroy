"""M22E adversarial signed-session authority -> fake CDC regression.

No ROS, serial.Serial, motor I/O, or device discovery. All keys are fake
test-only bytes. No secret provisioning or operator authentication claimed.
"""
import ast
from dataclasses import replace
from pathlib import Path
import unittest

from leroy_bridge.authorized_session import (
    MotionEnvelope, RearmEnvelope, SessionVerifier, OWNER,
    sign_test_motion, sign_test_rearm,
)
from leroy_bridge.command_ingress import CommandRejected, command_to_bytes
from m22a_diagnostic_authority import Inputs
from m22c_fake_serial_boundary import FakeSerialTransport, ZERO_PWM
from m22e_fake_signed_ingress import FakeSignedIngress

HERE = Path(__file__).resolve().parent
BASE = 10_000_000_000
GOOD = Inputs(True, True, True, True, True)
SOURCES = (OWNER,)
MOTION_KEY = b"m" * 32
OPERATOR_KEY = b"o" * 32
UNAUTHORIZED_KEY = b"x" * 32


def setup_gate():
    session = SessionVerifier(
        motion_key=MOTION_KEY, operator_key=OPERATOR_KEY)
    serial = FakeSerialTransport()
    gate = FakeSignedIngress(transport=serial, session=session)
    return gate, serial, session


def packet(session, seq, now, *, age=10_000_000, expiry=100_000_000,
           key=MOTION_KEY, **fields):
    return sign_test_motion(
        key=key, session_id=session.session_id, sequence=seq,
        generated_ns=now - age, expires_ns=now + expiry, **fields)


def grant(session, counter, now, *, key=OPERATOR_KEY):
    return sign_test_rearm(
        key=key, session_id=session.session_id, counter=counter,
        generated_ns=now - 10_000_000, expires_ns=now + 200_000_000)


def cycle(gate, session, seq, now, *, inputs=GOOD, sources=SOURCES,
          rearm=None, envelope=None):
    return gate.cycle(
        now_ns=now, health=inputs, publishers=sources,
        command=packet(session, seq, now) if envelope is None else envelope,
        rearm=rearm)


def armed():
    gate, fake, session = setup_gate()
    first = cycle(gate, session, 0, BASE, rearm=grant(session, 0, BASE))
    assert first.state == "REARMING" and first.pwm == ZERO_PWM
    for i in range(1, 10):
        tick = cycle(gate, session, i, BASE + i * 100_000_000)
        assert tick.state == "REARMING" and tick.pwm == ZERO_PWM
    finished = cycle(gate, session, 10, BASE + 1_000_000_000)
    assert finished.state == "ARMED" and finished.pwm != ZERO_PWM
    return gate, fake, session


class SignedSessionTests(unittest.TestCase):
    def assert_blocked(self, result, reason=None):
        self.assertEqual(result.pwm, ZERO_PWM)
        self.assertEqual(result.state, "LATCHED")
        if reason is not None:
            self.assertEqual(result.reason, reason)

    def test_startup_latched_with_no_remote_or_implicit_rearm(self):
        gate, fake, session = setup_gate()
        r0 = cycle(gate, session, 0, BASE)
        self.assert_blocked(r0, "BLOCKED_MANUAL_REARM_REQUIRED")
        r1 = cycle(gate, session, 1, BASE + 100_000_000)
        self.assert_blocked(r1, "BLOCKED_MANUAL_REARM_REQUIRED")
        self.assertEqual(fake.writes, [b"set_wheels 0 0 0 0\n"] * 2)

    def test_valid_separate_key_rearm_dwell_produces_only_fake_pwm(self):
        gate, fake, _session = armed()
        self.assertEqual(gate.state, "ARMED")
        self.assertEqual(len(fake.writes), 11)
        self.assertNotEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")
        self.assertTrue(fake.writes[-1].startswith(b"set_wheels "))
        self.assertTrue(all(
            x == b"set_wheels 0 0 0 0\n" for x in fake.writes[:-1]))

    def test_session_id_changes_on_restart_and_old_packet_is_invalid(self):
        gate, _fake, old_session = armed()
        _new_gate, _new_fake, new_session = setup_gate()
        self.assertNotEqual(old_session.session_id, new_session.session_id)
        past = packet(old_session, 11, BASE + 1_100_000_000)
        r = cycle(_new_gate, new_session, 0, BASE + 1_100_000_000,
                  envelope=past)
        self.assert_blocked(r, "BLOCKED_SESSION_OR_OWNER")
        self.assertFalse(_new_fake.writes[-1] != b"set_wheels 0 0 0 0\n")

    def test_motion_hmac_tamper_blocks_writes(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        valid = packet(session, 11, now)
        forged = replace(valid, vx=0.08)
        result = cycle(gate, session, 11, now, envelope=forged)
        self.assert_blocked(result, "BLOCKED_MOTION_AUTH")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_wrong_secret_blocks_motion(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        r = cycle(gate, session, 11, now,
                  envelope=packet(session, 11, now, key=UNAUTHORIZED_KEY))
        self.assert_blocked(r, "BLOCKED_MOTION_AUTH")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_duplicate_sequence_and_reordered_generation_block(self):
        for item, why in (
            ("sequence", "BLOCKED_MOTION_REPLAY"),
            ("generation", "BLOCKED_MOTION_REPLAY"),
        ):
            with self.subTest(item=item):
                gate, fake, session = armed()
                now = BASE + 1_100_000_000
                command = packet(
                    session, 10 if item == "sequence" else 11, now,
                    age=110_000_000 if item == "generation" else 10_000_000)
                r = cycle(gate, session, 11, now, envelope=command)
                self.assert_blocked(r, why)
                self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_stale_expired_or_future_command_blocks(self):
        for age, expiry in ((300_000_000, -50_000_000), (-1, 100_000_000),
                            (10_000_000, 300_000_000)):
            with self.subTest(age=age, expiry=expiry):
                gate, fake, session = armed()
                now = BASE + 1_100_000_000
                r = cycle(gate, session, 11, now,
                          envelope=packet(session, 11, now,
                                          age=age, expiry=expiry))
                self.assert_blocked(r, "BLOCKED_MOTION_FRESHNESS")
                self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_absent_and_wrong_type_envelope_block(self):
        for value in (False, {}, "signed", 13):
            with self.subTest(value=value):
                gate, fake, session = armed()
                now = BASE + 1_100_000_000
                r = cycle(gate, session, 11, now, envelope=value)
                self.assert_blocked(r, "BLOCKED_ABSENT_OR_BAD_ENVELOPE")
                self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_wrong_owner_even_with_valid_hmac_blocks(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        r = cycle(gate, session, 11, now, envelope=packet(
            session, 11, now, owner="other_controller"))
        self.assert_blocked(r, "BLOCKED_SESSION_OR_OWNER")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_competing_publishers_latch_and_never_auto_rearm(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        r = cycle(gate, session, 11, now,
                  sources=(OWNER, "unauthorized"))
        self.assert_blocked(r, "BLOCKED_COMMAND_OWNERSHIP")
        recovered = cycle(gate, session, 12, now + 100_000_000)
        self.assert_blocked(recovered, "BLOCKED_MANUAL_REARM_REQUIRED")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_unhealthy_sensor_latches_and_requires_new_grant(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        r = cycle(gate, session, 11, now, inputs=replace(
            GOOD, localization_valid=False))
        self.assert_blocked(r, "BLOCKED_SENSOR_HEALTH")
        r2 = cycle(gate, session, 12, now + 100_000_000)
        self.assert_blocked(r2, "BLOCKED_MANUAL_REARM_REQUIRED")
        rr = cycle(gate, session, 13, now + 200_000_000,
                   rearm=grant(session, 1, now + 200_000_000))
        self.assertEqual(rr.state, "REARMING")
        self.assertEqual(rr.pwm, ZERO_PWM)

    def test_bad_operator_signature_latches_even_when_motion_valid(self):
        gate, fake, session = setup_gate()
        forged = grant(session, 0, BASE, key=UNAUTHORIZED_KEY)
        result = cycle(gate, session, 0, BASE, rearm=forged)
        self.assert_blocked(result, "BLOCKED_REARM_AUTH")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_old_operator_grant_replay_rejected(self):
        gate, fake, session = setup_gate()
        rearm = grant(session, 0, BASE)
        r0 = cycle(gate, session, 0, BASE, rearm=rearm)
        self.assertEqual(r0.state, "REARMING")
        r1 = cycle(gate, session, 1, BASE + 100_000_000, rearm=rearm)
        self.assert_blocked(r1, "BLOCKED_REARM_REPLAY")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_old_operator_grant_from_other_session_rejected(self):
        _old_gate, _old_fake, old_session = setup_gate()
        gate, fake, session = setup_gate()
        foreign_grant = grant(old_session, 0, BASE)
        r = cycle(gate, session, 0, BASE, rearm=foreign_grant)
        self.assert_blocked(r, "BLOCKED_REARM_FORMAT")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_operator_grant_while_armed_is_not_extra_permission(self):
        gate, fake, session = armed()
        now = BASE + 1_100_000_000
        r = cycle(gate, session, 11, now, rearm=grant(session, 1, now))
        self.assert_blocked(r, "BLOCKED_UNEXPECTED_REARM")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_clock_evaluator_gap_fails_closed(self):
        gate, fake, session = armed()
        now = BASE + 1_200_000_001
        r = cycle(gate, session, 11, now)
        self.assert_blocked(r, "BLOCKED_EVALUATOR_STALL")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_nan_signing_rejected_before_command_creation(self):
        _, _, session = setup_gate()
        with self.assertRaises(ValueError):
            packet(session, 0, BASE, vx=float("nan"))
        with self.assertRaises(ValueError):
            packet(session, 0, BASE, vx=float("inf"))

    def test_fake_serial_error_terminal_latch_without_retry(self):
        gate, fake, session = armed()
        before = len(fake.writes)
        fake.fail_next = True
        now = BASE + 1_100_000_000
        fail = cycle(gate, session, 11, now)
        self.assert_blocked(fail, "BLOCKED_FAKE_CDC_WRITE_FAILURE")
        self.assertTrue(fail.fake_transport_fault_latched)
        self.assertFalse(fail.fake_write_completed)
        self.assertEqual(len(fake.writes), before)
        again = cycle(gate, session, 12, now + 100_000_000,
                      rearm=grant(session, 1, now + 100_000_000))
        self.assert_blocked(again, "BLOCKED_FAKE_CDC_LATCHED")
        self.assertEqual(len(fake.writes), before)

    def test_key_separation_and_format_checks(self):
        with self.assertRaises(ValueError):
            SessionVerifier(motion_key=MOTION_KEY, operator_key=MOTION_KEY)
        with self.assertRaises(ValueError):
            SessionVerifier(motion_key=b"short", operator_key=OPERATOR_KEY)
        gate, fake, session = setup_gate()
        now = BASE
        invalid = replace(packet(session, 0, now), sequence=True)
        r = cycle(gate, session, 0, now, envelope=invalid)
        self.assert_blocked(r, "BLOCKED_MOTION_FORMAT")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_m22d_production_writer_remains_non_actuating(self):
        for cmd in ("set_wheels 1 0 0 0",
                    "set_wheels 0 -20 0 0", "set_watchdog_ms 0"):
            with self.assertRaises(CommandRejected):
                command_to_bytes(cmd)
        self.assertEqual(command_to_bytes("set_wheels 0 0 0 0"),
                         b"set_wheels 0 0 0 0\n")
        node = (HERE / "src/leroy_bridge/leroy_bridge/node.py").read_text()
        tree = ast.parse(node)
        cls = next(x for x in tree.body
                   if isinstance(x, ast.ClassDef)
                   and x.name == "LeroyBridgeNode")
        writes = [n for n in ast.walk(cls)
                  if isinstance(n, ast.Call)
                  and isinstance(n.func, ast.Attribute)
                  and n.func.attr == "write"]
        self.assertEqual(len(writes), 1)
        self.assertNotIn("self._cmd_sub =", node)
        self.assertIn("payload = command_to_bytes(command)", node)

    def test_m22e_does_not_import_physical_io(self):
        for path in (
            HERE / "m22e_fake_signed_ingress.py",
            HERE / "src/leroy_bridge/leroy_bridge/authorized_session.py",
        ):
            tree = ast.parse(path.read_text())
            imports = set()
            for n in ast.walk(tree):
                if isinstance(n, ast.Import):
                    imports.update(x.name.split(".")[0] for x in n.names)
                elif isinstance(n, ast.ImportFrom):
                    imports.add((n.module or "").split(".")[0])
            self.assertFalse(imports & {
                "serial", "rclpy", "subprocess", "socket", "os",
            })
        self.assertFalse(
            (HERE / "m22e_fake_signed_ingress.py").read_text().find(
                "FakeSerialTransport") < 0)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(SignedSessionTests))
    if result.wasSuccessful():
        print("M22E_SIGNED_SESSION_FAKE_INGRESS_SELFTEST=PASS", flush=True)
        print("M22E_VERDICT=OFFLINE_SIGNED_SESSION_FAKE_CDC_ONLY", flush=True)
        print("M22E_PRODUCTION_NONZERO_CDC_ENABLED=FALSE", flush=True)
        print("M22E_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22E_SIGNED_SESSION_FAKE_INGRESS_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
