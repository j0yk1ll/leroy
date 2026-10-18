"""M22J offline exact production bridge audit method, WITHOUT ROS or USB."""
import ast
from dataclasses import asdict, replace
from pathlib import Path
import unittest

from leroy_bridge.authorized_session import (
    SessionVerifier, sign_test_motion, sign_test_rearm,
)
from leroy_bridge.bridge_signed_audit import (
    BridgeSignedAudit, SignedAuditResult,
)
from leroy_bridge.command_ingress import CommandRejected, command_to_bytes
from test_m22d_command_ingress import bridge_class, method_named, source_tree

NOW = 10_000_000_000
KEY_MOTION = b"m" * 32
KEY_OPERATOR = b"o" * 32
HERE = Path(__file__).resolve().parents[1]
NODE = HERE / "leroy_bridge" / "node.py"
AUDIT_SOURCE = HERE / "leroy_bridge" / "bridge_signed_audit.py"


def make():
    session = SessionVerifier(
        motion_key=KEY_MOTION, operator_key=KEY_OPERATOR)
    return session, BridgeSignedAudit(session)


def motion(session, seq=0, now=NOW, *, key=KEY_MOTION):
    return sign_test_motion(
        key=key, session_id=session.session_id,
        sequence=seq, generated_ns=now - 10_000_000,
        expires_ns=now + 150_000_000)


def rearm(session, counter=0, now=NOW, *, key=KEY_OPERATOR):
    return sign_test_rearm(
        key=key, session_id=session.session_id,
        counter=counter, generated_ns=now - 10_000_000,
        expires_ns=now + 150_000_000)


def exact_production_method():
    target = method_named(
        bridge_class(source_tree()), "_inspect_signed_diagnostic")
    module = ast.fix_missing_locations(
        ast.Module(body=[target], type_ignores=[]))
    result = {}
    exec(compile(module, str(NODE), "exec"),
         {"BridgeSignedAudit": BridgeSignedAudit}, result)
    return result["_inspect_signed_diagnostic"]


class PoisonBridge:
    def __init__(self):
        self.serial_writes_attempted = 0

    def _send_command(self, _command):
        self.serial_writes_attempted += 1
        raise AssertionError("M22J audit attempted serial I/O")


class M22JBridgeAuditTests(unittest.TestCase):
    def test_valid_signed_command_is_diagnostic_never_motion(self):
        session, gate = make()
        result = gate.inspect(motion=motion(session), now_ns=NOW)
        self.assertIs(type(result), SignedAuditResult)
        self.assertTrue(result.command_authenticated)
        self.assertEqual(result.command_reason,
                         "MOTION_ENVELOPE_VERIFIED_SYNTHETIC")
        self.assertFalse(result.physical_motion_authorized)
        self.assertTrue(result.diagnostic_only)

    def test_valid_signed_rearm_does_not_enable_motion(self):
        session, gate = make()
        result = gate.inspect(
            motion=motion(session), rearm=rearm(session), now_ns=NOW)
        self.assertTrue(result.command_authenticated)
        self.assertTrue(result.operator_grant_authenticated)
        self.assertFalse(result.physical_motion_authorized)

    def test_missing_rearm_is_not_treated_as_operator_grant(self):
        session, gate = make()
        result = gate.inspect(motion=motion(session), now_ns=NOW)
        self.assertFalse(result.operator_grant_authenticated)
        self.assertEqual(result.rearm_reason, "REARM_NOT_SUBMITTED")

    def test_wrong_motion_key_rejected(self):
        session, gate = make()
        result = gate.inspect(
            motion=motion(session, key=b"u" * 32), now_ns=NOW)
        self.assertFalse(result.command_authenticated)
        self.assertEqual(result.command_reason, "BLOCKED_MOTION_AUTH")
        self.assertFalse(result.physical_motion_authorized)

    def test_wrong_operator_key_rejected_without_motion_side_effect(self):
        session, gate = make()
        result = gate.inspect(
            motion=motion(session),
            rearm=rearm(session, key=b"u" * 32),
            now_ns=NOW)
        self.assertTrue(result.command_authenticated)
        self.assertFalse(result.operator_grant_authenticated)
        self.assertEqual(result.rearm_reason, "BLOCKED_REARM_AUTH")
        self.assertFalse(result.physical_motion_authorized)

    def test_motion_replay_is_blocked(self):
        session, gate = make()
        pkt = motion(session)
        self.assertTrue(gate.inspect(
            motion=pkt, now_ns=NOW).command_authenticated)
        duplicate = gate.inspect(motion=pkt, now_ns=NOW + 1)
        self.assertFalse(duplicate.command_authenticated)
        self.assertEqual(duplicate.command_reason, "BLOCKED_MOTION_REPLAY")

    def test_rearm_replay_is_blocked(self):
        session, gate = make()
        pkt = rearm(session)
        first = gate.inspect(
            motion=motion(session), rearm=pkt, now_ns=NOW)
        self.assertTrue(first.operator_grant_authenticated)
        second = gate.inspect(
            motion=motion(session, seq=1, now=NOW + 100_000_000),
            rearm=pkt, now_ns=NOW + 100_000_000)
        self.assertFalse(second.operator_grant_authenticated)
        self.assertEqual(second.rearm_reason, "BLOCKED_REARM_REPLAY")

    def test_future_and_expired_envelopes_rejected(self):
        for at in (NOW - 50_000_000, NOW + 200_000_000):
            with self.subTest(at=at):
                session, gate = make()
                result = gate.inspect(motion=motion(session), now_ns=at)
                self.assertFalse(result.command_authenticated)
                self.assertEqual(result.command_reason,
                                 "BLOCKED_MOTION_FRESHNESS")

    def test_prior_session_rejected_after_new_verifier(self):
        old, _gate = make()
        prior = motion(old)
        fresh, fresh_gate = make()
        self.assertNotEqual(old.session_id, fresh.session_id)
        result = fresh_gate.inspect(motion=prior, now_ns=NOW)
        self.assertFalse(result.command_authenticated)
        self.assertEqual(result.command_reason, "BLOCKED_SESSION_OR_OWNER")

    def test_tampered_signed_motion_rejected(self):
        session, gate = make()
        signed = motion(session)
        result = gate.inspect(
            motion=replace(signed, vx=0.13), now_ns=NOW)
        self.assertFalse(result.command_authenticated)
        self.assertEqual(result.command_reason, "BLOCKED_MOTION_AUTH")

    def test_exact_production_method_never_calls_serial(self):
        session, gate = make()
        fake = PoisonBridge()
        execute = exact_production_method()
        result = execute(fake, gate, motion=motion(session),
                         rearm=rearm(session), now_ns=NOW)
        self.assertTrue(result.command_authenticated)
        self.assertTrue(result.operator_grant_authenticated)
        self.assertFalse(result.physical_motion_authorized)
        self.assertEqual(fake.serial_writes_attempted, 0)

    def test_production_method_rejects_untrusted_audit_object(self):
        session, _gate = make()
        fake = PoisonBridge()
        with self.assertRaises(TypeError):
            exact_production_method()(
                fake, object(), motion=motion(session), now_ns=NOW)
        self.assertEqual(fake.serial_writes_attempted, 0)

    def test_result_contains_no_secret_key_or_actuator_command(self):
        session, gate = make()
        report = asdict(gate.inspect(
            motion=motion(session), rearm=rearm(session), now_ns=NOW))
        self.assertEqual(set(report), {
            "command_authenticated", "operator_grant_authenticated",
            "command_reason", "rearm_reason",
            "physical_motion_authorized", "diagnostic_only",
        })
        self.assertNotIn("key", repr(report).lower())
        self.assertNotIn("set_wheels", repr(report))

    def test_library_and_bridge_have_no_motion_bypass(self):
        library = AUDIT_SOURCE.read_text()
        node = NODE.read_text()
        imports = set()
        for item in ast.walk(ast.parse(library)):
            if isinstance(item, ast.Import):
                imports.update(n.name.split(".")[0] for n in item.names)
            elif isinstance(item, ast.ImportFrom):
                imports.add((item.module or "").split(".")[0])
        self.assertEqual(imports, {"dataclasses", "leroy_bridge"})
        self.assertNotIn("serial", library.lower())
        self.assertNotIn("mecanum", library.lower())
        self.assertNotIn("set_wheels", library)
        self.assertNotIn("self._cmd_sub =", node)
        self.assertIn("payload = command_to_bytes(command)", node)
        self.assertEqual(node.count("self._serial.write("), 1)
        assert "_inspect_signed_diagnostic" in node
        with self.assertRaises(CommandRejected):
            command_to_bytes("set_wheels 1 0 0 0")


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M22JBridgeAuditTests))
    if result.wasSuccessful():
        print("M22J_BRIDGE_SIGNED_AUDIT_SELFTEST=PASS", flush=True)
        print("M22J_VERDICT=ACTUAL_BRIDGE_AUDIT_NO_ACTUATOR_OUTPUT",
              flush=True)
        print("M22J_PRODUCTION_MOTION_ENABLED=FALSE", flush=True)
        print("M22J_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22J_BRIDGE_SIGNED_AUDIT_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
