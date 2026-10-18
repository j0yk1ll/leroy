"""M22K: exact real-bridge diagnostic method + stateful authority, NO ROS.

The imported bridge module is NEVER instantiated. We AST-compile only
the actual production method and use a fake with no real serial handles.
"""
import ast
from dataclasses import asdict, FrozenInstanceError
from pathlib import Path
import threading
import unittest

from leroy_bridge.authorized_session import (
    SessionVerifier, sign_test_motion, sign_test_rearm,
)
from leroy_bridge.bridge_signed_audit import BridgeSignedAudit
from leroy_bridge.bridge_diagnostic_supervisor import (
    BridgeDiagnosticSupervisor, DiagnosticAssessment, RECOVERY_NS,
)
from leroy_bridge.command_ingress import CommandRejected, command_to_bytes
from test_m22d_command_ingress import (
    bridge_class, method_named, source_tree,
)

BASE = 10_000_000_000
MOTION_KEY = b"m" * 32
OPERATOR_KEY = b"o" * 32
HEALTH = {
    "odom_fresh": True, "depth_fresh": True, "tf_fresh": True,
    "costmap_valid": True, "localization_valid": True,
}
SOURCES = ("controller_server",)
NODE = Path(__file__).resolve().parents[1] / "leroy_bridge" / "node.py"


def signed(session, number, at, **changes):
    return sign_test_motion(
        key=changes.pop("key", MOTION_KEY),
        session_id=session.session_id,
        sequence=number, generated_ns=at - 10_000_000,
        expires_ns=at + 150_000_000, **changes)


def grant(session, number, at, *, key=OPERATOR_KEY):
    return sign_test_rearm(
        key=key, session_id=session.session_id,
        counter=number, generated_ns=at - 10_000_000,
        expires_ns=at + 150_000_000)


def exact_method():
    target = method_named(
        bridge_class(source_tree()), "_supervise_signed_diagnostic")
    compiled = ast.fix_missing_locations(
        ast.Module(body=[target], type_ignores=[]))
    namespace = {}
    exec(compile(compiled, str(NODE), "exec"),
         {"BridgeDiagnosticSupervisor": BridgeDiagnosticSupervisor},
         namespace)
    return namespace["_supervise_signed_diagnostic"]


class FakeBridge:
    def __init__(self):
        self._transport_fault = threading.Event()
        self._clock_fault = False
        self._stop_event = threading.Event()
        self.writes = 0

    def _send_command(self, _command):
        self.writes += 1
        raise AssertionError("diagnostic supervisor attempted CDC")

    def inspect(self, supervisor, *, motion, now, rearm=None,
                health=HEALTH, publishers=SOURCES):
        return exact_method()(
            self, supervisor, motion=motion, rearm=rearm,
            now_ns=now, health=health, publishers=publishers)


def make():
    sess = SessionVerifier(
        motion_key=MOTION_KEY, operator_key=OPERATOR_KEY)
    return sess, BridgeDiagnosticSupervisor(BridgeSignedAudit(sess)), FakeBridge()


def arm():
    sess, model, node = make()
    start = node.inspect(
        model, motion=signed(sess, 0, BASE),
        rearm=grant(sess, 0, BASE), now=BASE)
    assert start.state == "REARMING" and not start.diagnostic_permission
    for number in range(1, 10):
        at = BASE + number * 100_000_000
        result = node.inspect(
            model, motion=signed(sess, number, at), now=at)
        assert result.state == "REARMING"
        assert not result.diagnostic_permission
    at = BASE + RECOVERY_NS
    result = node.inspect(
        model, motion=signed(sess, 10, at), now=at)
    assert result.state == "ARMED" and result.diagnostic_permission
    assert not result.physical_motion_authorized
    return sess, model, node


class M22KBridgeSupervisorTests(unittest.TestCase):
    def assert_inhibited(self, result, reason=None):
        self.assertFalse(result.diagnostic_permission)
        self.assertFalse(result.physical_motion_authorized)
        self.assertTrue(result.diagnostic_only)
        self.assertEqual(result.state, "LATCHED")
        if reason is not None:
            self.assertEqual(result.reason, reason)

    def test_bridge_starts_inhibited_without_grant(self):
        sess, model, node = make()
        self.assert_inhibited(node.inspect(
            model, motion=signed(sess, 0, BASE), now=BASE),
            "BLOCKED_EXPLICIT_REARM_REQUIRED")
        self.assertEqual(node.writes, 0)

    def test_valid_rearm_requires_dwell_and_never_motors(self):
        sess, model, node = arm()
        self.assertEqual(model.state, "ARMED")
        self.assertEqual(node.writes, 0)
        self.assertEqual(model.last_fault, "INITIAL_INHIBIT")

    def test_rearm_dwell_cannot_be_skipped_by_backdated_grant(self):
        sess, model, node = make()
        result = node.inspect(
            model, motion=signed(sess, 0, BASE),
            rearm=grant(sess, 0, BASE), now=BASE)
        self.assertEqual(result.state, "REARMING")
        self.assertFalse(result.diagnostic_permission)
        self.assertEqual(model.healthy_since_ns, BASE)

    def test_new_command_must_arrive_at_each_tick(self):
        sess, model, node = arm()
        result = node.inspect(
            model, motion=None, now=BASE + 1_100_000_000)
        self.assert_inhibited(result, "BLOCKED_ABSENT_OR_BAD_ENVELOPE")

    def test_forged_motion_mac_latches(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at, key=b"x" * 32), now=at)
        self.assert_inhibited(result, "BLOCKED_MOTION_AUTH")

    def test_forged_operator_rearm_latches(self):
        sess, model, node = make()
        result = node.inspect(
            model, motion=signed(sess, 0, BASE),
            rearm=grant(sess, 0, BASE, key=b"x" * 32), now=BASE)
        self.assert_inhibited(result, "BLOCKED_REARM_AUTH")

    def test_replayed_signed_command_latches(self):
        sess, model, node = arm()
        result = node.inspect(
            model, motion=signed(sess, 10, BASE + RECOVERY_NS),
            now=BASE + 1_100_000_000)
        self.assert_inhibited(result, "BLOCKED_MOTION_REPLAY")

    def test_old_session_packet_latches(self):
        old, _, _ = make()
        sess, model, node = make()
        self.assertNotEqual(old.session_id, sess.session_id)
        result = node.inspect(
            model, motion=signed(old, 0, BASE), now=BASE)
        self.assert_inhibited(result, "BLOCKED_SESSION_OR_OWNER")

    def test_sensor_loss_invalidates_and_no_auto_rearm(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        unhealthy = dict(HEALTH, depth_fresh=False)
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at,
            health=unhealthy)
        self.assert_inhibited(result, "BLOCKED_SENSOR_HEALTH")
        recovered = node.inspect(
            model, motion=signed(sess, 12, at + 100_000_000),
            now=at + 100_000_000)
        self.assert_inhibited(recovered, "BLOCKED_EXPLICIT_REARM_REQUIRED")

    def test_invalid_or_missing_health_is_fail_closed(self):
        for state in (None, {}, dict(HEALTH, tf_fresh=1),
                      dict(HEALTH, localization_valid=False)):
            with self.subTest(health=state):
                sess, model, node = make()
                result = node.inspect(
                    model, motion=signed(sess, 0, BASE),
                    now=BASE, health=state)
                self.assert_inhibited(result, "BLOCKED_SENSOR_HEALTH")

    def test_duplicate_synthetic_sources_latch(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at,
            publishers=("controller_server", "other"))
        self.assert_inhibited(result, "BLOCKED_SYNTHETIC_SOURCE_CONFLICT")

    def test_missing_synthetic_owner_is_denied(self):
        sess, model, node = make()
        result = node.inspect(
            model, motion=signed(sess, 0, BASE), now=BASE,
            publishers=())
        self.assert_inhibited(result, "BLOCKED_SYNTHETIC_SOURCE_CONFLICT")

    def test_valid_signed_overlimit_is_denied(self):
        sess, model, node = make()
        result = node.inspect(
            model, motion=signed(sess, 0, BASE, vx=0.16),
            rearm=grant(sess, 0, BASE), now=BASE)
        self.assert_inhibited(result, "BLOCKED_SYNTHETIC_LIMITS")

    def test_unexpected_second_rearm_requires_new_sequence(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at),
            rearm=grant(sess, 1, at), now=at)
        self.assert_inhibited(result, "BLOCKED_UNEXPECTED_REARM")

    def test_evaluator_stall_fault_latches(self):
        sess, model, node = arm()
        at = BASE + 1_250_000_001
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at)
        self.assert_inhibited(result, "BLOCKED_SUPERVISOR_STALL")

    def test_actual_bridge_transport_fault_overrides_good_hmac(self):
        sess, model, node = arm()
        node._transport_fault.set()
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at)
        self.assert_inhibited(result, "BLOCKED_LOCAL_TRANSPORT_FAULT")
        self.assertTrue(node._transport_fault.is_set())
        self.assertEqual(node.writes, 0)

    def test_actual_bridge_clock_fault_overrides_good_hmac(self):
        sess, model, node = arm()
        node._clock_fault = True
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at)
        self.assert_inhibited(result, "BLOCKED_LOCAL_CLOCK_FAULT")
        self.assertEqual(node.writes, 0)

    def test_actual_bridge_stop_event_overrides_good_hmac(self):
        sess, model, node = arm()
        node._stop_event.set()
        at = BASE + 1_100_000_000
        result = node.inspect(
            model, motion=signed(sess, 11, at), now=at)
        self.assert_inhibited(result, "BLOCKED_BRIDGE_STOP_EVENT")
        self.assertEqual(node.writes, 0)

    def test_rearm_after_nonterminal_health_fault_starts_new_dwell(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        node.inspect(
            model, motion=signed(sess, 11, at), now=at,
            health=dict(HEALTH, odom_fresh=False))
        result = node.inspect(
            model, motion=signed(sess, 12, at + 100_000_000),
            rearm=grant(sess, 1, at + 100_000_000),
            now=at + 100_000_000)
        self.assertEqual(result.state, "REARMING")
        self.assertFalse(result.diagnostic_permission)
        self.assertFalse(result.physical_motion_authorized)

    def test_fresh_supervisor_is_not_automatically_armed(self):
        _old_sess, old_model, _old_node = arm()
        self.assertEqual(old_model.state, "ARMED")
        new_sess, new_model, new_node = make()
        result = new_node.inspect(
            new_model, motion=signed(new_sess, 0, BASE), now=BASE)
        self.assert_inhibited(result, "BLOCKED_EXPLICIT_REARM_REQUIRED")

    def test_invalid_supervisor_object_is_rejected(self):
        sess, _model, node = make()
        with self.assertRaises(TypeError):
            node.inspect(
                object(), motion=signed(sess, 0, BASE), now=BASE)
        self.assertEqual(node.writes, 0)

    def test_report_immutable_and_never_contains_pwm_or_secret(self):
        sess, model, node = arm()
        at = BASE + 1_100_000_000
        report = node.inspect(
            model, motion=signed(sess, 11, at), now=at)
        self.assertIs(type(report), DiagnosticAssessment)
        self.assertEqual(set(asdict(report)), {
            "state", "reason", "diagnostic_permission",
            "physical_motion_authorized", "diagnostic_only",
        })
        with self.assertRaises(FrozenInstanceError):
            report.physical_motion_authorized = True
        self.assertNotIn("set_wheels", repr(report))
        self.assertNotIn("secret", repr(report))

    def test_production_writer_and_raw_velocity_remain_inhibited(self):
        tree = source_tree()
        klass = bridge_class(tree)
        method = method_named(klass, "_supervise_signed_diagnostic")
        called_names = [
            n.func.attr for n in ast.walk(method)
            if isinstance(n, ast.Call)
            and isinstance(n.func, ast.Attribute)
        ]
        self.assertNotIn("_send_command", called_names)
        self.assertNotIn("write", called_names)
        source = NODE.read_text()
        self.assertNotIn("self._cmd_sub =", source)
        self.assertEqual(source.count("self._serial.write("), 1)
        self.assertIn("payload = command_to_bytes(command)", source)
        with self.assertRaises(CommandRejected):
            command_to_bytes("set_wheels 1 0 0 0")


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M22KBridgeSupervisorTests))
    if result.wasSuccessful():
        print("M22K_BRIDGE_DIAGNOSTIC_SUPERVISOR_SELFTEST=PASS", flush=True)
        print("M22K_VERDICT=ACTUAL_BRIDGE_FAULT_BOUND_DIAGNOSTIC_ONLY",
              flush=True)
        print("M22K_PRODUCTION_NONZERO_CDC_ENABLED=FALSE", flush=True)
        print("M22K_OPERATOR_AUTHENTICATION_IMPLEMENTED=FALSE", flush=True)
        print("M22K_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22K_BRIDGE_DIAGNOSTIC_SUPERVISOR_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
