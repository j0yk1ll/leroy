"""M22A purely synthetic authority tests; no ROS, serial or motor access."""
from dataclasses import replace
import math
import unittest

from m22a_diagnostic_authority import (
    DiagnosticAuthority, Inputs, Proposal, ZERO,
    EXPECTED_OWNER, MAX_AGE_NS, RECOVERY_NS,
)


GOOD = Inputs(True, True, True, True, True)
BASE = 10_000_000_000


def packet(seq, at, **values):
    args = dict(owner=EXPECTED_OWNER, sequence=seq,
                generated_mono_ns=at - 10_000_000,
                vx=0.04, vy=0.0, wz=0.1)
    args.update(values)
    return Proposal(**args)


def tick(gate, seq, at, *, inputs=GOOD, publishers=(EXPECTED_OWNER,),
         operator_rearm=False, proposal=None):
    if proposal is None:
        proposal = packet(seq, at)
    return gate.evaluate(now_ns=at, inputs=inputs,
                         publishers=publishers, proposal=proposal,
                         operator_rearm=operator_rearm)


def armed_gate():
    gate = DiagnosticAuthority()
    assert tick(gate, 0, BASE, operator_rearm=True).state == "REARMING"
    for i in range(1, 10):
        waiting = tick(gate, i, BASE + i * 100_000_000)
        assert waiting.state == "REARMING" and waiting.output == ZERO
    good = tick(gate, 10, BASE + RECOVERY_NS)
    assert good.state == "ARMED" and good.output != ZERO
    return gate


class AuthorityTests(unittest.TestCase):
    def assert_latched(self, result):
        self.assertEqual(result.output, ZERO)
        self.assertEqual(result.state, "LATCHED")

    def test_initially_inhibited_no_automatic_rearm(self):
        gate = DiagnosticAuthority()
        result = tick(gate, 0, BASE)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_MANUAL_REARM_REQUIRED")
        result = tick(gate, 1, BASE + RECOVERY_NS * 5)
        self.assert_latched(result)

    def test_explicit_rearm_requires_continuous_dwell(self):
        gate = DiagnosticAuthority()
        a = tick(gate, 0, BASE, operator_rearm=True)
        self.assertEqual(a.output, ZERO)
        self.assertEqual(a.state, "REARMING")
        for i in range(1, 10):
            b = tick(gate, i, BASE + 100_000_000 * i)
            self.assertEqual(b.output, ZERO)
            self.assertEqual(b.reason, "BLOCKED_REARM_DWELL")
        c = tick(gate, 10, BASE + RECOVERY_NS)
        self.assertEqual(c.state, "ARMED")
        self.assertEqual(c.output, (0.04, 0.0, 0.1))

    def test_replay_sequence_latches_even_when_other_inputs_healthy(self):
        gate = armed_gate()
        result = tick(gate, 10, BASE + RECOVERY_NS + 25_000_000)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_REPLAY_SEQUENCE")

    def test_generation_time_replay_with_new_sequence_latches(self):
        gate = armed_gate()
        at = BASE + RECOVERY_NS + 100_000_000
        stale_generation = BASE + RECOVERY_NS - 10_000_000
        result = tick(gate, 11, at,
                      proposal=packet(11, at, generated_mono_ns=stale_generation))
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_REPLAY_GENERATION_TIME")

    def test_clock_regression_latches(self):
        gate = armed_gate()
        result = tick(gate, 11, BASE + RECOVERY_NS - 100_000_000)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_MONOTONIC_CLOCK")

    def test_duplicate_or_missing_owner_fails_closed(self):
        for publishers in ((), (EXPECTED_OWNER, EXPECTED_OWNER),
                           ("other_controller",)):
            with self.subTest(publishers=publishers):
                gate = armed_gate()
                result = tick(gate, 11, BASE + RECOVERY_NS + 25_000_000,
                              publishers=publishers)
                self.assert_latched(result)
                self.assertEqual(result.reason, "BLOCKED_COMMAND_OWNERSHIP")

    def test_proposal_owner_mismatch_fails_closed(self):
        gate = armed_gate()
        at = BASE + RECOVERY_NS + 25_000_000
        result = tick(gate, 11, at,
                      proposal=packet(11, at, owner="unauthorized"))
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_COMMAND_FORMAT")

    def test_every_health_gate_is_required(self):
        for name in GOOD.__dataclass_fields__:
            with self.subTest(name=name):
                gate = armed_gate()
                result = tick(gate, 11, BASE + RECOVERY_NS + 25_000_000,
                              inputs=replace(GOOD, **{name: False}))
                self.assert_latched(result)
                self.assertEqual(result.reason, "BLOCKED_SENSOR_HEALTH")

    def test_stale_or_future_generated_stamp_rejected(self):
        for offset in (MAX_AGE_NS + 1, -1):
            with self.subTest(offset=offset):
                gate = armed_gate()
                at = BASE + RECOVERY_NS + 25_000_000
                result = tick(gate, 11, at, proposal=packet(
                    3, at, generated_mono_ns=at - offset))
                self.assert_latched(result)
                self.assertEqual(result.reason,
                                 "BLOCKED_STALE_OR_FUTURE_COMMAND")

    def test_unbounded_nonplanar_and_nonfinite_commands_rejected(self):
        faults = (
            (dict(vx=0.16), "BLOCKED_COMMAND_LIMITS"),
            (dict(vy=-0.151), "BLOCKED_COMMAND_LIMITS"),
            (dict(wz=0.301), "BLOCKED_COMMAND_LIMITS"),
            (dict(vz=0.001), "BLOCKED_COMMAND_LIMITS"),
            (dict(wx=0.001), "BLOCKED_COMMAND_LIMITS"),
            (dict(wy=0.001), "BLOCKED_COMMAND_LIMITS"),
            (dict(vx=float("nan")), "BLOCKED_NONFINITE_COMMAND"),
            (dict(wz=float("inf")), "BLOCKED_NONFINITE_COMMAND"),
            (dict(vx=True), "BLOCKED_NONFINITE_COMMAND"),
        )
        for data, reason in faults:
            with self.subTest(data=data):
                gate = armed_gate()
                at = BASE + RECOVERY_NS + 25_000_000
                result = tick(gate, 11, at, proposal=packet(11, at, **data))
                self.assert_latched(result)
                self.assertEqual(result.reason, reason)

    def test_absent_proposal_latches_and_does_not_replay_old_output(self):
        gate = armed_gate()
        result = gate.evaluate(
            now_ns=BASE + RECOVERY_NS + 25_000_000,
            inputs=GOOD, publishers=(EXPECTED_OWNER,), proposal=None)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_COMMAND_FORMAT")

    def test_fault_requires_fresh_explicit_rearm(self):
        gate = armed_gate()
        t = BASE + RECOVERY_NS + 25_000_000
        fault = tick(gate, 11, t, inputs=replace(GOOD, depth_fresh=False))
        self.assert_latched(fault)
        restored = tick(gate, 12, t + 25_000_000)
        self.assert_latched(restored)
        self.assertEqual(restored.reason, "BLOCKED_MANUAL_REARM_REQUIRED")
        recovering = tick(gate, 13, t + 50_000_000, operator_rearm=True)
        self.assertEqual(recovering.state, "REARMING")
        self.assertEqual(recovering.output, ZERO)

    def test_rearm_while_already_armed_is_not_an_override(self):
        gate = armed_gate()
        result = tick(gate, 11, BASE + RECOVERY_NS + 25_000_000,
                      operator_rearm=True)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_UNEXPECTED_REARM")

    def test_evaluator_stall_latches_armed(self):
        gate = armed_gate()
        result = tick(gate, 11, BASE + RECOVERY_NS + 200_000_001)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_EVALUATOR_STALL")

    def test_evaluator_stall_interrupts_rearm_dwell(self):
        gate = DiagnosticAuthority()
        self.assertEqual(tick(gate, 0, BASE, operator_rearm=True).state,
                         "REARMING")
        result = tick(gate, 1, BASE + 200_000_001)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_EVALUATOR_STALL")
        result = tick(gate, 2, BASE + 300_000_000)
        self.assert_latched(result)
        self.assertEqual(result.reason, "BLOCKED_MANUAL_REARM_REQUIRED")

    def test_invalid_sequence_and_rearm_flag_rejected(self):
        at = BASE + RECOVERY_NS + 25_000_000
        for bad_sequence in (-1, 2.5, True, 1 << 64):
            with self.subTest(bad_sequence=bad_sequence):
                gate = armed_gate()
                r = tick(gate, 11, at,
                         proposal=packet(bad_sequence, at))
                self.assert_latched(r)
                self.assertEqual(r.reason, "BLOCKED_COMMAND_FORMAT")
        gate = armed_gate()
        r = gate.evaluate(
            now_ns=at, inputs=GOOD,
            publishers=(EXPECTED_OWNER,),
            proposal=packet(11, at), operator_rearm="true")
        self.assert_latched(r)
        self.assertEqual(r.reason, "BLOCKED_INVALID_REARM")


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(AuthorityTests))
    if result.wasSuccessful():
        print("M22A_SOFTWARE_AUTHORITY_SELFTEST=PASS", flush=True)
        print("M22A_VERDICT=SYNTHETIC_DIAGNOSTIC_AUTHORITY_ONLY", flush=True)
        print("M22A_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    raise SystemExit(2)
