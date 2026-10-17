"""M22C: unit-test fake serial command binding and zero-on-inhibit.

No ROS, physical CDC, hardware, Podman, motors or production bridge startup.
"""
import ast
from dataclasses import replace
from pathlib import Path
import unittest

from leroy_bridge.motor_pwm import mecanum_twist_to_pwm
from m22a_diagnostic_authority import Inputs, Proposal, ZERO
from m22c_fake_serial_boundary import (
    FakeSerialTransport, SyntheticBridgeBoundary, SYNTHETIC_WHEEL_PARAMS,
    ZERO_PWM,
)

HERE = Path(__file__).resolve().parent
BASE = 10_000_000_000
HEALTH = Inputs(True, True, True, True, True)
SOURCE = ("controller_server",)


def proposal(seq, at, **changes):
    args = dict(owner="controller_server", sequence=seq,
                generated_mono_ns=at - 10_000_000,
                vx=0.04, vy=0.0, wz=0.07)
    args.update(changes)
    return Proposal(**args)


def cycle(boundary, sequence, at, *, health=HEALTH, source=SOURCE,
          rearm=False, override=None):
    return boundary.evaluate_and_write(
        now_ns=at, inputs=health, publishers=source,
        proposal=proposal(sequence, at) if override is None else override,
        operator_rearm=rearm)


def initialized():
    serial = FakeSerialTransport()
    boundary = SyntheticBridgeBoundary(serial)
    return boundary, serial


def armed():
    boundary, serial = initialized()
    first = cycle(boundary, 0, BASE, rearm=True)
    assert first.emitted_pwm == ZERO_PWM and first.authority_state == "REARMING"
    for index in range(1, 10):
        report = cycle(boundary, index, BASE + index * 100_000_000)
        assert report.emitted_pwm == ZERO_PWM
    report = cycle(boundary, 10, BASE + 1_000_000_000)
    assert report.authority_state == "ARMED"
    assert report.emitted_pwm != ZERO_PWM
    return boundary, serial


class FakeSerialAuthorityTests(unittest.TestCase):
    def assert_inhibited(self, result):
        self.assertEqual(result.emitted_pwm, ZERO_PWM)
        self.assertEqual(result.authority_state, "LATCHED")

    def test_initial_state_emits_only_exact_zero(self):
        boundary, serial = initialized()
        report = cycle(boundary, 0, BASE)
        self.assert_inhibited(report)
        self.assertEqual(serial.writes, [b"set_wheels 0 0 0 0\n"])
        self.assertTrue(report.fake_write_completed)
        self.assertFalse(report.physical_stop_acknowledgement_validated)

    def test_rearm_dwell_and_production_pwm_math(self):
        boundary, serial = armed()
        expected = tuple(mecanum_twist_to_pwm(
            vx=0.04, vy=0.0, wz=0.07, **SYNTHETIC_WHEEL_PARAMS))
        self.assertNotEqual(expected, ZERO_PWM)
        self.assertEqual(serial.writes[-1],
                         ("set_wheels {} {} {} {}\n".format(*expected)).encode())
        self.assertEqual(len(serial.writes), 11)
        self.assertEqual(boundary.state, "ARMED")

    def test_duplicate_publisher_zero_and_rearm_required(self):
        boundary, serial = armed()
        now = BASE + 1_100_000_000
        fault = cycle(boundary, 11, now,
                      source=("controller_server", "rogue_controller"))
        self.assert_inhibited(fault)
        self.assertEqual(fault.authority_reason, "BLOCKED_COMMAND_OWNERSHIP")
        self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")
        recovered = cycle(boundary, 12, now + 100_000_000)
        self.assert_inhibited(recovered)
        self.assertEqual(recovered.authority_reason,
                         "BLOCKED_MANUAL_REARM_REQUIRED")

    def test_wrong_or_missing_owner_always_zero(self):
        for publishers in ((), ("rogue",), ("controller_server", "controller_server")):
            with self.subTest(publishers=publishers):
                boundary, _serial = armed()
                result = cycle(boundary, 11, BASE + 1_100_000_000,
                               source=publishers)
                self.assert_inhibited(result)

    def test_incorrect_proposal_owner_rejected(self):
        boundary, serial = armed()
        t = BASE + 1_100_000_000
        result = cycle(boundary, 11, t,
                       override=proposal(11, t, owner="rogue"))
        self.assert_inhibited(result)
        self.assertEqual(result.authority_reason, "BLOCKED_COMMAND_FORMAT")
        self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_stale_and_future_command_generation_zero(self):
        for age in (251_000_000, -1):
            with self.subTest(age_ns=age):
                boundary, serial = armed()
                t = BASE + 1_100_000_000
                result = cycle(boundary, 11, t,
                               override=proposal(
                                   11, t, generated_mono_ns=t - age))
                self.assert_inhibited(result)
                self.assertEqual(result.authority_reason,
                                 "BLOCKED_STALE_OR_FUTURE_COMMAND")
                self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_replay_sequence_and_generation_zero(self):
        for changes, reason in (
            ({"sequence": 10}, "BLOCKED_REPLAY_SEQUENCE"),
            ({"generated_mono_ns": BASE + 990_000_000},
             "BLOCKED_REPLAY_GENERATION_TIME"),
        ):
            with self.subTest(changes=changes):
                boundary, serial = armed()
                t = BASE + 1_100_000_000
                result = cycle(boundary, 11, t,
                               override=proposal(11, t, **changes))
                self.assert_inhibited(result)
                self.assertEqual(result.authority_reason, reason)
                self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_sensor_loss_zero_and_does_not_automatically_rearm(self):
        boundary, serial = armed()
        t = BASE + 1_100_000_000
        fault = cycle(boundary, 11, t,
                      health=replace(HEALTH, depth_fresh=False))
        self.assert_inhibited(fault)
        self.assertEqual(fault.authority_reason, "BLOCKED_SENSOR_HEALTH")
        self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")
        restored = cycle(boundary, 12, t + 100_000_000)
        self.assert_inhibited(restored)

    def test_missing_proposal_cannot_replay_prior_duty(self):
        boundary, serial = armed()
        t = BASE + 1_100_000_000
        result = cycle(boundary, 11, t, override=False)
        self.assert_inhibited(result)
        self.assertEqual(result.authority_reason, "BLOCKED_COMMAND_FORMAT")
        self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_nonfinite_and_overlimit_zero(self):
        for changes in ({"vx": float("nan")}, {"vx": 0.16},
                        {"wz": float("inf")}, {"vz": 0.001}):
            with self.subTest(changes=changes):
                boundary, serial = armed()
                t = BASE + 1_100_000_000
                result = cycle(boundary, 11, t,
                               override=proposal(11, t, **changes))
                self.assert_inhibited(result)
                self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_evaluator_stall_zero(self):
        boundary, serial = armed()
        result = cycle(boundary, 11, BASE + 1_200_000_001)
        self.assert_inhibited(result)
        self.assertEqual(result.authority_reason, "BLOCKED_EVALUATOR_STALL")
        self.assertEqual(serial.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_invalid_pwm_calibration_fails_closed(self):
        fake = FakeSerialTransport()
        config = dict(SYNTHETIC_WHEEL_PARAMS, wheel_radius_m=0.0)
        boundary = SyntheticBridgeBoundary(fake, pwm_parameters=config)
        for index in range(10):
            cycle(boundary, index, BASE + index * 100_000_000,
                  rearm=(index == 0))
        result = cycle(boundary, 10, BASE + 1_000_000_000)
        self.assert_inhibited(result)
        self.assertEqual(result.authority_reason, "BLOCKED_PWM_CONVERSION")
        self.assertEqual(fake.writes[-1], b"set_wheels 0 0 0 0\n")

    def test_fake_write_error_latches_and_never_retries(self):
        boundary, fake = armed()
        before = len(fake.writes)
        fake.fail_next = True
        failed = cycle(boundary, 11, BASE + 1_100_000_000)
        self.assert_inhibited(failed)
        self.assertTrue(failed.fake_transport_fault_latched)
        self.assertFalse(failed.fake_write_completed)
        self.assertFalse(failed.physical_stop_acknowledgement_validated)
        self.assertEqual(len(fake.writes), before)
        blocked = cycle(boundary, 12, BASE + 1_200_000_000,
                        rearm=True)
        self.assert_inhibited(blocked)
        self.assertEqual(blocked.authority_reason,
                         "BLOCKED_FAKE_TRANSPORT_LATCHED")
        self.assertEqual(len(fake.writes), before)

    def test_fake_cdc_protocol_allowlist(self):
        fake = FakeSerialTransport()
        illegal = (
            b"set_wheels 10 10 10 10\nX",
            b"set_wheels 999 0 0 0\n",
            b"set_watchdog_ms 0\n",
            b"set_wheels 1 2 3\n",
            "set_wheels 1 2 3 4\n",
        )
        for packet in illegal:
            with self.subTest(packet=packet), self.assertRaises(ValueError):
                fake.write(packet)
        self.assertEqual(fake.writes, [])

    def test_module_stays_offline_and_production_bridge_unchanged(self):
        script = (HERE / "m22c_fake_serial_boundary.py").read_text()
        imports = set()
        for node in ast.walk(ast.parse(script)):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            if isinstance(node, ast.ImportFrom):
                imports.add((node.module or "").split(".")[0])
        self.assertFalse(imports & {"serial", "rclpy", "subprocess", "os", "socket"})
        # An explicit reminder: deployed bridge still accepts untrusted Twist
        # on /cmd_vel. This PR intentionally DOES NOT fix that bypass.
        real_node = (HERE / "src/leroy_bridge/leroy_bridge/node.py").read_text()
        self.assertIn("Twist, '/cmd_vel', self._on_cmd_vel, 10", real_node)
        self.assertIn("mecanum_twist_to_pwm(", real_node)


if __name__ == "__main__":
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(unittest.defaultTestLoader.loadTestsFromTestCase(
        FakeSerialAuthorityTests))
    if result.wasSuccessful():
        print("M22C_FAKE_SERIAL_AUTHORITY_SELFTEST=PASS", flush=True)
        print("M22C_VERDICT=SYNTHETIC_AUTHORITY_TO_FAKE_WHEEL_PROTOCOL_ONLY",
              flush=True)
        print("M22C_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22C_FAKE_SERIAL_AUTHORITY_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
