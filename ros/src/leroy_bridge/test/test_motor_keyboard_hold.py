"""No-robot regressions for physical-key hold and sequenced motor updates."""
import errno
import os
import tempfile
import unittest
from unittest.mock import patch

from leroy_bridge.motor_keyboard_hold import (
    HeldKeyboard, KeyboardExit, _EVENT, MAX_HOLD_NS,
    HIGH_PWM_MAX_HOLD_NS,
)
from leroy_bridge.motor_manual_commission import (
    ManualPulseProtocol, MotorSequenceLedger, MAX_PWM, SIGNED_PWM_MAX,
)


class FakeMcu:
    def __init__(self):
        self.sent = []
        self.reject = False

    def write(self, raw):
        self.sent.append(raw.decode("ascii").strip())
        return len(raw)

    def flush(self):
        pass

    def readline(self):
        command = self.sent[-1]
        if self.reject and command.startswith(
                ("set_wheels_tx", "set_wheels_calibration_tx")):
            return b"error set_wheels_tx rejected\r\n"
        if command.startswith((
                "set_wheels_tx ", "set_wheels_calibration_tx ",
                "stop_wheels_tx ")):
            kind, seq = command.split()[:2]
            return f"ok {kind} seq={seq}\r\n".encode()
        if command.startswith("set_watchdog_ms "):
            return b"ok set_watchdog_ms\r\n"
        if command.startswith("set_platform_ramp "):
            return b"ok set_platform_ramp\r\n"
        raise AssertionError(command)


class KeyboardHoldTest(unittest.TestCase):
    def setUp(self):
        self.keyb = object.__new__(HeldKeyboard)
        self.keyb.fd = 987
        self.keyb._grabbed = False
        self.keyb._inhibited = True
        self.keyb._started_ns = None
        self.keyb._last_direction = None
        self.pressed = set()
        self.t = 5_000_000_000

    def fake_ioctl(self, fd, cmd, data):
        keys = {"w": 17, "a": 30, "s": 31, "d": 32, "q": 16, "e": 18}
        bits = bytearray(96)
        for k in self.pressed:
            code = keys[k]
            bits[code // 8] |= (1 << (code % 8))
        return bytes(bits)

    def read(self):
        with patch("os.read", side_effect=BlockingIOError), \
             patch("fcntl.ioctl", side_effect=self.fake_ioctl):
            return self.keyb.read(self.t)

    def test_requires_first_release_and_stops_on_release(self):
        self.pressed = {"w"}
        self.assertIsNone(self.read())           # no key already held at start
        self.pressed.clear()
        self.assertIsNone(self.read())
        self.pressed = {"w"}
        self.assertEqual(self.read(), "w")
        self.t += 25_000_000
        self.assertEqual(self.read(), "w")       # no auto-repeat needed
        self.pressed.clear()
        self.assertIsNone(self.read())           # immediate zero proposal

    def test_multiple_keys_inhibit_until_release(self):
        self.assertIsNone(self.read())
        self.pressed = {"w"}
        self.assertEqual(self.read(), "w")
        self.pressed = {"w", "a"}
        self.assertIsNone(self.read())
        self.pressed = {"a"}
        self.assertIsNone(self.read())          # cannot jump to new direction
        self.pressed.clear()
        self.assertIsNone(self.read())
        self.pressed = {"a"}
        self.assertEqual(self.read(), "a")

    def test_high_pwm_hold_expires_after_half_second(self):
        self.keyb.max_hold_ns = HIGH_PWM_MAX_HOLD_NS
        self.assertIsNone(self.read())
        self.pressed = {"a"}
        self.assertEqual(self.read(), "a")
        self.t += HIGH_PWM_MAX_HOLD_NS + 1
        self.assertIsNone(self.read())
        self.assertIsNone(self.read())
        self.pressed.clear()
        self.assertIsNone(self.read())
        self.pressed = {"a"}
        self.assertEqual(self.read(), "a")

    def test_max_hold_latches_until_release(self):
        self.assertIsNone(self.read())
        self.pressed = {"w"}
        self.assertEqual(self.read(), "w")
        self.t += MAX_HOLD_NS + 1
        self.assertIsNone(self.read())
        self.assertIsNone(self.read())
        self.pressed.clear()
        self.assertIsNone(self.read())
        self.pressed = {"w"}
        self.assertEqual(self.read(), "w")

    def test_key_stream_overrun_and_escape_fail_closed(self):
        with patch("os.read", return_value=_EVENT.pack(0, 0, 0, 3, 0)):
            with self.assertRaisesRegex(RuntimeError, "overrun"):
                self.keyb.read(self.t)
        with patch("os.read", return_value=_EVENT.pack(0, 0, 1, 1, 1)):
            with self.assertRaises(KeyboardExit):
                self.keyb.read(self.t)


    def test_keyboard_ungrab_uses_integer_zero_and_closes_fd(self):
        self.keyb._grabbed = True
        with patch("fcntl.ioctl") as ioctl, patch("os.close") as close:
            self.keyb.close()
        ioctl.assert_called_once_with(987, 0x40044590, 0)
        close.assert_called_once_with(987)
        self.assertIsNone(self.keyb.fd)

    def test_keyboard_release_error_does_not_mask_original_fault(self):
        self.keyb._grabbed = True
        with patch("fcntl.ioctl", side_effect=OSError(errno.EBUSY, "busy")), \
             patch("os.close") as close:
            self.keyb.close()
        close.assert_called_once_with(987)


class SequenceLedgerTest(unittest.TestCase):
    def test_persists_across_separate_process_sessions(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "motor_seq")
            with patch("time.time", return_value=1793098162):
                first = MotorSequenceLedger(path)
                self.assertEqual(first.current, 1793098162)
                self.assertEqual(first.claim(), 1793098163)
                self.assertEqual(first.claim(), 1793098164)
                first.close()
                # Clock can stay the same or move backward; file wins.
                second = MotorSequenceLedger(path)
                self.assertEqual(second.current, 1793098164)
                self.assertEqual(second.claim(), 1793098165)
                second.close()
            with open(path, encoding="ascii") as f:
                self.assertEqual(f.read(), "1793098165\n")

    def test_exclusive_lock_and_corruption_fail_closed(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "motor_seq")
            first = MotorSequenceLedger(path)
            with self.assertRaises(BlockingIOError):
                MotorSequenceLedger(path)
            first.close()
            with open(path, "w", encoding="ascii") as f:
                f.write("not a motor sequence\n")
            with self.assertRaisesRegex(ValueError, "corrupt"):
                MotorSequenceLedger(path)

    def test_ledger_advances_before_command_and_stop(self):
        with tempfile.TemporaryDirectory() as td:
            path = os.path.join(td, "motor_seq")
            mcu = FakeMcu()
            with patch("time.time", return_value=1793098162):
                ledger = MotorSequenceLedger(path)
            try:
                p = ManualPulseProtocol(
                    mcu, first_sequence=ledger.current, pulse_ms=100,
                    sequence_ledger=ledger)
                p.configure()
                p.drive((48, 48, 48, 48))
                p.stop()
                with open(path, encoding="ascii") as f:
                    self.assertEqual(f.read().strip(), str(p.sequence))
                self.assertEqual(
                    mcu.sent[-1], f"stop_wheels_tx {p.sequence}")
            finally:
                ledger.close()


class HoldCdcTest(unittest.TestCase):
    def test_hold_refresh_uses_exact_acks_and_watchdog(self):
        mcu = FakeMcu()
        p = ManualPulseProtocol(mcu, first_sequence=100, pulse_ms=100)
        p.configure()
        p.drive((32, 32, 32, 32))
        p.drive((32, 32, 32, 32))
        p.stop()
        self.assertEqual(mcu.sent, [
            "stop_wheels_tx 101",
            "set_watchdog_ms 200",
            "set_platform_ramp 4",
            "set_wheels_tx 102 32 32 32 32",
            "set_wheels_tx 103 32 32 32 32",
            "stop_wheels_tx 104",
        ])

    def test_upper_limit_matches_existing_mcu_ceiling(self):
        self.assertEqual(SIGNED_PWM_MAX, 64)
        self.assertEqual(MAX_PWM, 255)
        mcu = FakeMcu()
        p = ManualPulseProtocol(mcu, first_sequence=1100, pulse_ms=100)
        p.configure()
        p.drive((64, -64, 64, -64))
        self.assertEqual(
            mcu.sent[-1], "set_wheels_tx 1102 64 -64 64 -64")
        with self.assertRaises(ValueError):
            p.drive((65, 0, 0, 0))
        self.assertEqual(len(mcu.sent), 4)
        p.stop()

    def test_opt_in_high_pwm_uses_distinct_255_command(self):
        mcu = FakeMcu()
        p = ManualPulseProtocol(
            mcu, first_sequence=5000, pulse_ms=100,
            high_pwm_enabled=True)
        p.configure()
        self.assertEqual(mcu.sent[2], "set_platform_ramp 8")
        p.drive((128, -128, 255, -255))
        self.assertEqual(
            mcu.sent[-1],
            "set_wheels_calibration_tx 5002 128 -128 255 -255")
        p.drive((64, 64, 64, 64))
        self.assertEqual(
            mcu.sent[-1], "set_wheels_tx 5003 64 64 64 64")
        with self.assertRaises(ValueError):
            p.drive((256, 0, 0, 0))
        p.stop()

    def test_high_pwm_fails_without_opt_in(self):
        mcu = FakeMcu()
        p = ManualPulseProtocol(mcu, first_sequence=6000, pulse_ms=100)
        p.configure()
        with self.assertRaisesRegex(ValueError, "opt-in"):
            p.drive((128, 128, 128, 128))
        self.assertEqual(len(mcu.sent), 3)

    def test_drive_bound_and_failure_latch(self):
        mcu = FakeMcu()
        p = ManualPulseProtocol(mcu, first_sequence=1000, pulse_ms=100)
        p.configure()
        with self.assertRaises(ValueError):
            p.drive((MAX_PWM + 1, 0, 0, 0))
        self.assertEqual(len(mcu.sent), 3)
        mcu.reject = True
        with self.assertRaises(RuntimeError):
            p.drive((32, 32, 32, 32))
        self.assertTrue(p.faulted)
        with self.assertRaises(RuntimeError):
            p.drive((1, 1, 1, 1))
        mcu.reject = False
        p.stop()
        self.assertEqual(mcu.sent[-1], "stop_wheels_tx 1003")


if __name__ == "__main__":
    unittest.main()
