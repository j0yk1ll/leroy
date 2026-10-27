"""No-robot regressions for physical-key hold and sequenced motor updates."""
import unittest
from unittest.mock import patch

from leroy_bridge.motor_keyboard_hold import (
    HeldKeyboard, KeyboardExit, _EVENT, MAX_HOLD_NS,
)
from leroy_bridge.motor_manual_commission import ManualPulseProtocol, MAX_PWM


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
        if self.reject and command.startswith("set_wheels_tx"):
            return b"error set_wheels_tx rejected\r\n"
        if command.startswith(("set_wheels_tx ", "stop_wheels_tx ")):
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
        self.assertEqual(MAX_PWM, 64)
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
