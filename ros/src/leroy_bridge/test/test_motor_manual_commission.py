"""Focused commissioning protocol tests: no motor or ROS device."""
import unittest
from leroy_bridge.motor_manual_commission import (
    OneShotEnable, ManualPulseProtocol, DIRECTIONS, MAX_PWM, wheel_polarities,
)

class FakeMcu:
    def __init__(self, reject=False):
        self.commands = []
        self.reject = reject
    def write(self, data):
        self.commands.append(data.decode().strip())
        return len(data)
    def flush(self):
        pass
    def readline(self):
        command = self.commands[-1]
        if self.reject and command.startswith("set_wheels_tx"):
            return b"error set_wheels_tx stale_sequence\r\n"
        if command.startswith(("set_wheels_tx", "stop_wheels_tx")):
            kind, seq = command.split()[:2]
            return f"ok {kind} seq={seq}\r\n".encode()
        if command.startswith("set_watchdog_ms"):
            return b"ok set_watchdog_ms\r\n"
        if command.startswith("set_platform_ramp"):
            return b"ok set_platform_ramp\r\n"
        raise AssertionError(command)

class ManualTests(unittest.TestCase):
    def test_one_shot_gate(self):
        enable = OneShotEnable()
        self.assertFalse(enable.consume("w", 1_000_000_000))
        enable.press_space(1_000_000_000)
        self.assertTrue(enable.consume("w", 1_100_000_000))
        self.assertFalse(enable.consume("w", 1_101_000_000))
        enable.press_space(1_000_000_000)
        self.assertFalse(enable.consume("w", 1_600_000_000))

    def test_directions_and_polarity(self):
        self.assertEqual(len(DIRECTIONS), 6)
        self.assertEqual(DIRECTIONS["a"], (-1, 1, 1, -1))
        self.assertEqual(wheel_polarities("+1,-1,+1,-1"), (1, -1, 1, -1))

    def test_protocol_sequence_stop_and_watchdog(self):
        mcu = FakeMcu()
        proto = ManualPulseProtocol(mcu, first_sequence=100, pulse_ms=20)
        proto.configure()
        proto.pulse((24, 24, 24, 24))
        self.assertEqual(mcu.commands, [
            "stop_wheels_tx 101", "set_watchdog_ms 120",
            "set_platform_ramp 4", "set_wheels_tx 102 24 24 24 24",
            "stop_wheels_tx 103"])

    def test_rejected_motion_stops_and_latches(self):
        mcu = FakeMcu(reject=True)
        proto = ManualPulseProtocol(mcu, first_sequence=200, pulse_ms=20)
        proto.configure()
        with self.assertRaises(RuntimeError):
            proto.pulse((MAX_PWM, -MAX_PWM, MAX_PWM, -MAX_PWM))
        self.assertEqual(mcu.commands[-1], "stop_wheels_tx 203")
        self.assertTrue(proto.faulted)
        with self.assertRaises(RuntimeError):
            proto.pulse((1, 1, 1, 1))

    def test_invalid_pwm_not_transmitted(self):
        mcu = FakeMcu()
        proto = ManualPulseProtocol(mcu, first_sequence=50, pulse_ms=20)
        proto.configure()
        for values in ((0, 0, 0, 0), (MAX_PWM+1, 0, 0, 0),
                       (1, 2, 3), [1, 1, 1, 1]):
            with self.assertRaises(ValueError):
                proto.pulse(values)
        self.assertEqual(len(mcu.commands), 3)

if __name__ == "__main__":
    unittest.main()
