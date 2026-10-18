"""M22G: offline exact-method terminal bridge transport fault regressions.

Import and compile ONLY production method AST from node.py. Never import
the production node module, start ROS or construct pyserial's Serial.
"""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

from test_m22d_command_ingress import (
    FakeBridge, FakeSerialError, MemorySerial, FakeLogger,
    load_exact_production_send_method, method_named, bridge_class,
    source_tree, NODE_PATH,
)

STOP = b"set_wheels 0 0 0 0\n"


def bridge():
    return FakeBridge(load_exact_production_send_method())


def compile_reader_method():
    method = method_named(bridge_class(source_tree()), "_reader_loop")
    code = ast.fix_missing_locations(
        ast.Module(body=[method], type_ignores=[]))
    space = {
        "serial": SimpleNamespace(
            SerialException=FakeSerialError,
            SerialTimeoutException=FakeSerialError),
        "rclpy": SimpleNamespace(ok=lambda: True),
    }
    compiled = {}
    exec(compile(code, str(NODE_PATH), "exec"), space, compiled)
    return compiled["_reader_loop"]


class BrokenReaderSerial(MemorySerial):
    def readline(self):
        raise FakeSerialError("injected fake CDC reader disconnect")


class ShortWriteSerial(MemorySerial):
    def write(self, payload):
        self.writes.append(payload[:2])
        return 2


class M22GTransportFaultTests(unittest.TestCase):
    def test_initial_bridge_transport_has_no_fault(self):
        b = bridge()
        self.assertFalse(b._transport_fault.is_set())
        self.assertFalse(b._stop_event.is_set())
        self.assertTrue(b.send("keepalive"))
        self.assertEqual(b._serial.writes, [b"keepalive\n"])

    def test_serial_write_failure_terminally_latches(self):
        b = bridge()
        b._serial.fail_next_write = True
        self.assertFalse(b.send("keepalive"))
        self.assertTrue(b._transport_fault.is_set())
        self.assertTrue(b._stop_event.is_set())
        self.assertFalse(b._serial.writes)
        self.assertIn("Terminal MCU transport fault", b._logger.errors[-1])

    def test_serial_flush_failure_terminally_latches(self):
        b = bridge()
        b._serial.fail_next_flush = True
        self.assertFalse(b.send("set_imu_stream_hz 50"))
        self.assertTrue(b._transport_fault.is_set())
        self.assertTrue(b._stop_event.is_set())
        self.assertEqual(b._serial.writes, [b"set_imu_stream_hz 50\n"])
        # A successful write but failed flush is NOT proof of delivery.
        self.assertEqual(b._serial.flush_count, 0)

    def test_short_write_is_terminal_fault(self):
        b = bridge()
        b._serial = ShortWriteSerial()
        self.assertFalse(b.send("set_wheels 0 0 0 0"))
        self.assertTrue(b._transport_fault.is_set())
        self.assertTrue(b._stop_event.is_set())
        self.assertEqual(b._serial.writes, [STOP[:2]])
        self.assertIn("short CDC write", b._logger.errors[-1])

    def test_terminal_fault_refuses_keepalive_stop_and_stream_commands(self):
        b = bridge()
        b._serial.fail_next_write = True
        self.assertFalse(b.send("keepalive"))
        old_count = len(b._serial.writes)
        for text in (
            "keepalive",
            "set_wheels 0 0 0 0",
            "set_imu_stream_hz 0",
            "set_encoder_stream_hz 0",
            "set_watchdog_ms 500",
        ):
            with self.subTest(command=text):
                self.assertFalse(b.send(text))
                self.assertEqual(len(b._serial.writes), old_count)
        self.assertTrue(b._transport_fault.is_set())

    def test_invalid_nonzero_wheel_command_does_not_trigger_io_or_latch(self):
        b = bridge()
        self.assertFalse(b.send("set_wheels 10 0 0 0"))
        self.assertFalse(b._serial.writes)
        self.assertFalse(b._transport_fault.is_set())
        self.assertFalse(b._stop_event.is_set())
        self.assertTrue(b.send("set_wheels 0 0 0 0"))
        self.assertEqual(b._serial.writes, [STOP])

    def test_unexpected_reader_exception_terminally_latches(self):
        b = bridge()
        b._serial = BrokenReaderSerial()
        compile_reader_method()(b)
        self.assertTrue(b._transport_fault.is_set())
        self.assertTrue(b._stop_event.is_set())
        self.assertIn("Terminal MCU read fault", b._logger.errors[-1])
        self.assertFalse(b.send("keepalive"))
        self.assertFalse(b._serial.writes)

    def test_expected_shutdown_reader_exception_does_not_add_fault(self):
        b = bridge()
        b._serial = BrokenReaderSerial()
        b._stop_event.set()
        compile_reader_method()(b)
        self.assertFalse(b._transport_fault.is_set())
        self.assertEqual(b._logger.errors, [])

    def test_fault_cannot_be_rearmed_in_same_instance(self):
        old = bridge()
        old._serial.fail_next_write = True
        self.assertFalse(old.send("keepalive"))
        self.assertFalse(old.send("set_watchdog_ms 500"))
        self.assertFalse(old.send("set_wheels 0 0 0 0"))
        self.assertTrue(old._transport_fault.is_set())
        fresh = bridge()  # Simulates a freshly constructed bridge process.
        self.assertFalse(fresh._transport_fault.is_set())
        self.assertTrue(fresh.send("keepalive"))
        self.assertTrue(old._transport_fault.is_set())

    def test_actual_bridge_guards_and_single_writer_remain(self):
        klass = bridge_class(source_tree())
        source = NODE_PATH.read_text()
        self.assertNotIn("self._cmd_sub =", source)
        self.assertNotIn("def _on_cmd_vel(", source)
        self.assertIn("payload = command_to_bytes(command)", source)
        self.assertIn("self._transport_fault = threading.Event()", source)
        self.assertIn("if self._clock_fault or self._transport_fault.is_set():",
                      source)
        self.assertEqual(source.count("self._serial.write("), 1)
        self.assertTrue(any(
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "set"
            for node in ast.walk(method_named(klass, "_reader_loop"))))


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M22GTransportFaultTests))
    if result.wasSuccessful():
        print("M22G_TERMINAL_TRANSPORT_FAULT_SELFTEST=PASS", flush=True)
        print("M22G_VERDICT=PRODUCTION_BRIDGE_SOURCE_FAKE_CDC_LATCH_ONLY",
              flush=True)
        print("M22G_NONZERO_PRODUCTION_CDC_ENABLED=FALSE", flush=True)
        print("M22G_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22G_TERMINAL_TRANSPORT_FAULT_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
