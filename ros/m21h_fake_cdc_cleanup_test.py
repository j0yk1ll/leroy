#!/usr/bin/env python3
"""M21H: exercise M20's actual cleanup control flow with a fake CDC port.

No serial device, camera hardware or motor-controlling process is opened.
The fake port accepts ONLY the two already-approved IMU rate commands.
"""
import contextlib
import io
import os
import unittest
from unittest import mock

import serial

import m20_combined_observer as m20
from m21f_imu_source_hook_selftest import fake_packet, assert_no_physical_devices

ENV = {
    "LEROY_M20_SENSOR_ONLY": "1", "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170", "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
}


class FakeCdc:
    def __init__(self, failure, stop_ack=True):
        self.failure = failure
        self.stop_ack = stop_ack
        self.writes = []
        self.acks = []
        self.read_error_raised = False
        self.packet_sent = False
        self.closed = False
        self.opened = False

    def __enter__(self):
        self.opened = True
        return self

    def __exit__(self, exc_type, exc, tb):
        self.closed = True
        return False

    def reset_input_buffer(self):
        if self.failure == "reset_error":
            raise serial.SerialException("simulated CDC input reset failure")

    def write(self, payload):
        if payload not in (m20.START, m20.STOP):
            raise AssertionError("attempted nonallowlisted serial write")
        self.writes.append(payload)
        if payload == m20.START or self.stop_ack:
            self.acks.append(b"ok set_imu_stream_hz\r\n")
        return len(payload)

    def flush(self):
        pass

    def readline(self):
        if self.acks:
            return self.acks.pop(0)
        if (self.failure == "callback_error"
                and m20.START in self.writes and not self.packet_sent):
            self.packet_sent = True
            return fake_packet(0)
        if (self.failure == "read_error"
                and m20.START in self.writes and not self.read_error_raised):
            self.read_error_raised = True
            raise serial.SerialException("simulated CDC read failure")
        return b""


class RunnerCleanupTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        assert_no_physical_devices()
        for key, expected in ENV.items():
            if os.environ.get(key) != expected:
                raise RuntimeError("M21H requires isolated sensor-only env " + key)

    def execute(self, failure, *, stop_ack=True, injected_callback=False,
                forbidden_graph=False):
        port = FakeCdc(failure, stop_ack)
        opens = []
        output = io.StringIO()
        source_class = m20.ImuStreamSource
        real_wait_ack = m20.wait_ack

        def serial_factory(*args, **kwargs):
            opens.append((args, kwargs))
            if kwargs.get("port") != m20.PORT or kwargs.get("exclusive") is not True:
                raise AssertionError("unexpected serial port configuration")
            return port

        def quick_ack(device, timeout_s=2.0):
            # Exercise the real ACK parser with a smaller deadline when
            # simulating a lost STOP response.
            return real_wait_ack(device, timeout_s=min(timeout_s, 0.075))

        def metadata_failure(*_args):
            raise ValueError("synthetic metadata write failure")

        def source_factory():
            if injected_callback:
                return source_class(on_imu_published=metadata_failure)
            return source_class()

        def reject_graph(_node):
            raise RuntimeError("synthetic_forbidden_graph")

        with contextlib.ExitStack() as stack:
            stack.enter_context(mock.patch.object(
                m20.serial, "Serial", side_effect=serial_factory))
            stack.enter_context(mock.patch.object(
                m20, "wait_ack", side_effect=quick_ack))
            stack.enter_context(mock.patch.object(
                m20, "ImuStreamSource", side_effect=source_factory))
            if forbidden_graph:
                stack.enter_context(mock.patch.object(
                    m20, "check_isolated_graph", side_effect=reject_graph))
            with contextlib.redirect_stdout(output):
                result = m20.run()
        return result, output.getvalue(), port, opens

    def test_serial_reset_fault_still_attempts_stop(self):
        code, text, port, opens = self.execute("reset_error")
        self.assertEqual(code, 2)
        self.assertEqual(port.writes, [m20.STOP])
        self.assertIn("M20_IMU_STREAM_STOP_ACK=PASS", text)
        self.assertIn("M20_COMBINED_ERROR=SerialException:", text)
        self.assertTrue(opens and port.closed)

    def test_metadata_callback_failure_attempts_stop(self):
        code, text, port, opens = self.execute(
            "callback_error", injected_callback=True)
        self.assertEqual(code, 2)
        self.assertEqual(port.writes, [m20.START, m20.STOP])
        self.assertIn("M20_IMU_STREAM_START_ACK=PASS", text)
        self.assertIn("imu_metadata_callback_failed", text)
        self.assertIn("M20_IMU_STREAM_STOP_ACK=PASS", text)
        self.assertTrue(opens and port.closed)

    def test_serial_read_fault_still_attempts_stop(self):
        code, text, port, _opens = self.execute("read_error")
        self.assertEqual(code, 2)
        self.assertEqual(port.writes, [m20.START, m20.STOP])
        self.assertIn("M20_IMU_STREAM_STOP_ACK=PASS", text)
        self.assertIn("M20_COMBINED_ERROR=SerialException:", text)

    def test_missing_stop_ack_is_not_accepted(self):
        code, text, port, _opens = self.execute(
            "read_error", stop_ack=False)
        self.assertEqual(code, 2)
        self.assertEqual(port.writes, [m20.START, m20.STOP])
        self.assertIn("M20_IMU_STREAM_STOP_ACK=UNVERIFIED", text)
        self.assertNotIn("M20_COMBINED_VERDICT=OBSERVED_CONCURRENT", text)

    def test_forbidden_graph_prevents_serial_open(self):
        code, text, port, opens = self.execute(
            "read_error", forbidden_graph=True)
        self.assertEqual(code, 2)
        self.assertEqual(opens, [])
        self.assertEqual(port.writes, [])
        self.assertFalse(port.opened)
        self.assertIn("synthetic_forbidden_graph", text)


if __name__ == "__main__":
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RunnerCleanupTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if result.wasSuccessful():
        print("M21H_STOP_CLEANUP_SELFTEST=PASS", flush=True)
        print("M21H_VERDICT=SIMULATED_M20_SERIAL_CLEANUP_ONLY", flush=True)
        print("M21H_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M21H_STOP_CLEANUP_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
