"""M22D: actual LeroyBridgeNode CDC method contract, WITHOUT importing ROS.

The test AST-compiles the unchanged method body of the production node
after guarding edits. Its fake serial receiver is in memory; no Node is
constructed, no ROS is started, and no USB/MCU/camera device is opened.
"""
import ast
from pathlib import Path
import threading
from types import SimpleNamespace
import unittest

from leroy_bridge.command_ingress import (
    CommandRejected, ZERO_WHEELS, command_to_bytes,
)

BRIDGE_ROOT = Path(__file__).resolve().parents[1]
NODE_PATH = BRIDGE_ROOT / "leroy_bridge" / "node.py"
PARSER_PATH = BRIDGE_ROOT / "leroy_bridge" / "command_ingress.py"


class FakeSerialError(Exception):
    pass


class MemorySerial:
    def __init__(self):
        self.writes = []
        self.flush_count = 0
        self.fail_next_write = False
        self.fail_next_flush = False

    def write(self, data):
        if self.fail_next_write:
            self.fail_next_write = False
            raise FakeSerialError("injected fake CDC write failure")
        if type(data) is not bytes:
            raise AssertionError("serial writer bypassed command guard")
        self.writes.append(data)
        return len(data)

    def flush(self):
        if self.fail_next_flush:
            self.fail_next_flush = False
            raise FakeSerialError("injected fake CDC flush failure")
        self.flush_count += 1


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(str(message))


class FakeBridge:
    """Only attributes needed by exact production _send_command method."""

    def __init__(self, send_command):
        self._serial = MemorySerial()
        self._serial_lock = threading.Lock()
        self._transport_fault = threading.Event()
        self._stop_event = threading.Event()
        self._logger = FakeLogger()
        self._method = send_command

    def get_logger(self):
        return self._logger

    def send(self, command):
        return self._method(self, command)


def source_tree():
    return ast.parse(NODE_PATH.read_text())


def bridge_class(tree):
    return next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "LeroyBridgeNode")


def method_named(klass, name):
    return next(
        node for node in klass.body
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
        and node.name == name)


def load_exact_production_send_method():
    # Compile ONLY _send_command from the production source AST.
    # Loading the full LeroyBridgeNode module would import rclpy/pyserial;
    # instantiating it could open the physical MCU.
    target = method_named(bridge_class(source_tree()), "_send_command")
    module = ast.fix_missing_locations(
        ast.Module(body=[target], type_ignores=[]))
    context = {
        "serial": SimpleNamespace(
            SerialException=FakeSerialError,
            SerialTimeoutException=FakeSerialError),
        "CommandRejected": CommandRejected,
        "command_to_bytes": command_to_bytes,
    }
    namespace = {}
    exec(compile(module, str(NODE_PATH), "exec"), context, namespace)
    return namespace["_send_command"]


class M22DBridgeLockdownTests(unittest.TestCase):
    def test_zero_wheels_and_readonly_telemetry_commands_allowed(self):
        commands = [
            ZERO_WHEELS, "keepalive", "set_watchdog_ms 500",
            "set_watchdog_ms 1", "set_watchdog_ms 60000",
            "set_imu_stream_hz 100", "set_encoder_stream_hz 50",
            "set_imu_stream_hz 0", "set_encoder_stream_hz 0",
        ]
        for command in commands:
            with self.subTest(command=command):
                self.assertEqual(
                    command_to_bytes(command),
                    (command + "\n").encode("ascii"))

    def test_any_nonzero_wheel_duty_denied(self):
        commands = [
            "set_wheels 1 0 0 0", "set_wheels 0 -1 0 0",
            "set_wheels 0 0 1 0", "set_wheels 0 0 0 -255",
            "set_wheels -64 -64 -64 -64",
            "set_wheels 999 0 0 0", "set_wheels -0 0 0 0",
            "set_wheels 00 0 0 0",
        ]
        for command in commands:
            with self.subTest(command=command), self.assertRaises(CommandRejected):
                command_to_bytes(command)

    def test_protocol_smuggling_and_noncanonical_commands_denied(self):
        commands = [
            "set_wheels 0 0 0 0\nset_wheels 100 0 0 0",
            "set_wheels 0 0 0 0\n",
            "set_wheels 0 0 0 0\r",
            "set_wheels 0 0 0 0\0",
            "set_wheels 0 0 0", "set_wheels 0 0 0 0 ",
            "set_wheels\t0 0 0 0", "KEEPALIVE", "keepalive\n",
            "set_pwm 1 2 3 4", "speed 100", "reset_mcu",
            "set_wheels 0 0 0 0 🚗",
        ]
        for command in commands:
            with self.subTest(command=command), self.assertRaises(CommandRejected):
                command_to_bytes(command)

    def test_watchdog_cannot_be_disabled(self):
        for command in (
            "set_watchdog_ms 0", "set_watchdog_ms 000",
            "set_watchdog_ms -1", "set_watchdog_ms 60001",
            "set_watchdog_ms 9999999999999999999999999",
        ):
            with self.subTest(command=command), self.assertRaises(CommandRejected):
                command_to_bytes(command)

    def test_sensor_rates_and_argument_types_are_restricted(self):
        for command in (
            "set_imu_stream_hz 11", "set_encoder_stream_hz 999",
            "set_imu_stream_hz 010", "set_encoder_stream_hz -10",
            "set_encoder_stream_hz 100000000000000000000000000000000000",
            "set_watchdog_ms 1; set_wheels 10 0 0 0",
            "set_watchdog_ms 1.0",
            None, b"keepalive", True, 1, ["keepalive"],
        ):
            with self.subTest(command=command), self.assertRaises(CommandRejected):
                command_to_bytes(command)

    def test_existing_bridge_node_has_no_direct_cmd_vel_subscription(self):
        tree = source_tree()
        cls = bridge_class(tree)
        methods = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
        self.assertNotIn("_on_cmd_vel", methods)
        self.assertNotIn("_check_cmd_timeout", methods)
        calls = [node for node in ast.walk(cls) if isinstance(node, ast.Call)]
        subs = [node for node in calls
                if isinstance(node.func, ast.Attribute)
                and node.func.attr == "create_subscription"]
        self.assertFalse(any(
            isinstance(arg, ast.Constant) and arg.value == "/cmd_vel"
            for call in subs for arg in call.args))
        self.assertFalse(any(
            isinstance(node, ast.Constant) and node.value == "/cmd_vel"
            for node in ast.walk(cls)))
        self.assertFalse(any(
            isinstance(node, ast.Attribute) and node.attr == "_cmd_sub"
            for node in ast.walk(cls)))

    def test_production_serial_writer_is_guarded_and_unique(self):
        cls = bridge_class(source_tree())
        send = method_named(cls, "_send_command")
        writer_calls = [
            node for node in ast.walk(cls) if isinstance(node, ast.Call)
            and isinstance(node.func, ast.Attribute)
            and node.func.attr == "write"
            and isinstance(node.func.value, ast.Attribute)
            and node.func.value.attr == "_serial"
        ]
        self.assertEqual(len(writer_calls), 1)
        self.assertTrue(any(node is writer_calls[0] for node in ast.walk(send)))
        calls = [node for node in ast.walk(send) if isinstance(node, ast.Call)]
        self.assertTrue(any(isinstance(node.func, ast.Name)
                            and node.func.id == "command_to_bytes"
                            for node in calls))
        self.assertTrue(any(
            isinstance(node, ast.ExceptHandler)
            and isinstance(node.type, ast.Name)
            and node.type.id == "CommandRejected"
            for node in ast.walk(send)))
        self.assertIn("from leroy_bridge.command_ingress import",
                      NODE_PATH.read_text())

    def test_actual_bridge_send_method_rejects_motion_before_io(self):
        bridge = FakeBridge(load_exact_production_send_method())
        self.assertFalse(bridge.send("set_wheels 12 0 0 0"))
        self.assertEqual(bridge._serial.writes, [])
        self.assertEqual(bridge._serial.flush_count, 0)
        self.assertTrue(bridge._logger.errors)
        self.assertTrue(bridge.send("set_wheels 0 0 0 0"))
        self.assertEqual(bridge._serial.writes, [b"set_wheels 0 0 0 0\n"])

    def test_actual_bridge_send_method_handles_fake_write_exception(self):
        bridge = FakeBridge(load_exact_production_send_method())
        bridge._serial.fail_next_write = True
        self.assertFalse(bridge.send("set_wheels 0 0 0 0"))
        self.assertEqual(bridge._serial.writes, [])
        self.assertEqual(len(bridge._logger.errors), 1)

    def test_actual_bridge_send_method_handles_fake_flush_exception(self):
        bridge = FakeBridge(load_exact_production_send_method())
        bridge._serial.fail_next_flush = True
        self.assertFalse(bridge.send("set_imu_stream_hz 50"))
        self.assertEqual(bridge._serial.flush_count, 0)
        self.assertEqual(len(bridge._logger.errors), 1)

    def test_bridge_keepalive_and_stream_configuration_still_allowed(self):
        bridge = FakeBridge(load_exact_production_send_method())
        for text in ("set_watchdog_ms 500", "set_imu_stream_hz 100",
                     "set_encoder_stream_hz 50", "keepalive"):
            self.assertTrue(bridge.send(text))
        self.assertEqual(
            bridge._serial.writes,
            [(command + "\n").encode("ascii")
             for command in ("set_watchdog_ms 500",
                             "set_imu_stream_hz 100",
                             "set_encoder_stream_hz 50", "keepalive")])

    def test_other_bridge_code_cannot_send_nonzero_via_writer(self):
        bridge = FakeBridge(load_exact_production_send_method())
        for cmd in (
            "set_wheels 64 64 64 64",
            "set_wheels 0 0 0 1",
            "set_wheels 0 0 0 0\nset_wheels 1 1 1 1",
        ):
            self.assertFalse(bridge.send(cmd))
        self.assertFalse(bridge._serial.writes)
        self.assertEqual(len(bridge._logger.errors), 3)

    def test_no_external_ros_device_test_dependencies(self):
        source = PARSER_PATH.read_text()
        imports = set()
        for node in ast.walk(ast.parse(source)):
            if isinstance(node, ast.Import):
                imports.update(alias.name.split(".")[0] for alias in node.names)
            if isinstance(node, ast.ImportFrom):
                imports.add((node.module or "").split(".")[0])
        self.assertEqual(imports, {"re"})
        self.assertNotIn("import rclpy", source)
        self.assertNotIn("/dev/", source)

    def test_real_bridge_settings_keep_positive_watchdog(self):
        text = NODE_PATH.read_text()
        self.assertIn("watchdog_ms must be in 1..60000 during M22D", text)
        self.assertIn("command_to_bytes(command)", text)
        self.assertNotIn("self._cmd_sub =", text)


if __name__ == "__main__":
    result = unittest.TextTestRunner(verbosity=2).run(
        unittest.defaultTestLoader.loadTestsFromTestCase(
            M22DBridgeLockdownTests))
    if result.wasSuccessful():
        print("M22D_BRIDGE_INGRESS_LOCKDOWN_SELFTEST=PASS", flush=True)
        print("M22D_VERDICT=RAW_CMD_VEL_REMOVED_ZERO_WHEEL_ALLOWLIST_ONLY",
              flush=True)
        print("M22D_DEVICE_ACCESS=NONE", flush=True)
        raise SystemExit(0)
    print("M22D_BRIDGE_INGRESS_LOCKDOWN_SELFTEST=FAIL", flush=True)
    raise SystemExit(2)
