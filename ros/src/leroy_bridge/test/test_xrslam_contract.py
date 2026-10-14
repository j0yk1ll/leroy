"""Source-level safety regression guards (no MCU or ROS graph required)."""

import ast
import re
from pathlib import Path


BRIDGE = Path(__file__).resolve().parents[1]
SOURCE = (BRIDGE / 'leroy_bridge' / 'node.py').read_text(encoding='utf-8')
CONFIG = (BRIDGE / 'config' / 'leroy_bridge.yaml').read_text(encoding='utf-8')


def _default(name):
    tree = ast.parse(SOURCE)
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
            if node.func.attr == 'declare_parameter' and len(node.args) >= 2:
                if isinstance(node.args[0], ast.Constant) and node.args[0].value == name:
                    return ast.literal_eval(node.args[1])
    raise AssertionError(f'Missing declared parameter: {name}')


def _yaml_value(name):
    match = re.search(r'^\s*' + re.escape(name) + r':\s*(\S+)\s*$', CONFIG, re.M)
    assert match is not None, f'Missing bridge setting {name}'
    return match.group(1)


def test_no_competing_odometry_default():
    assert _default('degraded_odometry_enabled') is False
    assert _default('publish_odom_tf') is False
    assert _yaml_value('degraded_odometry_enabled') == 'false'
    assert _yaml_value('publish_odom_tf') == 'false'


def test_odom_endpoints_are_created_only_in_diagnostic_mode():
    assert 'if self._degraded_odometry_enabled else None' in SOURCE
    assert 'if self._degraded_odometry_enabled and self._publish_odom_tf else None' in SOURCE


def test_supervised_low_pwm_and_watchdog_interlocks():
    # No explicit YAML => motor commands remain disabled by default.
    assert _default('wheel_velocity_to_pwm_gain') == 0.0
    # The robot YAML explicitly opts into provisional, bounded low-duty output.
    assert _yaml_value('wheel_velocity_to_pwm_gain') == '12.75'
    assert _yaml_value('max_pwm') == '64'
    assert _yaml_value('watchdog_ms') == '500'
    assert _yaml_value('keepalive_period_s') == '0.10'
    assert _yaml_value('cmd_vel_timeout_s') == '0.25'
    assert _yaml_value('wheel_radius_m') == '0.020'
    assert _yaml_value('half_wheelbase_m') == '0.180'
    assert _yaml_value('half_track_m') == '0.275'
    assert 'max_wheel_angular_velocity_rad_s' not in SOURCE
    assert 'max_wheel_angular_velocity_rad_s' not in CONFIG
    assert 'pwm_full_scale_wheel_velocity_rad_s' not in SOURCE
    assert 'pwm_full_scale_wheel_velocity_rad_s' not in CONFIG


def test_cached_motor_safety_parameters_are_read_only():
    # A live ROS parameter set must not report a value different from the
    # cached PWM/watchdog fields actually used by the bridge.
    tree = ast.parse(SOURCE)
    readonly = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != 'declare_parameter' or not node.args:
            continue
        name = node.args[0]
        if not isinstance(name, ast.Constant) or not isinstance(name.value, str):
            continue
        for keyword in node.keywords:
            if keyword.arg != 'descriptor':
                continue
            desc = keyword.value
            if not isinstance(desc, ast.Call):
                continue
            if not isinstance(desc.func, ast.Name) or desc.func.id != 'ParameterDescriptor':
                continue
            if any(option.arg == 'read_only'
                   and isinstance(option.value, ast.Constant)
                   and option.value.value is True for option in desc.keywords):
                readonly.add(name.value)
    expected = {
        'wheel_velocity_to_pwm_gain', 'max_pwm', 'watchdog_ms',
        'cmd_vel_timeout_s', 'keepalive_period_s',
        'wheel_radius_m', 'half_wheelbase_m', 'half_track_m',
    }
    assert expected <= readonly


def test_clock_discontinuity_latches_and_withholds_keepalives():
    assert "except ClockSyncError as exc:" in SOURCE
    assert "self._clock_fault = True" in SOURCE
    assert "if self._clock_fault:" in SOURCE
    assert "self._send_command('set_wheels 0 0 0 0')" in SOURCE
    assert "def _send_keepalive(self)" in SOURCE
    assert "if self._clock_fault:\n            return\n        self._send_command('keepalive')" in SOURCE
