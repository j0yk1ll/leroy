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


def test_motor_and_watchdog_interlocks_unchanged():
    assert _default('max_wheel_angular_velocity_rad_s') == 0.0
    assert _yaml_value('max_wheel_angular_velocity_rad_s') == '0.0'
    assert _yaml_value('watchdog_ms') == '500'
    assert _yaml_value('keepalive_period_s') == '0.10'
    assert _yaml_value('cmd_vel_timeout_s') == '0.25'
    assert _yaml_value('wheel_radius_m') == '0.020'
    assert _yaml_value('half_wheelbase_m') == '0.180'
    assert _yaml_value('half_track_m') == '0.275'


def test_clock_discontinuity_latches_and_withholds_keepalives():
    assert "except ClockSyncError as exc:" in SOURCE
    assert "self._clock_fault = True" in SOURCE
    assert "if self._clock_fault:" in SOURCE
    assert "self._send_command('set_wheels 0 0 0 0')" in SOURCE
    assert "def _send_keepalive(self)" in SOURCE
    assert "if self._clock_fault:\n            return\n        self._send_command('keepalive')" in SOURCE
