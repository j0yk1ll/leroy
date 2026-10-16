"""M11 pure synthetic blackout-window accounting and source boundaries.

Unit tests never launch ROS, send an action, or access real hardware.
"""

import ast
import json
from pathlib import Path

import pytest

from leroy_nav2_integration.m11_blackout_metrics import (
    encode_window, parse_window, summarize,
)


ROOT = Path(__file__).resolve().parents[1]
PY = ROOT / 'leroy_nav2_integration'
LAUNCH = ROOT / 'launch/m11_timed_fault_fixture.launch.py'


def test_trigger_window_exact_monotonic_roundtrip():
    message = encode_window(start=100.0, end=105.0)
    assert parse_window(message) == (100.0, 105.0)
    assert json.loads(message)['m11_synthetic_only'] is True


@pytest.mark.parametrize('bad', [
    'not json', '{}',
    '{"m11_synthetic_only":false,"scheduled_start_mono":100,"scheduled_end_mono":105}',
    '{"m11_synthetic_only":true,"scheduled_start_mono":100,"scheduled_end_mono":104}',
    '{"m11_synthetic_only":true,"scheduled_start_mono":"100","scheduled_end_mono":105}',
    '{"m11_synthetic_only":true,"scheduled_start_mono":true,"scheduled_end_mono":105}',
    '{"m11_synthetic_only":true,"scheduled_start_mono":100,"scheduled_end_mono":1e999}',
])
def test_invalid_or_ambiguous_window_fails_closed(bad):
    with pytest.raises(ValueError):
        parse_window(bad)


def test_counts_are_only_within_scheduled_five_seconds():
    samples = [
        (99.99, True), (100.5, True),
        (100.75, False), (101.6, True),
        (104.7, False), (104.8, True),
        (105.0, True),
    ]
    out = summarize(
        samples, start=100., end=105.,
        first_veto=101.6, sensor_recv=100.)
    assert out['scheduled_zero'] == 2
    assert out['scheduled_nonzero'] == 3
    assert out['inner_zero'] == 2
    assert out['inner_nonzero'] == 1
    assert out['after_veto_zero'] == 1
    assert out['after_veto_nonzero'] == 2
    assert out['veto_from_schedule_s'] == 1.6
    assert out['veto_from_last_sensor_rx_s'] == 1.6
    assert out['veto_vs_recv_age_threshold_s'] == 0.1


def test_zero_diagnostic_messages_is_measured_not_assumed_failure():
    result = summarize(
        [], start=100., end=105.,
        first_veto=101.6, sensor_recv=99.95)
    for key in ('scheduled_zero', 'scheduled_nonzero',
                'inner_zero', 'inner_nonzero',
                'after_veto_zero', 'after_veto_nonzero'):
        assert result[key] == 0


@pytest.mark.parametrize('first_veto,sensor_recv', [
    (None, 100.), (99., 100.), (105., 100.),
    (101.6, None), (101.6, 100.1),
])
def test_veto_must_be_inside_window_with_pre_trigger_sensor_baseline(
        first_veto, sensor_recv):
    with pytest.raises(ValueError):
        summarize([], start=100., end=105.,
                  first_veto=first_veto, sensor_recv=sensor_recv)


@pytest.mark.parametrize('sample', [
    (float('nan'), True), (102., 1), (102., None),
])
def test_bad_sample_is_not_silently_counted(sample):
    with pytest.raises(ValueError):
        summarize([sample], start=100., end=105.,
                  first_veto=101.6, sensor_recv=100.)


def test_m11_trigger_only_changes_response_evidence_not_sensor_path():
    source = (PY / 'm11_timed_fake_fault.py').read_text()
    ast.parse(source)
    assert 'class TimedDepth(TriggeredDepth)' in source
    assert 'class TimedOdom(TriggeredOdom)' in source
    assert 'response = super()._trigger(request, response)' in source
    assert 'response.message = encode_window(' in source
    assert 'self._fault_until - BLACKOUT_S' in source
    assert 'self._fault_until)' in source
    assert 'create_publisher(' not in source
    assert 'cmd_vel' not in source


def test_m11_still_five_disconnected_remapped_fixture_processes():
    source = LAUNCH.read_text()
    ast.parse(source)
    assert source.count('Node(') == 5
    assert "'m11_timed_odom'" in source
    assert "'m11_timed_depth'" in source
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in source
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in source
    assert "'autostart': False" in source
    assert "'ROS_DOMAIN_ID': '167'" in source
    assert "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'" in source
    for forbidden in ('leroy_bridge', 'bt_navigator', 'planner_server',
                      'velocity_smoother', 'ActionClient', 'FollowPath.Goal'):
        assert forbidden not in source


def test_m11_probe_can_observe_but_not_publish_trajectory():
    source = (PY / 'm11_blackout_probe.py').read_text()
    tree = ast.parse(source)
    calls = {
        n.func.attr if isinstance(n.func, ast.Attribute)
        else n.func.id if isinstance(n.func, ast.Name) else None
        for n in ast.walk(tree) if isinstance(n, ast.Call)
    }
    assert 'create_publisher' not in calls
    assert 'publish' not in calls
    assert 'send_goal_async' in calls
    assert 'cancel_goal_async' in calls
    assert 'call_async' in calls
    assert 'M10ActiveFaultProbe' in source
    assert 'super()._on_cmd(msg)' in source
    assert 'self.receipt_samples.append((stamp, nonzero))' in source
    assert 'MAX_SAMPLES = 512' in source
    assert 'node.action_nonzero_samples >= 1' in source
    assert 'node.healthy_samples >= 1' in source
    assert 'if goal_result.done():' in source
    assert 'node.triggered = True' in source
    assert 'if not node.sustained_veto:' in source
    assert 'first_veto=node.first_veto_mono' in source
    assert 'sensor_recv=pre_sensor_recv' in source
    assert 'parse_window(reply.message)' in source
    assert 'BLACKOUT METRICS:' in source
    assert 'NO proof of braking' in source
    assert "'/cmd_vel'" not in source
    assert 'm8_synthetic_candidate' not in source


def test_m11_optional_image_has_executables_tests_and_launch():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    for exe, target in (
        ('m11_timed_depth', 'm11_timed_fake_fault:main_depth'),
        ('m11_timed_odom', 'm11_timed_fake_fault:main_odom'),
        ('m11_blackout_probe', 'm11_blackout_probe:main'),
    ):
        assert exe + ' = leroy_nav2_integration.' + target in setup
        assert "'leroy_nav2_integration " + exe + "'" in docker
    assert 'm11_timed_fault_fixture.launch.py' in docker
    assert 'colcon test --python-testing pytest' in docker
    for bad in ('COPY src/leroy_bridge', 'COPY src/leroy_depthart',
                'COPY src/leroy_bringup'):
        assert bad not in docker
