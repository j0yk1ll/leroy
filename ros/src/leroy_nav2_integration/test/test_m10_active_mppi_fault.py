"""M10 contracts: synthetic-only trigger overlapping an accepted MPPI goal.

No pytest test starts a ROS publisher, action server or physical bridge.
Actual Jazzy ROS integration and teardown remain operator-run.
"""

import ast
from pathlib import Path

import pytest

from leroy_nav2_integration.shadow_policy import ShadowWindow
from leroy_nav2_integration.shadow_replay import snapshot


ROOT = Path(__file__).resolve().parents[1]
FAULT = ROOT / 'leroy_nav2_integration/m10_triggered_fake_fault.py'
PROBE = ROOT / 'leroy_nav2_integration/m10_active_fault_probe.py'
LAUNCH = ROOT / 'launch/m10_active_fault_fixture.launch.py'
M6 = ROOT / 'launch/m6_fault_fixture.launch.py'


def read(path):
    source = path.read_text()
    ast.parse(source)
    return source


def calls(path):
    return {
        n.func.attr if isinstance(n.func, ast.Attribute)
        else n.func.id if isinstance(n.func, ast.Name) else None
        for n in ast.walk(ast.parse(path.read_text()))
        if isinstance(n, ast.Call)
    }


def test_m10_fake_source_pause_is_only_one_bounded_trigger():
    source = read(FAULT)
    assert "SERVICE = '/leroy_nav2/diagnostics/m10_trigger_fake_fault'" in source
    assert 'BLACKOUT_S = 5.0' in source
    assert 'self._fault_started = True' in source
    assert "response.success = False" in source
    assert 'time.monotonic() + BLACKOUT_S' in source
    assert 'self._previous = time.monotonic()' in source
    assert 'class TriggeredDepth(TriggeredFakeFault, SyntheticDepthFixture)' in source
    assert 'class TriggeredOdom(TriggeredFakeFault, SyntheticOdom)' in source
    assert 'super()._publish()' in source
    assert 'super()._tick()' in source
    assert 'create_service' in calls(FAULT)
    assert 'create_subscription' not in calls(FAULT)
    assert 'ActionClient' not in source
    assert "'/cmd_vel'" not in source
    assert 'm10_fake_depth' not in source  # registration lives in setup.py


def test_triggered_fake_fault_rejects_second_arm_and_expires(monkeypatch):
    from types import SimpleNamespace
    from leroy_nav2_integration.m10_triggered_fake_fault import (
        TriggeredFakeFault,
    )

    class FakeLogger:
        def warning(self, message):
            assert 'M10 SYNTHETIC FAULT' in message or 'M10 SYNTHETIC SOURCE' in message

    class FakeFixture:
        _fault_started = False
        _fault_until = None
        _was_blocked = False
        def get_logger(self):
            return FakeLogger()

    clock = {'time': 100.0}
    monkeypatch.setattr(
        'leroy_nav2_integration.m10_triggered_fake_fault.time.monotonic',
        lambda: clock['time'],
    )
    fixture = FakeFixture()
    ack = TriggeredFakeFault._trigger(
        fixture, object(), SimpleNamespace(success=False, message=''))
    assert ack.success is True
    assert fixture._fault_until == 105.0
    assert TriggeredFakeFault._blocked(fixture) is True
    second = TriggeredFakeFault._trigger(
        fixture, object(), SimpleNamespace(success=True, message=''))
    assert second.success is False
    clock['time'] = 105.1
    assert TriggeredFakeFault._blocked(fixture) is False


def test_m10_launch_is_five_fixture_processes_and_hard_remapped():
    source = read(LAUNCH)
    assert source.count('Node(') == 5
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in source
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in source
    assert "DIAGNOSTIC_CMD_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'" in source
    assert "executable=('m10_fake_odom'" in source
    assert "executable=('m10_fake_depth'" in source
    assert "choices=['depth', 'odom']" in source
    assert "'autostart': False" in source
    assert "'LEROY_NAV2_DRY_RUN': '1'" in source
    assert "'ROS_DOMAIN_ID': '167'" in source
    assert "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'" in source
    for forbidden in (
        'leroy_bridge', 'planner_server', 'bt_navigator',
        'velocity_smoother', 'FollowPath.Goal', 'cmd_vel_pub',
    ):
        assert forbidden not in source
    assert M6.is_file()


def test_m10_probe_is_a_goal_client_but_never_a_velocity_publisher():
    source = read(PROBE)
    cs = calls(PROBE)
    assert 'create_publisher' not in cs
    assert 'publish' not in cs
    assert 'ActionServer' not in source
    assert 'send_goal_async' in cs
    assert 'cancel_goal_async' in cs
    assert 'call_async' in cs
    assert 'Trigger.Request()' in source
    assert 'M9MppiShadowProbe' in source
    assert 'ControllerProbe.check_graph(self)' in source
    assert 'inspect_mppi_source(endpoints)' in source
    assert 'GRAPH_DISCOVERY_TIMEOUT_S' in source
    assert "node.verified_source" in source
    assert 'node.action_nonzero_samples >= 1' in source
    assert 'node.healthy_samples >= 1' in source
    assert 'if goal_result.done():' in source
    assert 'node.triggered = True' in source
    assert 'FAULT_DWELL_S = 0.5' in source
    assert 'OBSERVATION_AFTER_TRIGGER_S = 9.0' in source
    assert 'node.sustained_veto' in source
    assert 'proposals_after_veto' in source
    assert 'NO controller safe-stop or motor claim' in source
    assert "('cmd_vel'," not in source  # only remapped in the launch
    assert 'm10_trigger_fake_fault' not in source or 'SERVICE' in source


@pytest.mark.parametrize('mode', ['depth', 'odom'])
def test_m7_prioritizes_fake_sensor_blackout_over_fresh_mppi_proposal(mode):
    window = ShadowWindow()
    assert window.observe(snapshot(0.0)) == 'WOULD_VETO_REARM'
    assert window.observe(snapshot(1.2)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    assert window.observe(snapshot(3.0, fault=mode)) == (
        'WOULD_VETO_BLOCKED_' + mode.upper())
    assert window.observe(snapshot(4.0, fault=mode)) == (
        'WOULD_VETO_BLOCKED_' + mode.upper())
    assert window.observe(snapshot(5.0)) == 'WOULD_VETO_REARM'
    assert window.observe(snapshot(6.1)) == 'HEALTHY_DIAGNOSTIC_ONLY'


def test_m10_setup_and_image_keep_tests_and_no_hardware():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    pkg = (ROOT / 'package.xml').read_text()
    for entry, target in (
        ('m10_fake_depth', 'm10_triggered_fake_fault:main_depth'),
        ('m10_fake_odom', 'm10_triggered_fake_fault:main_odom'),
        ('m10_active_fault_probe', 'm10_active_fault_probe:main'),
    ):
        assert entry + ' = leroy_nav2_integration.' + target in setup
        assert "'leroy_nav2_integration " + entry + "'" in docker
    assert '<exec_depend>std_srvs</exec_depend>' in pkg
    assert 'ros-jazzy-std-srvs' in docker
    assert 'colcon test --python-testing pytest' in docker
    assert 'COPY src/leroy_bridge' not in docker
    assert 'COPY src/leroy_depthart' not in docker
