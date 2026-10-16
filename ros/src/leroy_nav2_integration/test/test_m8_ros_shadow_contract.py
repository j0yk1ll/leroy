"""M8 disconnected ROS-shadow contracts; no ROS nodes started by pytest."""

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from leroy_nav2_integration.shadow_policy import (
    DiagnosticProposal, ShadowWindow,
)
from leroy_nav2_integration.shadow_replay import snapshot


ROOT = Path(__file__).resolve().parents[1]
AUDIT = ROOT / 'leroy_nav2_integration/ros_shadow_audit.py'
CANDIDATE = ROOT / 'leroy_nav2_integration/synthetic_shadow_candidate.py'
LAUNCH = ROOT / 'launch/m8_ros_shadow_fixture.launch.py'
M6_LAUNCH = ROOT / 'launch/m6_fault_fixture.launch.py'


def calls(path):
    tree = ast.parse(path.read_text())
    return {
        (node.func.attr if isinstance(node.func, ast.Attribute)
         else node.func.id if isinstance(node.func, ast.Name) else None)
        for node in ast.walk(tree) if isinstance(node, ast.Call)
    }


def test_shadow_ros_observer_cannot_publish_or_send_action():
    audit = AUDIT.read_text()
    cs = calls(AUDIT)
    assert 'create_subscription' in cs
    assert 'create_publisher' not in cs
    assert 'send_goal_async' not in cs
    assert 'ActionClient' not in cs
    assert 'publish' not in cs
    assert 'get_publishers_info_by_topic' in cs
    assert 'ShadowWindow' in audit
    assert 'ShadowSnapshot' in audit
    assert 'diagnostic_state' not in audit  # M7 policy is actually reused
    assert 'node.window.observe(node.snapshot())' in audit
    assert 'WOULD_VETO_REARM' in audit
    assert 'HEALTHY_DIAGNOSTIC_ONLY' in audit
    assert "FAULTS = {'depth': 'WOULD_VETO_BLOCKED_DEPTH'" in audit
    assert "'odom': 'WOULD_VETO_BLOCKED_ODOM'" in audit
    assert "'/cmd_vel' in topics or '/odom' in topics" in audit
    assert '/leroy_nav2/diagnostics/proposed_cmd_vel' not in audit
    for blocked in ('leroy_bridge', 'planner_server', 'velocity_smoother'):
        assert blocked in audit


def test_only_new_fictional_topic_is_published_by_fixture():
    source = CANDIDATE.read_text()
    assert 'create_publisher' in calls(CANDIDATE)
    assert 'create_subscription' not in calls(CANDIDATE)
    assert source.count('self.create_publisher(') == 1
    assert "SYNTHETIC_CANDIDATE = '/leroy_nav2/diagnostics/synthetic_shadow_candidate'" in source
    assert 'Twist, SYNTHETIC_CANDIDATE, 10' in source
    assert 'msg.linear.x = 0.04' in source
    assert 'msg.linear.y = 0.02' in source
    assert 'msg.angular.z = 0.1' in source
    assert "'/cmd_vel'" not in source
    assert '/leroy_nav2/diagnostics/proposed_cmd_vel' not in source
    assert 'ActionClient' not in source
    assert 'leroy_bridge' not in source


def test_m8_launch_reuses_isolated_m6_fixture_without_new_action():
    launch = LAUNCH.read_text()
    assert 'IncludeLaunchDescription(' in launch
    assert 'm6_fault_fixture.launch.py' in launch
    assert "choices=['depth', 'odom']" in launch
    assert "executable='synthetic_shadow_candidate'" in launch
    assert "'autostart':" not in launch  # inherited M6 autostart=False
    assert "parameters=[" not in launch  # inherited M6 controller config
    for required in (
        "'LEROY_NAV2_DRY_RUN': '1'",
        "'ROS_DOMAIN_ID': '167'",
        "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'",
    ):
        assert required in launch
    for forbidden in ('FollowPath.Goal', 'controller_probe', 'ros2 action send_goal',
                      'leroy_bridge', 'ExecuteProcess'):
        assert forbidden not in launch
    m6 = M6_LAUNCH.read_text()
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in m6
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in m6
    assert "'autostart': False" in m6
    assert 'FollowPath.Goal' not in m6


@pytest.mark.parametrize('fault', ['depth', 'odom'])
def test_shadow_window_contract_requires_real_freshness_then_rearm(fault):
    window = ShadowWindow()
    assert window.observe(snapshot(0.0)) == 'WOULD_VETO_REARM'
    assert window.observe(snapshot(1.2)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    label = 'WOULD_VETO_BLOCKED_' + fault.upper()
    assert window.observe(snapshot(10.0, fault=fault)) == label
    assert window.observe(snapshot(12.0, fault=fault)) == label
    assert window.observe(snapshot(16.0)) == 'WOULD_VETO_REARM'
    assert window.observe(snapshot(16.5)) == 'WOULD_VETO_REARM'
    assert window.observe(snapshot(17.2)) == 'HEALTHY_DIAGNOSTIC_ONLY'


def test_good_stimulus_never_masks_missing_both_sensors():
    s = snapshot(10.0, fault='both')
    assert ShadowWindow().observe(s) == 'WOULD_VETO_BLOCKED_BOTH'


def test_no_positive_label_from_nonfinite_or_out_of_limits_stimulus():
    w = ShadowWindow()
    s = snapshot(0.0)
    bad = replace(s.proposal, vx=float('nan'))
    assert w.observe(replace(s, proposal=bad)) == 'WOULD_VETO_INVALID_PROPOSAL'
    too_fast = DiagnosticProposal(.5, 0., 0., 0., 0., 0.,
                                  received_mono=101.0)
    assert w.observe(snapshot(1.0, proposal=too_fast)) == 'WOULD_VETO_INVALID_LIMITS'


def test_m8_executables_registered_in_optional_image():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    for program in ('synthetic_shadow_candidate', 'ros_shadow_audit'):
        assert ("'"+program+" = leroy_nav2_integration."+program+":main'") in setup
        assert ("'leroy_nav2_integration "+program+"'") in docker
    assert 'COPY src/leroy_nav2_integration' in docker
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in docker.replace('\\n', ' ')
    for forbidden in ('COPY src/leroy_bridge', 'COPY src/leroy_depthart',
                      'COPY src/leroy_bringup'):
        assert forbidden not in docker
