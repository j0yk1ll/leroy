"""M7 shadow policy tests. These tests never start ROS or publish Twist."""

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from leroy_nav2_integration.shadow_policy import (
    DiagnosticProposal, MAX_PROPOSAL_AGE_S, ShadowWindow, classify,
)
from leroy_nav2_integration.shadow_replay import run, snapshot


ROOT = Path(__file__).resolve().parents[1]
POLICY = ROOT / 'leroy_nav2_integration/shadow_policy.py'
REPLAY = ROOT / 'leroy_nav2_integration/shadow_replay.py'


@pytest.mark.parametrize('mode', ['depth', 'odom'])
def test_offline_replay_matches_finite_veto_and_rearm_sequence(mode, capsys):
    run(mode)
    output = capsys.readouterr().out
    assert 'M7 REPLAY: t=0.0 WOULD_VETO_REARM' in output
    assert 'M7 REPLAY: t=1.2 HEALTHY_DIAGNOSTIC_ONLY' in output
    assert 'M7 REPLAY: t=10.0 WOULD_VETO_BLOCKED_' + mode.upper() in output
    assert 'M7 REPLAY: t=12.0 WOULD_VETO_BLOCKED_' + mode.upper() in output
    assert 'M7 REPLAY: t=16.0 WOULD_VETO_REARM' in output
    assert 'M7 REPLAY: t=17.2 HEALTHY_DIAGNOSTIC_ONLY' in output
    assert 'PASS: M7 ' + mode + ' synthetic shadow replay' in output


def test_initially_unhealthy_and_unobserved_candidate_not_positive():
    s = snapshot(0.0)
    assert classify(replace(s, odom_recv=None)) == 'WOULD_VETO_BLOCKED_ODOM'
    assert classify(replace(s, depth_recv=None)) == 'WOULD_VETO_BLOCKED_DEPTH'
    assert classify(replace(s, map_valid=False)) == 'WOULD_VETO_BLOCKED_COSTMAP'
    assert classify(replace(
        s, odom_recv=None, depth_recv=None)) == 'WOULD_VETO_BLOCKED_BOTH'
    assert classify(replace(s, proposal=None)) == 'WOULD_VETO_NO_PROPOSAL'


@pytest.mark.parametrize('attribute,bad_value', [
    ('vx', .15101), ('vx', -.15101),
    ('vy', .15101), ('vy', -.15101),
    ('wz', .30101), ('wz', -.30101),
    ('vz', .001), ('wx', .001), ('wy', -.001),
])
def test_fictional_m5_limits_and_nonplanar_candidates_veto(attribute, bad_value):
    s = snapshot(0.0)
    bad = replace(s.proposal, **{attribute: bad_value})
    assert classify(replace(s, proposal=bad)) == 'WOULD_VETO_INVALID_LIMITS'


@pytest.mark.parametrize('attribute', [
    'vx', 'vy', 'vz', 'wx', 'wy', 'wz', 'received_mono',
])
@pytest.mark.parametrize('value', [float('nan'), float('inf')])
def test_all_nonfinite_proposal_components_veto(attribute, value):
    s = snapshot(0.0)
    bad = replace(s.proposal, **{attribute: value})
    assert classify(replace(s, proposal=bad)) == 'WOULD_VETO_INVALID_PROPOSAL'


def test_stale_future_and_bounded_proposals():
    s = snapshot(0.0)
    assert classify(s) == 'HEALTHY_DIAGNOSTIC_ONLY'
    for age in (-0.001, MAX_PROPOSAL_AGE_S + 0.001):
        bad = replace(s.proposal, received_mono=s.now_mono - age)
        assert classify(replace(s, proposal=bad)) == 'WOULD_VETO_STALE_PROPOSAL'
    high_but_bounded = replace(s.proposal, vx=0.15, vy=-0.15, wz=0.3)
    assert classify(replace(s, proposal=high_but_bounded)) == 'HEALTHY_DIAGNOSTIC_ONLY'


def test_stale_depth_takes_precedence_over_sane_velocity():
    s = snapshot(12.0, fault='depth')
    assert classify(s) == 'WOULD_VETO_BLOCKED_DEPTH'
    s = snapshot(12.0, fault='odom')
    assert classify(s) == 'WOULD_VETO_BLOCKED_ODOM'


def test_one_second_rearm_restarts_on_each_fault_or_clock_reset():
    w = ShadowWindow()
    assert w.observe(snapshot(0)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(1.01)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    assert w.observe(snapshot(2, fault='depth')) == 'WOULD_VETO_BLOCKED_DEPTH'
    assert w.observe(snapshot(2.1)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(3.05)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(3.11)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    assert w.observe(snapshot(3.0)) == 'WOULD_VETO_CLOCK_RESET'
    assert w.observe(snapshot(3.2)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(4.3)) == 'HEALTHY_DIAGNOSTIC_ONLY'


def test_one_second_rearm_restarts_on_bad_diagnostic_proposal():
    w = ShadowWindow()
    assert w.observe(snapshot(0)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(1.1)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    invalid = DiagnosticProposal(.8, 0., 0., 0., 0., 0., 102.0)
    assert w.observe(snapshot(2., proposal=invalid)) == 'WOULD_VETO_INVALID_LIMITS'
    assert w.observe(snapshot(2.1)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(3.2)) == 'HEALTHY_DIAGNOSTIC_ONLY'


@pytest.mark.parametrize('bad', [0., -1., float('nan'), float('inf')])
def test_invalid_recovery_window_rejected(bad):
    with pytest.raises(ValueError):
        ShadowWindow(recovery_s=bad)


def test_shadow_components_are_ros_free_and_do_not_produce_plant_commands():
    for path in (POLICY, REPLAY):
        source = path.read_text()
        ast.parse(source)
        assert 'import rclpy' not in source
        assert 'create_publisher(' not in source
        assert 'ActionClient(' not in source
        assert 'genui' not in source
        assert 'leroy_bridge' not in source
        assert "def main()" in REPLAY.read_text()
    assert 'return \'HEALTHY_DIAGNOSTIC_ONLY\'' in POLICY.read_text()
    assert 'diagnostic_state(' in POLICY.read_text()
    assert "return 'WOULD_VETO_REARM'" in POLICY.read_text()


def test_m7_executable_is_included_in_disconnected_development_image():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert "'shadow_replay = leroy_nav2_integration.shadow_replay:main'" in setup
    assert "leroy_nav2_integration shadow_replay" in docker
    assert "colcon test --python-testing pytest --return-code-on-test-failure" in docker.replace(
        '\\n', ' ')
    assert 'COPY src/leroy_bridge' not in docker
    assert 'COPY src/leroy_depthart' not in docker
