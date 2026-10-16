"""M12 release-boundary tests. Pure Python: NEVER launch ROS/hardware."""

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from leroy_nav2_integration import m12_no_go_audit as audit


ROOT = Path(__file__).resolve().parents[1]
NAV2 = ROOT.parent / 'leroy_nav2'
BRIDGE = ROOT.parent / 'leroy_bridge'
BRINGUP = ROOT.parent / 'leroy_bringup'
AUDIT = ROOT / 'leroy_nav2_integration/m12_no_go_audit.py'
M11 = ROOT / 'M11_BLACKOUT_WINDOW_METRICS.md'


def test_m12_never_emits_hardware_permission():
    result = audit.report()
    assert result['verdict'] == 'NO_GO_LIVE_ROBOT_NOT_AUTHORIZED'
    assert result['scope'] == 'DISCONNECTED_SYNTHETIC_READINESS_REVIEW_ONLY'
    gates = result['gates']
    assert len(gates) == 11
    assert {g['code'] for g in gates} == {
        f'G{i:02}' for i in range(1, 12)}
    assert all(g['state'] in {'BLOCKED', 'UNVERIFIED'} for g in gates)
    assert all(g['why'] and g['evidence_needed'] and g['source']
               for g in gates)
    # The schema provides NO field for hardware-enable or motor-arm output.
    assert not any(k in result for k in (
        'is_safe', 'can_move', 'motion_enabled', 'motor_enable',
        'motor_permit', 'ready', 'go'))


def test_an_accidental_green_gate_fails_closed(monkeypatch):
    changed = (
        replace(audit.GATES[0], evidence_state='READY'),
        *audit.GATES[1:],
    )
    monkeypatch.setattr(audit, 'GATES', changed)
    with pytest.raises(RuntimeError, match='cannot declare a gate passed'):
        audit.report()


def test_duplicate_or_missing_gate_fails_closed(monkeypatch):
    monkeypatch.setattr(
        audit, 'GATES', audit.GATES + (audit.GATES[0],))
    with pytest.raises(RuntimeError, match='malformed'):
        audit.report()
    monkeypatch.setattr(audit, 'GATES', ())
    with pytest.raises(RuntimeError, match='malformed'):
        audit.report()


def test_report_requires_optional_disconnected_environment(monkeypatch, capsys):
    for key in ('LEROY_NAV2_DRY_RUN', 'ROS_DOMAIN_ID',
                'ROS_AUTOMATIC_DISCOVERY_RANGE'):
        monkeypatch.delenv(key, raising=False)
    with pytest.raises(RuntimeError, match='isolated offline'):
        audit.main([])
    monkeypatch.setenv('LEROY_NAV2_DRY_RUN', '1')
    monkeypatch.setenv('ROS_DOMAIN_ID', '167')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    audit.main([])
    out = capsys.readouterr().out
    assert 'M12 READINESS: NO_GO_LIVE_ROBOT_NOT_AUTHORIZED' in out
    assert out.count('M12 GATE:') == 11
    assert 'PASS: M12 offline NO-GO evidence inventory generated' in out
    assert 'live robot motion is NOT authorized' in out


def test_m12_is_pure_audit_not_ros_subscriber_action_or_motor():
    source = AUDIT.read_text()
    tree = ast.parse(source)
    imports = {
        alias.name for n in ast.walk(tree)
        if isinstance(n, ast.Import) for alias in n.names
    }
    imports |= {
        n.module for n in ast.walk(tree)
        if isinstance(n, ast.ImportFrom)
    }
    assert imports == {'dataclasses', 'os'}
    calls = {
        n.func.attr if isinstance(n.func, ast.Attribute)
        else n.func.id if isinstance(n.func, ast.Name) else None
        for n in ast.walk(tree) if isinstance(n, ast.Call)
    }
    for forbidden in (
        'create_publisher', 'create_subscription', 'publish',
        'send_goal_async', 'ActionClient', 'create_client',
        'open', 'socket', 'Serial', 'Popen', 'system',
    ):
        assert forbidden not in calls
    assert 'rclpy.init' not in source


def test_existing_hardware_nav2_manifest_remains_explicitly_disabled():
    path = NAV2 / 'config/hardware.disabled.yaml'
    cfg = path.read_text()
    assert 'enabled: false' in cfg
    assert 'bridge_command_output_enabled: false' in cfg
    assert 'odometry_validated: false' in cfg
    assert 'obstacle_coverage_validated: false' in cfg
    assert 'footprint_measured: false' in cfg
    assert 'operator_acceptance_completed: false' in cfg


def test_original_mppi_fixture_keeps_diagnostics_only_remaps():
    launch = (
        ROOT / 'launch/m11_timed_fault_fixture.launch.py').read_text()
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in launch
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in launch
    assert "DIAGNOSTIC_CMD_TOPIC = '/leroy_nav2/diagnostics/proposed_cmd_vel'" in launch
    assert "'autostart': False" in launch


def test_report_does_not_erase_real_bridge_software_controls():
    bridge = BRIDGE / 'config/leroy_bridge.yaml'
    if bridge.is_file():
        cfg = bridge.read_text()
        assert 'watchdog_ms: 500' in cfg
        assert 'cmd_vel_timeout_s: 0.25' in cfg
        assert 'max_pwm: 64' in cfg
        assert 'wheel_velocity_to_pwm_gain: 12.75' in cfg
        assert 'front_right_encoder_sign: 0' in cfg
        assert 'rear_right_encoder_sign: 0' in cfg
    # Optional image deliberately does not COPY the real hardware bridge.
    reason = next(g.reason for g in audit.GATES if g.code == 'G02')
    assert 'software watchdogs/timeouts' in reason
    assert 'not independent E-stop' in reason


def test_report_carries_explicit_m11_measurement_nonclaims():
    source = AUDIT.read_text()
    assert '39 depth and 35 odom nonzero samples' in source
    assert '1.0983s depth and 1.4921s odom' in source
    assert 'shadow' in source.lower()
    assert 'NOT_AUTHORIZED' in source
    assert 'blackout' in M11.read_text().lower()


def test_m12_entrypoint_and_optional_image_checks():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert (
        "'m12_no_go_audit = leroy_nav2_integration.m12_no_go_audit:main'"
        in setup)
    assert "'leroy_nav2_integration m12_no_go_audit'" in docker
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in docker.replace(
        '\\n', ' ')
    for excluded in ('COPY src/leroy_bridge', 'COPY src/leroy_depthart',
                     'COPY src/leroy_bringup'):
        assert excluded not in docker
