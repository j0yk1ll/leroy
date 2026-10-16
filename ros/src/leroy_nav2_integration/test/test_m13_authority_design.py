"""M13 non-actuating command-authority DESIGN tests; no ROS test runs."""

import ast
from dataclasses import replace
from pathlib import Path

import pytest

from leroy_nav2_integration import m13_authority_design as design


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'leroy_nav2_integration/m13_authority_design.py'
M12 = ROOT / 'leroy_nav2_integration/m12_no_go_audit.py'
M11_LAUNCH = ROOT / 'launch/m11_timed_fault_fixture.launch.py'
M12_DOC = ROOT / 'M12_HARDWARE_READINESS_NO_GO.md'
M13_DOC = ROOT / 'M13_AUTHORITY_AND_STOP_DESIGN.md'


def scenario(**kwargs):
    return replace(design.IMAGINED_BASELINE, **kwargs)


def test_all_scenarios_fail_even_with_other_imagined_evidence_true():
    matrix = design.scenario_matrix()
    assert len(matrix) == 9
    assert len({name for name, _ in matrix}) == 9
    assert all(rejections for _, rejections in matrix)
    assert all(
        label.startswith('MODEL_WOULD_REJECT_')
        for _, rejections in matrix for label in rejections)
    assert dict(matrix)['physical_stop_unknown'] == (
        'MODEL_WOULD_REJECT_INDEPENDENT_STOP_UNVERIFIED',)
    assert dict(matrix)['duplicate_command_owner'] == (
        'MODEL_WOULD_REJECT_OWNER_COUNT',)
    assert dict(matrix)['stale_odom'] == (
        'MODEL_WOULD_REJECT_STALE_ODOM',)
    assert dict(matrix)['stale_depth'] == (
        'MODEL_WOULD_REJECT_STALE_DEPTH',)
    assert dict(matrix)['tf_loss'] == ('MODEL_WOULD_REJECT_BAD_TF',)


def test_physical_stop_missing_preempts_every_other_model_fault():
    r = design.reasons(scenario(
        independent_stop_evidence=False, logical_owner_count=3,
        odom_fresh=False, depth_fresh=False))
    assert r[0] == 'MODEL_WOULD_REJECT_INDEPENDENT_STOP_UNVERIFIED'
    assert 'MODEL_WOULD_REJECT_OWNER_COUNT' in r
    assert 'MODEL_WOULD_REJECT_STALE_ODOM' in r
    assert 'MODEL_WOULD_REJECT_STALE_DEPTH' in r


@pytest.mark.parametrize('owner_count', [0, 2, 3])
def test_duplicate_or_absent_authority_is_rejected(owner_count):
    assert 'MODEL_WOULD_REJECT_OWNER_COUNT' in design.reasons(
        scenario(logical_owner_count=owner_count))


@pytest.mark.parametrize('field,label', [
    ('proposed_source_authenticated',
     'MODEL_WOULD_REJECT_SOURCE_UNAUTHENTICATED'),
    ('odom_fresh', 'MODEL_WOULD_REJECT_STALE_ODOM'),
    ('depth_fresh', 'MODEL_WOULD_REJECT_STALE_DEPTH'),
    ('tf_healthy', 'MODEL_WOULD_REJECT_BAD_TF'),
    ('proposal_present', 'MODEL_WOULD_REJECT_NO_PROPOSAL'),
    ('proposal_receipt_fresh', 'MODEL_WOULD_REJECT_STALE_PROPOSAL'),
])
def test_each_single_bad_signal_is_sufficient_to_reject(field, label):
    assert design.reasons(scenario(**{field: False})) == (label,)


def test_initial_condition_never_assumes_software_model_armed():
    d = design.DesignLatch()
    assert d.fault_latched is True
    assert d.observe(scenario()) == 'MODEL_WOULD_HOLD_MANUAL_REVIEW'
    assert d.fault_latched is True
    assert d.observe(scenario(now_mono=11.0)) == (
        'MODEL_WOULD_HOLD_MANUAL_REVIEW')


def test_manual_review_and_full_illustrative_dwell_are_both_required():
    d = design.DesignLatch()
    assert d.observe(scenario()) == 'MODEL_WOULD_HOLD_MANUAL_REVIEW'
    assert d.observe(scenario(now_mono=10.1, manual_review_ack=True)) == (
        'MODEL_WOULD_HOLD_RECOVERY')
    assert d.observe(scenario(now_mono=11.099)) == (
        'MODEL_WOULD_HOLD_RECOVERY')
    assert d.observe(scenario(now_mono=11.101)) == (
        design.POSITIVE_DIAGNOSTIC)
    assert design.POSITIVE_DIAGNOSTIC.endswith('DIAGNOSTIC_ONLY')
    assert 'PERMIT' not in design.POSITIVE_DIAGNOSTIC


def test_any_fault_relocks_and_blocks_automatic_recovery():
    d = design.DesignLatch()
    d.observe(scenario())
    d.observe(scenario(now_mono=10.1, manual_review_ack=True))
    assert d.observe(scenario(now_mono=11.2)) == design.POSITIVE_DIAGNOSTIC
    assert d.observe(scenario(
        now_mono=11.3, depth_fresh=False, manual_review_ack=True)) == (
            'MODEL_WOULD_REJECT_STALE_DEPTH')
    assert d.fault_latched is True
    assert d.observe(scenario(now_mono=12.4)) == (
        'MODEL_WOULD_HOLD_MANUAL_REVIEW')
    assert d.observe(scenario(
        now_mono=12.5, manual_review_ack=True)) == 'MODEL_WOULD_HOLD_RECOVERY'
    assert d.observe(scenario(now_mono=13.4)) == 'MODEL_WOULD_HOLD_RECOVERY'
    assert d.observe(scenario(now_mono=13.6)) == design.POSITIVE_DIAGNOSTIC


def test_nonmonotonic_clock_forces_new_manual_review():
    d = design.DesignLatch()
    d.observe(scenario())
    d.observe(scenario(now_mono=10.1, manual_review_ack=True))
    assert d.observe(scenario(now_mono=11.2)) == design.POSITIVE_DIAGNOSTIC
    assert d.observe(scenario(now_mono=11.2)) == (
        'MODEL_WOULD_REJECT_NONMONOTONIC_CLOCK')
    assert d.fault_latched
    assert d.observe(scenario(now_mono=11.3)) == (
        'MODEL_WOULD_HOLD_MANUAL_REVIEW')


@pytest.mark.parametrize('params', [
    {'now_mono': float('nan')},
    {'now_mono': float('inf')},
    {'now_mono': -1.0},
    {'now_mono': True},
    {'logical_owner_count': -1},
    {'logical_owner_count': 1.5},
    {'logical_owner_count': True},
    {'depth_fresh': 1},
    {'independent_stop_evidence': None},
    {'manual_review_ack': 'yes'},
])
def test_malformed_synthetic_inputs_fail_closed(params):
    with pytest.raises(ValueError):
        design.reasons(scenario(**params))


def test_readiness_report_always_stays_no_go_after_hypothetical_positive():
    r = design.report()
    assert r['verdict'] == 'NO_GO_LIVE_ROBOT_NOT_AUTHORIZED'
    assert r['scope'] == 'OFFLINE_NON_ACTUATING_DESIGN_THOUGHT_EXPERIMENT'
    assert len(r['unclosed_gates']) == 11
    assert r['synthetic_rearm_example'][-1] == design.POSITIVE_DIAGNOSTIC
    assert r['verdict'] != r['synthetic_rearm_example'][-1]
    assert M12.read_text().find("NO_GO_LIVE_ROBOT_NOT_AUTHORIZED") >= 0


def test_command_line_is_no_go_only_with_offline_environment(monkeypatch, capsys):
    monkeypatch.setenv('ROS_DOMAIN_ID', '167')
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    monkeypatch.delenv('LEROY_NAV2_DRY_RUN', raising=False)
    with pytest.raises(RuntimeError, match='disconnected offline'):
        design.main()
    monkeypatch.setenv('LEROY_NAV2_DRY_RUN', '1')
    design.main()
    out = capsys.readouterr().out
    assert 'M13 DESIGN: NO_GO_LIVE_ROBOT_NOT_AUTHORIZED' in out
    assert out.count('M13 CASE:') == 9
    assert 'M13 M12-GATES-STILL-OPEN: G01' in out
    assert 'PASS: M13 offline design thought experiment' in out


def test_static_scope_does_not_publish_subscribe_or_control_robot():
    source = SCRIPT.read_text()
    tree = ast.parse(source)
    imports = {
        a.name for n in ast.walk(tree)
        if isinstance(n, ast.Import) for a in n.names
    } | {
        n.module for n in ast.walk(tree)
        if isinstance(n, ast.ImportFrom) and n.level == 0
    }
    assert imports == {'dataclasses', 'math', 'os'}
    assert 'rclpy' not in source
    calls = {
        n.func.attr if isinstance(n.func, ast.Attribute)
        else n.func.id if isinstance(n.func, ast.Name) else None
        for n in ast.walk(tree) if isinstance(n, ast.Call)
    }
    for forbidden in ('create_publisher', 'create_subscription', 'publish',
                      'send_goal_async', 'ActionClient', 'create_client',
                      'socket', 'Serial', 'Popen', 'system'):
        assert forbidden not in calls
    assert "'/cmd_vel'" not in source


def test_existing_m11_remap_and_m12_disabled_manifest_not_changed():
    launch = M11_LAUNCH.read_text()
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in launch
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in launch
    manifest = (ROOT.parent / 'leroy_nav2/config/hardware.disabled.yaml').read_text()
    assert 'enabled: false' in manifest
    assert 'bridge_command_output_enabled: false' in manifest
    assert 'operator_acceptance_completed: false' in manifest


def test_m13_spec_traceability_and_no_real_implementation():
    doc = M13_DOC.read_text()
    for req in ('AR-01', 'AR-02', 'AR-03', 'AR-04', 'AR-05',
                'FS-01', 'FS-02', 'FS-03', 'FS-04', 'FS-05',
                'IV-01', 'IV-02', 'IV-03', 'IV-04'):
        assert req in doc
    for gate in ('G01', 'G02', 'G03', 'G08', 'G09'):
        assert gate in doc
    assert 'NO-GO' in doc
    assert 'NOT' in doc
    assert 'wheel_velocity_to_pwm_gain' in doc
    assert '0.721249' in doc


def test_optional_image_registration_only():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert (
        "'m13_authority_design = leroy_nav2_integration.m13_authority_design:main'"
        in setup)
    assert "'leroy_nav2_integration m13_authority_design'" in docker
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in docker.replace(
        '\\n', ' ')
    for forbidden in ('COPY src/leroy_bridge', 'COPY src/leroy_bringup',
                      'COPY src/leroy_depthart'):
        assert forbidden not in docker
