"""M14 real SOFTWARE suppression, offline ROS diagnostic-only regression."""

import ast
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import pytest

from leroy_nav2_integration.m14_gate_graph_contract import inspect_gate_publisher
from leroy_nav2_integration.m9_mppi_source import (
    UNKNOWN_NODE, UNKNOWN_NAMESPACE,
)
from leroy_nav2_integration.m14_gate_core import (
    DiagnosticCommandGate, ZERO, proposal_valid,
)
from leroy_nav2_integration.shadow_policy import (
    DiagnosticProposal, ShadowSnapshot,
)


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'leroy_nav2_integration'
LAUNCH = ROOT / 'launch/m14_gate_fault_fixture.launch.py'


def snapshot(now, *, proposal=None, **updates):
    if proposal is None:
        proposal = DiagnosticProposal(
            vx=.1, vy=.02, vz=0., wx=0., wy=0., wz=.05,
            received_mono=now)
    base = ShadowSnapshot(
        now_mono=now, now_ros=100.+now,
        odom_recv=now, odom_stamp=100.+now,
        depth_recv=now, depth_stamp=100.+now,
        map_recv=now, map_valid=True,
        proposal=proposal)
    return replace(base, **updates)


def test_no_data_or_missing_source_publishes_exact_zero():
    gate = DiagnosticCommandGate()
    value, label = gate.evaluate(
        snapshot(10.), source_ok=False, graph_ok=True, tf_ok=True)
    assert value == ZERO and label == 'BLOCKED_SOURCE'
    value, label = gate.evaluate(
        snapshot(10.1), source_ok=True, graph_ok=False, tf_ok=True)
    assert value == ZERO and label == 'BLOCKED_GRAPH'
    assert gate.healthy_since is None


def test_one_second_rearm_then_real_bounded_proposal_forwarding():
    gate = DiagnosticCommandGate()
    for now in (10., 10.5, 10.999):
        out, label = gate.evaluate(
            snapshot(now), source_ok=True, graph_ok=True, tf_ok=True)
        assert (out, label) == (ZERO, 'BLOCKED_REARM')
    out, label = gate.evaluate(
        snapshot(11.001), source_ok=True, graph_ok=True, tf_ok=True)
    assert out == (.1, .02, .05)
    assert label == 'FORWARD_DIAGNOSTIC_ONLY'


@pytest.mark.parametrize('blocked,field', [
    ('BLOCKED_ODOM', 'odom_recv'),
    ('BLOCKED_DEPTH', 'depth_recv'),
    ('BLOCKED_COSTMAP', 'map_valid'),
])
def test_sensor_or_costmap_loss_zeroes_even_previous_nonzero(blocked, field):
    gate = DiagnosticCommandGate()
    gate.evaluate(snapshot(10.), source_ok=True, graph_ok=True, tf_ok=True)
    gate.evaluate(snapshot(11.1), source_ok=True, graph_ok=True, tf_ok=True)
    broken = {field: 8.} if field != 'map_valid' else {field: False}
    out, label = gate.evaluate(
        snapshot(11.2, **broken), source_ok=True, graph_ok=True, tf_ok=True)
    assert out == ZERO and label == blocked
    # An immediate good input must NOT resume without recovery dwell.
    out, label = gate.evaluate(
        snapshot(11.3), source_ok=True, graph_ok=True, tf_ok=True)
    assert (out, label) == (ZERO, 'BLOCKED_REARM')



def test_tf_loss_blocks_even_fresh_other_inputs():
    gate = DiagnosticCommandGate()
    gate.evaluate(snapshot(10.), source_ok=True, graph_ok=True, tf_ok=True)
    gate.evaluate(snapshot(11.1), source_ok=True, graph_ok=True, tf_ok=True)
    out, reason = gate.evaluate(
        snapshot(11.2), source_ok=True, graph_ok=True, tf_ok=False)
    assert (out, reason) == (ZERO, 'BLOCKED_TF')
    assert gate.healthy_since is None

def test_stale_proposal_is_zero_even_when_sensors_fresh():
    g = DiagnosticCommandGate()
    g.evaluate(snapshot(10.), source_ok=True, graph_ok=True, tf_ok=True)
    stale = DiagnosticProposal(
        .1, 0., 0., 0., 0., .05, 10.)
    out, label = g.evaluate(
        snapshot(11., proposal=stale), source_ok=True, graph_ok=True, tf_ok=True)
    assert (out, label) == (ZERO, 'BLOCKED_STALE_PROPOSAL')


@pytest.mark.parametrize('bad', [
    DiagnosticProposal(.151, 0., 0., 0., 0., 0., 10.),
    DiagnosticProposal(0., 0., 0.01, 0., 0., 0., 10.),
    DiagnosticProposal(float('nan'), 0., 0., 0., 0., 0., 10.),
])
def test_malformed_or_nonplanar_candidate_never_forwarded(bad):
    g = DiagnosticCommandGate()
    out, reason = g.evaluate(
        snapshot(10., proposal=bad), source_ok=True, graph_ok=True, tf_ok=True)
    assert out == ZERO and reason.startswith('BLOCKED_INVALID')


def test_clock_reversal_and_duplicate_tick_deny_output():
    g = DiagnosticCommandGate()
    g.evaluate(snapshot(10.), source_ok=True, graph_ok=True, tf_ok=True)
    g.evaluate(snapshot(11.1), source_ok=True, graph_ok=True, tf_ok=True)
    for ts in (11.1, 10.8):
        out, label = g.evaluate(
            snapshot(ts), source_ok=True, graph_ok=True, tf_ok=True)
        assert (out, label) == (ZERO, 'BLOCKED_CLOCK')


def test_source_identity_loss_requires_new_rearm():
    g = DiagnosticCommandGate()
    g.evaluate(snapshot(10.), source_ok=True, graph_ok=True, tf_ok=True)
    out, _ = g.evaluate(snapshot(11.1), source_ok=True, graph_ok=True, tf_ok=True)
    assert out != ZERO
    out, reason = g.evaluate(
        snapshot(11.2), source_ok=False, graph_ok=True, tf_ok=True)
    assert (out, reason) == (ZERO, 'BLOCKED_SOURCE')
    out, reason = g.evaluate(
        snapshot(11.3), source_ok=True, graph_ok=True, tf_ok=True)
    assert (out, reason) == (ZERO, 'BLOCKED_REARM')


def test_m14_ros_gate_actual_publisher_contract():
    s = (PKG / 'm14_software_gate.py').read_text()
    ast.parse(s)
    assert "GATED_TOPIC = '/leroy_nav2/diagnostics/gated_cmd_vel'" in s
    assert "STATUS_TOPIC = '/leroy_nav2/diagnostics/m14_gate_status'" in s
    assert "Twist, GATED_TOPIC" in s
    assert "Twist, MPPI_TOPIC" in s
    assert "String, STATUS_TOPIC" in s
    assert 'self.out.publish(cmd)' in s
    assert 'self.status.publish(msg)' in s
    assert 'cmd.linear.x, cmd.linear.y, cmd.angular.z = output' in s
    assert 'self.gate.evaluate(' in s
    assert "TFMessage, '/tf'" in s
    assert 'self._on_tf' in s
    assert 'tf_ok=tf_ok' in s
    assert 'M14 diagnostic topic remapping forbidden' in s
    assert "'/cmd_vel' in topics" in s
    assert 'send_goal_async' not in s
    # Inspect executable Python syntax, not prose in the module docstring.
    # The existing source says "serial" only to document excluded I/O.
    tree = ast.parse(s)
    imports = {
        alias.name.split('.')[0]
        for node in ast.walk(tree) if isinstance(node, ast.Import)
        for alias in node.names
    } | {
        (node.module or '').split('.')[0]
        for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)
    }
    assert not {'serial', 'serial_asyncio'} & imports
    calls = [
        node for node in ast.walk(tree) if isinstance(node, ast.Call)
    ]
    assert not any(
        (isinstance(call.func, ast.Name)
         and call.func.id in {'Serial', 'serial_for_url'})
        or (isinstance(call.func, ast.Attribute)
            and call.func.attr in {'Serial', 'serial_for_url'})
        for call in calls
    )
    # Exactly two ROS publishers, both on the diagnostic-only topics.
    publishers = [
        call for call in calls
        if isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_publisher'
    ]
    assert len(publishers) == 2
    assert {ast.unparse(call.args[1]) for call in publishers} == {
        'GATED_TOPIC', 'STATUS_TOPIC'}
    assert 'self.create_timer(0.1, self._tick)' in s


def test_six_nodes_and_both_mppi_diagnostic_remaps():
    s = LAUNCH.read_text()
    ast.parse(s)
    assert s.count('Node(') == 6
    assert "'m14_software_gate'" in s
    assert "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in s
    assert "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)" in s
    assert "'autostart': False" in s
    assert "'ROS_DOMAIN_ID': '167'" in s
    assert "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'" in s
    assert "'m11_timed_odom'" in s and "'m11_timed_depth'" in s
    for bad in ('leroy_bridge', 'bt_navigator', 'planner_server'):
        assert bad not in s


def test_audit_asserts_real_suppression_not_only_labels():
    s = (PKG / 'm14_gate_audit.py').read_text()
    ast.parse(s)
    assert 'blocked_input_nonzero < 3' in s
    assert 'any(gated_during)' in s
    assert "any(item['output_nonzero'] for item in all_post_veto)" in s
    assert 'ts >= self.blackout_end' in s
    assert 'len(gated_during) < 5' in s
    assert 'node.gate_recovered_nonzero < 1' in s
    assert 'node.gate_pre_nonzero >= 1' in s
    assert 'node.sustained_veto' in s
    assert 'parse_window(reply.message)' in s
    assert 'self.gate_first_block = ts' in s
    assert 'create_publisher' not in s
    assert 'blocked_gate_nonzero_ticks=0' in s


def test_image_registers_only_diagnostic_gate_and_audit():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    for name, target in (
        ('m14_software_gate', 'm14_software_gate:main'),
        ('m14_gate_audit', 'm14_gate_audit:main'),
    ):
        assert (f"'{name} = leroy_nav2_integration.{target}'"
                in setup)
        assert f"'leroy_nav2_integration {name}'" in docker
    assert 'm14_gate_fault_fixture.launch.py' in docker
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in docker
    for excluded in ('COPY src/leroy_bridge', 'COPY src/leroy_bringup'):
        assert excluded not in docker



def _publisher(name='m14_software_gate', namespace='/',
               typ='geometry_msgs/msg/Twist'):
    return SimpleNamespace(
        node_name=name, node_namespace=namespace, topic_type=typ)


@pytest.mark.parametrize('typ', [
    'geometry_msgs/msg/Twist', 'std_msgs/msg/String',
])
def test_m14_gated_graph_source_exact_identity_is_required(typ):
    verified, pending, evidence = inspect_gate_publisher(
        [_publisher(typ=typ)], expected_type=typ)
    assert verified is True and pending is False
    assert evidence == [('m14_software_gate', '/', typ)]


@pytest.mark.parametrize('name,namespace', [
    (UNKNOWN_NODE, '/'),
    ('m14_software_gate', UNKNOWN_NAMESPACE),
    (UNKNOWN_NODE, UNKNOWN_NAMESPACE),
    (None, '/'),
    ('m14_software_gate', ''),
])
def test_m14_gated_graph_incomplete_discovery_is_only_pending(
        name, namespace):
    valid, pending, _ = inspect_gate_publisher(
        [_publisher(name=name, namespace=namespace)],
        expected_type='geometry_msgs/msg/Twist')
    assert valid is False and pending is True


def test_m14_gated_graph_empty_discovery_is_pending_not_verified():
    assert inspect_gate_publisher(
        [], expected_type='geometry_msgs/msg/Twist') == (
            False, True, [])


@pytest.mark.parametrize('endpoints,expected_type', [
    ([_publisher(name='spoofed_gate')], 'geometry_msgs/msg/Twist'),
    ([_publisher(namespace='/other')], 'geometry_msgs/msg/Twist'),
    ([_publisher(typ='std_msgs/msg/String')], 'geometry_msgs/msg/Twist'),
    ([_publisher(), _publisher()], 'geometry_msgs/msg/Twist'),
    ([_publisher(name=UNKNOWN_NODE), _publisher()],
     'geometry_msgs/msg/Twist'),
])
def test_m14_gated_graph_invalid_source_or_duplicate_fails_immediately(
        endpoints, expected_type):
    verified, pending, evidence = inspect_gate_publisher(
        endpoints, expected_type=expected_type)
    assert (verified, pending) == (False, False)
    assert evidence


def test_m14_audit_requires_resolved_outputs_and_bounded_discovery():
    audit = (PKG / 'm14_gate_audit.py').read_text()
    assert 'inspect_gate_publisher(' in audit
    assert 'GATE_DISCOVERY_TIMEOUT_S = 12.0' in audit
    assert 'self.gate_publishers_verified = verified_all' in audit
    assert 'and node.gate_publishers_verified' in audit
    assert 'self.triggered or time.monotonic()' in audit
    assert "repr((topic, evidence))" in audit

def test_physical_hardware_policy_still_off():
    text = (ROOT.parent / 'leroy_nav2/config/hardware.disabled.yaml').read_text()
    assert 'enabled: false' in text
    assert 'bridge_command_output_enabled: false' in text
