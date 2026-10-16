"""M9: source-verified actual MPPI diagnostic ROS observer contracts.

These unit tests never launch ROS, issue a FollowPath goal, or publish
velocity. The ROS integration acceptance is an operator-only Podman run.
"""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest

from leroy_nav2_integration.m9_mppi_source import (
    MPPI_TOPIC, inspect_mppi_source,
)
from leroy_nav2_integration.shadow_policy import ShadowWindow
from leroy_nav2_integration.shadow_replay import snapshot


ROOT = Path(__file__).resolve().parents[1]
M9 = ROOT / 'leroy_nav2_integration/m9_mppi_shadow_probe.py'
M5_LAUNCH = ROOT / 'launch/m5_controller_fixture.launch.py'
M5_PROBE = ROOT / 'leroy_nav2_integration/controller_probe.py'
M8_PRODUCER = ROOT / 'leroy_nav2_integration/synthetic_shadow_candidate.py'


def ep(name='controller_server', namespace='/',
       typ='geometry_msgs/msg/Twist'):
    return SimpleNamespace(
        node_name=name, node_namespace=namespace, topic_type=typ)


def test_exact_source_has_one_correct_node_namespace_and_twist_type():
    assert MPPI_TOPIC == '/leroy_nav2/diagnostics/proposed_cmd_vel'
    assert inspect_mppi_source([ep()]) == (
        True, False,
        [('controller_server', '/', 'geometry_msgs/msg/Twist')],
    )
    assert inspect_mppi_source([]) == (False, True, [])


@pytest.mark.parametrize('node,namespace,typ,pending', [
    ('_NODE_NAME_UNKNOWN_', '/', 'geometry_msgs/msg/Twist', True),
    ('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_',
     'geometry_msgs/msg/Twist', True),
    ('controller_server', '_NODE_NAMESPACE_UNKNOWN_',
     'geometry_msgs/msg/Twist', True),
    ('other_node', '/', 'geometry_msgs/msg/Twist', False),
    ('controller_server', '/other', 'geometry_msgs/msg/Twist', False),
    ('controller_server', '/', 'std_msgs/msg/String', False),
    ('_NODE_NAME_UNKNOWN_', '/', 'std_msgs/msg/String', False),
])
def test_unresolved_only_waits_without_considering_identity_verified(
        node, namespace, typ, pending):
    verified, actually_pending, _ = inspect_mppi_source([
        ep(node, namespace, typ)])
    assert not verified
    assert actually_pending is pending


def test_two_publishers_are_never_accepted_or_pending():
    for endpoints in ([ep(), ep()], [ep(), ep('other_node')]):
        verified, pending, report = inspect_mppi_source(endpoints)
        assert not verified and not pending and len(report) == 2


def test_reuses_m5_unchanged_hard_remap_and_synthetic_only_controller():
    launch = M5_LAUNCH.read_text()
    assert "('cmd_vel', PROPOSED_CMD)" in launch
    assert "('/cmd_vel', PROPOSED_CMD)" in launch
    assert "PROPOSED_CMD = '/leroy_nav2/diagnostics/proposed_cmd_vel'" in launch
    assert "executable='synthetic_odom'" in launch
    assert "executable='synthetic_depth_fixture'" in launch
    assert "executable='depth_obstacles'" in launch
    assert "'autostart': False" in launch
    assert "'ROS_DOMAIN_ID': '167'" in launch
    for bad in ('leroy_bridge', 'robot.launch.py', 'FollowPath.Goal',
                'm8_synthetic_candidate', 'synthetic_shadow_candidate'):
        assert bad not in launch


def test_m9_is_action_client_and_read_only_ros_subscriber():
    source = M9.read_text()
    ast.parse(source)
    tree = ast.parse(source)
    calls = {
        node.func.attr if isinstance(node.func, ast.Attribute)
        else node.func.id if isinstance(node.func, ast.Name) else None
        for node in ast.walk(tree) if isinstance(node, ast.Call)
    }
    assert 'create_subscription' in calls
    assert 'create_publisher' not in calls
    assert 'publish' not in calls
    assert 'send_goal_async' in calls
    assert 'cancel_goal_async' in calls
    assert 'spin_until_future_complete' in calls
    assert 'ActionServer' not in source
    assert 'ControllerProbe' in source
    assert 'ShadowWindow' in source
    assert 'ShadowSnapshot' in source
    assert 'DiagnosticProposal' in source
    assert 'MPPI_TOPIC' in source
    assert 'inspect_mppi_source(endpoints)' in source
    assert 'GRAPH_DISCOVERY_TIMEOUT_S = 12.0' in source
    assert 'WAITING_MPPI_GRAPH_IDENTITY' in source
    assert 'node.window = ShadowWindow()' not in source  # instance-local
    assert 'self.window = ShadowWindow()' in source
    assert 'self.healthy_samples = 0' in source
    assert "self.action_nonzero_samples >= 1" not in source  # node state used
    assert 'node.action_nonzero_samples >= 1' in source
    assert 'node.healthy_samples >= 1' in source
    assert 'CLOUD = ' in source
    assert "'/leroy_nav2/diagnostics/obstacle_points'" in source
    assert 'map_valid' in source
    assert 'self.map_recv' in source
    assert 'NO motor command' in source
    assert 'synthetic_shadow_candidate' not in source
    assert 'create_publisher' not in M5_PROBE.read_text()
    assert 'self._publisher.publish' in M8_PRODUCER.read_text()


def test_m7_policy_checks_both_sensors_and_new_candidate_after_rearm():
    w = ShadowWindow()
    assert w.observe(snapshot(0.0)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(1.1)) == 'HEALTHY_DIAGNOSTIC_ONLY'
    assert w.observe(snapshot(2.0, fault='depth')) == 'WOULD_VETO_BLOCKED_DEPTH'
    assert w.observe(snapshot(2.1)) == 'WOULD_VETO_REARM'
    assert w.observe(snapshot(3.3)) == 'HEALTHY_DIAGNOSTIC_ONLY'


def test_m9_docker_build_requires_console_entrypoint():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert (
        "'mppi_shadow_probe = leroy_nav2_integration.m9_mppi_shadow_probe:main'"
        in setup
    )
    assert "'leroy_nav2_integration mppi_shadow_probe'" in docker
    assert 'COPY src/leroy_bridge' not in docker
    assert 'COPY src/leroy_depthart' not in docker
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in docker.replace(
        '\\n', ' ')



def test_per_message_m9_strict_m7_limits_cannot_be_hidden_by_good_followups():
    source = M9.read_text()
    assert 'abs(msg.linear.x) > VX_LIMIT + EPS' in source
    assert 'abs(msg.linear.y) > VY_LIMIT + EPS' in source
    assert 'abs(msg.angular.z) > WZ_LIMIT + EPS' in source
    assert 'FAIL: actual MPPI proposal violates M7 fictional bounds' in source
    assert 'self.failure =' in source
    from dataclasses import replace
    from leroy_nav2_integration.shadow_policy import classify
    base = snapshot(0.0)
    bad = replace(base.proposal, vx=0.15001)
    assert classify(replace(base, proposal=bad)) == 'WOULD_VETO_INVALID_LIMITS'
