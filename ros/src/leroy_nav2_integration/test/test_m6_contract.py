"""M6 synthetic fault contract tests; pure helpers + launch safety inspection."""

from pathlib import Path
import ast

import pytest

from leroy_nav2_integration.fault_contract import (
    MAX_AGE_S, diagnostic_state, in_blackout,
)


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch/m6_fault_fixture.launch.py'
FAULT_AUDIT = ROOT / 'leroy_nav2_integration/fault_audit.py'
FAULT_DEPTH = ROOT / 'leroy_nav2_integration/fault_depth_fixture.py'
FAULT_ODOM = ROOT / 'leroy_nav2_integration/fault_odom_fixture.py'


@pytest.mark.parametrize(('second', 'expected'), [
    (0., False), (9.999, False), (10., True), (12., True),
    (15.999, True), (16., False), (21.999, False),
    (22., False), (32., True), (38., False),
])
def test_cyclic_synthetic_blackout_exact_boundaries(second, expected):
    assert in_blackout(second) is expected


@pytest.mark.parametrize('bad', [-0.01, float('nan'), float('inf')])
def test_invalid_blackout_schedule_rejected(bad):
    with pytest.raises(ValueError):
        in_blackout(bad)


def healthy_inputs():
    return {
        'now_mono': 100.0,
        'now_ros': 500.0,
        'odom_recv': 99.9,
        'odom_stamp': 499.9,
        'depth_recv': 99.8,
        'depth_stamp': 499.8,
        'map_recv': 99.6,
        'map_valid': True,
    }


def test_all_three_independent_freshness_inputs_required():
    a = healthy_inputs()
    assert diagnostic_state(**a) == 'READY'
    for missing in ('odom_recv', 'odom_stamp'):
        case = {**a, missing: None}
        assert diagnostic_state(**case) == 'BLOCKED_ODOM'
    for missing in ('depth_recv', 'depth_stamp'):
        case = {**a, missing: None}
        assert diagnostic_state(**case) == 'BLOCKED_DEPTH'
    assert diagnostic_state(**{**a, 'map_valid': False}) == 'BLOCKED_COSTMAP'
    assert diagnostic_state(**{**a, 'map_recv': None}) == 'BLOCKED_COSTMAP'


def test_depth_blackout_never_deemed_ready_from_persistent_costmap():
    a = healthy_inputs()
    a.update(depth_recv=100.0 - MAX_AGE_S - 0.02,
             depth_stamp=500.0 - MAX_AGE_S - 0.02,
             map_recv=100., map_valid=True)
    assert diagnostic_state(**a) == 'BLOCKED_DEPTH'
    a.update(depth_recv=99.9, depth_stamp=499.9)
    assert diagnostic_state(**a) == 'READY'


def test_fake_wheel_blackout_blocks_even_with_fresh_depth_and_grid():
    a = healthy_inputs()
    a.update(odom_recv=96., odom_stamp=496.,
             depth_recv=100., depth_stamp=500.,
             map_recv=100., map_valid=True)
    assert diagnostic_state(**a) == 'BLOCKED_ODOM'
    a.update(odom_recv=99.9, odom_stamp=499.9)
    assert diagnostic_state(**a) == 'READY'


@pytest.mark.parametrize(('field', 'value', 'expected'), [
    ('odom_stamp', 498., 'BLOCKED_ODOM'),  # replayed stamp, new arrival
    ('depth_stamp', 498., 'BLOCKED_DEPTH'),
    ('depth_stamp', 500.3, 'BLOCKED_DEPTH'),  # future stamp
    ('odom_recv', 100.3, 'BLOCKED_ODOM'),  # future local receive clock
    ('map_recv', 97.0, 'BLOCKED_COSTMAP'),
    ('now_ros', float('nan'), 'BLOCKED_CLOCK'),
])
def test_outdated_future_and_nonfinite_source_stamps_rejected(
        field, value, expected):
    a = healthy_inputs()
    a[field] = value
    assert diagnostic_state(**a) == expected


def test_m6_launch_has_no_hardware_or_auto_action():
    source = LAUNCH.read_text()
    ast.parse(source)
    for expected in (
        "choices=['depth', 'odom']",
        "'LEROY_NAV2_DRY_RUN': '1'",
        "'ROS_DOMAIN_ID': '167'",
        "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'",
        "('cmd_vel', DIAGNOSTIC_CMD_TOPIC)",
        "('/cmd_vel', DIAGNOSTIC_CMD_TOPIC)",
        "'autostart': False",
        "'node_names': ['controller_server']",
        "executable='controller_server'",
        'fault_depth_fixture', 'fault_odom_fixture',
        'synthetic_depth_fixture', 'synthetic_odom',
        'm5_controller.yaml',
    ):
        assert expected in source
    for forbidden in (
        'leroy_bridge', 'robot.launch.py', 'FollowPath.Goal',
        "executable='planner_server'", 'nav2_bt_navigator',
        'leroy_depthart', 'xrslam', 'publishers=[', 'ExecuteProcess',
    ):
        assert forbidden not in source


def test_sensor_faults_override_only_existing_fake_publisher_callbacks():
    depth = FAULT_DEPTH.read_text()
    odom = FAULT_ODOM.read_text()
    for text in (depth, odom):
        ast.parse(text)
        assert 'in_blackout' in text
        assert 'get_logger().warning' in text
        assert 'create_publisher(' not in text
        assert 'create_subscription(' not in text
        assert "'/cmd_vel'" not in text
    assert 'class FaultDepthFixture(SyntheticDepthFixture)' in depth
    assert 'super()._publish()' in depth
    assert 'class FaultOdomFixture(SyntheticOdom)' in odom
    assert 'super()._tick()' in odom
    assert 'self._previous = now' in odom


def test_health_probe_is_read_only_and_requires_block_then_recovery():
    source = FAULT_AUDIT.read_text()
    ast.parse(source)
    assert 'create_publisher(' not in source
    assert 'ActionClient' not in source
    assert 'FollowPath.Goal' not in source
    assert 'diagnostic_state(' in source
    assert "seen_expected_block" in source
    assert "status == MODES[node.mode]" in source
    assert "status == 'READY'" in source
    assert "'/cmd_vel' in topics or '/odom' in topics" in source
    assert "node.get_clock().now().nanoseconds" not in source  # uses self in node
    assert 'self.get_clock().now().nanoseconds' in source


def test_m6_executables_exist_in_optional_image_recipe():
    recipe = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    setup = (ROOT / 'setup.py').read_text()
    assert 'COPY src/leroy_nav2_integration' in recipe
    assert 'colcon test --python-testing pytest --return-code-on-test-failure' in recipe.replace(
        '\\n', ' ')
    for script in ('fault_depth_fixture', 'fault_odom_fixture', 'fault_audit'):
        assert "'"+script+" = leroy_nav2_integration." in setup
        assert "'leroy_nav2_integration " + script + "'" in recipe
    for forbidden in ('COPY src/leroy_bridge', 'COPY src/leroy_bringup',
                      'COPY src/leroy_depthart'):
        assert forbidden not in recipe

def test_both_missing_streams_cannot_masquerade_as_one_fault():
    a = healthy_inputs()
    a.update(odom_recv=None, odom_stamp=None,
             depth_recv=None, depth_stamp=None)
    assert diagnostic_state(**a) == 'BLOCKED_BOTH'
