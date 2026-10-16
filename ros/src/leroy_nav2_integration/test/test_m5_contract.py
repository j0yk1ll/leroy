"""M5 offline diagnostic controller contract: no motor-capable graph."""

import ast
import importlib.util
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT.parents[1]
M5 = ROOT / 'config/m5_controller.yaml'
LAUNCH = ROOT / 'launch/m5_controller_fixture.launch.py'
PROBE = ROOT / 'leroy_nav2_integration/controller_probe.py'


def test_m5_mppi_configuration_uses_diagnostic_odometry():
    params = yaml.safe_load(M5.read_text())
    controller = params['controller_server']['ros__parameters']
    plugin = controller['FollowPath']
    cm = params['local_costmap']['local_costmap']['ros__parameters']
    m4 = yaml.safe_load(
        (ROOT / 'config/rolling_costmap.yaml').read_text()
    )['costmap']['ros__parameters']

    assert controller['odom_topic'] == '/leroy_nav2/diagnostics/synthetic_odom'
    assert controller['enable_stamped_cmd_vel'] is False
    assert controller['controller_plugins'] == ['FollowPath']
    assert controller['use_realtime_priority'] is False
    assert plugin['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert plugin['motion_model'] == 'Omni'
    assert plugin['vx_min'] < 0 < plugin['vx_max']
    assert plugin['vy_max'] > 0
    assert plugin['wz_max'] > 0
    assert plugin['model_dt'] <= 1.0 / controller['controller_frequency']
    assert cm['global_frame'] == 'odom'
    assert cm['robot_base_frame'] == 'base_link'
    assert cm['rolling_window'] is True
    assert cm['track_unknown_space'] is True
    assert type(cm['width']) is int and type(cm['height']) is int
    assert cm['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert cm['footprint'] == m4['footprint']
    assert cm['inflation_layer']['inflation_radius'] == m4['inflation_layer']['inflation_radius']
    assert cm['obstacle_layer']['footprint_clearing_enabled'] is False
    cloud = cm['obstacle_layer']['depth_points']
    assert cloud['marking'] is True and cloud['clearing'] is False
    assert cloud['topic'] == '/leroy_nav2/diagnostics/obstacle_points'
    assert 'amcl' not in params and 'map_server' not in params


def test_launch_remaps_every_motor_cmd_vel_and_requires_offline():
    source = LAUNCH.read_text()
    spec = importlib.util.spec_from_file_location('m5_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.PROPOSED_CMD == '/leroy_nav2/diagnostics/proposed_cmd_vel'
    assert "('cmd_vel', PROPOSED_CMD)" in source
    assert "('/cmd_vel', PROPOSED_CMD)" in source
    assert "'autostart': False" in source
    assert "'node_names': ['controller_server']" in source
    for name in (
        "package='leroy_nav2_odometry'",
        "executable='synthetic_odom'",
        "package='leroy_nav2_perception'",
        "executable='synthetic_depth_fixture'",
        "executable='depth_obstacles'",
        "package='nav2_controller'",
        "executable='controller_server'",
    ):
        assert name in source
    for forbidden in (
        'leroy_bridge', 'robot.launch.py', 'nav2_bt_navigator',
        "executable='planner_server'", 'velocity_smoother',
        'leroy_depthart', 'xrslam'):
        assert forbidden not in source
    assert module.generate_launch_description() is not None


@pytest.mark.parametrize(('key', 'bad'), [
    ('LEROY_NAV2_DRY_RUN', '0'),
    ('ROS_DOMAIN_ID', '0'),
    ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'SUBNET'),
])
def test_controller_launch_rejects_wrong_environment(monkeypatch, key, bad):
    spec = importlib.util.spec_from_file_location('m5_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    for k, v in (
        ('LEROY_NAV2_DRY_RUN', '1'),
        ('ROS_DOMAIN_ID', '167'),
        ('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    ):
        monkeypatch.setenv(k, v)
    assert module._guard(None) == []
    monkeypatch.setenv(key, bad)
    with pytest.raises(RuntimeError):
        module._guard(None)


def test_action_probe_is_bounded_and_does_not_publish_commands():
    source = PROBE.read_text()
    ast.parse(source)
    assert "'/follow_path'" in source
    assert "controller_id = 'FollowPath'" in source
    assert 'send_goal_async(goal)' in source
    assert 'cancel_goal_async()' in source
    assert "'/leroy_nav2/diagnostics/proposed_cmd_vel'" in source
    assert "Twist, PROPOSED_CMD, self._on_cmd" in source
    assert "'/cmd_vel' in topics or '/odom' in topics" in source
    assert "'/local_costmap/costmap'" in source
    assert 'self.map_ready' in source
    assert 'self.nonzero_samples >= 1' in source
    assert 'abs(linear.y) <= 0.151' in source
    assert 'no /cmd_vel or live /odom' in source
    assert 'create_publisher(' not in source
    assert 'ActionServer' not in source
    assert 'leroy_bridge' in source
    assert 'time.monotonic()' in source


def test_dockerfile_remains_hardware_free_and_package_is_tested():
    recipe = (ROS / 'Dockerfile.nav2').read_text()
    assert 'COPY src/leroy_nav2_integration' in recipe
    assert 'COPY src/leroy_bridge' not in recipe
    assert 'COPY src/leroy_bringup' not in recipe
    assert 'COPY src/leroy_depthart' not in recipe
    assert 'colcon test --python-testing pytest' in recipe
    setup = (ROOT / 'setup.py').read_text()
    assert "'controller_probe = leroy_nav2_integration.controller_probe:main'" in setup
    assert "tests_require=['pytest']" in setup
