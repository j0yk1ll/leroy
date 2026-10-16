"""Offline-only M4 source and geometry contracts; no devices or live graph."""

import importlib.util
from pathlib import Path

import pytest
import yaml

from leroy_nav2_integration.probe_math import spans, tracks_robot


ROOT = Path(__file__).resolve().parents[1]
ROS = ROOT.parents[1]
CONFIG = ROOT / 'config/rolling_costmap.yaml'
LAUNCH = ROOT / 'launch/integrated_fixture.launch.py'


def test_config_integrates_m2_marking_into_odom_frame():
    p = yaml.safe_load(CONFIG.read_text())['costmap']['ros__parameters']
    m2 = yaml.safe_load(
        (ROS / 'src/leroy_nav2_perception/config/fixture_costmap.yaml').read_text()
    )['costmap']['ros__parameters']
    assert p['global_frame'] == 'odom'
    assert p['robot_base_frame'] == 'base_link'
    assert p['rolling_window'] is True
    assert p['track_unknown_space'] is True
    assert type(p['width']) is int and type(p['height']) is int
    assert p['width'] == 4 and p['height'] == 4
    assert p['resolution'] == pytest.approx(0.05)
    assert p['plugins'] == ['obstacle_layer', 'inflation_layer']
    assert p['footprint'] == m2['footprint']
    assert p['inflation_layer']['inflation_radius'] >= 0.51
    obstacle = p['obstacle_layer']
    source = obstacle['depth_points']
    assert obstacle['footprint_clearing_enabled'] is False
    assert obstacle['observation_sources'] == 'depth_points'
    assert source['topic'] == '/leroy_nav2/diagnostics/obstacle_points'
    assert source['topic'] == m2['obstacle_layer']['depth_points']['topic']
    assert source['data_type'] == 'PointCloud2'
    assert source['marking'] is True
    assert source['clearing'] is False
    assert p['always_send_full_costmap'] is True


def test_m4_uses_only_synthetic_nodes_and_no_controller():
    text = LAUNCH.read_text()
    for pair in (
        ("package='leroy_nav2_odometry'", "executable='synthetic_odom'"),
        ("package='leroy_nav2_perception'", "executable='synthetic_depth_fixture'"),
        ("package='leroy_nav2_perception'", "executable='depth_obstacles'"),
        ("package='leroy_nav2_costmap_fixture'", "executable='costmap_fixture'"),
    ):
        assert pair[0] in text and pair[1] in text
    assert "'autostart': False" in text
    assert 'ROS_DOMAIN_ID' in text and 'LEROY_NAV2_DRY_RUN' in text
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' in text
    for forbidden in (
        'controller_server', 'planner_server', 'bt_navigator',
        'NavigateToPose', 'FollowPath', 'leroy_bridge', 'cmd_vel',
        'robot.launch.py', 'leroy_depthart', 'xrslam',
    ):
        assert forbidden not in text
    assert 'nav2_costmap_2d' not in text  # stock executable is not launched


def test_guard_denies_nonisolated_environment(monkeypatch):
    spec = importlib.util.spec_from_file_location('integrated_fixture', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    expected = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    for key, val in expected.items():
        monkeypatch.setenv(key, val)
    assert mod._guard(None) == []
    for key in expected:
        monkeypatch.setenv(key, 'invalid')
        with pytest.raises(RuntimeError):
            mod._guard(None)
        monkeypatch.setenv(key, expected[key])


@pytest.mark.parametrize(('robot', 'origin', 'expected'), [
    ((0., 0.), (-2., -2.), True),
    ((0.2, 0.1), (-1.8, -1.9), True),
    ((0.2, 0.1), (-2.4, -2.0), False),
    ((0.2, 0.1), (-2.0, -2.5), False),
])
def test_rolling_window_center_follows_synthetic_odom(robot, origin, expected):
    assert tracks_robot(*origin, 80, 80, 0.05, *robot) is expected


@pytest.mark.parametrize('origin', [
    (float('nan'), -2.0), (-2.0, float('inf')),
])
def test_invalid_costmap_origin_rejected(origin):
    assert tracks_robot(*origin, 80, 80, .05, 0., 0.) is False


def test_pose_and_rolling_origin_must_actually_move():
    assert spans([0., .1, .18], .05)
    assert not spans([0., .01, .02], .05)
    assert not spans([float('nan'), .2], .05)
    assert not spans([], .05)
    probe = (ROOT / 'leroy_nav2_integration/integrated_probe.py').read_text()
    assert 'self.lethal_and_unknown' in probe
    assert "100 in msg.data and -1 in msg.data" in probe
    assert 'self.center_matches' in probe
    assert 'spans(self.origins_x, 0.05)' in probe
    assert 'spans(self.origins_y, 0.05)' in probe
    assert 'self.cloud_pairs >= 2' in probe
    assert 'self.odom_tf_matches >= 5' in probe
    assert 'rclpy.spin_once' in probe


def test_no_real_odom_or_hardware_velocity_authority():
    probe = (ROOT / 'leroy_nav2_integration/integrated_probe.py').read_text()
    launch = LAUNCH.read_text()
    assert "'/cmd_vel' in topics or '/odom' in topics" in probe
    assert 'leroy_bridge' in probe
    assert 'create_publisher(' not in probe
    assert 'create_subscription(' not in launch
    assert "'/cmd_vel'" not in launch
    m3 = (
        ROS / 'src/leroy_nav2_odometry/leroy_nav2_odometry/synthetic_odom.py'
    ).read_text()
    assert 'create_subscription' not in m3
    assert "'/leroy_nav2/diagnostics/synthetic_odom'" in m3
    recipe = (ROS / 'Dockerfile.nav2').read_text()
    assert 'COPY src/leroy_nav2_integration' in recipe
    assert 'leroy_nav2_integration integrated_probe' in recipe
    assert 'COPY src/leroy_bridge' not in recipe
    assert 'COPY src/leroy_bringup' not in recipe
    assert 'COPY src/leroy_depthart' not in recipe
