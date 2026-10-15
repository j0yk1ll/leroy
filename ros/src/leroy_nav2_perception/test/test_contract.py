"""Guard against accidental hardware/odometry introduction to M2."""

import os
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


def test_diagnostic_defaults_deny_obstacle_authority():
    p = yaml.safe_load(
        (ROOT / 'config/depth_diagnostics.yaml').read_text()
    )['/depth_obstacles']['ros__parameters']
    assert p['publish_obstacles'] is False
    assert p['extrinsics_validated'] is False
    assert p['ground_reference_validated'] is False
    assert p['depth_topic'] == '/depthart/depth'
    assert p['camera_info_topic'] == '/camera/camera_info'


def test_synthetic_fixture_obstacle_gates_and_costmap():
    sim = yaml.safe_load(
        (ROOT / 'config/fixture_obstacles.yaml').read_text()
    )['/depth_obstacles']['ros__parameters']
    assert sim['publish_obstacles'] is True
    assert sim['extrinsics_validated'] is True
    assert sim['ground_reference_validated'] is True

    config = yaml.safe_load(
        (ROOT / 'config/fixture_costmap.yaml').read_text()
    )['costmap']['ros__parameters']
    assert config['global_frame'] == 'base_link'
    assert config['robot_base_frame'] == 'base_link'
    assert config['track_unknown_space'] is True
    assert type(config['width']) is int
    assert type(config['height']) is int
    source = config['obstacle_layer']['depth_points']
    assert config['obstacle_layer']['observation_sources'] == 'depth_points'
    assert source['data_type'] == 'PointCloud2'
    assert source['marking'] is True
    assert source['clearing'] is False
    assert source['topic'] == sim['obstacle_cloud_topic']
    assert config['plugins'] == ['obstacle_layer', 'inflation_layer']


@pytest.mark.parametrize('name', [
    'depth_diagnostics.launch.py', 'depth_fixture.launch.py'
])
def test_launch_modes_are_offline_and_motor_free(name):
    launch = (ROOT / 'launch' / name).read_text()
    assert "ROS_DOMAIN_ID" in launch
    assert "LEROY_NAV2_DRY_RUN" in launch
    assert "ROS_AUTOMATIC_DISCOVERY_RANGE" in launch
    for forbidden in ('leroy_bridge', 'cmd_vel', 'robot.launch.py',
                      'nav2_controller', 'nav2_bt_navigator',
                      'nav2_velocity_smoother'):
        assert forbidden not in launch


def test_obstacle_output_never_clears_on_invalid_depth():
    node = (ROOT / 'leroy_nav2_perception/depth_obstacles.py').read_text()
    assert 'no_valid_depth_pixels' in node
    assert 'missing_exact_timestamp_camera_info' in node
    assert 'Time.from_msg(msg.header.stamp)' in node
    assert 'header = Header(stamp=msg.header.stamp' in node
    assert 'self._obstacle_pub.publish' in node
    assert 'self._raw_pub.publish' in node
    assert 'self.p[\'extrinsics_validated\']' in node
    assert 'self.p[\'ground_reference_validated\']' in node


def test_image_contains_no_hardware_bridge():
    recipe = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert 'COPY src/leroy_bridge' not in recipe
    assert 'COPY src/leroy_bringup' not in recipe
    assert 'COPY src/leroy_depthart' not in recipe
    assert 'COPY src/leroy_nav2_perception' in recipe
    assert 'COPY Dockerfile.nav2 /leroy_ws/Dockerfile.nav2' in recipe
    assert recipe.count('CMD ["/bin/bash"]') == 1


def test_nav2_dockerfile_run_command_is_valid_shell():
    import subprocess

    recipe = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert recipe.count('RUN . /opt/ros/jazzy/setup.sh ') == 1
    run_lines = recipe.split('RUN . /opt/ros/jazzy/setup.sh ', 1)[1]
    command = '. /opt/ros/jazzy/setup.sh ' + run_lines.split('CMD ["/bin/bash"]', 1)[0]
    command = command.replace(chr(92) + chr(10), ' ')
    subprocess.run(['/bin/sh', '-n', '-c', command], check=True)
    assert "nav2_costmap_2d nav2_costmap_2d" in command
    assert "leroy_nav2_perception depth_obstacles" in command
