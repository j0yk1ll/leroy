"""Contract tests: no motor topic, no fake localization, Jazzy Omni config."""

import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext

ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / 'launch' / 'dry_run.launch.py'
SIM = ROOT / 'config' / 'simulation.yaml'
HARDWARE = ROOT / 'config' / 'hardware.disabled.yaml'


def load_launch():
    spec = importlib.util.spec_from_file_location('leroy_nav2_dry_run', LAUNCH)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_simulation_yaml_is_jazzy_omni_and_inert():
    data = yaml.safe_load(SIM.read_text())
    controller = data['controller_server']['ros__parameters']
    mppi = controller['FollowPath']
    costmap = data['local_costmap']['local_costmap']['ros__parameters']
    assert controller['controller_plugins'] == ['FollowPath']
    assert mppi['plugin'] == 'nav2_mppi_controller::MPPIController'
    assert mppi['motion_model'] == 'Omni'
    assert controller['min_y_velocity_threshold'] < mppi['vy_max']
    assert mppi['model_dt'] <= 1.0 / controller['controller_frequency']
    assert mppi['vy_max'] > 0
    assert costmap['global_frame'] == 'odom'
    assert costmap['robot_base_frame'] == 'base_link'
    assert costmap['track_unknown_space'] is True
    assert costmap['plugins'] == ['inflation_layer']
    assert 'obstacle_layer' not in costmap
    assert 'voxel_layer' not in costmap
    assert 'observation_sources' not in str(costmap)
    assert 'map' not in data
    assert 'amcl' not in data


def test_hardware_manifest_remains_disabled():
    manifest = yaml.safe_load(HARDWARE.read_text())['deployment']
    assert manifest['mode'] == 'hardware'
    assert manifest['enabled'] is False
    assert manifest['bridge_command_output_enabled'] is False
    assert manifest['odometry_validated'] is False
    assert manifest['operator_acceptance_completed'] is False


def test_launch_has_only_diagnostic_output():
    mod = load_launch()
    assert mod.PROPOSED_CMD_TOPIC == '/leroy_nav2/diagnostics/proposed_cmd_vel'
    content = LAUNCH.read_text()
    assert "('cmd_vel', PROPOSED_CMD_TOPIC)" in content
    assert "('/cmd_vel', PROPOSED_CMD_TOPIC)" in content
    assert 'leroy_bridge' not in content.replace('motor bridge', '')
    assert 'robot.launch.py' not in content
    assert "'autostart': False" in content
    # No launch parameter can change output topic or load a hardware profile.
    assert 'DeclareLaunchArgument(\'params_file\'' not in content
    assert 'DeclareLaunchArgument(\'cmd_vel\'' not in content
    assert mod.generate_launch_description() is not None


@pytest.mark.parametrize('bad_mode', ['passive_live', 'hardware', '', 'SIMULATION'])
def test_non_simulation_modes_rejected(bad_mode):
    mod = load_launch()
    with pytest.raises(RuntimeError, match='simulation only'):
        mod.require_dry_run(bad_mode, {})


@pytest.mark.parametrize('env', [
    {},
    {'LEROY_NAV2_DRY_RUN': '1', 'ROS_DOMAIN_ID': '0',
     'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'},
    {'LEROY_NAV2_DRY_RUN': '1', 'ROS_DOMAIN_ID': '167',
     'ROS_AUTOMATIC_DISCOVERY_RANGE': 'SUBNET'},
])
def test_missing_or_invalid_isolation_environment_rejected(env):
    with pytest.raises(RuntimeError):
        load_launch().require_dry_run('simulation', env)


def test_explicit_offline_launch_contract(monkeypatch):
    mod = load_launch()
    env = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    mod.require_dry_run('simulation', env)
    context = LaunchContext()
    context.launch_configurations['mode'] = 'simulation'
    for k, value in env.items():
        monkeypatch.setenv(k, value)
    assert len(mod._guard(context)) == 1
