"""M16 sensor-only ROS launch and capture image: no motor/bridge authority.

Static offline contract checks, executable in optional Nav2 image without
building or starting any camera/DepthART hardware runtime.
"""
import ast
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = ROOT.parents[1]


def test_sensor_launch_starts_only_three_expected_sensor_processes():
    s = (WORKSPACE / 'm16_sensor_only.launch.py').read_text()
    tree = ast.parse(s)
    body = next(n for n in ast.walk(tree)
                if isinstance(n, ast.FunctionDef)
                and n.name == '_only_sensor_sources')
    processes = [
        n for n in ast.walk(body)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id in ('Node', 'ComposableNodeContainer')
    ]
    assert len(processes) == 3
    assert s.count("package='usb_cam'") == 1
    assert s.count("package='leroy_depthart'") == 1
    assert s.count("package='rclcpp_components'") == 1
    assert "plugin='image_proc::RectifyNode'" in s
    assert "namespace='/camera'" in s
    assert "('image', 'image_raw')" in s
    assert "('camera_info', 'camera_info')" in s
    assert "namespace='camera'" in s
    assert "device': 'cpu'" in s


def test_sensor_launch_only_offline_no_bridge_or_loc_state_override():
    s = (WORKSPACE / 'm16_sensor_only.launch.py').read_text()
    tree = ast.parse(s)
    assert "'LEROY_M16_SENSOR_ONLY': '1'" in s
    assert "'LEROY_NAV2_DRY_RUN': '1'" in s
    assert "'ROS_DOMAIN_ID': '168'" in s
    assert "'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST'" in s
    assert "'/config/camera/front_camera.yaml'" in s
    assert "'/dev/video2'" in s
    forbidden = (
        'leroy_bridge', 'leroy_xrslam', 'controller_server',
        'planner_server', 'bt_navigator', 'cmd_vel',
        'pose_convention_validated', 'publish_obstacles',
        'synthetic_odom', 'm15_replay_source',
        'IncludeLaunchDescription', 'robot_state_publisher',
    )
    for text in forbidden:
        assert text not in s
    assert not any(
        isinstance(n, ast.Call)
        and isinstance(n.func, ast.Name)
        and n.func.id == 'ExecuteProcess'
        for n in ast.walk(tree))


def test_runner_executes_one_passive_observer_and_shutdown():
    sh = (WORKSPACE / 'm16_sensor_only_run.sh').read_text()
    assert 'set -Eeuo pipefail' in sh
    assert 'ros2 launch /opt/m16/launch/m16_sensor_only.launch.py' in sh
    assert 'python3 -m leroy_nav2_integration.m16_passive_observer' in sh
    assert sh.count('python3 -m leroy_nav2_integration.m16_passive_observer') == 1
    assert 'trap cleanup EXIT' in sh
    assert 'kill -INT "$launch_pid"' in sh
    assert 'M16_PASSIVE_REPORT_JSON=' in sh
    assert 'M16_CAMERA_DEPTH_VERDICT=' in sh
    assert 'OBSERVED_CAMERA_DEPTH_MESSAGE_CONTRACT' in sh
    assert "'depthart'" in sh
    assert "'/cmd_vel|/odom'" not in sh  # grep uses exact anchored regex
    assert "'^(/cmd_vel|/odom)$'" in sh
    assert 'podman' not in sh
    assert 'ros2 lifecycle ' not in sh
    assert 'ros2 action ' not in sh


def test_capture_image_inherits_existing_camera_runtime_not_robot_cmd():
    df = (WORKSPACE / 'Dockerfile.m16-sensors').read_text()
    assert 'localhost/leroy-ros:jazzy-lowpwm' in df
    assert 'FROM ${LEROY_SENSOR_BASE_IMAGE}' in df
    assert 'CMD ["/usr/local/bin/leroy-m16-sensor-run"]' in df
    assert 'LEROY_M16_SENSOR_ONLY=1' in df
    assert 'ROS_DOMAIN_ID=168' in df
    assert 'ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST' in df
    assert 'COPY src/leroy_nav2_integration/leroy_nav2_integration/m16_passive_observer.py' in df
    assert 'COPY m16_sensor_only.launch.py /opt/m16/launch/m16_sensor_only.launch.py' in df
    assert 'COPY src/leroy_bridge' not in df
    assert 'COPY src/leroy_xrslam' not in df
    assert 'COPY src/leroy_bringup' not in df
    assert 'CMD ["/usr/local/bin/leroy-entrypoint"]' not in df
    assert "leroy_bridge.launch.py" not in df
    assert "robot.launch.py" not in df


def test_optional_image_runs_static_sensor_only_contract_regression():
    source = (WORKSPACE / 'Dockerfile.nav2').read_text()
    assert 'COPY src/leroy_bringup' not in source
    for name in (
        'Dockerfile.m16-sensors', 'm16_sensor_only_run.sh',
        'm16_sensor_only.launch.py',
    ):
        assert name in source
