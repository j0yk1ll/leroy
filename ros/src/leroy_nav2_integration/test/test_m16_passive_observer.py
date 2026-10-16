"""M16 passive real-sensor observer: bounded metadata and no-control contract."""

import ast
import math
import struct
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from leroy_nav2_integration.m16_passive_contract import (
    INPUTS, StreamStats, check_camera_info, check_image,
    check_imu, depth_sample,
)
from leroy_nav2_integration.m16_passive_observer import (
    requested_duration, endpoint_evidence,
)


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'leroy_nav2_integration'


def image(*, depth=True):
    enc = '32FC1' if depth else 'rgb8'
    values = struct.pack('<ffff', 1.0, 2.0, math.nan, -1.0)
    return NS(
        header=NS(stamp=NS(sec=10, nanosec=0),
                  frame_id='camera_optical_frame'),
        width=2, height=2, encoding=enc, is_bigendian=False,
        step=8 if depth else 6,
        data=values if depth else b'\0'*12)


def test_image_shape_and_sample_bounds():
    msg = image()
    assert check_image(msg, depth=True) is None
    assert depth_sample(msg) == (2, 4)
    msg.step = 0
    assert check_image(msg, depth=True) == 'bad_image_stride_or_buffer'
    assert depth_sample(msg) is None
    assert check_image(image(depth=False)) is None


def test_wrong_depth_encoding_is_not_mistaken_for_meters():
    msg = image()
    msg.encoding = 'mono8'
    assert check_image(msg, depth=True) == 'not_32FC1_depth'


def test_camera_info_rejects_invalid_calibration_shape():
    msg = NS(
        header=NS(frame_id='camera_optical_frame'),
        width=640, height=480,
        p=[0.0]*12)
    assert check_camera_info(msg) == 'invalid_camera_calibration_shape'
    msg.p[0] = 480.0
    msg.p[5] = 479.0
    assert check_camera_info(msg) is None
    msg.p[2] = math.nan
    assert check_camera_info(msg) == 'invalid_camera_calibration_shape'


def test_imu_is_read_only_and_requires_finite_measurement():
    msg = NS(
        header=NS(frame_id='imu_link'),
        angular_velocity=NS(x=0., y=.01, z=.02),
        linear_acceleration=NS(x=0., y=0., z=9.8))
    assert check_imu(msg) is None
    msg.angular_velocity.z = math.inf
    assert check_imu(msg) == 'nonfinite_imu_sample'


def test_stream_staleness_and_malformed_latest_invalidates():
    s = StreamStats()
    assert s.observe(
        stamp=9.9, frame='base_link', mono=100., ros_now=10.)
    assert s.export(100.2)['fresh_valid_receipt']
    assert not s.observe(
        stamp=9.9, frame='base_link', mono=100.3, ros_now=10.3)
    assert s.export(100.3)['invalid'] == 1
    assert not s.export(100.3)['fresh_valid_receipt']
    assert s.observe(
        stamp=10.4, frame='base_link', mono=100.4, ros_now=10.4)
    assert s.export(100.5)['fresh_valid_receipt']
    assert not s.export(102.5)['fresh_valid_receipt']


def test_future_and_reordered_samples_never_accepted():
    s = StreamStats()
    assert not s.observe(
        stamp=11., frame='base_link', mono=10., ros_now=10.)
    assert s.accepted == 0
    assert s.observe(
        stamp=10., frame='base_link', mono=10.1, ros_now=10.)
    assert not s.observe(
        stamp=9.9, frame='base_link', mono=10.2, ros_now=10.2)
    assert s.invalid == 2


@pytest.mark.parametrize('text', ['NaN', 'inf', '-1', '0', '4', '181', 'oops'])
def test_invalid_bounded_duration_rejected(text):
    with pytest.raises(ValueError):
        requested_duration({'LEROY_M16_DURATION_S': text})


def test_valid_duration_and_default():
    assert requested_duration({}) == 30.0
    assert requested_duration({'LEROY_M16_DURATION_S': '45'}) == 45.0


def test_graph_evidence_records_raw_provenance_not_auth():
    items = endpoint_evidence([
        NS(node_name='leroy_bridge', node_namespace='/',
           topic_type='sensor_msgs/msg/Imu'),
        NS(node_name='unexpected', node_namespace='/other',
           topic_type='sensor_msgs/msg/Imu')])
    assert items[0]['node'] == 'leroy_bridge'
    assert items[1]['namespace'] == '/other'


def test_topic_contract_matches_checked_in_ros_drivers():
    expected = {
        'camera_image': '/camera/image_rect',
        'camera_info': '/camera/camera_info',
        'depth_image': '/depthart/depth',
        'imu': '/imu/data_raw',
        'xrslam_tracking': '/xrslam/tracking_status',
        'xrslam_valid': '/xrslam/localization_valid',
        'odom': '/odom',
        'tf': '/tf',
        'obstacle_cloud': '/leroy_nav2/diagnostics/obstacle_points',
    }
    assert {key: topic for key, (topic, _) in INPUTS.items()} == expected


def test_m16_uses_subscriptions_only_not_ros_publishers_services_actions():
    source = (PKG / 'm16_passive_observer.py').read_text()
    root = ast.parse(source)
    names = [
        n.func.attr for n in ast.walk(root)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
    ]
    assert names.count('create_subscription') == 9
    # Jazzy rclpy.Node.subscriptions is a read-only property. Static
    # guard complements the real-node constructor test in the image build.
    assert "self._m16_subscriptions = [" in source
    assert "self.subscriptions =" not in source
    assert "zip(INPUTS,self._m16_subscriptions)" in source
    for forbidden in (
        'create_publisher', 'create_service', 'create_client',
        'create_timer', 'send_goal_async', 'set_parameters',
        'publish', 'call_async', 'sendTransform',
    ):
        assert forbidden not in names
    assert "enable_rosout=False" in source
    assert "start_parameter_services=False" in source
    assert 'M16_PASSIVE_REPORT_JSON=' in source
    assert "M16_VERDICT=" in source
    assert "M16 refuses ROS subscription remapping" in source
    assert "INCOMPLETE_OR_UNVERIFIED_REAL_STREAMS" in source
    assert "OBSERVED_MESSAGE_CONTRACTS_ONLY" in source
    assert "camera_info_recent = deque(maxlen=512)" in source
    assert "LEROY_NAV2_DRY_RUN" in source


def test_m16_optional_image_build_only_registers_passive_executable():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    assert ("'m16_passive_observer = "
            "leroy_nav2_integration.m16_passive_observer:main'") in setup
    assert "leroy_nav2_integration m16_passive_observer" in docker
    assert 'COPY src/leroy_bridge' not in docker
    assert 'COPY src/leroy_xrslam' not in docker
    assert 'COPY src/leroy_depthart' not in docker
