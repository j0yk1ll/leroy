"""M15 XRSLAM/DepthART message-contract and diagnostic gate tests."""

import ast
import math
import struct
from pathlib import Path
from types import SimpleNamespace as NS

import pytest

from leroy_nav2_integration.m15_sensor_contract import (
    SensorIntake, validate_odom, validate_base_cloud, validate_tf,
    stamp_seconds,
)


ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / 'leroy_nav2_integration'


def stamp(t=10.0):
    return NS(sec=int(t), nanosec=round((t-int(t))*1e9))


def quat(w=1.0):
    return NS(x=0.0, y=0.0, z=0.0, w=w)


def odom(t=10.0):
    pos = NS(x=0.2, y=0.0, z=0.0)
    twist = NS(
        linear=NS(x=0., y=0., z=0.),
        angular=NS(x=0., y=0., z=0.))
    return NS(
        header=NS(stamp=stamp(t), frame_id='odom'),
        child_frame_id='base_link',
        pose=NS(pose=NS(position=pos, orientation=quat()),
                covariance=[0.0]*36),
        twist=NS(twist=twist, covariance=[0.0]*36))


def cloud(t=10.0):
    fields = [
        NS(name=name, offset=off, datatype=7, count=1)
        for name, off in (('x', 0), ('y', 4), ('z', 8))]
    data = struct.pack('<fff', 1., .2, .4)
    return NS(
        header=NS(stamp=stamp(t), frame_id='base_link'),
        fields=fields, is_bigendian=False,
        height=1, width=1, row_step=12, point_step=12,
        data=data)


def tf(t=10.0):
    return NS(
        header=NS(stamp=stamp(t), frame_id='odom'),
        child_frame_id='base_link',
        transform=NS(
            translation=NS(x=.2, y=0., z=0.),
            rotation=quat()))


def intake_observe(thing, t=10.):
    intake = SensorIntake()
    intake.tracking('tracking')
    intake.localization(True)
    assert intake.odom(odom(t), now_mono=t, now_ros=t)
    assert intake.cloud(cloud(t), now_mono=t, now_ros=t)
    assert intake.tf(tf(t), now_mono=t, now_ros=t)
    assert intake.healthy(t+.1, t+.1)
    return intake


def test_valid_real_message_shapes_can_be_accepted():
    assert validate_odom(
        odom(), now_ros=10., previous_stamp=None) == 10.
    assert validate_base_cloud(
        cloud(), now_ros=10., previous_stamp=None) == 10.
    assert validate_tf(
        tf(), now_ros=10., previous_stamp=None) == 10.
    intake_observe(None)


@pytest.mark.parametrize('bad', [
    NS(sec=10, nanosec=-1), NS(sec=10, nanosec=1_000_000_000),
    NS(sec=0, nanosec=0), NS(sec=10.0, nanosec=0),
])
def test_invalid_ros_timestamps_are_rejected(bad):
    assert stamp_seconds(bad) is None


def test_odom_rejects_bad_frame_signals_and_covariance():
    x = odom()
    x.child_frame_id = 'wrong'
    assert validate_odom(x, now_ros=10., previous_stamp=None) is None
    x = odom()
    x.pose.pose.orientation.w = .2
    assert validate_odom(x, now_ros=10., previous_stamp=None) is None
    x = odom()
    x.twist.covariance[0] = -1.
    assert validate_odom(x, now_ros=10., previous_stamp=None) is None
    x = odom()
    x.pose.pose.position.x = float('nan')
    assert validate_odom(x, now_ros=10., previous_stamp=None) is None


def test_odom_rejects_reordered_stale_and_future_stamps():
    assert validate_odom(
        odom(), now_ros=10., previous_stamp=10.) is None
    assert validate_odom(
        odom(), now_ros=12., previous_stamp=None) is None
    assert validate_odom(
        odom(), now_ros=9.5, previous_stamp=None) is None


def test_depth_cloud_rejects_wrong_layout_nonfinite_points():
    x = cloud()
    x.header.frame_id = 'camera_optical_frame'
    assert validate_base_cloud(x, now_ros=10., previous_stamp=None) is None
    x = cloud()
    x.data = struct.pack('<fff', float('inf'), .2, .4)
    assert validate_base_cloud(x, now_ros=10., previous_stamp=None) is None
    x = cloud()
    x.point_step = 16
    assert validate_base_cloud(x, now_ros=10., previous_stamp=None) is None
    x = cloud()
    x.data = b''
    assert validate_base_cloud(x, now_ros=10., previous_stamp=None) is None


def test_tf_requires_correct_direction_and_normalized_quaternion():
    x = tf()
    x.header.frame_id = 'base_link'
    x.child_frame_id = 'odom'
    assert validate_tf(x, now_ros=10., previous_stamp=None) is None
    x = tf()
    x.transform.rotation.w = 3.0
    assert validate_tf(x, now_ros=10., previous_stamp=None) is None
    assert validate_tf(
        tf(), now_ros=10., previous_stamp=10.) is None


def test_full_intake_false_tracking_latches_until_new_valid_status():
    obj = intake_observe(None)
    obj.tracking('tracking_lost')
    assert not obj.healthy(10.2, 10.2)
    obj.tracking('tracking')
    assert not obj.healthy(10.2, 10.2)
    obj.localization(True)
    assert obj.healthy(10.2, 10.2)


def test_stale_odom_depth_and_tf_each_block_intake():
    for kind in ('odom', 'cloud', 'tf'):
        obj = intake_observe(None)
        setattr(obj, kind + '_recv', None)
        assert not obj.healthy(10.1, 10.1)
    obj = intake_observe(None)
    assert not obj.healthy(11.6, 11.6)


def test_invalid_new_odom_clears_last_healthy_observation():
    obj = intake_observe(None)
    bad = odom(10.3)
    bad.pose.pose.position.x = math.nan
    assert not obj.odom(bad, now_mono=10.3, now_ros=10.3)
    assert obj.odom_recv is None
    assert not obj.healthy(10.3, 10.3)
    assert obj.odom(odom(10.4), now_mono=10.4, now_ros=10.4)
    assert obj.tf(tf(10.4), now_mono=10.4, now_ros=10.4)
    assert obj.healthy(10.4, 10.4)


def test_valid_odom_and_tf_must_be_aligned_in_timestamp():
    obj = intake_observe(None)
    assert obj.tf(tf(10.5), now_mono=10.5, now_ros=10.5)
    assert not obj.healthy(10.5, 10.5)


def test_real_topic_mapping_and_diagnostic_outputs_only():
    s = (PKG / 'm15_real_input_gate.py').read_text()
    tree = ast.parse(s)
    assert "return {" in s
    for line in (
        "'odom': '/odom'",
        "'tracking': '/xrslam/tracking_status'",
        "'valid': '/xrslam/localization_valid'",
        "'cloud': '/leroy_nav2/diagnostics/obstacle_points'",
        "'tf': '/tf'",
        "'proposal': MPPI_TOPIC",
        "OUTPUT = '/leroy_nav2/diagnostics/m15_gated_cmd_vel'",
        "'m15_real_input_gate'",
        "self.intake.odom(msg",
        "self.intake.cloud(msg",
        "self.intake.tf(transform",
        "self.intake.tracking(msg.data)",
        "self.intake.localization(msg.data)",
        "self.gate.evaluate(",
        "M15 refuses topic remapping",
    ):
        assert line in s
    publishers = [
        call for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Attribute)
        and call.func.attr == 'create_publisher']
    assert len(publishers) == 2
    assert {ast.unparse(call.args[1]) for call in publishers} == {
        'OUTPUT', 'STATUS'}
    assert 'leroy_bridge' not in s.split('from .')[0]


def test_replay_uses_only_diagnostic_input_topics_and_trigger():
    replay = (PKG / 'm15_replay_source.py').read_text()
    launch = (ROOT / 'launch/m15_real_input_replay.launch.py').read_text()
    probe = (PKG / 'm15_input_probe.py').read_text()
    for s in (replay, launch, probe):
        ast.parse(s)
    assert "'offline_replay'" in launch
    assert launch.count('Node(') == 2
    assert "fault_mode" in launch
    assert "'ROS_DOMAIN_ID': '167'" in launch
    assert 'self.create_service(' in replay
    assert 'self.proposal_pub.publish(proposal)' in replay
    assert "'/cmd_vel'" not in replay
    assert "m15_replay/odom" not in replay  # topic is centralized
    assert "'/cmd_vel' in topics or '/odom' in topics" in probe
    assert "any(inner)" in probe
    assert "with_input < 3" in probe
    assert "recovered < 3" in probe


def test_source_contract_requires_exact_single_named_publisher():
    from leroy_nav2_integration.m15_real_input_gate import (
        source_matches, topics_for, PROFILE_REAL, PROFILE_REPLAY,
    )
    assert topics_for(PROFILE_REAL)['odom'] == '/odom'
    assert (topics_for(PROFILE_REPLAY)['odom']
            == '/leroy_nav2/diagnostics/m15_replay/odom')
    good = NS(node_name='leroy_xrslam', node_namespace='/',
              topic_type='nav_msgs/msg/Odometry')
    assert source_matches(
        [good], name='leroy_xrslam', msg_type='nav_msgs/msg/Odometry')
    assert not source_matches(
        [good, good], name='leroy_xrslam',
        msg_type='nav_msgs/msg/Odometry')
    for node, namespace, msg_type in (
        ('untrusted_node', '/', 'nav_msgs/msg/Odometry'),
        ('leroy_xrslam', '/wrong', 'nav_msgs/msg/Odometry'),
        ('leroy_xrslam', '/', 'std_msgs/msg/String'),
        ('_NODE_NAME_UNKNOWN_', '/', 'nav_msgs/msg/Odometry'),
    ):
        assert not source_matches(
            [NS(node_name=node, node_namespace=namespace,
                topic_type=msg_type)],
            name='leroy_xrslam', msg_type='nav_msgs/msg/Odometry')
    assert not source_matches(
        [], name='leroy_xrslam', msg_type='nav_msgs/msg/Odometry')


def test_invalid_depth_clears_good_stream_before_receipt_age_limit():
    obj = intake_observe(None)
    bad = cloud(10.3)
    bad.data = struct.pack('<fff', float('nan'), .2, .4)
    assert not obj.cloud(bad, now_mono=10.3, now_ros=10.3)
    assert obj.cloud_recv is None
    assert not obj.healthy(10.3, 10.3)


def test_m15_build_registers_all_console_tools():
    setup = (ROOT / 'setup.py').read_text()
    docker = (ROOT.parents[1] / 'Dockerfile.nav2').read_text()
    for name in (
        'm15_real_input_gate', 'm15_replay_source',
        'm15_input_probe',
    ):
        assert (f"'{name} = leroy_nav2_integration.{name}:main'"
                in setup)
        assert f"'leroy_nav2_integration {name}'" in docker
    assert 'm15_real_input_replay.launch.py' in docker
    assert 'COPY src/leroy_bridge' not in docker
    assert 'COPY src/leroy_xrslam' not in docker
