"""M15 ROS gate input adapter for XRSLAM/DepthART MESSAGE CONTRACTS.

Subscribes to the actual interfaces in 'real_interfaces' profile, or
namespaced fake replay topics in 'offline_replay' profile. Software
gating selects a bounded proposal or exact zero on an UNCONSUMED
diagnostic output. No XRSLAM, camera, DepthART driver, serial/bridge
or motor access in the M15 container.

Not calibrated localization/obstacle validation or production ROS
command authority. Even 'real_interfaces' may only execute in the
disconnected fixture during M15.
"""

import json
import math
import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)

from .m9_mppi_source import MPPI_TOPIC
from .m9_mppi_shadow_probe import require_offline
from .m14_gate_core import DiagnosticCommandGate, ZERO, is_nonzero
from .fault_contract import _recent
from .m15_sensor_contract import SensorIntake
from .shadow_policy import DiagnosticProposal, ShadowSnapshot


OUTPUT = '/leroy_nav2/diagnostics/m15_gated_cmd_vel'
STATUS = '/leroy_nav2/diagnostics/m15_gate_status'
REPLAY_PREFIX = '/leroy_nav2/diagnostics/m15_replay'
PROFILE_REAL = 'real_interfaces'
PROFILE_REPLAY = 'offline_replay'


def topics_for(profile):
    if profile == PROFILE_REAL:
        return {
            'odom': '/odom',
            'cloud': '/leroy_nav2/diagnostics/obstacle_points',
            'tracking': '/xrslam/tracking_status',
            'valid': '/xrslam/localization_valid',
            'tf': '/tf',
            'map': '/local_costmap/costmap',
            'proposal': MPPI_TOPIC,
        }
    if profile == PROFILE_REPLAY:
        return {
            'odom': REPLAY_PREFIX + '/odom',
            'cloud': REPLAY_PREFIX + '/obstacle_points',
            'tracking': REPLAY_PREFIX + '/tracking_status',
            'valid': REPLAY_PREFIX + '/localization_valid',
            'tf': REPLAY_PREFIX + '/tf',
            'map': REPLAY_PREFIX + '/costmap',
            'proposal': REPLAY_PREFIX + '/proposed_cmd_vel',
        }
    raise ValueError('M15 requires known real_interfaces/offline_replay profile')


def source_matches(endpoints, *, name, msg_type):
    """Exact graph identity only; unknown or extra publishers reject."""
    return (len(endpoints) == 1
            and getattr(endpoints[0], 'node_name', None) == name
            and getattr(endpoints[0], 'node_namespace', None) == '/'
            and getattr(endpoints[0], 'topic_type', None) == msg_type)


class M15RealInputGate(Node):
    def __init__(self):
        require_offline(os.environ)
        super().__init__('m15_real_input_gate')
        self.declare_parameter('profile', PROFILE_REAL)
        self.profile = self.get_parameter('profile').value
        self.topics = topics_for(self.profile)
        self.expected_odom = (
            'leroy_xrslam' if self.profile == PROFILE_REAL
            else 'm15_replay_source')
        self.expected_depth = (
            'depth_obstacles' if self.profile == PROFILE_REAL
            else 'm15_replay_source')
        self.expected_proposal = (
            'controller_server' if self.profile == PROFILE_REAL
            else 'm15_replay_source')
        self.intake = SensorIntake()
        self.gate = DiagnosticCommandGate()
        self.proposal = None
        self.map_recv = None
        self.map_valid = False
        self.counter = 0
        self.last_reason = None
        self.odom_sub = self.create_subscription(
            Odometry, self.topics['odom'], self._odom, qos_profile_sensor_data)
        self.depth_sub = self.create_subscription(
            PointCloud2, self.topics['cloud'], self._cloud,
            qos_profile_sensor_data)
        self.tf_sub = self.create_subscription(
            TFMessage, self.topics['tf'], self._tf,
            qos_profile_sensor_data)
        latched = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.valid_sub = self.create_subscription(
            Bool, self.topics['valid'], self._valid, latched)
        self.tracking_sub = self.create_subscription(
            String, self.topics['tracking'], self._tracking, latched)
        self.map_sub = self.create_subscription(
            OccupancyGrid, self.topics['map'], self._map,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.proposal_sub = self.create_subscription(
            Twist, self.topics['proposal'], self._proposal, 10)
        self.out = self.create_publisher(Twist, OUTPUT, 10)
        self.status = self.create_publisher(String, STATUS, 10)
        # Startup guarantee: output can never be remapped to /cmd_vel.
        for endpoint, expected in (
            (self.odom_sub, self.topics['odom']),
            (self.depth_sub, self.topics['cloud']),
            (self.tf_sub, self.topics['tf']),
            (self.valid_sub, self.topics['valid']),
            (self.tracking_sub, self.topics['tracking']),
            (self.map_sub, self.topics['map']),
            (self.proposal_sub, self.topics['proposal']),
            (self.out, OUTPUT), (self.status, STATUS),
        ):
            if endpoint.topic_name != expected:
                raise RuntimeError('M15 refuses topic remapping: ' + expected)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            'M15 subscribes to ' + self.profile +
            ' sensor interfaces and gates DIAGNOSTIC Twist only; '
            'no bridge or motor output')

    def _times(self):
        return time.monotonic(), self.get_clock().now().nanoseconds * 1e-9

    def _odom(self, msg):
        mono, ros = self._times()
        self.intake.odom(msg, now_mono=mono, now_ros=ros)

    def _cloud(self, msg):
        mono, ros = self._times()
        self.intake.cloud(msg, now_mono=mono, now_ros=ros)

    def _tf(self, msg):
        mono, ros = self._times()
        for transform in msg.transforms:
            if (transform.header.frame_id == 'odom'
                    and transform.child_frame_id == 'base_link'):
                self.intake.tf(transform, now_mono=mono, now_ros=ros)

    def _tracking(self, msg):
        self.intake.tracking(msg.data)

    def _valid(self, msg):
        self.intake.localization(msg.data)

    def _map(self, msg):
        info = msg.info
        self.map_valid = (
            msg.header.frame_id == 'odom'
            and info.width > 0 and info.height > 0
            and len(msg.data) == info.width * info.height
            and 100 in msg.data and -1 in msg.data)
        self.map_recv = time.monotonic() if self.map_valid else None

    def _proposal(self, msg):
        self.proposal = DiagnosticProposal(
            vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
            wx=msg.angular.x, wy=msg.angular.y, wz=msg.angular.z,
            received_mono=time.monotonic())

    def _graph_ok(self):
        topics = dict(self.get_topic_names_and_types())
        if '/cmd_vel' in topics or any(
                'leroy_bridge' in node for node in self.get_node_names()):
            return False
        if self.profile == PROFILE_REPLAY and '/odom' in topics:
            return False
        requirements = (
            ('odom', self.expected_odom, 'nav_msgs/msg/Odometry'),
            ('cloud', self.expected_depth, 'sensor_msgs/msg/PointCloud2'),
            ('tracking', self.expected_odom, 'std_msgs/msg/String'),
            ('valid', self.expected_odom, 'std_msgs/msg/Bool'),
            ('proposal', self.expected_proposal, 'geometry_msgs/msg/Twist'),
        )
        for topic, source, typ in requirements:
            full = self.topics[topic]
            if not source_matches(
                    self.get_publishers_info_by_topic(full),
                    name=source, msg_type=typ):
                return False
        return True

    def _tick(self):
        mono, ros = self._times()
        sources_ok = self._graph_ok()
        status_ok = (self.intake.localization_valid and
                     self.intake.tracking_state == 'tracking')
        tf_ok = (
            _recent(mono, ros, self.intake.tf_recv, self.intake.tf_stamp)
            and self.intake.odom_stamp is not None
            and self.intake.tf_stamp is not None
            and abs(self.intake.odom_stamp -
                    self.intake.tf_stamp) <= 0.25)
        snapshot = ShadowSnapshot(
            now_mono=mono, now_ros=ros,
            odom_recv=self.intake.odom_recv,
            odom_stamp=self.intake.odom_stamp,
            depth_recv=self.intake.cloud_recv,
            depth_stamp=self.intake.cloud_stamp,
            map_recv=self.map_recv,
            map_valid=self.map_valid,
            proposal=self.proposal)
        values, reason = self.gate.evaluate(
            snapshot, graph_ok=sources_ok,
            source_ok=status_ok, tf_ok=tf_ok)
        if not sources_ok:
            reason = 'BLOCKED_GRAPH'
        elif not status_ok:
            reason = 'BLOCKED_LOCALIZATION'
        if reason != self.last_reason:
            print('M15 GATE: ' + reason, flush=True)
            self.last_reason = reason
        cmd = Twist()
        cmd.linear.x, cmd.linear.y, cmd.angular.z = values
        self.out.publish(cmd)
        self.counter += 1
        incoming_nonzero = False
        if self.proposal is not None:
            p = self.proposal
            if all(math.isfinite(v) for v in (p.vx, p.vy, p.wz)):
                incoming_nonzero = is_nonzero((p.vx, p.vy, p.wz))
        state = {
            'diagnostics_only': True, 'profile': self.profile,
            'sequence': self.counter, 'monotonic': mono,
            'reason': reason,
            'input_nonzero': incoming_nonzero,
            'output_nonzero': is_nonzero(values),
            'output_xyz': list(values),
            'tracking_state': self.intake.tracking_state,
            'localization_valid': self.intake.localization_valid,
        }
        out = String()
        out.data = json.dumps(state, sort_keys=True)
        self.status.publish(out)


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = M15RealInputGate()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
