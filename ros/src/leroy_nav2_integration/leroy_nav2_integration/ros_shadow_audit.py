"""M8 ROS-connected SHADOW OBSERVER, output is only diagnostic text.

Reads M6's fake odom, fake obstacle cloud, embedded costmap and an
INDEPENDENT fictional Twist stimulus. Does not subscribe to MPPI output,
publish any ROS topic, send a FollowPath goal or control a motor.
"""

import math
import os
import time

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2

from .shadow_policy import DiagnosticProposal, ShadowSnapshot, ShadowWindow
from .synthetic_shadow_candidate import SYNTHETIC_CANDIDATE


ODOM = '/leroy_nav2/diagnostics/synthetic_odom'
CLOUD = '/leroy_nav2/diagnostics/obstacle_points'
GRID = '/local_costmap/costmap'
FAULTS = {'depth': 'WOULD_VETO_BLOCKED_DEPTH',
          'odom': 'WOULD_VETO_BLOCKED_ODOM'}


def _stamp(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def require_offline(environment):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(k) != v for k, v in required.items()):
        raise RuntimeError('M8 ROS shadow audit requires isolated offline Podman')


class RosShadowAudit(Node):
    def __init__(self):
        require_offline(os.environ)
        super().__init__('m8_ros_shadow_audit')
        self.declare_parameter('fault_mode', 'depth')
        self.mode = self.get_parameter('fault_mode').value
        if self.mode not in FAULTS:
            raise ValueError('M8 fault_mode must be depth or odom')
        self.window = ShadowWindow()
        self.odom_recv = None
        self.odom_stamp = None
        self.depth_recv = None
        self.depth_stamp = None
        self.map_recv = None
        self.map_valid = False
        self.proposal = None
        self.last_odom_stamp = None
        self.last_depth_stamp = None
        self.failure = None

        self.create_subscription(
            Odometry, ODOM, self._odom, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, CLOUD, self._depth, qos_profile_sensor_data)
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, GRID, self._map, grid_qos)
        self.create_subscription(
            Twist, SYNTHETIC_CANDIDATE, self._candidate, 10)
        self.get_logger().warning(
            'M8 READ-ONLY SHADOW AUDIT using FAKE Twist; not MPPI output '
            'or a motor interlock; fault mode=' + self.mode)

    def _odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.failure = 'invalid synthetic odometry frames'
            return
        stamp = _stamp(msg.header.stamp)
        position = msg.pose.pose.position
        twist = msg.twist.twist
        if (not math.isfinite(stamp) or stamp <= 0 or
                (self.last_odom_stamp is not None
                 and stamp <= self.last_odom_stamp) or
                not all(math.isfinite(v) for v in (
                    position.x, position.y, twist.linear.x,
                    twist.linear.y, twist.angular.z))):
            self.failure = 'nonfinite/repeated synthetic odometry'
            return
        self.last_odom_stamp = stamp
        self.odom_stamp = stamp
        self.odom_recv = time.monotonic()

    def _depth(self, msg):
        if msg.header.frame_id != 'base_link' or msg.width <= 0:
            self.failure = 'invalid or empty synthetic obstacle cloud'
            return
        stamp = _stamp(msg.header.stamp)
        if (not math.isfinite(stamp) or stamp <= 0 or
                (self.last_depth_stamp is not None
                 and stamp <= self.last_depth_stamp)):
            self.failure = 'nonfinite/repeated synthetic depth stamp'
            return
        self.last_depth_stamp = stamp
        self.depth_stamp = stamp
        self.depth_recv = time.monotonic()

    def _map(self, msg):
        if msg.header.frame_id != 'odom':
            self.failure = 'wrong costmap frame'
            return
        info = msg.info
        if (info.width <= 0 or info.height <= 0 or
                len(msg.data) != info.width * info.height):
            self.failure = 'invalid costmap dimensions or data'
            return
        # A new costmap message does NOT count as fresh sensor data.
        self.map_valid = 100 in msg.data and -1 in msg.data
        self.map_recv = time.monotonic()

    def _candidate(self, msg):
        self.proposal = DiagnosticProposal(
            vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
            wx=msg.angular.x, wy=msg.angular.y, wz=msg.angular.z,
            received_mono=time.monotonic(),
        )

    def snapshot(self):
        return ShadowSnapshot(
            now_mono=time.monotonic(),
            now_ros=self.get_clock().now().nanoseconds * 1e-9,
            odom_recv=self.odom_recv,
            odom_stamp=self.odom_stamp,
            depth_recv=self.depth_recv,
            depth_stamp=self.depth_stamp,
            map_recv=self.map_recv,
            map_valid=self.map_valid,
            proposal=self.proposal,
        )

    def check_graph(self):
        topics = dict(self.get_topic_names_and_types())
        if '/cmd_vel' in topics or '/odom' in topics:
            raise RuntimeError('FAIL: hardware-compatible topic detected')
        # The M6 controller may advertise its remapped diagnostic topic,
        # but this M8 observer never subscribes to that MPPI output.
        for name in self.get_node_names():
            if any(blocked in name for blocked in (
                    'leroy_bridge', 'planner_server', 'bt_navigator',
                    'velocity_smoother', 'robot_state_publisher')):
                raise RuntimeError('FAIL: forbidden hardware/navigation node')
        endpoints = self.get_publishers_info_by_topic(SYNTHETIC_CANDIDATE)
        if endpoints and (
                len(endpoints) != 1 or
                endpoints[0].node_name != 'm8_synthetic_candidate'):
            raise RuntimeError('FAIL: untrusted fictional proposal publisher')
        if self.failure:
            raise RuntimeError('FAIL: ' + self.failure)


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = RosShadowAudit()
        baseline = False
        blocked = False
        rearmed = False
        blocked_start = None
        last_label = None
        until = time.monotonic() + 85.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.12)
            node.check_graph()
            label = node.window.observe(node.snapshot())
            if label != last_label:
                print('M8 SHADOW: ' + label, flush=True)
                last_label = label

            if not baseline:
                if label == 'HEALTHY_DIAGNOSTIC_ONLY':
                    baseline = True
                continue

            if not blocked:
                if label == FAULTS[node.mode]:
                    if blocked_start is None:
                        blocked_start = time.monotonic()
                    if time.monotonic() - blocked_start >= 0.5:
                        blocked = True
                        print('M8 SHADOW: expected fake source loss vetoed',
                              flush=True)
                else:
                    blocked_start = None
                continue

            if label == 'WOULD_VETO_REARM':
                rearmed = True
            if label == 'HEALTHY_DIAGNOSTIC_ONLY' and rearmed:
                # Check the producer still belongs only to this fixture.
                endpoints = node.get_publishers_info_by_topic(SYNTHETIC_CANDIDATE)
                if (len(endpoints) != 1 or
                        endpoints[0].node_name != 'm8_synthetic_candidate'):
                    raise RuntimeError('FAIL: fake candidate publisher missing')
                print(
                    'PASS: M8 ROS shadow ' + node.mode +
                    ' observed healthy fake data, vetoed stale input, '
                    'required 1s rearm and recovered; independent '
                    'FICTIONAL Twist only, no MPPI action, /cmd_vel, '
                    'or motor interlock',
                    flush=True,
                )
                return
        raise RuntimeError(
            'FAIL: M8 ROS shadow timed out before positive baseline, '
            'sustained fault veto and recovery: '
            f'mode={node.mode if node else "unknown"} '
            f'baseline={baseline} blocked={blocked} '
            f'rearmed={rearmed} last={last_label}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
