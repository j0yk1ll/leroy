"""M6 read-only synthetic fault audit: NO velocity publisher or motor interlock.

Requires READY -> expected sensor STALE -> READY while the *same*
Nav2 controller's marked/unknown costmap is present. This is diagnostic
evidence only: it does NOT prevent controller output or authorize motion.
"""

import math
import os
import time

from nav_msgs.msg import Odometry, OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import PointCloud2

from .fault_contract import diagnostic_state


ODOM = '/leroy_nav2/diagnostics/synthetic_odom'
DEPTH_POINTS = '/leroy_nav2/diagnostics/obstacle_points'
COSTMAP = '/local_costmap/costmap'
MODES = {'depth': 'BLOCKED_DEPTH', 'odom': 'BLOCKED_ODOM'}


def _stamp(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class FaultAudit(Node):
    def __init__(self):
        super().__init__('m6_fault_audit')
        self.declare_parameter('fault_mode', 'depth')
        self.mode = self.get_parameter('fault_mode').value
        if self.mode not in MODES:
            raise ValueError('M6 fault_mode must be depth or odom')
        self.odom_recv = None
        self.odom_stamp = None
        self.last_odom_stamp = None
        self.depth_recv = None
        self.depth_stamp = None
        self.last_depth_stamp = None
        self.map_recv = None
        self.map_valid = False
        self.failure = None
        self.create_subscription(
            Odometry, ODOM, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, DEPTH_POINTS, self._on_depth, qos_profile_sensor_data)
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, COSTMAP, self._on_map, grid_qos)
        self.get_logger().warning(
            'READ-ONLY M6 health AUDIT, not a safety interlock; '
            'fault mode=' + self.mode)

    def _on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.failure = 'FAIL: fake odometry frame changed'
            return
        stamp = _stamp(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            self.failure = 'FAIL: invalid odometry stamp'
            return
        if self.last_odom_stamp is not None and stamp <= self.last_odom_stamp:
            self.failure = 'FAIL: repeated/out-of-order odometry stamp'
            return
        pose = msg.pose.pose.position
        twist = msg.twist.twist
        if not all(math.isfinite(v) for v in (
                pose.x, pose.y, twist.linear.x, twist.linear.y,
                twist.angular.z)):
            self.failure = 'FAIL: invalid odometry value'
            return
        self.last_odom_stamp = stamp
        self.odom_stamp = stamp
        self.odom_recv = time.monotonic()

    def _on_depth(self, msg):
        if msg.header.frame_id != 'base_link' or msg.width <= 0:
            self.failure = 'FAIL: invalid or clearing fake obstacle cloud'
            return
        stamp = _stamp(msg.header.stamp)
        if not math.isfinite(stamp) or stamp <= 0:
            self.failure = 'FAIL: invalid obstacle acquisition stamp'
            return
        if self.last_depth_stamp is not None and stamp <= self.last_depth_stamp:
            self.failure = 'FAIL: repeated/out-of-order depth stamp'
            return
        self.last_depth_stamp = stamp
        self.depth_stamp = stamp
        self.depth_recv = time.monotonic()

    def _on_map(self, msg):
        if msg.header.frame_id != 'odom':
            self.failure = 'FAIL: costmap frame changed'
            return
        info = msg.info
        if (info.width <= 0 or info.height <= 0
                or len(msg.data) != info.width * info.height):
            self.failure = 'FAIL: malformed costmap'
            return
        self.map_valid = 100 in msg.data and -1 in msg.data
        self.map_recv = time.monotonic()

    def status(self):
        return diagnostic_state(
            now_mono=time.monotonic(),
            now_ros=self.get_clock().now().nanoseconds * 1e-9,
            odom_recv=self.odom_recv,
            odom_stamp=self.odom_stamp,
            depth_recv=self.depth_recv,
            depth_stamp=self.depth_stamp,
            map_recv=self.map_recv,
            map_valid=self.map_valid,
        )

    def check_graph(self):
        topics = {name for name, _ in self.get_topic_names_and_types()}
        if '/cmd_vel' in topics or '/odom' in topics:
            raise RuntimeError('FAIL: motor-compatible topic exists')
        for name in self.get_node_names():
            if any(bad in name for bad in (
                    'leroy_bridge', 'planner_server', 'bt_navigator',
                    'velocity_smoother', 'robot_state_publisher')):
                raise RuntimeError('FAIL: forbidden hardware/navigation node')
        if self.failure:
            raise RuntimeError(self.failure)


def main(args=None):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError('M6 diagnostic audit requires isolated Podman ROS')

    rclpy.init(args=args)
    node = None
    try:
        node = FaultAudit()
        first_ready = False
        seen_expected_block = False
        blocked_since = None
        last_status = None
        deadline = time.monotonic() + 58.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.12)
            node.check_graph()
            status = node.status()
            if status != last_status:
                print('M6 AUDIT: ' + status, flush=True)
                last_status = status

            if not first_ready:
                first_ready = status == 'READY'
                continue

            if not seen_expected_block:
                if status == MODES[node.mode]:
                    if blocked_since is None:
                        blocked_since = time.monotonic()
                    if time.monotonic() - blocked_since >= 0.5:
                        seen_expected_block = True
                        print('M6 AUDIT: expected synthetic sensor loss detected',
                              flush=True)
                else:
                    blocked_since = None
                continue

            if status == 'READY':
                print(
                    'PASS: M6 synthetic ' + node.mode +
                    ' loss was diagnosed as stale, then recovered only '
                    'on fresh sensor observations; diagnostic audit only, '
                    'no /cmd_vel or live /odom, NO motor interlock claim',
                    flush=True)
                return
        raise RuntimeError(
            'FAIL: M6 audit did not observe READY -> ' +
            MODES[node.mode] + ' -> READY within 58s; '
            f'first_ready={first_ready} blocked={seen_expected_block} '
            f'last={last_status}, costmap_ok={node.map_valid}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
