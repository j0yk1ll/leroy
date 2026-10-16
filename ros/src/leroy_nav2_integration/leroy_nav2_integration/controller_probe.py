"""M5 one-shot FollowPath smoke test, isolated synthetic diagnostics ONLY.

The action goal is sent exclusively to an offline Nav2 controller whose
cmd_vel topic is hard-remapped to a diagnostic topic. No motor commands,
closed-loop driving, live odometry, or hardware transport.
"""

import math
import os
import time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import FollowPath
from nav_msgs.msg import Odometry, OccupancyGrid, Path
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)


PROPOSED_CMD = '/leroy_nav2/diagnostics/proposed_cmd_vel'
ODOM = '/leroy_nav2/diagnostics/synthetic_odom'
MAP = '/local_costmap/costmap'


def _safe_twist(twist):
    """Do not accept invalid/nonplanar or out-of-config diagnostic outputs."""
    linear = twist.linear
    angular = twist.angular
    values = (
        linear.x, linear.y, linear.z,
        angular.x, angular.y, angular.z)
    return (
        all(math.isfinite(v) for v in values)
        and abs(linear.x) <= 0.151
        and abs(linear.y) <= 0.151
        and abs(angular.z) <= 0.301
        and all(abs(v) <= 1e-6 for v in (
            linear.z, angular.x, angular.y))
    )


def _make_synthetic_path(pose, stamp):
    """A short lateral fake plan relative to the most recent fake odometry."""
    yaw = 2.0 * math.atan2(
        pose.pose.pose.orientation.z, pose.pose.pose.orientation.w)
    path = Path()
    path.header.frame_id = 'odom'
    path.header.stamp = stamp
    for i in range(7):
        distance = 0.07 * i
        waypoint = PoseStamped()
        waypoint.header = path.header
        waypoint.pose.position.x = (
            pose.pose.pose.position.x - math.sin(yaw) * distance)
        waypoint.pose.position.y = (
            pose.pose.pose.position.y + math.cos(yaw) * distance)
        waypoint.pose.orientation = pose.pose.pose.orientation
        path.poses.append(waypoint)
    return path


class ControllerProbe(Node):
    def __init__(self):
        super().__init__('m5_controller_probe')
        self.latest_odom = None
        self.map_ready = False
        self.diag_samples = 0
        self.nonzero_samples = 0
        self.failure = None
        self.controller = ActionClient(self, FollowPath, '/follow_path')
        self.create_subscription(
            Odometry, ODOM, self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            Twist, PROPOSED_CMD, self._on_cmd, 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, MAP, self._on_map, map_qos)

    def _on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            self.failure = 'FAIL: unexpected odometry frames'
        else:
            self.latest_odom = msg

    def _on_map(self, msg):
        if msg.header.frame_id != 'odom':
            self.failure = 'FAIL: controller costmap not in odom frame'
        elif (msg.info.width and msg.info.height
              and len(msg.data) == msg.info.width * msg.info.height
              and 100 in msg.data and -1 in msg.data):
            self.map_ready = True

    def _on_cmd(self, msg):
        if not _safe_twist(msg):
            self.failure = 'FAIL: invalid/nonplanar/out-of-bounds MPPI proposal'
            return
        self.diag_samples += 1
        if any(abs(v) > 1e-4 for v in (
                msg.linear.x, msg.linear.y, msg.angular.z)):
            self.nonzero_samples += 1

    def check_graph(self):
        topics = dict(self.get_topic_names_and_types())
        if '/cmd_vel' in topics or '/odom' in topics:
            raise RuntimeError('FAIL: motor command or live odom topic detected')
        for name in self.get_node_names():
            if any(blocked in name for blocked in (
                    'leroy_bridge', 'bt_navigator',
                    'planner_server', 'velocity_smoother')):
                raise RuntimeError('FAIL: forbidden navigation/hardware node')
        if PROPOSED_CMD in topics and topics[PROPOSED_CMD] != ['geometry_msgs/msg/Twist']:
            raise RuntimeError('FAIL: unexpected diagnostic output message type')
        if self.failure:
            raise RuntimeError(self.failure)


def _spin_until(node, condition, duration_s):
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.15)
        node.check_graph()
        if condition():
            return True
    return False


def main(args=None):
    guard = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(os.environ.get(k) != v for k, v in guard.items()):
        raise RuntimeError('M5 action probe requires isolated synthetic ROS graph')

    rclpy.init(args=args)
    node = ControllerProbe()
    goal_handle = None
    try:
        if not _spin_until(
                node, lambda: node.latest_odom is not None and node.map_ready,
                25.0):
            raise RuntimeError('FAIL: no synthetic odom or marked+unknown controller costmap')
        if not node.controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FAIL: FollowPath action server unavailable')

        goal = FollowPath.Goal()
        goal.path = _make_synthetic_path(
            node.latest_odom, node.get_clock().now().to_msg())
        goal.controller_id = 'FollowPath'
        if hasattr(goal, 'goal_checker_id'):
            goal.goal_checker_id = 'goal_checker'
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = 'progress_checker'

        sent = node.controller.send_goal_async(goal)
        if not _spin_until(node, sent.done, 8.0):
            raise RuntimeError('FAIL: FollowPath goal request timed out')
        goal_handle = sent.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FAIL: synthetic FollowPath action rejected')

        # The fake odometry continues independently of diagnostics. This
        # validates MPPI computations only, NOT convergence to a path goal.
        if not _spin_until(
                node, lambda: node.nonzero_samples >= 1, 9.0):
            raise RuntimeError(
                'FAIL: accepted synthetic path but no nonzero bounded diagnostic '
                f'proposal (samples={node.diag_samples}). Inspect costmap/MPPI logs.')

    finally:
        cancel_error = None
        if goal_handle is not None:
            try:
                # An accepted goal MUST receive a cancel acknowledgement
                # (or be confirmed terminal) before the probe can PASS.
                cancel = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=4.0)
                if not cancel.done() or cancel.result() is None:
                    cancel_error = 'FAIL: FollowPath cancellation was not acknowledged'
                elif not cancel.result().goals_canceling:
                    # It may have already aborted or reached its goal.
                    result = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(node, result, timeout_sec=3.0)
                    if not result.done():
                        cancel_error = 'FAIL: goal neither canceled nor terminal'
            except Exception as ex:
                cancel_error = f'FAIL: unable to confirm cancellation: {ex}'
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if cancel_error:
            raise RuntimeError(cancel_error)
    print(
        'PASS: offline MPPI accepted synthetic FollowPath, produced '
        'bounded diagnostic-only velocity, controller costmap marked '
        'lethal+unknown, no /cmd_vel or live /odom; NO path completion claim')


if __name__ == '__main__':
    main()
