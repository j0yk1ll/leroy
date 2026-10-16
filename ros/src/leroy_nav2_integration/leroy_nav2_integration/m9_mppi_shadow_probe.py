"""M9: inspect ACTUAL MPPI diagnostic proposals using M7 shadow policy.

Starts exactly one short M5-style synthetic FollowPath action in an
isolated ROS graph with hard-remapped cmd_vel, then confirms that real
MPPI-proposed Twist messages produce bounded diagnostic-only labels.
No topic publisher, hardware, bridge, watchdog, or motor safety gate.
"""

import math
import os
import time

from nav2_msgs.action import FollowPath
from sensor_msgs.msg import PointCloud2
import rclpy
from rclpy.qos import qos_profile_sensor_data

from .controller_probe import (
    ControllerProbe, _make_synthetic_path, _spin_until,
)
from .m9_mppi_source import MPPI_TOPIC, inspect_mppi_source
from .shadow_policy import (
    DiagnosticProposal, ShadowSnapshot, ShadowWindow,
    VX_LIMIT, VY_LIMIT, WZ_LIMIT, EPS,
)


CLOUD = '/leroy_nav2/diagnostics/obstacle_points'
GRAPH_DISCOVERY_TIMEOUT_S = 12.0


def _stamp(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def require_offline(environment):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(k) != v for k, v in required.items()):
        raise RuntimeError('M9 actual MPPI probe requires isolated synthetic ROS')


class M9MppiShadowProbe(ControllerProbe):
    """M5 read-only graph/action probe extended with M6/M7 timestamp policy.

    Inheritance preserves M5's strict velocity bounds, diagnostic
    remapping checks, and synthetic-only FollowPath action behavior.
    """

    def __init__(self):
        super().__init__()
        self.odom_recv = None
        self.odom_stamp = None
        self.last_odom_stamp = None
        self.depth_recv = None
        self.depth_stamp = None
        self.last_depth_stamp = None
        self.map_recv = None
        self.map_valid = False
        self.proposal = None
        self.window = ShadowWindow()
        self.goal_accepted = False
        self.verified_source = False
        self.graph_pending_since = None
        self.last_label = None
        self.healthy_samples = 0
        self.action_nonzero_samples = 0
        self.create_subscription(
            PointCloud2, CLOUD, self._on_depth, qos_profile_sensor_data)
        self.get_logger().warning(
            'M9 observes ACTUAL MPPI /diagnostics/proposed_cmd_vel only; '
            'synthetic FollowPath, NO motor permission or safety gate')

    def _on_odom(self, msg):
        super()._on_odom(msg)
        stamp = _stamp(msg.header.stamp)
        if (not math.isfinite(stamp) or stamp <= 0
                or (self.last_odom_stamp is not None
                    and stamp <= self.last_odom_stamp)):
            self.failure = 'FAIL: invalid/repeated synthetic odometry stamp'
            return
        self.last_odom_stamp = stamp
        self.odom_stamp = stamp
        self.odom_recv = time.monotonic()

    def _on_depth(self, msg):
        if msg.header.frame_id != 'base_link' or msg.width <= 0:
            self.failure = 'FAIL: malformed synthetic obstacle cloud'
            return
        stamp = _stamp(msg.header.stamp)
        if (not math.isfinite(stamp) or stamp <= 0
                or (self.last_depth_stamp is not None
                    and stamp <= self.last_depth_stamp)):
            self.failure = 'FAIL: invalid/repeated fake obstacle stamp'
            return
        self.last_depth_stamp = stamp
        self.depth_stamp = stamp
        self.depth_recv = time.monotonic()

    def _on_map(self, msg):
        super()._on_map(msg)
        info = msg.info
        if (msg.header.frame_id != 'odom'
                or info.width <= 0 or info.height <= 0
                or len(msg.data) != info.width * info.height):
            self.failure = 'FAIL: malformed controller rolling costmap'
            return
        self.map_valid = (100 in msg.data and -1 in msg.data)
        self.map_recv = time.monotonic()

    def _on_cmd(self, msg):
        super()._on_cmd(msg)  # M5 rejects invalid, nonplanar or oversized Twist.
        # M7 limits are slightly stricter than the M5 smoke tolerance:
        # reject EVERY out-of-M7-range message before it can be masked
        # by a later good sample or a temporarily stale sensor stream.
        if (abs(msg.linear.x) > VX_LIMIT + EPS
                or abs(msg.linear.y) > VY_LIMIT + EPS
                or abs(msg.angular.z) > WZ_LIMIT + EPS
                or any(abs(v) > EPS for v in (
                    msg.linear.z, msg.angular.x, msg.angular.y))):
            self.failure = 'FAIL: actual MPPI proposal violates M7 fictional bounds'
            return
        if not self.goal_accepted or self.failure:
            return
        self.proposal = DiagnosticProposal(
            vx=msg.linear.x, vy=msg.linear.y, vz=msg.linear.z,
            wx=msg.angular.x, wy=msg.angular.y, wz=msg.angular.z,
            received_mono=time.monotonic(),
        )
        if any(abs(v) > 1e-4 for v in (
                msg.linear.x, msg.linear.y, msg.angular.z)):
            self.action_nonzero_samples += 1

    def begin_goal(self):
        # Discard pre-goal candidates: only the accepted synthetic action
        # may contribute to M9 evidence. A fresh 1s rearm is mandatory.
        self.goal_accepted = True
        self.proposal = None
        self.window = ShadowWindow()
        self.healthy_samples = 0
        self.action_nonzero_samples = 0
        self.last_label = None

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
        super().check_graph()  # Check /cmd_vel, /odom, forbidden nodes/types.
        endpoints = self.get_publishers_info_by_topic(MPPI_TOPIC)
        verified, pending, evidence = inspect_mppi_source(endpoints)
        if not verified and not pending:
            raise RuntimeError(
                'FAIL: unexpected MPPI diagnostic publisher: ' + repr(evidence))
        self.verified_source = verified
        if not self.goal_accepted:
            # Some lifecycle publishers are not discovered until the action
            # is active. No positive M9 evidence is collected pre-action.
            return
        if pending:
            now = time.monotonic()
            if self.graph_pending_since is None:
                self.graph_pending_since = now
            if now - self.graph_pending_since >= GRAPH_DISCOVERY_TIMEOUT_S:
                raise RuntimeError(
                    'FAIL: MPPI diagnostic publisher unresolved after '
                    f'{GRAPH_DISCOVERY_TIMEOUT_S}s: {evidence!r}')
            if self.last_label != 'WAITING_MPPI_GRAPH_IDENTITY':
                print('M9 MPPI SHADOW: WAITING_MPPI_GRAPH_IDENTITY', flush=True)
            self.window = ShadowWindow()
            self.healthy_samples = 0
            self.last_label = 'WAITING_MPPI_GRAPH_IDENTITY'
            return
        self.graph_pending_since = None
        label = self.window.observe(self.snapshot())
        if label != self.last_label:
            print('M9 MPPI SHADOW: ' + label, flush=True)
            self.last_label = label
        if label == 'HEALTHY_DIAGNOSTIC_ONLY':
            self.healthy_samples += 1


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    goal_handle = None
    succeeded = False
    try:
        node = M9MppiShadowProbe()
        if not _spin_until(
                node, lambda: node.latest_odom is not None and node.map_ready
                and node.depth_recv is not None, 25.0):
            raise RuntimeError(
                'FAIL: no synthetic odom, depth observations or marked+unknown '
                'controller costmap')
        if not node.controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FAIL: synthetic FollowPath server unavailable')

        goal = FollowPath.Goal()
        goal.path = _make_synthetic_path(
            node.latest_odom, node.get_clock().now().to_msg())
        goal.controller_id = 'FollowPath'
        if hasattr(goal, 'goal_checker_id'):
            goal.goal_checker_id = 'goal_checker'
        if hasattr(goal, 'progress_checker_id'):
            goal.progress_checker_id = 'progress_checker'

        future = node.controller.send_goal_async(goal)
        if not _spin_until(node, future.done, 8.0):
            raise RuntimeError('FAIL: M9 synthetic action request timed out')
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FAIL: M9 synthetic action rejected')
        node.begin_goal()

        # Time-bounded, not a path-completion test. Require at least one
        # nonzero *actual* MPPI remapped proposal and M7's healthy rearm.
        if not _spin_until(
                node,
                lambda: (node.verified_source
                         and node.action_nonzero_samples >= 1
                         and node.healthy_samples >= 1),
                16.0):
            raise RuntimeError(
                'FAIL: no source-verified, nonzero MPPI proposal plus '
                'post-rearm healthy shadow sample within 16s; '
                f'nonzero={node.action_nonzero_samples}, '
                f'healthy={node.healthy_samples}, '
                f'last={node.last_label}, '
                f'source_verified={node.verified_source}')
        succeeded = True
    finally:
        cancel_error = None
        if goal_handle is not None and node is not None:
            try:
                cancel = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(node, cancel, timeout_sec=4.0)
                if not cancel.done() or cancel.result() is None:
                    cancel_error = 'FAIL: M9 synthetic FollowPath cancellation not acknowledged'
                elif not cancel.result().goals_canceling:
                    result = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(node, result, timeout_sec=3.0)
                    if not result.done():
                        cancel_error = 'FAIL: M9 goal neither canceled nor terminal'
            except Exception as exc:
                cancel_error = f'FAIL: M9 cancellation verification failed: {exc}'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if cancel_error:
            raise RuntimeError(cancel_error)
    if succeeded:
        print(
            'PASS: M9 actual MPPI remapped diagnostic Twist observed during '
            'accepted SYNTHETIC FollowPath; finite bounded nonzero proposal, '
            'M7 shadow 1s rearm, verified controller_server graph endpoint, '
            'cancellation acknowledged or terminal; NO motor command '
            'interlock, hardware, real tracking or safe-stop claim',
            flush=True,
        )


if __name__ == '__main__':
    main()
