"""M11: receipt-timed MPPI proposals within one TRIGGERED synthetic outage.

M10's active-goal fake-depth/odom test with precise scheduled-window
accounting. This node publishes NOTHING and never gates velocity.
"""

import os
import time

import rclpy
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger

from .controller_probe import _make_synthetic_path, _spin_until
from .m9_mppi_shadow_probe import require_offline
from .m10_active_fault_probe import (
    M10ActiveFaultProbe, FAULTS, OBSERVATION_AFTER_TRIGGER_S,
)
from .m11_blackout_metrics import parse_window, summarize


MAX_SAMPLES = 512


class M11BlackoutProbe(M10ActiveFaultProbe):
    def __init__(self):
        super().__init__()
        self.receipt_samples = []
        self.first_veto_mono = None
        self.get_logger().warning(
            'M11 measures actual MPPI diagnostic Twist during one FAKE '
            'sensor blackout; no motor authority or stopping claim')

    def _on_cmd(self, msg):
        # M9/M10 enforce finite, planar, M7-bounded real MPPI proposals.
        # Callback time is a receipt time, NOT time of generation.
        stamp = time.monotonic()
        super()._on_cmd(msg)
        if self.goal_accepted and not self.failure:
            if len(self.receipt_samples) >= MAX_SAMPLES:
                self.failure = 'FAIL: M11 bounded diagnostic sample buffer exhausted'
                return
            nonzero = any(abs(v) > 1e-4 for v in (
                msg.linear.x, msg.linear.y, msg.angular.z))
            self.receipt_samples.append((stamp, nonzero))

    def check_graph(self):
        super().check_graph()
        if (self.triggered and self.verified_source
                and self.last_label == FAULTS[self.mode]
                and self.first_veto_mono is None):
            self.first_veto_mono = time.monotonic()


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    goal_handle = None
    goal_result = None
    outcome = 'not observed'
    completed = False
    try:
        node = M11BlackoutProbe()
        if not _spin_until(
                node, lambda: node.latest_odom is not None
                and node.depth_recv is not None and node.map_ready, 25.0):
            raise RuntimeError(
                'FAIL: M11 synthetic odom/depth/marked costmap not ready')
        if not node.controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FAIL: isolated FollowPath unavailable')
        if not node.fault_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError('FAIL: M11 fake sensor trigger unavailable')

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
            raise RuntimeError('FAIL: M11 fake action request timed out')
        goal_handle = sent.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FAIL: M11 synthetic FollowPath rejected')
        node.begin_goal()
        goal_result = goal_handle.get_result_async()

        if not _spin_until(
                node, lambda: (node.verified_source
                              and node.action_nonzero_samples >= 1
                              and node.healthy_samples >= 1), 16.0):
            raise RuntimeError(
                'FAIL: M11 missing nonzero actual MPPI proposal and '
                'M7 healthy 1s baseline')
        if goal_result.done():
            raise RuntimeError(
                'FAIL: M11 synthetic goal terminal before fake fault')

        # Capture last input receipt BEFORE sending the trigger request.
        # This is an approximation to the precise sensor age threshold;
        # callbacks may deliver more samples before the fixture timer pauses.
        pre_sensor_recv = (
            node.depth_recv if node.mode == 'depth' else node.odom_recv)
        trigger = node.fault_client.call_async(Trigger.Request())
        if not _spin_until(node, trigger.done, 3.0):
            raise RuntimeError('FAIL: M11 fake trigger acknowledgment timeout')
        reply = trigger.result()
        if reply is None or not reply.success:
            raise RuntimeError(
                'FAIL: M11 fake sensor trigger refused: ' +
                str(getattr(reply, 'message', '<no response>')))
        start, end = parse_window(reply.message)
        if not (start <= time.monotonic() < end):
            raise RuntimeError('FAIL: M11 fake blackout schedule not current')
        node.triggered = True
        print(
            f'M11 WINDOW: mode={node.mode} '
            f'scheduled_start_mono={start:.6f} '
            f'scheduled_end_mono={end:.6f} '
            'clock=container_monotonic; NOT physical sensor timing',
            flush=True,
        )

        # Keep observing beyond the scheduled 5s blackout even if the
        # synthetic FollowPath terminates. Never assume MPPI emits zero.
        until = max(time.monotonic() + OBSERVATION_AFTER_TRIGGER_S, end + 1.0)
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_graph()
            if goal_result.done() and outcome == 'not observed':
                result = goal_result.result()
                outcome = 'terminal status=' + str(
                    getattr(result, 'status', '<unknown>'))
                print('M11 ACTION: ' + outcome, flush=True)
        if not node.sustained_veto:
            raise RuntimeError(
                'FAIL: M11 missing 0.5s sustained correct synthetic '
                f'{node.mode} veto; last={node.last_label}')
        if node.failure:
            raise RuntimeError(node.failure)
        metrics = summarize(
            node.receipt_samples, start=start, end=end,
            first_veto=node.first_veto_mono, sensor_recv=pre_sensor_recv)
        print(
            'M11 BLACKOUT METRICS: ' +
            ' '.join(f'{key}={value}' for key, value in metrics.items()) +
            f' action={outcome} rearm_seen={node.post_fault_rearm} '
            f'recovered_with_proposals={node.post_fault_healthy}',
            flush=True,
        )
        completed = True
    finally:
        cancel_error = None
        if node is not None and goal_handle is not None:
            try:
                if goal_result is not None and goal_result.done():
                    print('M11 ACTION: goal already terminal', flush=True)
                else:
                    cancel = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        node, cancel, timeout_sec=4.0)
                    if not cancel.done() or cancel.result() is None:
                        cancel_error = 'FAIL: M11 FollowPath cancellation unacknowledged'
                    elif not cancel.result().goals_canceling:
                        terminal = goal_result or goal_handle.get_result_async()
                        rclpy.spin_until_future_complete(
                            node, terminal, timeout_sec=3.0)
                        if not terminal.done():
                            cancel_error = 'FAIL: M11 goal neither canceled nor terminal'
            except Exception as exc:
                cancel_error = f'FAIL: M11 synthetic cancellation error: {exc}'
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if cancel_error:
            raise RuntimeError(cancel_error)
        if node is not None and node.failure:
            raise RuntimeError(node.failure)
    if completed:
        print(
            'PASS: M11 synthetic ' + node.mode +
            ' timed-blackout diagnostic sample accounting and M7 stale '
            'input veto; actual MPPI output only on isolated diagnostic '
            'topic; NO proof of braking, safe stopping or motor interlock',
            flush=True,
        )


if __name__ == '__main__':
    main()
