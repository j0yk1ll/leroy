"""M10: one triggered FAKE sensor outage while a synthetic MPPI goal runs.

Observes the controller's REAL remapped diagnostic Twist with M7 labels.
No velocity publisher, motor input, real camera, wheel or safety interlock.
A Trigger service only pauses an isolated synthetic sensor publisher.
"""

import os
import time

import rclpy
from nav2_msgs.action import FollowPath
from std_srvs.srv import Trigger

from .controller_probe import (
    ControllerProbe, _make_synthetic_path, _spin_until,
)
from .m9_mppi_shadow_probe import (
    M9MppiShadowProbe, require_offline, GRAPH_DISCOVERY_TIMEOUT_S,
)
from .m9_mppi_source import MPPI_TOPIC, inspect_mppi_source
from .m10_triggered_fake_fault import SERVICE, BLACKOUT_S
from .shadow_policy import ShadowWindow


FAULTS = {
    'depth': 'WOULD_VETO_BLOCKED_DEPTH',
    'odom': 'WOULD_VETO_BLOCKED_ODOM',
}
FAULT_DWELL_S = 0.5
OBSERVATION_AFTER_TRIGGER_S = 9.0


class M10ActiveFaultProbe(M9MppiShadowProbe):
    """M9's read-only MPPI observer + a fixture-only one-shot trigger client."""

    def __init__(self):
        super().__init__()
        self.declare_parameter('fault_mode', 'depth')
        self.mode = self.get_parameter('fault_mode').value
        if self.mode not in FAULTS:
            raise ValueError('M10 fault_mode must be depth or odom')
        self.fault_client = self.create_client(Trigger, SERVICE)
        self.triggered = False
        self.fault_since = None
        self.sustained_veto = False
        self.proposals_after_trigger = 0
        self.proposals_after_veto = 0
        self.post_fault_rearm = False
        self.post_fault_healthy = False
        self.get_logger().warning(
            'M10 observes REAL MPPI diagnostic proposals during a '
            'TRIGGERED FAKE sensor outage; NO robot drive authorization')

    def _on_cmd(self, msg):
        super()._on_cmd(msg)
        if self.triggered and self.goal_accepted and not self.failure:
            self.proposals_after_trigger += 1
            if self.sustained_veto:
                self.proposals_after_veto += 1

    def check_graph(self):
        # Bypass the M9 print/decision function only; retain M5's strict
        # no-/cmd_vel, no-/odom, output-type and forbidden-node checks.
        ControllerProbe.check_graph(self)
        endpoints = self.get_publishers_info_by_topic(MPPI_TOPIC)
        verified, pending, evidence = inspect_mppi_source(endpoints)
        if not verified and not pending:
            raise RuntimeError(
                'FAIL: M10 unexpected MPPI diagnostic publisher: ' +
                repr(evidence))
        self.verified_source = verified
        if not self.goal_accepted:
            return
        if pending:
            if self.triggered:
                raise RuntimeError(
                    'FAIL: publisher identity lost during synthetic fault: ' +
                    repr(evidence))
            now = time.monotonic()
            if self.graph_pending_since is None:
                self.graph_pending_since = now
            if now - self.graph_pending_since >= GRAPH_DISCOVERY_TIMEOUT_S:
                raise RuntimeError(
                    'FAIL: MPPI source unresolved after 12s: ' + repr(evidence))
            if self.last_label != 'WAITING_MPPI_GRAPH_IDENTITY':
                print('M10 MPPI SHADOW: WAITING_MPPI_GRAPH_IDENTITY', flush=True)
            self.window = ShadowWindow()
            self.healthy_samples = 0
            self.last_label = 'WAITING_MPPI_GRAPH_IDENTITY'
            return
        self.graph_pending_since = None
        label = self.window.observe(self.snapshot())
        if label != self.last_label:
            print('M10 MPPI SHADOW: ' + label, flush=True)
            self.last_label = label
        if label == 'HEALTHY_DIAGNOSTIC_ONLY':
            self.healthy_samples += 1
        if not self.triggered:
            return

        # Actual fake sensor receipt/acquisition freshness takes priority
        # over the candidate, even if MPPI continues publishing.
        if label == FAULTS[self.mode]:
            now = time.monotonic()
            if self.fault_since is None:
                self.fault_since = now
            if (not self.sustained_veto
                    and now - self.fault_since >= FAULT_DWELL_S):
                self.sustained_veto = True
                print('M10 FAULT: sustained ' + label, flush=True)
        else:
            self.fault_since = None

        if self.sustained_veto and label == 'WOULD_VETO_REARM':
            self.post_fault_rearm = True
        if (self.sustained_veto and self.post_fault_rearm
                and label == 'HEALTHY_DIAGNOSTIC_ONLY'):
            self.post_fault_healthy = True


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    goal_handle = None
    goal_result = None
    completed = False
    outcome = 'not observed'
    try:
        node = M10ActiveFaultProbe()
        if not _spin_until(
                node, lambda: node.latest_odom is not None
                and node.depth_recv is not None and node.map_ready,
                25.0):
            raise RuntimeError(
                'FAIL: initial synthetic odometry/depth/costmap not ready')
        if not node.controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FAIL: isolated FollowPath server unavailable')
        if not node.fault_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError('FAIL: fixture-only trigger service unavailable')

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
            raise RuntimeError('FAIL: synthetic FollowPath request timed out')
        goal_handle = sent.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FAIL: synthetic FollowPath goal rejected')
        node.begin_goal()
        goal_result = goal_handle.get_result_async()

        if not _spin_until(
                node, lambda: (
                    node.verified_source
                    and node.action_nonzero_samples >= 1
                    and node.healthy_samples >= 1),
                16.0):
            raise RuntimeError(
                'FAIL: actual MPPI pre-fault nonzero diagnostic and '
                'M7 healthy baseline not established')
        if goal_result.done():
            raise RuntimeError(
                'FAIL: synthetic goal already terminated before fake fault')

        # Trigger ONLY the synthetic depth/odom fixture, never a ROS
        # command output, FollowPath control, real camera or motor.
        request = node.fault_client.call_async(Trigger.Request())
        if not _spin_until(node, request.done, 3.0):
            raise RuntimeError('FAIL: M10 fixture trigger acknowledgement timeout')
        reply = request.result()
        if reply is None or not reply.success:
            raise RuntimeError(
                'FAIL: M10 fake sensor trigger refused: ' +
                str(getattr(reply, 'message', '<no response>')))
        node.triggered = True
        print('M10 FAULT: triggered ' + node.mode +
              ' fake blackout; goal was active before trigger', flush=True)

        end = time.monotonic() + OBSERVATION_AFTER_TRIGGER_S
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_graph()
            if goal_result.done() and outcome == 'not observed':
                result = goal_result.result()
                outcome = 'terminal status=' + str(
                    getattr(result, 'status', '<unknown>'))
                print('M10 ACTION: ' + outcome, flush=True)
        if not node.sustained_veto:
            raise RuntimeError(
                'FAIL: expected sustained shadow veto not observed during '
                f'triggered fake {node.mode} loss; last={node.last_label}')
        completed = True
        print(
            'M10 OBSERVATION: post_trigger_mppi_samples=' +
            str(node.proposals_after_trigger) +
            ' post_veto_mppi_samples=' + str(node.proposals_after_veto) +
            ' rearm_seen=' + str(node.post_fault_rearm) +
            ' recovered_with_proposals=' + str(node.post_fault_healthy) +
            ' action=' + outcome,
            flush=True,
        )
    finally:
        cancel_error = None
        if node is not None and goal_handle is not None:
            try:
                if goal_result is not None and goal_result.done():
                    print('M10 ACTION: already terminal before cleanup',
                          flush=True)
                else:
                    cancel = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        node, cancel, timeout_sec=4.0)
                    if not cancel.done() or cancel.result() is None:
                        cancel_error = 'FAIL: synthetic goal cancel unacknowledged'
                    elif not cancel.result().goals_canceling:
                        terminal = goal_result or goal_handle.get_result_async()
                        rclpy.spin_until_future_complete(
                            node, terminal, timeout_sec=3.0)
                        if not terminal.done():
                            cancel_error = 'FAIL: goal neither canceled nor terminal'
            except Exception as exc:
                cancel_error = f'FAIL: M10 action cancellation check failed: {exc}'
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
            'PASS: M10 synthetic ' + node.mode +
            ' blackout overlapped an initially active synthetic FollowPath '
            'goal; M7 shadow vetoed stale fake sensor input while actual '
            'MPPI diagnostic output was monitored, with explicit counts '
            'and action outcome; NO controller safe-stop or motor claim',
            flush=True,
        )


if __name__ == '__main__':
    main()
