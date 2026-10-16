"""M14 independent diagnostic-output audit during ACTUAL MPPI fake faults.

Exercises one synthetic FollowPath, the real M14 ROS gate, and the
M11 one-shot fake depth/odom blackout. NO velocity publisher in audit.
"""

import json
import math
import os
import time

from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from std_msgs.msg import String
from std_srvs.srv import Trigger
import rclpy

from .controller_probe import _make_synthetic_path, _spin_until
from .m11_blackout_metrics import parse_window
from .m11_blackout_probe import M11BlackoutProbe
from .m9_mppi_shadow_probe import require_offline
from .m14_gate_graph_contract import inspect_gate_publisher
from .m14_software_gate import GATED_TOPIC, STATUS_TOPIC


OBSERVE_S = 9.0
GATE_DISCOVERY_TIMEOUT_S = 12.0
MAX_RECEIPTS = 512
EXPECTED = {
    'depth': 'BLOCKED_DEPTH',
    'odom': 'BLOCKED_ODOM',
}


class M14GateAudit(M11BlackoutProbe):
    def __init__(self):
        super().__init__()
        self.gated_receipts = []
        self.statuses = []
        self.gate_first_block = None
        self.blackout_end = None
        self.gate_pre_nonzero = 0
        self.gate_recovered_nonzero = 0
        self.gate_blocked_input_nonzero = 0
        self.gate_blocked_ticks = 0
        self.gate_last_seq = 0
        self.gate_discovery_since = time.monotonic()
        self.gate_publishers_verified = False
        self.gate_output = self.create_subscription(
            Twist, GATED_TOPIC, self._on_gated, 10)
        self.gate_status = self.create_subscription(
            String, STATUS_TOPIC, self._on_gate_status, 10)

    def _on_gated(self, msg):
        if len(self.gated_receipts) >= MAX_RECEIPTS:
            self.failure = 'FAIL: M14 gated sample buffer overflow'
            return
        values = (msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z)
        if (any(not math.isfinite(v) for v in values)
                or any(abs(v) > 1e-6 for v in (
                    msg.linear.z, msg.angular.x, msg.angular.y))):
            self.failure = 'FAIL: M14 gate emitted malformed Twist'
            return
        self.gated_receipts.append((
            time.monotonic(),
            any(abs(v) > 1e-4 for v in (
                msg.linear.x, msg.linear.y, msg.angular.z))))

    def _on_gate_status(self, msg):
        try:
            item = json.loads(msg.data)
            seq = item['sequence']
            ts = item['now_mono']
            label = item['reason']
            nonzero = item['output_nonzero']
            input_nonzero = item['input_nonzero']
            vals = item['output_xyz']
            if (item.get('m14_diagnostic_only') is not True
                    or type(seq) is not int or seq <= self.gate_last_seq
                    or type(ts) not in (int, float)
                    or type(nonzero) is not bool
                    or type(input_nonzero) is not bool
                    or type(vals) is not list or len(vals) != 3
                    or not all(math.isfinite(v) for v in vals)
                    or nonzero != any(abs(v) > 1e-4 for v in vals)
                    or (label != 'FORWARD_DIAGNOSTIC_ONLY'
                        and any(v != 0.0 for v in vals))):
                raise ValueError('bad status schema, order or veto output')
        except (KeyError, TypeError, ValueError) as exc:
            self.failure = 'FAIL: M14 gate status invalid: ' + str(exc)
            return
        self.gate_last_seq = seq
        if len(self.statuses) >= MAX_RECEIPTS:
            self.failure = 'FAIL: M14 gate status buffer overflow'
            return
        self.statuses.append(item)
        if (self.triggered and self.gate_first_block is None
                and label == EXPECTED[self.mode]):
            self.gate_first_block = ts
        if label == 'FORWARD_DIAGNOSTIC_ONLY' and nonzero:
            if not self.triggered:
                self.gate_pre_nonzero += 1
            elif (self.gate_first_block is not None
                    and self.blackout_end is not None
                    and ts >= self.blackout_end):
                self.gate_recovered_nonzero += 1

    def check_graph(self):
        super().check_graph()
        topics = dict(self.get_topic_names_and_types())
        expected = (
            (GATED_TOPIC, 'geometry_msgs/msg/Twist'),
            (STATUS_TOPIC, 'std_msgs/msg/String'))
        verified_all = True
        for topic, typ in expected:
            if topic in topics and topics[topic] != [typ]:
                raise RuntimeError(
                    'FAIL: M14 gated topic wrong type: ' +
                    repr((topic, topics[topic])))
            pubs = self.get_publishers_info_by_topic(topic)
            verified, pending, evidence = inspect_gate_publisher(
                pubs, expected_type=typ)
            if not verified and not pending:
                raise RuntimeError(
                    'FAIL: M14 unexpected gated diagnostic publisher: ' +
                    repr((topic, evidence)))
            if not verified:
                verified_all = False
                if (self.triggered or time.monotonic() -
                        self.gate_discovery_since > GATE_DISCOVERY_TIMEOUT_S):
                    raise RuntimeError(
                        'FAIL: M14 gate output identity discovery timeout '
                        'or lost during fault: ' + repr((topic, evidence)))
        # Pending discovery is never sufficient for audit acceptance.
        self.gate_publishers_verified = verified_all


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    goal_handle = None
    goal_result = None
    completed = False
    outcome = 'not observed'
    try:
        node = M14GateAudit()
        if not _spin_until(
                node, lambda: node.latest_odom is not None
                and node.depth_recv is not None and node.map_ready,
                25.0):
            raise RuntimeError('FAIL: M14 synthetic fixtures not ready')
        if not node.controller.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('FAIL: M14 synthetic FollowPath unavailable')
        if not node.fault_client.wait_for_service(timeout_sec=6.0):
            raise RuntimeError('FAIL: M14 fake sensor trigger unavailable')

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
            raise RuntimeError('FAIL: M14 fake FollowPath request timed out')
        goal_handle = sent.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('FAIL: M14 fake FollowPath rejected')
        node.begin_goal()
        goal_result = goal_handle.get_result_async()

        if not _spin_until(
                node, lambda: (node.verified_source
                              and node.gate_publishers_verified
                              and node.action_nonzero_samples >= 1
                              and node.gate_pre_nonzero >= 1
                              and node.healthy_samples >= 1), 18.0):
            raise RuntimeError(
                'FAIL: M14 missing pre-fault real MPPI proposal '
                'and nonzero FORWARDED gated diagnostic after rearm')
        if goal_result.done():
            raise RuntimeError('FAIL: M14 fake goal terminal before trigger')

        request = node.fault_client.call_async(Trigger.Request())
        if not _spin_until(node, request.done, 3.0):
            raise RuntimeError('FAIL: M14 synthetic trigger timeout')
        reply = request.result()
        if reply is None or not reply.success:
            raise RuntimeError('FAIL: M14 fake trigger refused')
        start, end = parse_window(reply.message)
        if not start <= time.monotonic() < end:
            raise RuntimeError('FAIL: M14 fake scheduled window elapsed')
        node.blackout_end = end
        node.triggered = True
        print('M14 WINDOW: mode=' + node.mode, flush=True)

        until = max(time.monotonic() + OBSERVE_S, end + 1.0)
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.1)
            node.check_graph()
            if node.failure:
                raise RuntimeError(node.failure)
            if goal_result.done() and outcome == 'not observed':
                result = goal_result.result()
                outcome = 'terminal status=' + str(
                    getattr(result, 'status', '<unknown>'))
                print('M14 ACTION: ' + outcome, flush=True)

        if not node.sustained_veto:
            raise RuntimeError('FAIL: M14 missing sustained M7 sensor veto')

        blocked = [
            s for s in node.statuses
            if start <= s['now_mono'] < end
            and s['reason'] == EXPECTED[node.mode]
        ]
        if len(blocked) < 5:
            raise RuntimeError('FAIL: M14 missing >=5 timed gate veto cycles')
        first = blocked[0]['now_mono']
        # Once the gate declares the stale-sensor veto, NO selected
        # nonzero output may reappear anywhere in the remaining
        # blackout except the deliberately excluded last timer edge.
        all_post_veto = [
            item for item in node.statuses
            if first <= item['now_mono'] < end - 0.2
        ]
        if any(item['output_nonzero'] for item in all_post_veto):
            raise RuntimeError(
                'FAIL: M14 gate reforwarded nonzero before fake source returned')
        blocked_input_nonzero = sum(
            1 for s in blocked if s['input_nonzero'])
        if blocked_input_nonzero < 3:
            raise RuntimeError(
                'FAIL: M14 did not observe continued nonzero MPPI input '
                'during gate suppression')
        if any(s['output_nonzero'] or any(
                v != 0 for v in s['output_xyz']) for s in blocked):
            raise RuntimeError(
                'FAIL: M14 forwarded a nonzero diagnostic under veto')

        # Cross-topic DDS delivery is not ordered. Exclude a conservative
        # 0.3s startup boundary and 0.2s blackout tail for gated receipts.
        gated_during = [
            nonzero for stamp, nonzero in node.gated_receipts
            if first + 0.3 <= stamp < end - 0.2
        ]
        if len(gated_during) < 5 or any(gated_during):
            raise RuntimeError(
                'FAIL: M14 gated diagnostics nonzero or missing after '
                'confirmed blocked-depth/odom status in blackout')
        if node.gate_recovered_nonzero < 1:
            raise RuntimeError(
                'FAIL: M14 never forwarded a new nonzero synthetic '
                'proposal after recovering from the outage')
        if node.failure:
            raise RuntimeError(node.failure)
        print(
            f'M14 GATE OBSERVATION: mode={node.mode} '
            f'pre_nonzero_forward={node.gate_pre_nonzero} '
            f'blocked_gate_ticks={len(blocked)} '
            f'blocked_input_nonzero_ticks={blocked_input_nonzero} '
            'blocked_gate_nonzero_ticks=0 '
            f'blocked_gated_rx_zeros={len(gated_during)} '
            f'recovery_nonzero_forward={node.gate_recovered_nonzero} '
            f'action={outcome}',
            flush=True)
        completed = True
    finally:
        cancel_error = None
        if node is not None and goal_handle is not None:
            try:
                if goal_result is not None and goal_result.done():
                    print('M14 ACTION: already terminal', flush=True)
                else:
                    cancel = goal_handle.cancel_goal_async()
                    rclpy.spin_until_future_complete(
                        node, cancel, timeout_sec=4.0)
                    if not cancel.done() or cancel.result() is None:
                        cancel_error = 'FAIL: M14 fake goal cancel unacknowledged'
                    elif not cancel.result().goals_canceling:
                        terminal = goal_result or goal_handle.get_result_async()
                        rclpy.spin_until_future_complete(
                            node, terminal, timeout_sec=3.0)
                        if not terminal.done():
                            cancel_error = 'FAIL: M14 fake goal neither canceled nor terminal'
            except Exception as exc:
                cancel_error = 'FAIL: M14 cancellation: ' + str(exc)
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
            'PASS: M14 actual MPPI proposals passed through a real '
            'diagnostic SOFTWARE gate: bounded healthy proposal forwarded, '
            'nonzero proposals suppressed to zero throughout sustained '
            f'fake {node.mode} loss, then gated diagnostic recovered; '
            'NO live bridge, motor channel or physical safe-stop claim',
            flush=True)


if __name__ == '__main__':
    main()
