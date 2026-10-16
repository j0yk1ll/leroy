"""M15 autonomous read-only audit of diagnostic gating on replayed input.

This process subscribes to diagnostic gate status and Twist; it
triggers one synthetic fault service but publishes NO velocities.
No real hardware sensor or motor is used in this acceptance run.
"""

import json
import math
import os
import time

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

import rclpy
from rclpy.node import Node

from .m9_mppi_shadow_probe import require_offline
from .m15_real_input_gate import OUTPUT, STATUS, REPLAY_PREFIX


MAX_RECEIPTS = 500


class M15InputProbe(Node):
    def __init__(self):
        super().__init__('m15_input_probe')
        self.declare_parameter('fault_mode', 'depth')
        self.mode = self.get_parameter('fault_mode').value
        if self.mode not in ('depth', 'odom', 'tracking'):
            raise RuntimeError('M15 audit invalid replay fault_mode')
        self.samples = []
        self.outputs = []
        self.last_sequence = 0
        self.error = None
        self.status_sub = self.create_subscription(
            String, STATUS, self._status, 10)
        self.output_sub = self.create_subscription(
            Twist, OUTPUT, self._output, 10)
        self.client = self.create_client(
            Trigger, REPLAY_PREFIX + '/trigger')

    def _status(self, msg):
        try:
            value = json.loads(msg.data)
            if (value.get('diagnostics_only') is not True
                    or value.get('profile') != 'offline_replay'
                    or type(value.get('sequence')) is not int
                    or value['sequence'] <= self.last_sequence
                    or not math.isfinite(value['monotonic'])
                    or type(value.get('input_nonzero')) is not bool
                    or type(value.get('output_nonzero')) is not bool
                    or not isinstance(value.get('reason'), str)
                    or len(value.get('output_xyz', [])) != 3
                    or not all(math.isfinite(v)
                               for v in value['output_xyz'])
                    or value['output_nonzero'] != any(
                        abs(v) > 1e-4 for v in value['output_xyz'])
                    or (value['reason'] != 'FORWARD_DIAGNOSTIC_ONLY'
                        and any(v != 0.0 for v in value['output_xyz']))):
                raise ValueError('bad gate status ordering/schema or bypass')
            self.last_sequence = value['sequence']
            if len(self.samples) >= MAX_RECEIPTS:
                raise ValueError('gate status buffer overflow')
            self.samples.append(value)
        except (TypeError, KeyError, ValueError) as e:
            self.error = 'FAIL: M15 malformed status: ' + str(e)

    def _output(self, msg):
        values = (
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z)
        if (not all(math.isfinite(v) for v in values)
                or any(abs(v) > 1e-6 for v in (
                    msg.linear.z, msg.angular.x, msg.angular.y))
                or len(self.outputs) >= MAX_RECEIPTS):
            self.error = 'FAIL: M15 invalid/buffer-overflow gated Twist'
            return
        self.outputs.append((
            time.monotonic(),
            any(abs(v) > 1e-4 for v in (
                msg.linear.x, msg.linear.y, msg.angular.z))))

    def check(self):
        if self.error:
            raise RuntimeError(self.error)
        topics = dict(self.get_topic_names_and_types())
        if '/cmd_vel' in topics or '/odom' in topics:
            raise RuntimeError('FAIL: M15 replay leaked physical ROS topic')
        if any('leroy_bridge' in name for name in self.get_node_names()):
            raise RuntimeError('FAIL: M15 bridge node observed')
        if topics.get(STATUS) not in (None, ['std_msgs/msg/String']):
            raise RuntimeError('FAIL: M15 status topic wrong type')
        if topics.get(OUTPUT) not in (None, ['geometry_msgs/msg/Twist']):
            raise RuntimeError('FAIL: M15 gate output topic wrong type')


def _until(node, condition, timeout_s):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.1)
        node.check()
        if condition():
            return True
    return False


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = M15InputProbe()
        if not node.client.wait_for_service(timeout_sec=8.0):
            raise RuntimeError('FAIL: M15 replay trigger unavailable')
        if not _until(
                node,
                lambda: sum(
                    s['reason'] == 'FORWARD_DIAGNOSTIC_ONLY'
                    and s['output_nonzero'] for s in node.samples) >= 3
                and sum(nonzero for _, nonzero in node.outputs) >= 2,
                12.0):
            raise RuntimeError(
                'FAIL: M15 never forwarded healthy replayed proposal')
        request = node.client.call_async(Trigger.Request())
        if not _until(node, request.done, 3.0):
            raise RuntimeError('FAIL: M15 replay trigger timeout')
        reply = request.result()
        if reply is None or not reply.success:
            raise RuntimeError('FAIL: M15 replay trigger rejected')
        window = json.loads(reply.message)
        if window['mode'] != node.mode:
            raise RuntimeError('FAIL: M15 replay mode mismatch')
        start, end = window['start'], window['end']
        if not start <= time.monotonic() < end:
            raise RuntimeError('FAIL: M15 replay scheduled window invalid')
        print('M15 WINDOW: mode=' + node.mode, flush=True)

        until = end + 2.5
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=.1)
            node.check()
        expected = {
            'depth': 'BLOCKED_DEPTH',
            'odom': 'BLOCKED_ODOM',
            'tracking': 'BLOCKED_LOCALIZATION',
        }[node.mode]
        blocked = [
            s for s in node.samples
            if start <= s['monotonic'] < end
            and s['reason'] == expected
        ]
        first = blocked[0]['monotonic'] if blocked else None
        if len(blocked) < 5:
            raise RuntimeError(
                'FAIL: M15 insufficient blocked gate cycles: ' + expected)
        with_input = sum(s['input_nonzero'] for s in blocked)
        if with_input < 3 or any(s['output_nonzero'] for s in blocked):
            raise RuntimeError('FAIL: M15 did not suppress nonzero replayed proposal')
        for s in node.samples:
            if first <= s['monotonic'] < end - 0.2 and s['output_nonzero']:
                raise RuntimeError('FAIL: M15 forwarded during held fault')

        inner = [
            nonzero for ts, nonzero in node.outputs
            if first + 0.3 <= ts < end - 0.2]
        if len(inner) < 5 or any(inner):
            raise RuntimeError('FAIL: M15 gated Twist received nonzero during fault')
        recovered = sum(
            1 for s in node.samples
            if s['monotonic'] >= end + 1.0
            and s['reason'] == 'FORWARD_DIAGNOSTIC_ONLY'
            and s['output_nonzero'])
        if recovered < 3:
            raise RuntimeError('FAIL: M15 no post-rearm forward')
        print(
            f'M15 INPUT OBSERVATION: mode={node.mode} '
            f'blocked_gate_ticks={len(blocked)} '
            f'blocked_input_nonzero_ticks={with_input} '
            'blocked_gate_nonzero_ticks=0 '
            f'blocked_gated_rx_zeros={len(inner)} '
            f'recovery_nonzero_forward={recovered}', flush=True)
        print(
            'PASS: M15 real-interface MESSAGE-SHAPE software gate '
            'replayed ' + node.mode +
            ' outage; nonzero synthetic proposals suppressed to zero '
            'and restored after input recovery; no physical sensor or motor path',
            flush=True)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
