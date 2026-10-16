"""Finite offline-only acceptance: odometry+TF consistency, no motor topics."""

import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage


def _stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


class OdomProbe(Node):
    def __init__(self):
        super().__init__('odom_probe')
        self.odom = {}
        self.transforms = {}
        self.matched = 0
        self.last_ns = 0
        self.forward = False
        self.lateral = False
        self.yaw = False
        self.stopped = False
        self.create_subscription(
            Odometry, '/leroy_nav2/diagnostics/synthetic_odom',
            self._on_odom, qos_profile_sensor_data)
        self.create_subscription(
            TFMessage, '/tf', self._on_tf, qos_profile_sensor_data)

    def _on_odom(self, msg):
        if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
            raise RuntimeError('FAIL: incorrect odom/body frame')
        ns = _stamp_ns(msg.header.stamp)
        if ns <= self.last_ns:
            raise RuntimeError('FAIL: invalid/reordered odom acquisition stamps')
        self.last_ns = ns
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        t = msg.twist.twist
        if not all(math.isfinite(v) for v in (
                p.x, p.y, q.z, q.w, t.linear.x, t.linear.y, t.angular.z)):
            raise RuntimeError('FAIL: nonfinite odometry')
        if abs(q.z * q.z + q.w * q.w - 1.0) > 1e-5:
            raise RuntimeError('FAIL: nonunit quaternion')
        if any(msg.pose.covariance[i] <= 0 or msg.twist.covariance[i] <= 0
               for i in (0, 7, 35)):
            raise RuntimeError('FAIL: missing synthetic covariance')
        self.forward |= t.linear.x > 0.04
        self.lateral |= t.linear.y > 0.04
        self.yaw |= t.angular.z > 0.1
        self.stopped |= all(abs(v) < 1e-6 for v in (
            t.linear.x, t.linear.y, t.angular.z))
        self.odom[ns] = (p.x, p.y, q.z, q.w)
        self._match(ns)

    def _on_tf(self, msg):
        for tf in msg.transforms:
            if tf.header.frame_id != 'odom' or tf.child_frame_id != 'base_link':
                continue
            ns = _stamp_ns(tf.header.stamp)
            self.transforms[ns] = (
                tf.transform.translation.x, tf.transform.translation.y,
                tf.transform.rotation.z, tf.transform.rotation.w)
            self._match(ns)

    def _match(self, ns):
        if ns not in self.odom or ns not in self.transforms:
            return
        a, b = self.odom.pop(ns), self.transforms.pop(ns)
        if any(abs(x - y) > 1e-6 for x, y in zip(a, b)):
            raise RuntimeError('FAIL: same-stamp odometry/TF pose mismatch')
        self.matched += 1
        # Bound storage even if a broken publisher never matches.
        if len(self.odom) > 300 or len(self.transforms) > 300:
            raise RuntimeError('FAIL: TF/odom matching backlog')

    def ready(self):
        return (self.matched >= 5 and self.forward and self.lateral
                and self.yaw and self.stopped)


def main(args=None):
    rclpy.init(args=args)
    node = OdomProbe()
    try:
        until = time.monotonic() + 22.0
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.2)
            topics = {name for name, _ in node.get_topic_names_and_types()}
            names = set(node.get_node_names())
            if '/cmd_vel' in topics or '/odom' in topics:
                raise RuntimeError('FAIL: motor command or live odom topic exists')
            if any('controller_server' in name or 'leroy_bridge' in name
                   for name in names):
                raise RuntimeError('FAIL: controller/hardware bridge exists')
            if node.ready():
                print('PASS: synthetic forward, lateral and yaw odom; matching '
                      'odom->base_link TF and acquisition stamps; '
                      'no /cmd_vel, /odom or controller')
                return
        raise RuntimeError(
            f'FAIL: incomplete M3 fixture: matches={node.matched} '
            f'forward={node.forward} lateral={node.lateral} '
            f'yaw={node.yaw} stopped={node.stopped}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
