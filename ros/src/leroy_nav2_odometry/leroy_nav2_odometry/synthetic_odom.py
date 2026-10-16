"""M3 wheel-based odom/TF simulation. Absolutely no live wheel input or commands."""

import math
import os
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

from .kinematics import (
    Geometry, Pose2D, Twist2D, integrate, synthetic_twist,
    twist_from_wheels, wheels_from_twist,
)


# Fictional values. Leroy's effective roller/wheel geometry is NOT measured.
SIM_GEOMETRY = Geometry(0.05, 0.25, 0.20)


def _covariance(diagonal):
    result = [0.0] * 36
    for i, value in enumerate(diagonal):
        result[7 * i] = float(value)
    return result


class SyntheticOdom(Node):
    def __init__(self):
        guard = {
            'LEROY_NAV2_DRY_RUN': '1',
            'ROS_DOMAIN_ID': '167',
            'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
        }
        if any(os.environ.get(k) != v for k, v in guard.items()):
            raise RuntimeError('Synthetic odom requires isolated Nav2 environment')
        super().__init__('synthetic_odom')
        # Deliberately NOT /odom, and no subscribers or velocity publishers.
        self._pub = self.create_publisher(
            Odometry, '/leroy_nav2/diagnostics/synthetic_odom', 10)
        self._tf = TransformBroadcaster(self)
        self._pose = Pose2D()
        self._start = time.monotonic()
        self._previous = None
        self._last_stamp_ns = 0
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            'M3 SYNTHETIC ONLY: odom->base_link from fictional X-roller '
            'wheel speeds; NOT encoder data, calibration or motion authority')

    def _tick(self):
        now = time.monotonic()
        if self._previous is None:
            self._previous = now
            return
        dt = now - self._previous
        self._previous = now
        if not 0.0 < dt <= 0.25:
            self.get_logger().warning(
                'Skipped invalid/stalled synthetic wheel sample; no extrapolation')
            return

        # The timed pattern is independent of /cmd_vel, feedback or any input.
        commanded_for_fixture = synthetic_twist((now - self._start) % 14.0)
        wheels = wheels_from_twist(commanded_for_fixture, SIM_GEOMETRY)
        measured_synthetic = twist_from_wheels(wheels, SIM_GEOMETRY)
        self._pose = integrate(self._pose, measured_synthetic, dt)

        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1000000000 + stamp.nanosec
        if stamp_ns <= self._last_stamp_ns:
            self.get_logger().warning('Skipped nonmonotonic ROS stamp')
            return
        self._last_stamp_ns = stamp_ns
        self._publish(stamp, measured_synthetic)

    def _publish(self, stamp, twist):
        yaw = self._pose.yaw
        z, w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._pose.x
        odom.pose.pose.position.y = self._pose.y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.x = twist.vx
        odom.twist.twist.linear.y = twist.vy
        odom.twist.twist.angular.z = twist.wz
        # Arbitrary nonzero simulated uncertainty, NOT real sensor covariance.
        odom.pose.covariance = _covariance((0.02, 0.02, 10., 10., 10., 0.1))
        odom.twist.covariance = _covariance((0.01, 0.01, 10., 10., 10., 0.05))
        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = odom.child_frame_id
        tf.transform.translation.x = self._pose.x
        tf.transform.translation.y = self._pose.y
        tf.transform.rotation.z = z
        tf.transform.rotation.w = w
        self._pub.publish(odom)
        self._tf.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticOdom()
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
