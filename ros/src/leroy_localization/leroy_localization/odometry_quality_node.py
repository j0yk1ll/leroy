"""Observe /odom and report plausibility; do not publish pose or authorize motors."""
import json
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from leroy_localization.odometry_quality import OdometryQualityGate, Sample


def make_sample(msg):
    q = msg.pose.pose.orientation
    norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w
    if not math.isfinite(norm) or abs(norm - 1.0) > 1e-3:
        raise ValueError("invalid_orientation")
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                     1 - 2*(q.y*q.y + q.z*q.z))
    stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
    p = msg.pose.pose.position
    v = msg.twist.twist
    return Sample(
        stamp, p.x, p.y, yaw, v.linear.x, v.linear.y, v.angular.z,
        max(msg.pose.covariance[0], msg.pose.covariance[7]),
        msg.pose.covariance[35], msg.header.frame_id, msg.child_frame_id)


class OdometryQualityNode(Node):
    def __init__(self):
        super().__init__("leroy_odometry_quality")
        self.gate = OdometryQualityGate()
        self.last_rx_ns = None
        self.last_result = "no_odometry"
        self.pub = self.create_publisher(String, "/leroy/localization/quality", 10)
        self.sub = self.create_subscription(Odometry, "/odom", self.receive, 10)
        self.create_timer(0.1, self.tick)

    def receive(self, msg):
        self.last_rx_ns = time.monotonic_ns()
        try:
            sample = make_sample(msg)
            now_ros = self.get_clock().now().nanoseconds
            quality = self.gate.observe(sample, now_ns=now_ros)
            self.last_result = quality.reason
            self.last_valid = quality.valid
        except (ValueError, TypeError, OverflowError):
            self.gate.reset()
            self.last_result = "invalid_orientation_or_message"
            self.last_valid = False

    def tick(self):
        fresh = (self.last_rx_ns is not None and
                 0 <= time.monotonic_ns() - self.last_rx_ns <= 300_000_000)
        status = {
            "schema": "leroy_odometry_quality_v1",
            "plausible": bool(fresh and getattr(self, "last_valid", False)),
            "metric_calibration_verified": False,
            "reason": self.last_result if fresh else "stream_stale",
        }
        self.pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = OdometryQualityNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
