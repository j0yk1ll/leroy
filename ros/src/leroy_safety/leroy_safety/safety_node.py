"""Read-only ROS adapter for Leroy safety mode and motion request diagnostics.

Never signs motor packets, rearms hardware, or trusts ROS health as physical
qualification. Remains DISARMED until a separate commissioned local authority
integration is implemented.
"""
import json
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

from leroy_safety.command_gateway import CommandGateway, MotionRequest
from leroy_safety.motion_supervisor import MotionSupervisor


class SafetyNode(Node):
    def __init__(self):
        super().__init__("leroy_safety_supervisor")
        self._supervisor = MotionSupervisor()
        self._gateway = CommandGateway(self._supervisor)
        self._sequence = 0
        self._status = self.create_publisher(String, "/leroy/safety/status", 10)
        # Diagnostic input only. Not connected to signed motor ingress.
        self._input = self.create_subscription(
            Twist, "/leroy/safety/request/autonomous",
            self._on_request, 10)
        self.create_timer(0.2, self._publish_status)
        self.get_logger().warning(
            "Safety gateway is observation-only; physical qualification "
            "and trusted mode selection are not commissioned. Motors inhibited.")

    def _on_request(self, msg):
        self._sequence += 1
        now = time.monotonic_ns()
        request = MotionRequest(
            "nav2", self._sequence, now, float(msg.linear.x),
            float(msg.linear.y), float(msg.angular.z))
        decision = self._gateway.admit(
            request, now_ns=now + 1,
            prerequisites_verified=False,
            single_publisher_verified=self._input.get_publisher_count() == 1)
        self.get_logger().debug("Motion request blocked: %s", decision.reason)

    def _publish_status(self):
        status = {
            "schema": "leroy_safety_status_v1",
            "mode": self._supervisor.mode.value,
            "reason": self._supervisor.reason,
            "motion_authorized": False,
            "gateway_output_connected": False,
        }
        self._status.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
