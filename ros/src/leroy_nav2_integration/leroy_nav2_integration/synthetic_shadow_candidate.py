"""M8 FAKE proposal stimulus; never uses /cmd_vel or MPPI output.

This intentionally publishes a fictional Twist on its OWN synthetic-only
diagnostics topic. It is not an MPPI proposal or motor command.
"""

import os

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


SYNTHETIC_CANDIDATE = '/leroy_nav2/diagnostics/synthetic_shadow_candidate'


def require_offline(environment):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(key) != value for key, value in required.items()):
        raise RuntimeError('M8 fake proposal requires isolated offline ROS')


class SyntheticShadowCandidate(Node):
    def __init__(self):
        require_offline(os.environ)
        super().__init__('m8_synthetic_candidate')
        self._publisher = self.create_publisher(Twist, SYNTHETIC_CANDIDATE, 10)
        self.create_timer(0.1, self._tick)
        self.get_logger().warning(
            'M8 FICTIONAL Twist is NOT MPPI output; cannot connect to '
            'motors or /cmd_vel, for shadow audit only')

    def _tick(self):
        msg = Twist()
        msg.linear.x = 0.04
        msg.linear.y = 0.02
        msg.angular.z = 0.1
        self._publisher.publish(msg)


def main(args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticShadowCandidate()
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
