"""M10 explicitly triggered blackout of FAKE odom or fake depth ONLY.

The Trigger ROS service never controls a robot, Nav2 action, command
topic, hardware or a motor. It pauses ONLY the selected local fixture
stream for 5 seconds, with no fabricated clearing/free-space message.
"""

import os
import time

import rclpy
from std_srvs.srv import Trigger

from leroy_nav2_odometry.synthetic_odom import SyntheticOdom
from leroy_nav2_perception.synthetic_depth_fixture import SyntheticDepthFixture


SERVICE = '/leroy_nav2/diagnostics/m10_trigger_fake_fault'
BLACKOUT_S = 5.0


def require_offline(environment):
    required = {
        'LEROY_NAV2_DRY_RUN': '1',
        'ROS_DOMAIN_ID': '167',
        'ROS_AUTOMATIC_DISCOVERY_RANGE': 'LOCALHOST',
    }
    if any(environment.get(k) != v for k, v in required.items()):
        raise RuntimeError('M10 fake fault fixture requires isolated ROS')


class TriggeredFakeFault:
    """Mixin: only one explicitly triggered, one-shot source blackout."""

    def __init__(self):
        require_offline(os.environ)
        self._fault_until = None
        self._fault_started = False
        self._was_blocked = False
        super().__init__()
        self.create_service(Trigger, SERVICE, self._trigger)
        self.get_logger().warning(
            'M10 FAKE sensor only: one-shot triggered 5s blackout; '
            'service is not a motor or Nav2 control')

    def _trigger(self, request, response):
        del request
        if self._fault_started:
            response.success = False
            response.message = 'FAIL: fake fault already triggered once'
            return response
        self._fault_started = True
        self._fault_until = time.monotonic() + BLACKOUT_S
        response.success = True
        response.message = 'M10 synthetic sensor blackout armed for 5s'
        self.get_logger().warning('M10 SYNTHETIC FAULT TRIGGERED')
        return response

    def _blocked(self):
        blocked = (self._fault_until is not None
                   and time.monotonic() < self._fault_until)
        if blocked != self._was_blocked:
            self.get_logger().warning(
                'M10 SYNTHETIC SOURCE ' + ('PAUSED' if blocked else 'RESUMED'))
            self._was_blocked = blocked
        return blocked


class TriggeredDepth(TriggeredFakeFault, SyntheticDepthFixture):
    def _publish(self):
        if not self._blocked():
            super()._publish()


class TriggeredOdom(TriggeredFakeFault, SyntheticOdom):
    def _tick(self):
        if self._blocked():
            # Prevent integrating several fake wheel seconds on resume.
            self._previous = time.monotonic()
            return
        super()._tick()


def _run(node_cls, args=None):
    require_offline(os.environ)
    rclpy.init(args=args)
    node = None
    try:
        node = node_cls()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main_depth(args=None):
    _run(TriggeredDepth, args=args)


def main_odom(args=None):
    _run(TriggeredOdom, args=args)
