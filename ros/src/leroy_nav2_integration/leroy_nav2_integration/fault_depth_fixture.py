"""Fake camera ONLY: cyclically stop publishing both depth and CameraInfo."""

import time

import rclpy

from leroy_nav2_perception.synthetic_depth_fixture import SyntheticDepthFixture

from .fault_contract import in_blackout


class FaultDepthFixture(SyntheticDepthFixture):
    def __init__(self):
        self._fault_begin = time.monotonic()
        self._blackout_state = None
        super().__init__()
        self.get_logger().warning(
            'M6 FAKE CAMERA fault cycle: stop fake depth+CameraInfo for '
            '6 seconds per 22; never publish clearing clouds')

    def _publish(self):
        blocked = in_blackout(time.monotonic() - self._fault_begin)
        if blocked != self._blackout_state:
            self.get_logger().warning(
                'M6 SYNTHETIC DEPTH ' + ('PAUSED' if blocked else 'RESUMED'))
            self._blackout_state = blocked
        if not blocked:
            super()._publish()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaultDepthFixture()
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
