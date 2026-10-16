"""Fake wheel odometry ONLY: cyclically stop odometry and dynamic TF."""

import time

import rclpy

from leroy_nav2_odometry.synthetic_odom import SyntheticOdom

from .fault_contract import in_blackout


class FaultOdomFixture(SyntheticOdom):
    def __init__(self):
        self._fault_begin = time.monotonic()
        self._blackout_state = None
        super().__init__()
        self.get_logger().warning(
            'M6 FAKE WHEEL fault cycle: stop synthetic odom+TF for '
            '6 seconds per 22; NO command inputs')

    def _tick(self):
        now = time.monotonic()
        blocked = in_blackout(now - self._fault_begin)
        if blocked != self._blackout_state:
            self.get_logger().warning(
                'M6 SYNTHETIC ODOM ' + ('PAUSED' if blocked else 'RESUMED'))
            self._blackout_state = blocked
        if blocked:
            # Do not retroactively integrate the paused interval on resume.
            self._previous = now
            return
        super()._tick()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FaultOdomFixture()
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
