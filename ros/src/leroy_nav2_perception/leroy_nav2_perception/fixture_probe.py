"""Offline synthetic costmap acceptance probe. No motor outputs."""

import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2


class FixtureProbe(Node):
    def __init__(self):
        super().__init__('fixture_probe')
        self.raw_seen = False
        self.obstacle_seen = False
        self.costmap_seen = False
        self.lethal_seen = False
        self.unknown_seen = False
        self.create_subscription(
            PointCloud2, '/leroy_nav2/diagnostics/raw_depth_points',
            self._on_raw, qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, '/leroy_nav2/diagnostics/obstacle_points',
            self._on_obstacle, qos_profile_sensor_data)
        # Nav2's standalone Jazzy costmap publisher uses the node name as
        # the topic (/costmap), with RELIABLE + TRANSIENT_LOCAL QoS.
        # Late-joining the fixture probe must still see the last full grid.
        grid_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/costmap', self._on_costmap, grid_qos)

    def _on_raw(self, msg):
        if msg.width > 0 and msg.header.frame_id == 'sim_camera_optical_frame':
            self.raw_seen = True

    def _on_obstacle(self, msg):
        if msg.width > 0 and msg.header.frame_id == 'base_link':
            self.obstacle_seen = True

    def _on_costmap(self, msg):
        if msg.info.width < 1 or msg.info.height < 1:
            return
        if msg.header.frame_id != 'base_link':
            return
        self.costmap_seen = True
        # Both lethal and unknown cells must coexist in an actual full grid.
        self.lethal_seen = 100 in msg.data
        self.unknown_seen = -1 in msg.data


def main(args=None):
    rclpy.init(args=args)
    node = FixtureProbe()
    try:
        deadline = time.monotonic() + 20.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            names = {name for name, _ in node.get_topic_names_and_types()}
            if '/cmd_vel' in names:
                raise RuntimeError('FAIL: /cmd_vel topic exists')
            if all((node.raw_seen, node.obstacle_seen, node.costmap_seen,
                    node.lethal_seen, node.unknown_seen)):
                print('PASS: raw + filtered synthetic clouds, lethal obstacle '
                      'cells, unknown space, no /cmd_vel')
                return
        raise RuntimeError(
            'FAIL: incomplete synthetic costmap: '
            f'raw={node.raw_seen} obstacle={node.obstacle_seen} '
            f'costmap={node.costmap_seen} lethal={node.lethal_seen} '
            f'unknown={node.unknown_seen}')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
