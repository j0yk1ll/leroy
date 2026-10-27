"""Read-only live DepthART -> PointCloud2 -> Nav2 occupancy grid probe.

PASS proves a REAL sensor-produced raw cloud marked an isolated Nav2
diagnostic costmap. A provisional TF and no ground filtering mean lethal
cells ARE NOT evidence of collision-safe obstacle interpretation.
"""
import os
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2

DURATION_S = 90
REQUIRED_CLOUDS = 3


class LiveCostmapProbe(Node):
    def __init__(self):
        super().__init__("leroy_live_costmap_probe",
                         enable_rosout=False, start_parameter_services=False)
        self.cloud_count = 0
        self.last_cloud_age = None
        self.grid_count = 0
        self.unknown = False
        self.lethal = False
        self.grid_frame = None
        self.create_subscription(
            PointCloud2, "/leroy_nav2/diagnostics/raw_depth_points",
            self._cloud, qos_profile_sensor_data)
        self.create_subscription(
            OccupancyGrid, "/costmap", self._grid,
            QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def _cloud(self, m):
        stamp = (int(m.header.stamp.sec) * 1_000_000_000
                 + int(m.header.stamp.nanosec))
        age = (self.get_clock().now().nanoseconds - stamp) / 1e9
        if (m.header.frame_id != "camera_optical_frame"
                or stamp <= 0 or age < 0 or age > 2.0
                or m.width < 1 or m.point_step < 12):
            return
        self.cloud_count += 1
        self.last_cloud_age = age

    def _grid(self, m):
        if (m.header.frame_id != "base_link"
                or m.info.width < 1 or m.info.height < 1
                or len(m.data) != m.info.width * m.info.height):
            return
        self.grid_count += 1
        self.grid_frame = m.header.frame_id
        self.unknown = -1 in m.data
        self.lethal = 100 in m.data

    def graph_ok(self):
        names = {name for name, _ in self.get_topic_names_and_types()}
        if any(x in names for x in ("/cmd_vel", "/cmd_vel_nav")):
            return False
        nodes = set(self.get_node_names())
        if any(any(blocked in n for blocked in (
                "leroy_bridge", "controller_server", "planner_server",
                "bt_navigator", "synthetic_depth_fixture"))
               for n in nodes):
            return False
        def one_source(topic, expected):
            publishers = self.get_publishers_info_by_topic(topic)
            return len(publishers) == 1 and publishers[0].node_name == expected
        return (one_source(
                    "/leroy_nav2/diagnostics/raw_depth_points",
                    "depth_obstacles")
                and one_source("/costmap", "costmap"))


def main():
    required = {
        "LEROY_LIVE_PERCEPTION_ONLY": "1",
        "LEROY_LIVE_COSTMAP_DIAGNOSTIC": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    }
    if (any(os.environ.get(k) != v for k, v in required.items())
            or os.path.exists("/dev/ttyACM0")):
        raise SystemExit("BLOCKED: missing motor-disconnected diagnostic guard")
    rclpy.init(args=[])
    node = LiveCostmapProbe()
    try:
        deadline = time.monotonic() + DURATION_S
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            if not node.graph_ok():
                print("LIVE_DEPTH_COSTMAP=BLOCKED_UNSAFE_OR_MISSING_GRAPH",
                      flush=True)
                return 2
            if (node.cloud_count >= REQUIRED_CLOUDS and
                    node.grid_count > 0 and node.lethal and node.unknown):
                print(
                    "LIVE_DEPTH_COSTMAP=PASS " +
                    f"real_optical_clouds={node.cloud_count} " +
                    f"costmap_grids={node.grid_count} " +
                    f"cloud_age_last_s={node.last_cloud_age:.3f} " +
                    "lethal_cells=true unknown_cells=true " +
                    "frame=base_link",
                    flush=True)
                print("LIVE_DEPTH_COSTMAP_NOTE="
                      "provisional_TF_marks_unclassified_surfaces_"
                      "not_for_motion", flush=True)
                return 0
        print("LIVE_DEPTH_COSTMAP=INCOMPLETE " +
              f"clouds={node.cloud_count} grids={node.grid_count} " +
              f"lethal={node.lethal} unknown={node.unknown}",
              flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
