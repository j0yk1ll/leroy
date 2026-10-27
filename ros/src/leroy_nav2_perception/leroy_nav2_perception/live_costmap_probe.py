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

    def graph_status(self):
        """(state, details): unsafe is terminal; discovery-missing is not.

        A missing expected publisher can be a normal DDS discovery delay.
        Unexpected publishers, motor topics or actuator nodes inhibit
        immediately. No missing publisher is synthesized or ignored for PASS.
        """
        topic_types = dict(self.get_topic_names_and_types())
        forbidden_topics = sorted(
            t for t in ("/cmd_vel", "/cmd_vel_nav") if t in topic_types)
        if forbidden_topics:
            return "UNSAFE", "forbidden_topics=" + repr(forbidden_topics)

        blocked_node_names = (
            "leroy_bridge", "controller_server", "planner_server",
            "bt_navigator", "synthetic_depth_fixture",
            "velocity_smoother",
        )
        nodes = self.get_node_names_and_namespaces()
        forbidden_nodes = sorted(
            ns + "/" + name for name, ns in nodes
            if any(token in name for token in blocked_node_names))
        if forbidden_nodes:
            return "UNSAFE", "forbidden_nodes=" + repr(forbidden_nodes)

        required = (
            ("/leroy_nav2/diagnostics/raw_depth_points",
             "depth_obstacles", "sensor_msgs/msg/PointCloud2"),
            ("/costmap", "costmap", "nav_msgs/msg/OccupancyGrid"),
        )
        missing = []
        for topic, expected_name, expected_type in required:
            endpoints = self.get_publishers_info_by_topic(topic)
            if not endpoints:
                missing.append(topic)
                continue
            observed = [(e.node_namespace, e.node_name, e.topic_type)
                        for e in endpoints]
            if (len(endpoints) != 1
                    or observed[0] != ("/", expected_name, expected_type)):
                return "UNSAFE", (f"unexpected_publishers_on={topic} "
                                  + repr(observed))
        if missing:
            return "WAITING", "missing_publishers=" + repr(missing)
        return "READY", "required_publishers_verified"

    def graph_ok(self):
        # Backward-compatible predicate for callers testing full readiness.
        return self.graph_status()[0] == "READY"


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
        previous_state = None
        previous_detail = None
        seen_ready = False
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
            state, detail = node.graph_status()
            if state != previous_state or detail != previous_detail:
                print(f"LIVE_DEPTH_COSTMAP_GRAPH={state} {detail}", flush=True)
                previous_state, previous_detail = state, detail
            if state == "UNSAFE":
                print("LIVE_DEPTH_COSTMAP=BLOCKED_UNSAFE_GRAPH " + detail,
                      flush=True)
                return 2
            if state == "WAITING":
                if seen_ready:
                    print("LIVE_DEPTH_COSTMAP=BLOCKED_GRAPH_LOST " + detail,
                          flush=True)
                    return 2
                continue
            seen_ready = True
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
              f"graph={previous_state} details={previous_detail} " +
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
