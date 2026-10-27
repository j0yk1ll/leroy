"""Read-only real sensor probe for the guarded live DepthART cloud launch."""
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from .live_pipeline_contract import LiveChain, stamp_key

MAX_DURATION_S = 90
TARGET_FRAMES = 5


class LivePipelineProbe(Node):
    def __init__(self):
        super().__init__("leroy_live_perception_probe",
                         enable_rosout=False,
                         start_parameter_services=False)
        self.chain = LiveChain()
        self.sub_info = self.create_subscription(
            CameraInfo, "/camera/camera_info", self._info,
            qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, "/depthart/depth", self._depth,
            qos_profile_sensor_data)
        self.sub_cloud = self.create_subscription(
            PointCloud2, "/leroy_nav2/diagnostics/raw_depth_points",
            self._cloud, qos_profile_sensor_data)

    def _accept(self, action):
        try:
            before = len(self.chain.matched)
            action()
            if len(self.chain.matched) > before:
                print("LIVE_DEPTH_CHAIN_MATCHED=" +
                      str(len(self.chain.matched)), flush=True)
        except Exception as exc:
            self.chain.faults.append(str(exc))
            print("LIVE_DEPTH_CHAIN_REJECT=" + str(exc), flush=True)

    def _info(self, m):
        self._accept(lambda: self.chain.on_info(
            stamp_key(m.header.stamp), m.header.frame_id,
            m.width, m.height, m.p))

    def _depth(self, m):
        age = ((self.get_clock().now().nanoseconds -
                stamp_key(m.header.stamp)) / 1e9)
        self._accept(lambda: self.chain.on_depth(
            stamp_key(m.header.stamp), m.header.frame_id, m.width,
            m.height, m.encoding, m.step, len(m.data), age))

    def _cloud(self, m):
        self._accept(lambda: self.chain.on_cloud(
            stamp_key(m.header.stamp), m.header.frame_id,
            m.width, m.height, m.point_step, len(m.data)))

    def graph_ok(self):
        topics = dict(self.get_topic_names_and_types())
        if "/cmd_vel" in topics or "/cmd_vel_nav" in topics:
            return False
        nodes = self.get_node_names()
        if any("leroy_bridge" in x or "controller_server" in x
               for x in nodes):
            return False
        expected = {
            "/camera/camera_info": "sensor_msgs/msg/CameraInfo",
            "/depthart/depth": "sensor_msgs/msg/Image",
            "/leroy_nav2/diagnostics/raw_depth_points":
                "sensor_msgs/msg/PointCloud2",
        }
        for topic, expected_type in expected.items():
            if topic in topics and topics[topic] != [expected_type]:
                return False
        return True


def main():
    if (os.environ.get("LEROY_LIVE_PERCEPTION_ONLY") != "1"
            or os.environ.get("ROS_DOMAIN_ID") != "170"
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or os.path.exists("/dev/ttyACM0")):
        raise SystemExit("BLOCKED: not in isolated sensor-only container")

    rclpy.init(args=[])
    node = LivePipelineProbe()
    try:
        deadline = time.monotonic() + MAX_DURATION_S
        while rclpy.ok() and time.monotonic() < deadline:
            if not node.graph_ok():
                print("LIVE_DEPTH_CHAIN=BLOCKED_UNSAFE_ROS_GRAPH", flush=True)
                return 2
            if len(node.chain.matched) >= TARGET_FRAMES:
                ages = node.chain.ages
                print("LIVE_DEPTH_CHAIN=PASS matched=" +
                      str(len(node.chain.matched)) +
                      " acquisition_age_mean_s=" +
                      format(sum(ages) / len(ages), ".3f") +
                      " shape_faults=" + str(len(node.chain.faults)),
                      flush=True)
                print("LIVE_DEPTH_CHAIN_NOTE=diagnostic_only_no_obstacle_or_motion_validation",
                      flush=True)
                return 0
            rclpy.spin_once(node, timeout_sec=0.2)
        print("LIVE_DEPTH_CHAIN=INCOMPLETE matched=" +
              str(len(node.chain.matched)) + " faults=" +
              repr(node.chain.faults[-5:]), flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
