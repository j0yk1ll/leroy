"""Depth ray -> odom occupancy grid producer; never publishes motor commands.

Requires metric depth, calibrated extrinsics and ground datum, and odom TF.
Those prerequisites are deployment assertions, not hardware safety evidence.
"""
import math
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformListener

from leroy_navigation.depth_grid import MapSettings, rasterize
from leroy_navigation.evidence_map import EvidenceMap


def apply_transform(point, transform):
    """Rotate and translate point by unit quaternion (x,y,z,w)."""
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z = point
    # Quaternion vector transform: v + w*t + cross(qv,t), t=2*cross(qv,v)
    tx, ty, tz = (2*(q.y*z-q.z*y), 2*(q.z*x-q.x*z), 2*(q.x*y-q.y*x))
    return (x+q.w*tx+(q.y*tz-q.z*ty)+t.x,
            y+q.w*ty+(q.z*tx-q.x*tz)+t.y,
            z+q.w*tz+(q.x*ty-q.y*tx)+t.z)


class DepthGridNode(Node):
    def __init__(self):
        super().__init__("leroy_depth_grid")
        self.declare_parameter("metric_depth_calibrated", False)
        self.declare_parameter("extrinsics_calibrated", False)
        self.declare_parameter("ground_datum_calibrated", False)
        self.declare_parameter("resolution_m", 0.10)
        self.declare_parameter("width_cells", 100)
        self.declare_parameter("height_cells", 100)
        self.declare_parameter("max_depth_m", 3.0)
        self.declare_parameter("min_obstacle_height_m", 0.08)
        self.declare_parameter("max_obstacle_height_m", 1.20)
        self.declare_parameter("max_cloud_age_s", 0.25)
        self.declare_parameter("evidence_lifetime_s", 3.0)
        self.settings = MapSettings(
            resolution=float(self.get_parameter("resolution_m").value),
            width=int(self.get_parameter("width_cells").value),
            height=int(self.get_parameter("height_cells").value),
            max_depth=float(self.get_parameter("max_depth_m").value),
            min_obstacle_height=float(self.get_parameter("min_obstacle_height_m").value),
            max_obstacle_height=float(self.get_parameter("max_obstacle_height_m").value))
        self.max_age_ns = int(float(self.get_parameter("max_cloud_age_s").value)*1e9)
        if not 0 < self.max_age_ns <= 1_000_000_000:
            raise ValueError("invalid cloud freshness limit")
        lifetime_s = float(self.get_parameter("evidence_lifetime_s").value)
        if not math.isfinite(lifetime_s) or not 0 < lifetime_s <= 60:
            raise ValueError("invalid evidence lifetime")
        self.map = EvidenceMap(
            self.settings, evidence_ttl_ns=int(lifetime_s * 1e9))
        self.enabled = all(self.get_parameter(k).value is True for k in (
            "metric_depth_calibrated", "extrinsics_calibrated",
            "ground_datum_calibrated"))
        self.tf = Buffer(cache_time=Duration(seconds=5))
        self.listener = TransformListener(self.tf, self)
        self.pub = self.create_publisher(
            OccupancyGrid, "/leroy/navigation/occupancy_grid", 10)
        self.sub = self.create_subscription(
            PointCloud2, "/perception/raw_depth_points", self.on_cloud, 10)
        if not self.enabled:
            self.get_logger().warning(
                "Depth mapping disabled until metric depth, extrinsics and ground "
                "reference are independently calibrated.")

    def on_cloud(self, msg):
        if not self.enabled or not msg.header.frame_id:
            return
        stamp_ns = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
        now_ns = self.get_clock().now().nanoseconds
        if not 0 <= now_ns-stamp_ns <= self.max_age_ns:
            return
        try:
            at = Time.from_msg(msg.header.stamp)
            sensor_tf = self.tf.lookup_transform(
                "odom", msg.header.frame_id, at, timeout=Duration(seconds=0.05))
            robot_tf = self.tf.lookup_transform(
                "odom", "base_link", at, timeout=Duration(seconds=0.05))
            origin = apply_transform((0., 0., 0.), sensor_tf)
            robot = apply_transform((0., 0., 0.), robot_tf)
            points = [apply_transform((float(x), float(y), float(z)), sensor_tf)
                      for x, y, z in point_cloud2.read_points(
                          msg, field_names=("x", "y", "z"), skip_nans=True)]
            if not points:
                return
            ox, oy, observed = rasterize(points, origin, robot[:2], self.settings)
            ox, oy, data = self.map.update(
                origin_x=ox, origin_y=oy, observed=observed,
                now_ns=time.monotonic_ns())
            out = OccupancyGrid()
            out.header.stamp = msg.header.stamp
            out.header.frame_id = "odom"
            out.info.map_load_time = msg.header.stamp
            out.info.resolution = self.settings.resolution
            out.info.width = self.settings.width
            out.info.height = self.settings.height
            out.info.origin.position.x = ox
            out.info.origin.position.y = oy
            out.info.origin.orientation.w = 1.0
            out.data = list(data)
            self.pub.publish(out)
        except Exception as exc:
            self.get_logger().warning(
                "Depth grid frame rejected: %s" % str(exc),
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = DepthGridNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
