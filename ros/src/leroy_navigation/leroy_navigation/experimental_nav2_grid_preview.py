"""Non-authoritative 2D planning preview from the active experimental voxel map.

Never publishes to /leroy/navigation/occupancy_grid or the native Nav2 TF
tree. Unknown cells remain unknown: depth observations do NOT establish free
space. The frame and height datum are unvalidated experimental coordinates.
"""
import math
import time

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy, qos_profile_sensor_data)
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import String


def make_preview(points, resolution, max_cells, min_z, max_z):
    """Return (origin_x, origin_y, width, height, occupancy) or None.

    Only positive obstacle evidence is produced. In particular, an unseen cell
    is -1, NOT free (0). Reject pathological point coordinates and grid sizes.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] != 3 or not pts.size:
        return None
    pts = pts[np.isfinite(pts).all(axis=1)]
    pts = pts[(pts[:, 2] >= min_z) & (pts[:, 2] <= max_z)]
    if not len(pts):
        return None
    ij = np.floor(pts[:, :2] / resolution).astype(np.int64)
    lo = ij.min(axis=0)
    hi = ij.max(axis=0)
    dims = hi - lo + 1
    if np.any(dims <= 0) or np.any(dims > max_cells):
        return None
    width, height = int(dims[0]), int(dims[1])
    data = np.full((height, width), -1, dtype=np.int8)
    rc = ij - lo
    data[rc[:, 1], rc[:, 0]] = 100
    return (float(lo[0]) * resolution, float(lo[1]) * resolution,
            width, height, data.ravel().tolist())


class ExperimentalNav2GridPreview(Node):
    def __init__(self):
        super().__init__("leroy_experimental_nav2_grid_preview")
        defaults = {
            "input_cloud_topic": "/xrslam/experimental_scene/voxel_map",
            "metric_status_topic": "/xrslam/experimental_metric/status",
            "output_grid_topic": "/xrslam/experimental_navigation/occupancy_preview",
            "status_topic": "/xrslam/experimental_navigation/preview_status",
            "experimental_frame": "xrslam_experimental_metric_world",
            "resolution_m": 0.10,
            "max_cells_per_axis": 400,
            "min_world_z_m": 0.08,
            "max_world_z_m": 1.20,
            "max_cloud_age_s": 5.0,
        }
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        p = {key: self.get_parameter(key).value for key in defaults}
        self.frame = str(p["experimental_frame"])
        self.resolution = float(p["resolution_m"])
        self.max_cells = int(p["max_cells_per_axis"])
        self.z_min = float(p["min_world_z_m"])
        self.z_max = float(p["max_world_z_m"])
        self.max_age_ns = int(float(p["max_cloud_age_s"]) * 1e9)
        if (self.frame != "xrslam_experimental_metric_world"
                or not math.isfinite(self.resolution)
                or not 0.02 <= self.resolution <= 0.5
                or not 10 <= self.max_cells <= 1000
                or not all(map(math.isfinite, (self.z_min, self.z_max)))
                or self.z_min >= self.z_max
                or not 0 < self.max_age_ns <= 30_000_000_000):
            raise ValueError("Invalid isolated Nav2 preview configuration")
        self.last_segment = None
        self.metric_active = False
        self.last_cloud_stamp = -1
        self.last_error = "none"
        self.occupied_cells = 0
        self.last_grid = None
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, str(p["output_grid_topic"]), status_qos)
        self.status_pub = self.create_publisher(
            String, str(p["status_topic"]), status_qos)
        self.create_subscription(
            String, str(p["metric_status_topic"]), self.on_metric, status_qos)
        self.create_subscription(
            PointCloud2, str(p["input_cloud_topic"]), self.on_cloud,
            qos_profile_sensor_data)
        self.create_timer(1.0, self.report)
        self.get_logger().warning(
            "Experimental costmap preview only: no free-space inference, "
            "no odom TF, no Nav2 costmap or motor authority.")

    def report(self):
        text = (
            f"segment_id={self.last_segment if self.last_segment is not None else 0} "
            f"active={int(self.metric_active)} "
            f"occupied_cells={self.occupied_cells} "
            f"grid_ready={int(self.last_grid is not None)} "
            f"last_error={self.last_error} "
            "navigation_authority=0 tf=0 free_space_inferred=0")
        self.status_pub.publish(String(data=text))

    def empty_grid(self):
        # Publish empty map to invalidate old RViz data on each scale reset.
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = self.frame
        grid.info.map_load_time = grid.header.stamp
        grid.info.resolution = self.resolution
        grid.info.width = 1
        grid.info.height = 1
        grid.info.origin.orientation.w = 1.0
        grid.data = [-1]
        self.grid_pub.publish(grid)
        self.occupied_cells = 0
        self.last_grid = None
        self.last_cloud_stamp = -1

    def on_metric(self, msg):
        import re
        match_seg = re.search(r"(?:^|\s)segment_id=(\d+)(?:\s|$)", msg.data)
        match_state = re.search(r"(?:^|\s)state=([^\s]+)", msg.data)
        if not (match_seg and match_state):
            self.metric_active = False
            self.last_error = "malformed_metric_status"
            self.empty_grid()
            return
        segment = int(match_seg.group(1))
        if segment != self.last_segment:
            self.last_segment = segment
            self.empty_grid()
        self.metric_active = match_state.group(1) in (
            "prototype_scale_frozen", "resumed_same_segment")
        if not self.metric_active:
            # Preserve the last view during temporary suspension, but
            # absolutely no new observations are accepted.
            self.last_error = "metric_preview_inactive"

    def on_cloud(self, msg):
        if not self.metric_active:
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        now = self.get_clock().now().nanoseconds
        if (msg.header.frame_id != self.frame or stamp <= 0 or
                stamp < self.last_cloud_stamp or
                not 0 <= now - stamp <= self.max_age_ns):
            self.last_error = "invalid_frame_timestamp_or_age"
            return
        self.last_cloud_stamp = stamp
        try:
            pts = point_cloud2.read_points_numpy(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            result = make_preview(pts, self.resolution, self.max_cells,
                                  self.z_min, self.z_max)
            if result is None:
                self.last_error = "no_obstacle_height_points_or_out_of_bounds"
                self.empty_grid()
                return
            ox, oy, width, height, data = result
            grid = OccupancyGrid()
            grid.header = msg.header
            grid.info.map_load_time = msg.header.stamp
            grid.info.resolution = self.resolution
            grid.info.width = width
            grid.info.height = height
            grid.info.origin.position.x = ox
            grid.info.origin.position.y = oy
            grid.info.origin.orientation.w = 1.0
            grid.data = data
            self.grid_pub.publish(grid)
            self.last_grid = grid
            self.occupied_cells = sum(v == 100 for v in data)
            self.last_error = "none"
        except Exception as exc:
            self.last_error = f"{type(exc).__name__}:{str(exc)[:60]}"


def main(args=None):
    rclpy.init(args=args)
    node = ExperimentalNav2GridPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
