"""Experimental session-local depth fusion.

Combines the camera-optical depth cloud with timestamp-associated XRSLAM
body poses. This is a visual prototype map, NOT obstacle/free-space evidence,
navigation odometry, a calibrated world reference, or a safety-rated map.
"""

from __future__ import annotations

from bisect import bisect_left
from collections import OrderedDict, deque
import math
import re
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

from .geometry import transform_xyz


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def tf_components(transform):
    t = transform.translation
    q = transform.rotation
    return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)


def pose_components(pose):
    p = pose.position
    q = pose.orientation
    return (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)


def parse_status(value: str):
    """Only accept the dedicated Stage A4.2 status fields."""
    segment = re.search(r"(?:^|\s)segment_id=(\d+)(?:\s|$)", value)
    state = re.search(r"(?:^|\s)state=([^\s]+)", value)
    if segment is None or state is None:
        return None
    return int(segment.group(1)), state.group(1)


class ExperimentalVoxelMapper(Node):
    def __init__(self):
        super().__init__("experimental_voxel_mapper")
        defaults = {
            "input_cloud_topic": "/xrslam/experimental_scene/depth_cloud",
            "pose_topic": "/xrslam/experimental_metric/pose",
            "metric_status_topic": "/xrslam/experimental_metric/status",
            "output_cloud_topic": "/xrslam/experimental_scene/voxel_map",
            "mapper_status_topic": "/xrslam/experimental_scene/voxel_map_status",
            "world_frame": "xrslam_experimental_metric_world",
            "camera_frame": "camera_optical_frame",
            "body_frame": "base_link",
            "voxel_size_m": 0.05,
            "max_voxels": 60000,
            "max_depth_age_s": 8.0,
            "max_pose_delta_s": 0.12,
            "max_pose_history_s": 20.0,
            "max_pending_clouds": 8,
            "max_points_per_cloud": 4800,
            "publish_period_s": 1.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.input_cloud_topic = str(p["input_cloud_topic"])
        self.pose_topic = str(p["pose_topic"])
        self.metric_status_topic = str(p["metric_status_topic"])
        self.world_frame = str(p["world_frame"])
        self.camera_frame = str(p["camera_frame"])
        self.body_frame = str(p["body_frame"])
        self.voxel_size = float(p["voxel_size_m"])
        self.max_voxels = int(p["max_voxels"])
        self.max_age_ns = round(float(p["max_depth_age_s"]) * 1e9)
        self.match_ns = round(float(p["max_pose_delta_s"]) * 1e9)
        self.history_ns = round(float(p["max_pose_history_s"]) * 1e9)
        self.max_pending = int(p["max_pending_clouds"])
        self.max_points = int(p["max_points_per_cloud"])
        period = float(p["publish_period_s"])
        if (not self.world_frame or not self.camera_frame or not self.body_frame or
                len({self.world_frame, self.camera_frame, self.body_frame,
                     "odom", "map"}) != 5):
            raise ValueError("Experimental frames must be distinct from navigation frames")
        if (not math.isfinite(self.voxel_size) or not 0.01 <= self.voxel_size <= 1.0 or
                not 100 <= self.max_voxels <= 500000 or
                not 0 < self.max_age_ns <= 60_000_000_000 or
                not 0 < self.match_ns <= 500_000_000 or
                self.history_ns < self.max_age_ns or
                not 1 <= self.max_pending <= 64 or
                not 1 <= self.max_points <= 20000 or
                not math.isfinite(period) or period < 0.1):
            raise ValueError("Invalid experimental voxel mapper limits")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.voxels = OrderedDict()  # voxel key -> first observed XYZ; bounded
        self.poses = deque()  # (stamp_ns, PoseStamped), increasing time
        self.pending = deque()  # (arrival_monotonic, PointCloud2)
        self.segment = None
        self.active = False
        self.last_status_state = "waiting_metric_status"
        self.last_cloud_stamp = -1
        self.last_publish = 0.0
        self.accepted_clouds = 0
        self.rejected_clouds = 0
        self.unmatched_clouds = 0
        self.last_error = "none"
        self.map_pub = self.create_publisher(
            PointCloud2, str(p["output_cloud_topic"]), qos_profile_sensor_data
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_pub = self.create_publisher(
            String, str(p["mapper_status_topic"]), status_qos
        )
        self.status_sub = self.create_subscription(
            String, self.metric_status_topic, self.on_status, status_qos
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, rclpy.qos.QoSProfile(depth=100)
        )
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_timer(period, self.tick)
        self.report("initializing")
        self.get_logger().warning(
            "EXPERIMENTAL ONLY: voxel map is not registered to navigation TF, "
            "and cannot authorize obstacle avoidance, driving or free-space clearing."
        )

    def report(self, reason: str):
        msg = String()
        msg.data = (
            f"segment_id={self.segment if self.segment is not None else 0} "
            f"state={self.last_status_state} active={int(self.active)} "
            f"voxels={len(self.voxels)} pending={len(self.pending)} "
            f"accepted_clouds={self.accepted_clouds} "
            f"rejected_clouds={self.rejected_clouds} "
            f"unmatched_clouds={self.unmatched_clouds} "
            f"reason={reason} last_error={self.last_error} "
            "navigation_authority=0 tf=0"
        )
        self.status_pub.publish(msg)

    def clear_map(self, reason: str):
        self.voxels.clear()
        self.poses.clear()
        self.pending.clear()
        self.last_cloud_stamp = -1
        # RViz must not retain points from a previous scale segment.
        cloud = point_cloud2.create_cloud_xyz32(
            Header(stamp=self.get_clock().now().to_msg(),
                   frame_id=self.world_frame),
            np.empty((0, 3), dtype=np.float32),
        )
        self.map_pub.publish(cloud)
        self.report(reason)

    def on_status(self, msg: String):
        status = parse_status(msg.data)
        if status is None:
            self.active = False
            self.last_status_state = "invalid_metric_status"
            self.report("invalid_status")
            return
        segment, state = status
        if self.segment != segment:
            self.segment = segment
            self.active = False
            self.last_status_state = state
            self.clear_map("segment_boundary")
        active = state in ("prototype_scale_frozen", "resumed_same_segment")
        if state != self.last_status_state or active != self.active:
            self.last_status_state = state
            self.active = active
            if not active:
                self.pending.clear()
            self.report("metric_state_change")

    def on_pose(self, msg: PoseStamped):
        if not self.active or msg.header.frame_id != self.world_frame:
            return
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= 0:
            return
        xyz, quat = pose_components(msg.pose)
        if not np.all(np.isfinite((*xyz, *quat))):
            return
        if abs(np.linalg.norm(quat) - 1.0) > 0.02:
            return
        if self.poses and stamp <= self.poses[-1][0]:
            return
        self.poses.append((stamp, msg))
        while self.poses and stamp-self.poses[0][0] > self.history_ns:
            self.poses.popleft()
        self.try_match_pending()

    def on_cloud(self, msg: PointCloud2):
        if not self.active:
            return
        stamp = stamp_ns(msg.header.stamp)
        now = self.get_clock().now().nanoseconds
        if (stamp <= self.last_cloud_stamp or stamp <= 0 or
                now-stamp > self.max_age_ns or
                stamp-now > 100_000_000 or
                msg.header.frame_id != self.camera_frame):
            self.rejected_clouds += 1
            self.last_error = "stale_ordered_or_frame_mismatch"
            return
        self.last_cloud_stamp = stamp
        if len(self.pending) >= self.max_pending:
            self.pending.popleft()
            self.unmatched_clouds += 1
        self.pending.append((time.monotonic(), msg))
        self.try_match_pending()

    def nearest_pose(self, stamp):
        if not self.poses:
            return None
        ts = [entry[0] for entry in self.poses]
        i = bisect_left(ts, stamp)
        candidates = [self.poses[j] for j in (i-1, i) if 0 <= j < len(self.poses)]
        if not candidates:
            return None
        dt, pose = min(
            ((abs(t-stamp), p) for t, p in candidates),
            key=lambda entry: entry[0],
        )
        return pose if dt <= self.match_ns else None

    def try_match_pending(self):
        if not self.active:
            return
        # Delay processing until a nearby pose exists. Never extrapolate
        # beyond the bounded timestamp tolerance.
        remaining = deque()
        while self.pending:
            arrival, cloud = self.pending.popleft()
            pose = self.nearest_pose(stamp_ns(cloud.header.stamp))
            if pose is None:
                if time.monotonic()-arrival < 2.0:
                    remaining.append((arrival, cloud))
                else:
                    self.unmatched_clouds += 1
                continue
            try:
                self.accumulate(cloud, pose)
                self.accepted_clouds += 1
            except Exception as exc:
                self.rejected_clouds += 1
                self.last_error = f"{type(exc).__name__}:{str(exc)[:70]}"
                self.get_logger().warning(
                    f"Map input rejected: {self.last_error}",
                    throttle_duration_sec=3.0,
                )
        self.pending = remaining

    def accumulate(self, cloud: PointCloud2, pose: PoseStamped):
        # Cloud points are still optical-frame XYZ, and pose is the
        # experimental XRSLAM *body* pose. Use the robot's static body<-camera
        # transform, then pose's session-local experimental world<-body.
        tf = self.tf_buffer.lookup_transform(
            self.body_frame, self.camera_frame, Time(),
            timeout=Duration(seconds=0.05),
        )
        points = point_cloud2.read_points_numpy(
            cloud, field_names=("x", "y", "z"), skip_nans=True
        )
        points = np.asarray(points, dtype=np.float32)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError("Malformed XYZ point cloud")
        points = points[:self.max_points]
        if points.size == 0:
            return
        if not np.all(np.isfinite(points)):
            raise ValueError("Non-finite cloud points")
        optical_in_body = transform_xyz(
            points, *tf_components(tf.transform)
        )
        world_points = transform_xyz(
            optical_in_body, *pose_components(pose.pose)
        )
        if not np.all(np.isfinite(world_points)):
            raise ValueError("Non-finite transformed world points")
        cells = np.floor(world_points/self.voxel_size).astype(np.int64)
        # First point per occupied voxel is retained; FIFO evicts oldest
        # occupied cells when the fixed capacity is reached.
        for cell, xyz in zip(cells, world_points):
            key = tuple(int(k) for k in cell)
            if key not in self.voxels:
                self.voxels[key] = xyz.copy()
                if len(self.voxels) > self.max_voxels:
                    self.voxels.popitem(last=False)

    def tick(self):
        self.try_match_pending()
        if self.voxels and self.active:
            xyz = np.stack(tuple(self.voxels.values()), axis=0)
            cloud = point_cloud2.create_cloud_xyz32(
                Header(stamp=self.get_clock().now().to_msg(),
                       frame_id=self.world_frame),
                xyz,
            )
            self.map_pub.publish(cloud)
        self.report("periodic")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ExperimentalVoxelMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
