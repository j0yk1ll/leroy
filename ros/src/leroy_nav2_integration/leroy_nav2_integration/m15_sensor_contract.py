"""M15 real-message sensor intake validation; ROS-free and fail-closed.

For integration with XRSLAM Odometry/status and DepthART base_link
PointCloud2. The checks verify message shape, timestamps and data
quality, NOT calibration, accurate scale, or physical obstacle coverage.
"""
import math
import struct

from .fault_contract import MAX_AGE_S, _recent


def stamp_seconds(stamp):
    sec, ns = stamp.sec, stamp.nanosec
    if (type(sec) is not int or type(ns) is not int
            or ns < 0 or ns >= 1_000_000_000):
        return None
    value = sec + ns * 1e-9
    return value if math.isfinite(value) and value > 0 else None


def finite(values):
    return all(math.isfinite(v) for v in values)


def normalized_quaternion(q):
    xyz = (q.x, q.y, q.z, q.w)
    return finite(xyz) and abs(sum(x*x for x in xyz) - 1.0) <= 0.05


def validate_odom(msg, *, now_ros, previous_stamp):
    """Reject bad metric odom shape but never assert metric calibration."""
    stamp = stamp_seconds(msg.header.stamp)
    if (stamp is None or msg.header.frame_id != 'odom'
            or msg.child_frame_id != 'base_link'
            or (previous_stamp is not None and stamp <= previous_stamp)
            or not math.isfinite(now_ros)
            or not -0.1 <= now_ros - stamp <= MAX_AGE_S):
        return None
    pose = msg.pose.pose
    twist = msg.twist.twist
    if not normalized_quaternion(pose.orientation):
        return None
    coords = (pose.position.x, pose.position.y, pose.position.z,
              twist.linear.x, twist.linear.y, twist.linear.z,
              twist.angular.x, twist.angular.y, twist.angular.z)
    if not finite(coords):
        return None
    for cov in (msg.pose.covariance, msg.twist.covariance):
        if len(cov) != 36 or not finite(cov):
            return None
        if any(cov[i] < 0 for i in (0, 7, 14, 21, 28, 35)):
            return None
    return stamp


def validate_base_cloud(msg, *, now_ros, previous_stamp,
                        max_points=10000, max_bytes=400000):
    """Validate XYZ float32 point cloud, not surface accuracy/coverage."""
    stamp = stamp_seconds(msg.header.stamp)
    if (stamp is None or msg.header.frame_id != 'base_link'
            or (previous_stamp is not None and stamp <= previous_stamp)
            or not math.isfinite(now_ros)
            or not -0.1 <= now_ros - stamp <= MAX_AGE_S
            or msg.is_bigendian or msg.point_step != 12
            or msg.height < 1 or msg.width < 1
            or msg.height * msg.width > max_points
            or msg.row_step != msg.width * msg.point_step
            or len(msg.data) != msg.height * msg.row_step
            or len(msg.data) > max_bytes):
        return None
    fields = [(f.name, f.offset, f.datatype, f.count) for f in msg.fields]
    if fields != [('x', 0, 7, 1), ('y', 4, 7, 1), ('z', 8, 7, 1)]:
        return None
    for x, y, z in struct.iter_unpack('<fff', bytes(msg.data)):
        if not finite((x, y, z)):
            return None
    return stamp


def validate_tf(msg, *, now_ros, previous_stamp):
    if (msg.header.frame_id != 'odom'
            or msg.child_frame_id != 'base_link'):
        return None
    stamp = stamp_seconds(msg.header.stamp)
    if (stamp is None or (previous_stamp is not None
                         and stamp <= previous_stamp)
            or not math.isfinite(now_ros)
            or not -0.1 <= now_ros - stamp <= MAX_AGE_S):
        return None
    t = msg.transform.translation
    if not finite((t.x, t.y, t.z)):
        return None
    if not normalized_quaternion(msg.transform.rotation):
        return None
    return stamp


class SensorIntake:
    """Track last VALID observation and explicit XRSLAM state.

    A malformed new message invalidates the corresponding stream until
    a fresh, newly stamped valid message arrives. XRSLAM statuses are
    latched values: fresh odom still required to consider them current.
    """

    def __init__(self):
        self.odom_recv = self.odom_stamp = None
        self.cloud_recv = self.cloud_stamp = None
        self.tf_recv = self.tf_stamp = None
        self.last_odom_stamp = None
        self.last_cloud_stamp = None
        self.last_tf_stamp = None
        self.localization_valid = False
        self.tracking_state = None

    def odom(self, msg, *, now_mono, now_ros):
        stamp = validate_odom(
            msg, now_ros=now_ros, previous_stamp=self.last_odom_stamp)
        self.odom_recv = self.odom_stamp = None
        if stamp is not None:
            self.last_odom_stamp = self.odom_stamp = stamp
            self.odom_recv = now_mono
        return stamp is not None

    def cloud(self, msg, *, now_mono, now_ros):
        stamp = validate_base_cloud(
            msg, now_ros=now_ros, previous_stamp=self.last_cloud_stamp)
        self.cloud_recv = self.cloud_stamp = None
        if stamp is not None:
            self.last_cloud_stamp = self.cloud_stamp = stamp
            self.cloud_recv = now_mono
        return stamp is not None

    def tf(self, msg, *, now_mono, now_ros):
        stamp = validate_tf(
            msg, now_ros=now_ros, previous_stamp=self.last_tf_stamp)
        self.tf_recv = self.tf_stamp = None
        if stamp is not None:
            self.last_tf_stamp = self.tf_stamp = stamp
            self.tf_recv = now_mono
        return stamp is not None

    def tracking(self, state):
        self.tracking_state = state
        if state != 'tracking':
            self.localization_valid = False

    def localization(self, value):
        if type(value) is not bool:
            raise ValueError('localization valid must be Bool')
        self.localization_valid = value

    def healthy(self, now_mono, now_ros):
        return (
            self.localization_valid and self.tracking_state == 'tracking'
            and _recent(now_mono, now_ros, self.odom_recv, self.odom_stamp)
            and _recent(now_mono, now_ros, self.cloud_recv, self.cloud_stamp)
            and _recent(now_mono, now_ros, self.tf_recv, self.tf_stamp)
            and self.odom_stamp is not None and self.tf_stamp is not None
            and abs(self.odom_stamp - self.tf_stamp) <= 0.25
        )
