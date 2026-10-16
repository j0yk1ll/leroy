"""M16 pure passive ROS sensor quality metrics; no ROS imports.

These checks say whether messages have a usable *shape* and timestamp,
never whether tracking, camera calibration, metric scale, or obstacles
are correct. Input payloads and images are never written to a file.
"""
import math
import struct

from .m15_sensor_contract import stamp_seconds


INPUTS = {
    'camera_image': ('/camera/image_rect', 'sensor_msgs/msg/Image'),
    'camera_info': ('/camera/camera_info', 'sensor_msgs/msg/CameraInfo'),
    'depth_image': ('/depthart/depth', 'sensor_msgs/msg/Image'),
    'imu': ('/imu/data_raw', 'sensor_msgs/msg/Imu'),
    'xrslam_tracking': ('/xrslam/tracking_status', 'std_msgs/msg/String'),
    'xrslam_valid': ('/xrslam/localization_valid', 'std_msgs/msg/Bool'),
    'odom': ('/odom', 'nav_msgs/msg/Odometry'),
    'tf': ('/tf', 'tf2_msgs/msg/TFMessage'),
    'obstacle_cloud': ('/leroy_nav2/diagnostics/obstacle_points',
                       'sensor_msgs/msg/PointCloud2'),
}


def check_image(msg, *, depth=False):
    if not msg.header.frame_id or msg.width < 1 or msg.height < 1:
        return 'bad_image_frame_or_dimensions'
    if msg.width * msg.height > 16_000_000:
        return 'oversized_image'
    if depth:
        if msg.encoding != '32FC1':
            return 'not_32FC1_depth'
        bpp = 4
    else:
        bpp = {
            'mono8': 1, 'rgb8': 3, 'bgr8': 3,
            'rgba8': 4, 'bgra8': 4,
        }.get(msg.encoding)
        if bpp is None:
            return 'unexpected_rectified_image_encoding'
    if (msg.step < msg.width * bpp
            or msg.step > msg.width * bpp + 65536
            or len(msg.data) != msg.step * msg.height):
        return 'bad_image_stride_or_buffer'
    return None


def depth_sample(msg, sample_count=64):
    """Bounded, non-copying sample, never a depth-accuracy estimate."""
    if check_image(msg, depth=True):
        return None
    prefix = '>' if msg.is_bigendian else '<'
    count = max(1, min(sample_count, msg.width * msg.height))
    valid = 0
    for i in range(count):
        idx = (i * msg.width * msg.height) // count
        off = (idx // msg.width) * msg.step + (idx % msg.width) * 4
        value = struct.unpack_from(prefix+'f', msg.data, off)[0]
        if math.isfinite(value) and value > 0:
            valid += 1
    return valid, count


def check_camera_info(msg):
    if (not msg.header.frame_id or msg.width < 1 or msg.height < 1
            or len(msg.p) != 12
            or not all(math.isfinite(v) for v in msg.p)
            or msg.p[0] <= 0 or msg.p[5] <= 0):
        return 'invalid_camera_calibration_shape'
    return None


def check_imu(msg):
    if not msg.header.frame_id or msg.header.frame_id != 'imu_link':
        return 'unexpected_imu_frame'
    values = (
        msg.angular_velocity.x, msg.angular_velocity.y,
        msg.angular_velocity.z, msg.linear_acceleration.x,
        msg.linear_acceleration.y, msg.linear_acceleration.z)
    if not all(math.isfinite(v) for v in values):
        return 'nonfinite_imu_sample'
    return None


class StreamStats:
    """Track accepted and rejected samples using bounded summary fields."""

    def __init__(self):
        self.received = 0
        self.accepted = 0
        self.invalid = 0
        self.first_recv = None
        self.last_recv = None
        self.last_valid_recv = None
        self.last_stamp = None
        self.last_age_s = None
        self.min_age_s = None
        self.max_age_s = None
        self.max_gap_s = None
        self.last_error = None
        self.latest_invalid = False
        self.frame = None
        self.details = {}

    def observe(self, *, stamp, frame, mono, ros_now, error=None,
                max_age_s=2.0):
        self.received += 1
        if self.first_recv is None:
            self.first_recv = mono
        if self.last_recv is not None:
            gap = mono - self.last_recv
            self.max_gap_s = max(
                gap, self.max_gap_s if self.max_gap_s is not None else 0)
        self.last_recv = mono
        if not error and stamp is None:
            error = 'invalid_or_zero_acquisition_stamp'
        if not error and (
                not math.isfinite(ros_now)
                or not -0.1 <= ros_now-stamp <= max_age_s):
            error = 'stale_or_future_acquisition_stamp'
        if not error and self.last_stamp is not None and stamp <= self.last_stamp:
            error = 'duplicate_or_regressed_acquisition_stamp'
        if error:
            self.latest_invalid = True
            self.invalid += 1
            self.last_error = error
            return False
        self.accepted += 1
        self.latest_invalid = False
        self.last_valid_recv = mono
        self.last_stamp = stamp
        self.frame = frame
        age = ros_now-stamp
        self.last_age_s = round(age, 4)
        self.min_age_s = round(min(age, self.min_age_s if self.min_age_s is not None else age), 4)
        self.max_age_s = round(max(age, self.max_age_s if self.max_age_s is not None else age), 4)
        return True

    def export(self, mono, freshness_s=2.0):
        span = (
            self.last_recv - self.first_recv
            if self.first_recv is not None and self.last_recv is not None
            else 0)
        return {
            'received': self.received, 'accepted': self.accepted,
            'invalid': self.invalid, 'frame': self.frame,
            'last_error': self.last_error,
            'latest_invalid': self.latest_invalid,
            'last_acquisition_age_s': self.last_age_s,
            'min_acquisition_age_s': self.min_age_s,
            'max_acquisition_age_s': self.max_age_s,
            'max_receipt_gap_s': (
                round(self.max_gap_s, 3)
                if self.max_gap_s is not None else None),
            'mean_receipt_hz': (
                round((self.received - 1) / span, 2)
                if span > 0 and self.received > 1 else None),
            'fresh_valid_receipt': (
                self.last_valid_recv is not None
                and not self.latest_invalid
                and 0 <= mono-self.last_valid_recv <= freshness_s),
            **self.details,
        }
