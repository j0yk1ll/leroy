"""Opt-in, diagnostic VL53L1X anchoring of a RELATIVE depth image.

Never publishes without three explicit commissioning flags. This does not
produce validated odometry, motor authority, or a certified obstacle map.
"""
from collections import deque
import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Range
from std_msgs.msg import String

from leroy_perception.tof_scale_core import scale_relative_depth


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class TofMetricDepth(Node):
    def __init__(self):
        super().__init__("tof_relative_depth_scale")
        self.declare_parameter("relative_depth_topic", "/relative_depth/image")
        self.declare_parameter("camera_info_topic", "/camera/camera_info")
        self.declare_parameter("tof_topic", "/tof/range")
        self.declare_parameter("output_topic", "/experimental/tof_metric_depth")
        self.declare_parameter("tof_frame_id", "vl53l1x_link")
        self.declare_parameter("extrinsics_measured", False)
        self.declare_parameter("time_alignment_measured", False)
        self.declare_parameter("linear_relative_depth_validated", False)
        self.declare_parameter("tof_origin_in_camera_xyz_m", [0.0, 0.0, 0.0])
        # Invalid by default: forces the user to supply a measured rotation.
        self.declare_parameter("tof_rotation_in_camera_xyzw", [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter("max_stamp_difference_ms", 60.0)
        self.declare_parameter("roi_radius_px", 160)
        self.declare_parameter("max_relative_roi_spread", 0.20)
        p = lambda name: self.get_parameter(name).value
        self.ready = all(bool(p(flag)) for flag in (
            "extrinsics_measured", "time_alignment_measured",
            "linear_relative_depth_validated"))
        self.origin = list(p("tof_origin_in_camera_xyz_m"))
        self.rotation = list(p("tof_rotation_in_camera_xyzw"))
        self.max_ns = int(float(p("max_stamp_difference_ms")) * 1e6)
        self.radius = int(p("roi_radius_px"))
        self.spread = float(p("max_relative_roi_spread"))
        self.tof_frame = str(p("tof_frame_id"))
        if (self.max_ns <= 0 or self.max_ns > 100_000_000 or
                not 4 <= self.radius <= 220 or not 0.05 <= self.spread <= 0.5 or
                len(self.origin) != 3 or len(self.rotation) != 4):
            raise ValueError("unsafe depth scale configuration")
        self.bridge = CvBridge()
        self.k = None
        self.shape = None
        self.recent_ranges = deque(maxlen=100)
        self.last_used = -1
        self.out = self.create_publisher(
            Image, str(p("output_topic")), qos_profile_sensor_data)
        self.status = self.create_publisher(String, "/tof/scale_status", 10)
        self.create_subscription(
            CameraInfo, str(p("camera_info_topic")), self.on_info,
            qos_profile_sensor_data)
        self.create_subscription(
            Range, str(p("tof_topic")), self.on_range,
            qos_profile_sensor_data)
        self.create_subscription(
            Image, str(p("relative_depth_topic")), self.on_depth,
            qos_profile_sensor_data)
        self.get_logger().warning(
            "ToF relative-depth scaler: experimental diagnostic only; "
            f"commissioning flags valid={self.ready}, no navigation authority")

    def report(self, message):
        self.status.publish(String(data=message))

    def on_info(self, msg):
        if msg.width <= 0 or msg.height <= 0:
            return
        self.shape = (msg.height, msg.width)
        self.k = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def on_range(self, msg):
        if msg.header.frame_id != self.tof_frame:
            return
        value = float(msg.range)
        if (not math.isfinite(value) or value < msg.min_range or
                value > msg.max_range or not 0.04 <= value <= 1.30):
            return
        stamp = stamp_ns(msg.header.stamp)
        if stamp > 0:
            self.recent_ranges.append((stamp, value))

    def on_depth(self, msg):
        if not self.ready:
            self.report("BLOCKED_UNMEASURED_EXTRINSICS_TIMING_OR_DEPTH_SEMANTICS")
            return
        if self.k is None or (msg.height, msg.width) != self.shape:
            self.report("BLOCKED_NO_MATCHING_CAMERA_INTRINSICS")
            return
        t = stamp_ns(msg.header.stamp)
        candidates = [(abs(s-t), s, r) for s, r in self.recent_ranges
                      if s > self.last_used and abs(s-t) <= self.max_ns]
        if not candidates:
            self.report("BLOCKED_NO_FRESH_TIME_ALIGNED_TOF")
            return
        _, stamp, range_m = min(candidates)
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
            if np.asarray(img).dtype != np.float32:
                raise ValueError("relative input must be float32")
            scaled, stats = scale_relative_depth(
                img, range_m, self.k, self.origin, self.rotation,
                self.radius, self.spread)
        except (ValueError, TypeError) as exc:
            self.report(f"BLOCKED_GEOMETRY_OR_RELATIVE_DEPTH: {exc}")
            return
        out = self.bridge.cv2_to_imgmsg(scaled, encoding="32FC1")
        out.header = msg.header
        self.out.publish(out)
        self.last_used = stamp
        self.report("DIAGNOSTIC_ONLY " +
                    " ".join(f"{key}={value:.5g}" for key, value in stats.items()))


def main():
    rclpy.init()
    node = None
    try:
        node = TofMetricDepth()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
