#!/usr/bin/env python3
"""Single bounded LIVE read-only Stage 3 boot-readiness observation.

Consumes the already isolated physical camera/depth/XRSLAM DDS graph.
Never publishes ROS topics or controls the MCU, bridge, wheels or navigation.
The existing offline startup model is evaluated with *unproven* physical
qualification flags; its output is not a command.
"""
import json
import math
import os
import time

IMAGE = "/camera/image_rect"
INFO = "/camera/camera_info"
CLOUD = "/leroy_nav2/diagnostics/raw_depth_points"
STATUS = "/xrslam/tracking_status"
VALID = "/xrslam/localization_valid"
YAW = "/leroy_boot/relative_yaw_status"
EXPECTED = {
    IMAGE: ("rectify_node", "/camera"),
    INFO: ("usb_cam", "/camera"),
    CLOUD: ("depth_obstacles", "/"),
    STATUS: ("leroy_xrslam", "/"),
    VALID: ("leroy_xrslam", "/"),
}
DEVICE_PATHS = ("/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0")


def preflight(env=None, exists=None):
    env = os.environ if env is None else env
    exists = os.path.exists if exists is None else exists
    for key, value in (
        ("ROS_DOMAIN_ID", "170"),
        ("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST"),
        ("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        ("LEROY_LIVE_BOOT_READ_ONLY", "1"),
    ):
        if env.get(key) != value:
            raise RuntimeError("BOOT_LIVE_ENV_INVALID_" + key)
    if any(exists(path) for path in DEVICE_PATHS):
        raise RuntimeError("BOOT_LIVE_DEVICE_VISIBLE")
    return True


def graph_report(node):
    report = {}
    for topic, (owner, namespace) in EXPECTED.items():
        pubs = node.get_publishers_info_by_topic(topic)
        report[topic] = (len(pubs) == 1
                         and pubs[0].node_name == owner
                         and pubs[0].node_namespace == namespace)
    return report


def run():
    preflight()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy,
    )
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from std_msgs.msg import String, Bool
    from boot_localization_core import BootLocalizationModel, Observation

    sensor_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    status_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL)

    class Observer(Node):
        def __init__(self):
            super().__init__("leroy_live_boot_readiness",
                             enable_rosout=False,
                             start_parameter_services=False)
            self.counts = dict.fromkeys(EXPECTED, 0)
            self.received_at = {}
            self.info_stamps = set()
            self.image_stamps = set()
            self.frame_errors = 0
            self.tracking_status = "not_received"
            self.local_valid = None
            self.boot_yaw = None
            self.boot_yaw_at = None
            self.boot_yaw_count = 0
            self._boot_subscriptions = [
                self.create_subscription(Image, IMAGE, self.on_image,
                                         sensor_qos),
                self.create_subscription(CameraInfo, INFO, self.on_info,
                                         sensor_qos),
                self.create_subscription(PointCloud2, CLOUD, self.on_cloud,
                                         sensor_qos),
                self.create_subscription(String, STATUS, self.on_status,
                                         status_qos),
                self.create_subscription(Bool, VALID, self.on_valid,
                                         status_qos),
            ]
            if {sub.topic_name for sub in self._boot_subscriptions} != set(EXPECTED):
                raise RuntimeError("BOOT_LIVE_SUBSCRIPTION_REMAP_DENIED")

        def mark(self, topic):
            self.counts[topic] += 1
            self.received_at[topic] = time.monotonic()

        @staticmethod
        def stamp(msg):
            return (int(msg.header.stamp.sec) * 1_000_000_000
                    + int(msg.header.stamp.nanosec))

        def on_image(self, msg):
            if (msg.header.frame_id != "camera_optical_frame"
                    or msg.width != 640 or msg.height != 480
                    or msg.header.stamp.sec <= 0):
                self.frame_errors += 1
            self.image_stamps.add(self.stamp(msg))
            self.mark(IMAGE)

        def on_info(self, msg):
            if (msg.header.frame_id != "camera_optical_frame"
                    or msg.width != 640 or msg.height != 480
                    or msg.header.stamp.sec <= 0):
                self.frame_errors += 1
            self.info_stamps.add(self.stamp(msg))
            self.mark(INFO)

        def on_cloud(self, msg):
            if (msg.header.frame_id != "camera_optical_frame"
                    or msg.header.stamp.sec <= 0):
                self.frame_errors += 1
            self.mark(CLOUD)

        def on_status(self, msg):
            self.tracking_status = msg.data
            self.mark(STATUS)

        def on_valid(self, msg):
            self.local_valid = bool(msg.data)
            self.mark(VALID)

    rclpy.init(args=[])
    node = None
    try:
        node = Observer()
        start = time.monotonic()
        # Strictly bounded observation; no motor-capable device is mapped.
        while time.monotonic() - start < 12.0:
            rclpy.spin_once(node, timeout_sec=0.05)
        now = time.monotonic()
        owners = graph_report(node)
        ages = {
            topic: (None if topic not in node.received_at
                    else round(now - node.received_at[topic], 3))
            for topic in EXPECTED
        }
        info_matches = len(node.image_stamps & node.info_stamps)
        signal_ok = (
            all(owners.values())
            and node.counts[IMAGE] >= 20
            and node.counts[INFO] >= 20
            and node.counts[CLOUD] >= 1
            and node.counts[STATUS] >= 1
            and node.counts[VALID] >= 1
            and node.frame_errors == 0
            and ages[IMAGE] is not None and ages[IMAGE] < 1.0
            and ages[CLOUD] is not None and ages[CLOUD] < 3.0
        )

        # Live data IS used. The unmeasured swept-obstacle clearance,
        # stop distances, physical authority and trusted yaw odometry
        # intentionally remain False/None: never synthesize release.
        model = BootLocalizationModel()
        model.request_start()
        decision = model.tick(Observation(
            now=now,
            image_at=node.received_at.get(IMAGE),
            obstacle_at=node.received_at.get(CLOUD),
            yaw_at=None,
            localization_at=node.received_at.get(STATUS),
            yaw_rad=None,
            localization_status=(
                node.tracking_status if node.tracking_status in (
                    "initializing", "tracking", "lost") else "unknown"),
            metric_pose_validated=False,
            navigation_release_validated=False,
        ))
        if decision.state != "FAULT_LATCHED" or decision.diagnostic_yaw_target_rad is not None:
            raise RuntimeError("BOOT_LIVE_UNEXPECTED_MOTION_SETPOINT")

        print("LIVE_BOOT_OBSERVATION_JSON=" + json.dumps({
            "counts": node.counts,
            "sensor_last_receipt_age_s": ages,
            "publisher_owners_match": owners,
            "image_info_stamp_matches": info_matches,
            "frame_errors": node.frame_errors,
            "xrslam_tracking_status": node.tracking_status,
            "xrslam_localization_valid": node.local_valid,
            "imu_publisher_count": node.count_publishers("/imu/data_raw"),
            "odom_publisher_count": node.count_publishers("/odom"),
            "read_only": True,
            "source_calibration_validated": False,
            "swept_footprint_clearance_validated": False,
            "physical_motion_authorized": False,
            "boot_state": decision.state,
            "boot_blocker": decision.reason,
        }, sort_keys=True), flush=True)
        print("LIVE_BOOT_SENSORS=" + (
            "PASS_READ_ONLY" if signal_ok else "BLOCKED_INPUTS_INCOMPLETE"),
            flush=True)
        print("LIVE_BOOT_MOTION=BLOCKED_UNCOMMISSIONED", flush=True)
        return 0 if signal_ok else 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
