#!/usr/bin/env python3
"""Single bounded LIVE read-only Stage 3 boot-readiness observation.

Consumes the already isolated physical camera/depth/XRSLAM DDS graph.
Never publishes ROS topics or controls the MCU, bridge, wheels or navigation.
The existing offline startup model is evaluated with *unproven* physical
qualification flags; its output is not a command.
"""

# TODO(AUTONOMY) [trusted readiness]: Extend passive endpoint checks with
# measured camera/IMU phase, calibrated TF provenance, known camera mode,
# sensor age and real odom/costmap covariance before reporting readiness.
# Publish a versioned observation/evidence record to the future supervisor,
# not a Bool that can authorize movement by itself. Check independent owners.
# Acceptance: malformed/stale/duplicate topics, invalid map scale, missing
# coverage, transient tracking and simulated data never produce NAV_READY.

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
AUTO_IMU = "/leroy_boot/auto_imu_calibration_status"
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


def parse_auto_imu_candidate(text, now_ns):
    """Parse strictly recent automatic stationary consistency only.

    The ROS subscriber separately validates the physical publisher owner.
    Success is not a motor, absolute-heading or camera-IMU calibration grant.
    """
    try:
        if type(text) is not str or len(text) > 4096:
            return None
        d = json.loads(text)
        if (type(d) is not dict
                or d.get("schema") != "leroy_auto_imu_calibration_v1"
                or d.get("source") != "live_ros_m18_imu"
                or d.get("state") != "VERIFIED_STATIONARY_CANDIDATE"
                or d.get("owner_verified") is not True
                or d.get("motor_motion_authorized") is not False
                or d.get("physical_motion_authorized") is not False
                or d.get("physical_heading_accuracy_validated") is not False
                or d.get("absolute_heading_validated") is not False
                or d.get("camera_imu_extrinsics_validated") is not False
                or type(d.get("samples")) is not int
                or d.get("samples") < 350
                or type(d.get("window_elapsed_s")) not in (float, int)
                or not math.isfinite(d["window_elapsed_s"])
                or d["window_elapsed_s"] < 8.0):
            return None
        stamp = d.get("monotonic_status_ns")
        if type(stamp) is not int or not 0 <= now_ns - stamp <= 350_000_000:
            return None
        keys = (
            "independent_bias_rad_s", "heading_bias_rad_s",
            "bias_disagreement_rad_s", "split_bias_delta_rad_s",
            "gyro_z_noise_std_rad_s", "yaw_drift_deg")
        if any(type(d.get(k)) not in (int, float)
               or not math.isfinite(d[k]) for k in keys):
            return None
        return {
            "samples": d["samples"],
            "window_elapsed_s": d["window_elapsed_s"],
            "gyro_z_noise_std_rad_s": d["gyro_z_noise_std_rad_s"],
            "bias_disagreement_rad_s": d["bias_disagreement_rad_s"],
            "yaw_drift_deg": d["yaw_drift_deg"],
        }
    except (ValueError, TypeError, OverflowError):
        return None


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
            self.auto_imu_calibration = None
            self.auto_imu_calibration_at = None
            self._auto_cal_sub = self.create_subscription(
                String, AUTO_IMU, self.on_auto_calibration,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
            if self._auto_cal_sub.topic_name != AUTO_IMU:
                raise RuntimeError("BOOT_LIVE_AUTO_CALIBRATION_REMAP_DENIED")
            self._yaw_sub = self.create_subscription(
                String, YAW, self.on_yaw,
                QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE))
            if self._yaw_sub.topic_name != YAW:
                raise RuntimeError("BOOT_LIVE_YAW_REMAP_DENIED")
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

        def on_auto_calibration(self, msg):
            # Same already-verified yaw node, separate diagnostic publisher.
            pubs = self.get_publishers_info_by_topic(AUTO_IMU)
            self.auto_imu_calibration = None
            self.auto_imu_calibration_at = None
            if (len(pubs) != 1
                    or pubs[0].node_name != "leroy_live_boot_yaw"
                    or pubs[0].node_namespace != "/"
                    or pubs[0].topic_type != "std_msgs/msg/String"):
                return
            result = parse_auto_imu_candidate(
                msg.data, time.monotonic_ns())
            if result is not None:
                self.auto_imu_calibration = result
                self.auto_imu_calibration_at = time.monotonic()

        def on_yaw(self, msg):
            pubs = self.get_publishers_info_by_topic(YAW)
            if (len(pubs) != 1
                    or pubs[0].node_name != "leroy_live_boot_yaw"
                    or pubs[0].node_namespace != "/"
                    or pubs[0].topic_type != "std_msgs/msg/String"):
                return
            try:
                d = json.loads(msg.data)
                if type(d) is not dict:
                    return
                v = d.get("yaw_relative_rad")
                t = d.get("monotonic_status_ns")
                if type(t) is not int:
                    return
                age = time.monotonic_ns() - t
                if (d.get("schema") != "leroy_boot_yaw_v1"
                        or d.get("state") != "HEADING_ACTIVE"
                        or d.get("owner_verified") is not True
                        or d.get("motor_motion_authorized") is not False
                        or type(v) not in (int, float)
                        or not math.isfinite(v)
                        or type(t) is not int
                        or not 0 <= age <= 350_000_000):
                    return
                self.boot_yaw = float(v)
                self.boot_yaw_at = time.monotonic()
                self.boot_yaw_count += 1
            except (ValueError, TypeError):
                return

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
        if (node.boot_yaw_at is not None
                and now - node.boot_yaw_at > 0.35):
            node.boot_yaw = None
            node.boot_yaw_at = None
        if (node.auto_imu_calibration_at is not None
                and now - node.auto_imu_calibration_at > 0.35):
            node.auto_imu_calibration = None
            node.auto_imu_calibration_at = None
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
            yaw_at=node.boot_yaw_at,
            localization_at=node.received_at.get(STATUS),
            yaw_rad=node.boot_yaw,
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
            "boot_relative_yaw_samples": node.boot_yaw_count,
            "boot_relative_yaw_rad": node.boot_y,
            "auto_imu_stationary_candidate_observed": (
                node.auto_imu_calibration is not None),
            "auto_imu_stationary_candidate": node.auto_imu_calibration,
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
