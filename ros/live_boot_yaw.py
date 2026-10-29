#!/usr/bin/env python3
"""Read real M18 IMU on Stage 3 DDS; publish relative yaw status only.

This process does not own the MCU, create motion interfaces, accept motion
requests or grant an actuator session. It requires the separately controlled,
already accepted M18 IMU source in the isolated domain.
"""

# TODO(AUTONOMY) [heading observation]: Replace the current standalone
# diagnostic result with a typed, fresh estimator status carrying measured
# uncertainty, continuity and calibration provenance for a boot supervisor.
# Cross-check IMU frame/clock alignment against actual camera and wheel data
# before allowing other modules to rely on yaw for metric localization.
# Acceptance: stale/duplicate IMU publishers, nonmonotonic time and sensor
# losses never yield a trusted heading or independent motion authorization.

import json
import math
import os
import time

from boot_yaw_core import BootYaw
from boot_auto_calibration_core import AutoImuCalibration

IMU_TOPIC = "/imu/data_raw"
STATUS_TOPIC = "/leroy_boot/relative_yaw_status"
CALIBRATION_TOPIC = "/leroy_boot/auto_imu_calibration_status"
EXPECTED_OWNER = ("m18_imu_source", "/", "sensor_msgs/msg/Imu")
REQUIRED_ENV = {
    "LEROY_BOOT_IMU_YAW_ONLY": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
}
FORBIDDEN_DEVICES = ("/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0")


def preflight(env=None, exists=None):
    env = os.environ if env is None else env
    exists = os.path.exists if exists is None else exists
    for key, value in REQUIRED_ENV.items():
        if env.get(key) != value:
            raise RuntimeError("BOOT_YAW_ENV_" + key)
    if any(exists(path) for path in FORBIDDEN_DEVICES):
        raise RuntimeError("BOOT_YAW_DEVICE_VISIBLE")
    return True


def run():
    preflight()
    persistent = os.environ.get("LEROY_BOOT_YAW_PERSISTENT", "0")
    if persistent not in ("0", "1"):
        raise RuntimeError("BOOT_YAW_PERSISTENT_MODE_INVALID")
    persistent = persistent == "1"
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
    )
    # A deeper BEST_EFFORT subscriber queue absorbs brief Stage 3 CPU
    # scheduling stalls without claiming any guaranteed DDS delivery.
    # On receipt, sensor age and core stamp continuity are still enforced.
    imu_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=32,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE)
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu

    class YawNode(Node):
        def __init__(self):
            super().__init__(
                "leroy_live_boot_yaw", enable_rosout=False,
                start_parameter_services=False)
            self.heading = BootYaw()
            # Automatically verify the stationary heading-bias estimate
            # against an independent 8s MCU gyro holdout. No actuator APIs.
            self.calibration = AutoImuCalibration()
            self._calibration_announced = False
            self._source_ever_verified = False
            self._owner_ok = False
            self._start_mono = time.monotonic()
            self._imu_sub = self.create_subscription(
                Imu, IMU_TOPIC, self.on_imu, imu_qos)
            self._status_pub = self.create_publisher(
                String, STATUS_TOPIC, 5)
            self._calibration_pub = self.create_publisher(
                String, CALIBRATION_TOPIC, 5)
            if (self._imu_sub.topic_name != IMU_TOPIC
                    or self._status_pub.topic_name != STATUS_TOPIC
                    or self._calibration_pub.topic_name != CALIBRATION_TOPIC):
                raise RuntimeError("BOOT_YAW_TOPIC_REMAP_DENIED")
            self._tick_timer = self.create_timer(0.2, self.tick)

        def owner_valid(self):
            if any(name in ("leroy_bridge", "leroy_bridge_node")
                   for name, ns in self.get_node_names_and_namespaces()):
                return False
            if self.count_publishers("/cmd_vel") > 0:
                return False
            publishers = self.get_publishers_info_by_topic(IMU_TOPIC)
            return (len(publishers) == 1
                    and (publishers[0].node_name,
                         publishers[0].node_namespace,
                         publishers[0].topic_type) == EXPECTED_OWNER)

        def on_imu(self, msg):
            # Don't accept unverified publisher contents as real MCU data.
            if not self._owner_ok or self.heading.state == "FAULT_LATCHED":
                return
            if msg.header.frame_id != "imu_link":
                self.heading.fault("WRONG_IMU_FRAME")
                return
            stamp_ns = (int(msg.header.stamp.sec) * 1_000_000_000
                        + int(msg.header.stamp.nanosec))
            age = (self.get_clock().now().nanoseconds - stamp_ns) * 1e-9
            if not math.isfinite(age) or not -0.1 <= age <= 0.50:
                self.heading.fault("IMU_HEADER_AGE_INVALID")
                return
            a, g = msg.linear_acceleration, msg.angular_velocity
            receipt_ns = time.monotonic_ns()
            acc = (a.x, a.y, a.z)
            gyro = (g.x, g.y, g.z)
            estimate = self.heading.update(
                stamp_ns, receipt_ns, accel=acc, gyro=gyro)
            if estimate.state == "FAULT_LATCHED":
                self.calibration.fault("HEADING_SOURCE_FAULT")
                return
            self.calibration.update(
                stamp_ns, receipt_ns, accel=acc, gyro=gyro,
                heading_state=estimate.state,
                heading_yaw_rad=estimate.relative_yaw_rad,
                heading_bias_rad_s=estimate.gyro_z_bias_rad_s)

        def tick(self):
            self._owner_ok = self.owner_valid()
            if self._owner_ok:
                self._source_ever_verified = True
            elif self._source_ever_verified:
                self.heading.fault("IMU_SOURCE_LOST_OR_DUPLICATED")
            elif time.monotonic() - self._start_mono > 7.0:
                self.heading.fault("IMU_SOURCE_NOT_DISCOVERED")
            clock_ns = time.monotonic_ns()
            self.heading.check_freshness(clock_ns)
            if self.heading.state == "FAULT_LATCHED":
                self.calibration.fault("HEADING_SOURCE_FAULT")
            self.calibration.on_timeout(clock_ns)
            e = self.heading.estimate()
            status = {
                "schema": "leroy_boot_yaw_v1",
                "source": "live_ros_m18_imu",
                "state": e.state,
                "reason": e.reason,
                "yaw_relative_rad": e.relative_yaw_rad,
                "yaw_relative_deg": (
                    math.degrees(e.relative_yaw_rad)
                    if e.relative_yaw_rad is not None else None),
                "gyro_z_bias_rad_s": e.gyro_z_bias_rad_s,
                "accepted_samples": e.accepted_samples,
                "calibration_samples": e.calibration_samples,
                "bias_window_restarts": e.bias_window_restarts,
                "last_stamp_delta_s": e.last_stamp_delta_s,
                "last_receipt_delta_s": e.last_receipt_delta_s,
                "last_accel_norm_m_s2": e.last_accel_norm_m_s2,
                "dynamic_accel_samples": e.dynamic_accel_samples,
                "owner_verified": self._owner_ok,
                "physical_heading_accuracy_validated": False,
                "motor_motion_authorized": False,
                "monotonic_status_ns": time.monotonic_ns(),
            }
            msg = String()
            msg.data = json.dumps(status, allow_nan=False, sort_keys=True)
            self._status_pub.publish(msg)
            self.last_status = status

            # Autocalibration is informational only. It does not set the
            # physical_heading_accuracy_validated field of heading status.
            from dataclasses import asdict
            calibration = asdict(self.calibration.status())
            calibration.update({
                "schema": "leroy_auto_imu_calibration_v1",
                "source": "live_ros_m18_imu",
                "monotonic_status_ns": clock_ns,
                "owner_verified": self._owner_ok,
                "sensor_input_kind": "physical_stm32_50hz",
                "absolute_heading_validated": False,
                "physical_motion_authorized": False,
            })
            candidate = String()
            candidate.data = json.dumps(
                calibration, allow_nan=False, sort_keys=True)
            self._calibration_pub.publish(candidate)
            if (not self._calibration_announced
                    and calibration["state"] ==
                    "VERIFIED_STATIONARY_CANDIDATE"):
                self._calibration_announced = True
                print(
                    "BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY",
                    flush=True)
                print("BOOT_AUTO_CALIBRATION_JSON=" + candidate.data,
                      flush=True)

    rclpy.init(args=[])
    node = None
    try:
        node = YawNode()
        started = time.monotonic()
        announced_heading = False
        interrupted = False
        try:
            while rclpy.ok() and (
                    persistent or time.monotonic() - started < 25.0):
                rclpy.spin_once(node, timeout_sec=0.05)
                if persistent and (
                        not announced_heading
                        and node.heading.state == "HEADING_ACTIVE"):
                    announced_heading = True
                    print("BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY",
                          flush=True)
                if node.heading.state == "FAULT_LATCHED":
                    break
        except KeyboardInterrupt:
            interrupted = True
        if rclpy.ok():
            node.tick()
        status = node.last_status
        print("BOOT_LIVE_YAW_JSON=" +
              json.dumps(status, allow_nan=False, sort_keys=True),
              flush=True)
        ok = (status["state"] == "HEADING_ACTIVE"
              and status["owner_verified"] is True
              and status["accepted_samples"] >= 100)
        print("BOOT_LIVE_YAW=" + (
            "PASS_RELATIVE_GYRO_HEADING_ONLY" if ok
            else "BLOCKED_" + status["reason"]), flush=True)
        print("BOOT_LIVE_MOTION=BLOCKED_NO_ACTUATION", flush=True)
        if persistent:
            if status["state"] == "FAULT_LATCHED":
                return 2
            return 0 if interrupted and announced_heading and ok else 2
        return 0 if ok else 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
