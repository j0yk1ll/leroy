#!/usr/bin/env python3
"""Read real M18 IMU on Stage 3 DDS; publish relative yaw status only.

This process does not own the MCU, create motion interfaces, accept motion
requests or grant an actuator session. It requires the separately controlled,
already accepted M18 IMU source in the isolated domain.
"""
import json
import math
import os
import time

from boot_yaw_core import BootYaw

IMU_TOPIC = "/imu/data_raw"
STATUS_TOPIC = "/leroy_boot/relative_yaw_status"
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
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu

    class YawNode(Node):
        def __init__(self):
            super().__init__(
                "leroy_live_boot_yaw", enable_rosout=False,
                start_parameter_services=False)
            self.heading = BootYaw()
            self._source_ever_verified = False
            self._owner_ok = False
            self._start_mono = time.monotonic()
            self._imu_sub = self.create_subscription(
                Imu, IMU_TOPIC, self.on_imu, qos_profile_sensor_data)
            self._status_pub = self.create_publisher(
                String, STATUS_TOPIC, 5)
            if (self._imu_sub.topic_name != IMU_TOPIC
                    or self._status_pub.topic_name != STATUS_TOPIC):
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
            self.heading.update(
                stamp_ns, time.monotonic_ns(),
                accel=(a.x, a.y, a.z), gyro=(g.x, g.y, g.z))

        def tick(self):
            self._owner_ok = self.owner_valid()
            if self._owner_ok:
                self._source_ever_verified = True
            elif self._source_ever_verified:
                self.heading.fault("IMU_SOURCE_LOST_OR_DUPLICATED")
            elif time.monotonic() - self._start_mono > 7.0:
                self.heading.fault("IMU_SOURCE_NOT_DISCOVERED")
            self.heading.check_freshness(time.monotonic_ns())
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
                "owner_verified": self._owner_ok,
                "physical_heading_accuracy_validated": False,
                "motor_motion_authorized": False,
                "monotonic_status_ns": time.monotonic_ns(),
            }
            msg = String()
            msg.data = json.dumps(status, allow_nan=False, sort_keys=True)
            self._status_pub.publish(msg)
            self.last_status = status

    rclpy.init(args=[])
    node = None
    try:
        node = YawNode()
        started = time.monotonic()
        while time.monotonic() - started < 25.0:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.heading.state == "FAULT_LATCHED":
                break
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
        return 0 if ok else 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
