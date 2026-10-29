#!/usr/bin/env python3
"""Observe a real manually executed [-90,+90,0] yaw sweep; no actuation.

For use while the persistent M18 gyro source and yaw ROS status are already
running. Every status comes from the sole expected live-yaw publisher.
The diagnostic proportional response is NEVER published or transmitted.
"""
import json
import math
import os
import time

from boot_yaw_tracking_core import Heading, YawTracking

TOPIC = "/leroy_boot/relative_yaw_status"
EXPECTED_OWNER = ("leroy_live_boot_yaw", "/", "std_msgs/msg/String")
REQUIRED_ENV = {
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    "LEROY_MANUAL_SWEEP_READ_ONLY": "1",
}
DEVICES = ("/dev/ttyACM0", "/dev/video2", "/dev/gpiochip0")
MAX_START_WAIT_S = 10.0
MAX_OPERATOR_WINDOW_S = 240.0


def preflight(env=None, exists=None):
    env = os.environ if env is None else env
    exists = os.path.exists if exists is None else exists
    for name, value in REQUIRED_ENV.items():
        if env.get(name) != value:
            raise RuntimeError("MANUAL_YAW_ENV_" + name)
    if any(exists(name) for name in DEVICES):
        raise RuntimeError("MANUAL_YAW_DEVICE_VISIBLE")
    return True


def read_status(raw, now_ns):
    """Decode only one fresh, authentic ROS status schema (not motion proof).

    Publisher ownership is checked separately against the real DDS graph;
    JSON fields alone are insufficient for an authenticated command path.
    """
    try:
        if type(raw) is not str or len(raw) > 4096:
            return None
        d = json.loads(raw)
        if (type(d) is not dict or d.get("schema") != "leroy_boot_yaw_v1"
                or d.get("source") != "live_ros_m18_imu"
                or d.get("state") != "HEADING_ACTIVE"
                or d.get("owner_verified") is not True
                or d.get("motor_motion_authorized") is not False
                or d.get("physical_heading_accuracy_validated") is not False):
            return None
        t = d.get("monotonic_status_ns")
        yaw = d.get("yaw_relative_rad")
        if (type(t) is not int
                or type(yaw) not in (float, int)
                or not math.isfinite(yaw)
                or not 0 <= now_ns - t <= 350_000_000):
            return None
        return Heading(
            stamp_ns=t, receipt_ns=now_ns,
            yaw_rad=float(yaw),
            source_active=True, owner_verified=True)
    except (ValueError, TypeError, OverflowError):
        return None


def operator_action(decision):
    """Human instruction only; never a motion command."""
    if decision.state == "DONE":
        return "SEQUENCE_COMPLETE"
    if decision.state in ("FAULT_LATCHED", "STOPPED"):
        return "STOP_NO_FURTHER_TURNS"
    if decision.state == "AWAITING_HEADING":
        return "WAIT_FOR_LIVE_HEADING"
    if decision.state == "SETTLING":
        return "HOLD_STILL_UNTIL_WAYPOINT_ADVANCES"
    if decision.error_rad is None:
        return "WAIT_FOR_LIVE_HEADING"
    if abs(decision.error_rad) <= math.radians(3.0):
        return "STOP_TURNING_AND_HOLD_STILL"
    if decision.error_rad < 0:
        return "TURN_CLOCKWISE_TO_TARGET"
    return "TURN_COUNTERCLOCKWISE_TO_TARGET"


def run():
    preflight()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (
        QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy)
    from std_msgs.msg import String

    class Observer(Node):
        def __init__(self):
            super().__init__(
                "leroy_manual_yaw_witness", enable_rosout=False,
                start_parameter_services=False)
            self.tracker = YawTracking()
            self.tracker.start()
            self.owner_was_valid = False
            self.last_output = None
            self._yaw_status_sub = self.create_subscription(
                String, TOPIC, self.on_yaw,
                QoSProfile(
                    history=HistoryPolicy.KEEP_LAST, depth=5,
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE))
            if self._yaw_status_sub.topic_name != TOPIC:
                raise RuntimeError("MANUAL_YAW_REMAP_FORBIDDEN")

        def owner_valid(self):
            if self.count_publishers("/cmd_vel") > 0:
                return False
            if any(name in ("leroy_bridge", "leroy_bridge_node")
                   for name, _ in self.get_node_names_and_namespaces()):
                return False
            pubs = self.get_publishers_info_by_topic(TOPIC)
            return (len(pubs) == 1 and
                    (pubs[0].node_name, pubs[0].node_namespace,
                     pubs[0].topic_type) == EXPECTED_OWNER)

        def on_yaw(self, msg):
            now_ns = time.monotonic_ns()
            if not self.owner_valid():
                if self.owner_was_valid:
                    self.tracker._fault("LIVE_YAW_PUBLISHER_OWNER_CHANGED")
                return
            self.owner_was_valid = True
            heading = read_status(msg.data, now_ns)
            if heading is None:
                self.tracker._fault("LIVE_YAW_STATUS_INVALID_OR_NOT_ACTIVE")
                return
            self.last_output = self.tracker.tick(heading, now_ns)

    rclpy.init(args=[])
    node = None
    try:
        node = Observer()
        started = time.monotonic()
        previous = None
        last_log = 0.0
        while time.monotonic() - started < MAX_OPERATOR_WINDOW_S:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if (node.tracker.state == "AWAITING_HEADING"
                    and now - started > MAX_START_WAIT_S):
                node.tracker._fault("LIVE_YAW_PUBLISHER_NOT_READY")
            if node.owner_was_valid and not node.owner_valid():
                node.tracker._fault("LIVE_YAW_PUBLISHER_OWNER_CHANGED")
            d = node.tracker.on_timeout(time.monotonic_ns())
            signature = (d.state, d.waypoint_index, d.reason)
            if signature != previous or now - last_log >= 2.0:
                # Relative witness yaw differs from the session-level IMU
                # yaw: the witness origin is captured on first fresh status.
                offset = (
                    node.tracker.last_yaw - node.tracker.origin
                    if node.tracker.origin is not None
                    and node.tracker.last_yaw is not None else None)
                target_offset = (
                    d.target_rad - node.tracker.origin
                    if d.target_rad is not None
                    and node.tracker.origin is not None else None)
                print("MANUAL_YAW_PROGRESS_JSON=" + json.dumps({
                    "state": d.state, "reason": d.reason,
                    "operator_action": operator_action(d),
                    "current_relative_deg": (
                        round(math.degrees(offset), 2)
                        if offset is not None else None),
                    "target_relative_deg": (
                        round(math.degrees(target_offset), 2)
                        if target_offset is not None else None),
                    "waypoint_index": d.waypoint_index,
                    "target_deg": (
                        round(math.degrees(d.target_rad), 2)
                        if d.target_rad is not None else None),
                    "error_deg": (
                        round(math.degrees(d.error_rad), 2)
                        if d.error_rad is not None else None),
                    "observed_yaw_rate_rad_s": d.observed_rate_rad_s,
                    "diagnostic_wz_rad_s": d.diagnostic_wz_rad_s,
                    "motor_motion_authorized": False,
                }, allow_nan=False, sort_keys=True), flush=True)
                previous = signature
                last_log = now
            if d.state in ("FAULT_LATCHED", "DONE"):
                break
        else:
            node.tracker._fault("MANUAL_SWEEP_OPERATOR_WINDOW_EXPIRED")
        final = node.tracker.decision()
        print("MANUAL_YAW_SWEEP=" + (
            "PASS_REAL_GYRO_WITNESS_NO_ACTUATION"
            if final.state == "DONE"
            else "BLOCKED_" + final.reason), flush=True)
        print("MANUAL_YAW_MOTOR=BLOCKED_NO_COMMAND_PATH", flush=True)
        return 0 if final.state == "DONE" else 2
    except KeyboardInterrupt:
        if node is not None:
            node.tracker.stop()
        print("MANUAL_YAW_SWEEP=STOPPED_BY_OPERATOR", flush=True)
        print("MANUAL_YAW_MOTOR=BLOCKED_NO_COMMAND_PATH", flush=True)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(run())
