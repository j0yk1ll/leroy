#!/usr/bin/env python3
"""Stage 3 live real-camera observer: passive DDS only, no commands/recording."""
import os
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, String


def validate_env():
    for name, required in {
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "M23BV_STAGE3_CAMERA_SIDECAR": "1",
    }.items():
        if os.environ.get(name) != required:
            raise RuntimeError("BLOCKED_ENV_" + name)
    if os.path.exists("/dev/video2") or os.path.exists("/dev/ttyACM0"):
        raise RuntimeError("BLOCKED_HARDWARE_IN_DIAGNOSTIC_SIDECAR")


def stamp(msg):
    s = msg.header.stamp
    if (s.sec <= 0 or s.nanosec < 0 or s.nanosec >= 1_000_000_000
            or msg.header.frame_id != "camera_optical_frame"):
        raise ValueError("invalid camera header")
    return s.sec * 1000000000 + s.nanosec


def main():
    validate_env()
    rclpy.init()
    node = rclpy.create_node("m23bv_stage3_live_camera_audit")
    seen = {name: set() for name in ("raw", "info", "rect", "depth")}
    counts = {name: 0 for name in seen}
    faults = []
    xr_state, xr_valid, odom_count = [], [], [0]

    reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def got(which, msg):
        try:
            st = stamp(msg)
            if msg.width != 640 or msg.height != 480:
                raise ValueError("unexpected image size")
            if which in ("raw", "rect"):
                if (msg.encoding != "rgb8" or msg.step < 640 * 3
                        or len(msg.data) < msg.step * 480):
                    raise ValueError("unexpected camera encoding/stride")
            elif which == "info":
                if len(msg.p) != 12 or msg.p[0] <= 0 or msg.p[5] <= 0:
                    raise ValueError("uncalibrated CameraInfo")
            else:
                if (msg.encoding != "32FC1" or msg.step < 640 * 4
                        or len(msg.data) < msg.step * 480):
                    raise ValueError("invalid DepthART image")
            counts[which] += 1
            seen[which].add(st)
            if len(seen[which]) > 2000:
                # A bounded collection of integer stamps, not image payload.
                seen[which] = set(sorted(seen[which])[-1000:])
        except Exception as exc:
            faults.append(which + ":" + str(exc))

    subscriptions = [
        node.create_subscription(
            Image, "/camera/image_raw", lambda m: got("raw", m), reliable),
        node.create_subscription(
            CameraInfo, "/camera/camera_info",
            lambda m: got("info", m), reliable),
        node.create_subscription(
            Image, "/camera/image_rect", lambda m: got("rect", m), reliable),
        node.create_subscription(
            Image, "/depthart/depth", lambda m: got("depth", m), best_effort),
        node.create_subscription(
            String, "/xrslam/tracking_status",
            lambda m: xr_state.append(m.data), latched),
        node.create_subscription(
            Bool, "/xrslam/localization_valid",
            lambda m: xr_valid.append(m.data), latched),
        node.create_subscription(
            Odometry, "/odom",
            lambda _m: odom_count.__setitem__(0, odom_count[0] + 1), reliable),
    ]
    services = []
    try:
        required = {
            "/camera/usb_cam", "/camera/rectify_node", "/depthart",
            "/leroy_xrslam", "/controller_server", "/planner_server",
            "/bt_navigator",
        }
        until = time.monotonic() + 115
        success_since = None
        while time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.15)
            if faults:
                raise RuntimeError("BLOCKED_SENSOR_PAYLOAD:" + faults[0])
            nodes = {
                ((ns.rstrip("/") if ns != "/" else "") + "/" + name)
                for name, ns in node.get_node_names_and_namespaces()
            }
            topics = {name for name, _ in node.get_topic_names_and_types()}
            if "/cmd_vel" in topics or "/cmd_vel_nav" in topics:
                raise RuntimeError("BLOCKED_MOTOR_COMPATIBLE_CMD_TOPIC")
            if any("leroy_bridge" in name or "velocity_smoother" in name
                   for name in nodes):
                raise RuntimeError("BLOCKED_BRIDGE_OR_SMOOTHER")
            if odom_count[0] != 0:
                raise RuntimeError("BLOCKED_UNVALIDATED_METRIC_ODOMETRY")
            if any(xr_valid):
                raise RuntimeError("BLOCKED_XRSLAM_VALIDATION_UNEXPECTED")
            if any(x != "waiting_calibration" for x in xr_state):
                raise RuntimeError("BLOCKED_XRSLAM_UNEXPECTED_STATE:" +
                                   repr(xr_state[-5:]))
            paired = seen["raw"] & seen["info"] & seen["rect"]
            depth_matches = seen["depth"] & seen["rect"]
            ready = (required.issubset(nodes)
                     and len(paired) >= 90
                     and len(depth_matches) >= 1
                     and xr_state and xr_valid and not xr_valid[-1])
            if ready:
                if success_since is None:
                    success_since = time.monotonic()
                if time.monotonic() - success_since >= 2.0:
                    break
            else:
                success_since = None
        else:
            raise RuntimeError(
                "BLOCKED_REAL_CAMERA_OR_DEPTH_TIMEOUT:" + repr({
                    "counts": counts,
                    "matched_camera": len(seen["raw"] & seen["info"] & seen["rect"]),
                    "matched_depth": len(seen["depth"] & seen["rect"]),
                    "missing_nodes": sorted(required - nodes),
                    "xr_state": xr_state[-3:],
                }))
        for name in ("controller_server", "planner_server", "bt_navigator"):
            service = node.create_client(GetState, "/" + name + "/get_state")
            services.append(service)
            if not service.wait_for_service(timeout_sec=6):
                raise RuntimeError("BLOCKED_LIFECYCLE_SERVICE_" + name)
            future = service.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=6)
            if not future.done() or future.result() is None:
                raise RuntimeError("BLOCKED_LIFECYCLE_QUERY_" + name)
            state = future.result().current_state
            if state.id != 1:
                raise RuntimeError(
                    "BLOCKED_NAV2_ACTIVE_" + name + ":" + state.label)
            print("STAGE3_REAL_CAMERA_NAV2_" + name + "=unconfigured", flush=True)
        print("STAGE3_REAL_CAMERA_COUNTS=" + repr(counts), flush=True)
        print("STAGE3_REAL_CAMERA_MATCHED_TRIPLE=" +
              str(len(seen["raw"] & seen["info"] & seen["rect"])), flush=True)
        print("STAGE3_REAL_CAMERA_MATCHED_DEPTH=" +
              str(len(seen["depth"] & seen["rect"])), flush=True)
        print("STAGE3_REAL_CAMERA_XRSLAM_STATUS=waiting_calibration", flush=True)
        print("STAGE3_REAL_CAMERA_ODOM_MESSAGES=0", flush=True)
        print("M23BV_STAGE3_REAL_CAMERA_AUDIT=PASS", flush=True)
    finally:
        for service in services:
            node.destroy_client(service)
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
