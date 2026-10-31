#!/usr/bin/env bash
# Read ONE fresh yaw status from the actual isolated ROS domain, then exit.
# This command never opens /dev/ttyACM0, publishes ROS topics, or drives motors.
set -Eeuo pipefail
[[ $# -eq 0 ]] || { echo 'BOOT_YAW_SNAPSHOT=BLOCKED_UNEXPECTED_ARGS'; exit 2; }
command -v podman >/dev/null || { echo 'BOOT_YAW_SNAPSHOT=BLOCKED_NO_PODMAN'; exit 2; }
[[ "$(podman inspect -f '{{.State.Running}}' leroy-live-costmap 2>/dev/null)" == true ]] ||
  { echo 'BOOT_YAW_SNAPSHOT=BLOCKED_STAGE3_NOT_RUNNING'; exit 2; }

exec podman exec -i leroy-live-costmap bash -lc '
set -e
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
exec timeout --signal=INT --kill-after=2s 9s python3 -B -u -
' <<'PY'
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy)
from std_msgs.msg import String

TOPIC = "/leroy_boot/relative_yaw_status"
OWNER = ("leroy_live_boot_yaw", "/", "std_msgs/msg/String")
rclpy.init(args=[])
node = Node("leroy_one_shot_yaw_snapshot", enable_rosout=False,
            start_parameter_services=False)
received = []
qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE)
sub = node.create_subscription(
    String, TOPIC, lambda msg: received.append(msg.data), qos)
try:
    deadline = time.monotonic() + 6.0
    seen_owner = False
    while time.monotonic() < deadline:
        pubs = node.get_publishers_info_by_topic(TOPIC)
        valid_owner = (
            len(pubs) == 1 and
            (pubs[0].node_name, pubs[0].node_namespace,
             pubs[0].topic_type) == OWNER)
        if not valid_owner:
            received.clear()
        else:
            seen_owner = True
        rclpy.spin_once(node, timeout_sec=0.1)
        while received and valid_owner:
            raw = received.pop(0)
            try:
                data = json.loads(raw)
                if type(data) is not dict:
                    continue
                if (data.get("schema") != "leroy_boot_yaw_v1"
                        or data.get("source") != "live_ros_m18_imu"
                        or data.get("motor_motion_authorized") is not False):
                    continue
                t = data.get("monotonic_status_ns")
                if type(t) is not int:
                    continue
                age_ns = time.monotonic_ns() - t
                if not 0 <= age_ns <= 500_000_000:
                    continue
                state = data.get("state")
                if state != "HEADING_ACTIVE":
                    print("BOOT_YAW_SNAPSHOT=BLOCKED_" +
                          str(data.get("reason", state)), flush=True)
                    raise SystemExit(2)
                yaw = data.get("yaw_relative_deg")
                if (type(yaw) not in (int, float)
                        or not math.isfinite(yaw)
                        or data.get("owner_verified") is not True):
                    continue
                print("BOOT_YAW_SNAPSHOT_JSON=" + json.dumps({
                    "state": state,
                    "yaw_relative_deg": yaw,
                    "accepted_samples": data.get("accepted_samples"),
                    "gyro_z_bias_rad_s": data.get("gyro_z_bias_rad_s"),
                    "dynamic_accel_samples": data.get("dynamic_accel_samples"),
                    "last_accel_norm_m_s2": data.get("last_accel_norm_m_s2"),
                    "status_age_ms": round(age_ns / 1e6, 1),
                    "motor_motion_authorized": False,
                }, sort_keys=True), flush=True)
                print("BOOT_YAW_SNAPSHOT=PASS_FRESH_READ_ONLY", flush=True)
                raise SystemExit(0)
            except (ValueError, TypeError):
                continue
    print("BOOT_YAW_SNAPSHOT=BLOCKED_" +
          ("NO_FRESH_YAW_STATUS" if seen_owner
           else "YAW_PUBLISHER_NOT_PRESENT"), flush=True)
    raise SystemExit(2)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
PY
