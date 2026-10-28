"""Stage 3: no-device audit of real XRSLAM, DepthART and Nav2 processes.

No fake sensor measurements, publishers, actuation, or navigation actions.
A ROS graph/lifecycle check is not localization or obstacle validation.
"""
import os
import time

import rclpy
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

REQUIRED = {
    "/leroy_xrslam",
    "/depthart",
    "/controller_server",
    "/planner_server",
    "/bt_navigator",
    "/lifecycle_manager_stage3_no_device",
}
LIFECYCLE = ("/controller_server", "/planner_server", "/bt_navigator")


def guard():
    for key, expected in {
        "M23BV_STAGE3_NO_DEVICE_APP_SMOKE": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }.items():
        if os.environ.get(key) != expected:
            raise RuntimeError("BLOCKED_ENV_" + key)
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/video2"):
        raise RuntimeError("BLOCKED_HARDWARE_DEVICE_VISIBLE")


def inspect():
    guard()
    rclpy.init()
    node = rclpy.create_node("stage3_fullstack_no_device_audit")
    statuses, localizations, odom_count = [], [], [0]
    latched = QoSProfile(depth=1,
                         reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subs = [
        node.create_subscription(
            String, "/xrslam/tracking_status",
            lambda msg: statuses.append(msg.data), latched),
        node.create_subscription(
            Bool, "/xrslam/localization_valid",
            lambda msg: localizations.append(msg.data), latched),
        node.create_subscription(
            Odometry, "/odom",
            lambda _msg: odom_count.__setitem__(0, odom_count[0] + 1), 10),
    ]
    clients = []
    try:
        deadline = time.monotonic() + 35.0
        discovered = set()
        stable_since = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.3)
            discovered = {
                ((namespace.rstrip("/") if namespace != "/" else "")
                 + "/" + name)
                for name, namespace in node.get_node_names_and_namespaces()
            }
            topics = {name for name, _ in node.get_topic_names_and_types()}
            if any(t in topics for t in ("/cmd_vel", "/cmd_vel_nav")):
                raise RuntimeError("BLOCKED_HARDWARE_COMMAND_TOPIC")
            if (any("leroy_bridge" in n for n in discovered)
                    or "/velocity_smoother" in discovered):
                raise RuntimeError("BLOCKED_BRIDGE_OR_COMMAND_FORWARDER")
            if odom_count[0]:
                raise RuntimeError("BLOCKED_UNVALIDATED_METRIC_ODOM")
            if REQUIRED.issubset(discovered) and statuses and localizations:
                if stable_since is None:
                    stable_since = time.monotonic()
                if time.monotonic() - stable_since > 3.0:
                    break
            else:
                stable_since = None
        else:
            raise RuntimeError(
                "BLOCKED_MISSING_APP_NODES_OR_XRSLAM_STATUS:"
                + repr({
                    "missing": sorted(REQUIRED - discovered),
                    "statuses": statuses[-5:],
                    "localization": localizations[-5:],
                }))
        if any(value for value in localizations):
            raise RuntimeError("BLOCKED_XRSLAM_VALID_WITHOUT_CALIBRATION")
        if any(value != "waiting_calibration" for value in statuses):
            raise RuntimeError("BLOCKED_UNEXPECTED_XRSLAM_STATE:" +
                               repr(statuses))
        print("STAGE3_XRSLAM_STATUS=waiting_calibration", flush=True)
        print("STAGE3_XRSLAM_LOCALIZATION_VALID=false", flush=True)
        for name in LIFECYCLE:
            client = node.create_client(GetState, name + "/get_state")
            clients.append(client)
            if not client.wait_for_service(timeout_sec=6.0):
                raise RuntimeError("BLOCKED_MISSING_LIFECYCLE_SERVICE:" + name)
            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=6.0)
            if not future.done() or future.result() is None:
                raise RuntimeError("BLOCKED_LIFECYCLE_CALL_FAILED:" + name)
            current = future.result().current_state
            if current.id != 1:
                raise RuntimeError(
                    "BLOCKED_NAV2_NOT_UNCONFIGURED:" + name + ":" +
                    str(current.id) + ":" + current.label)
            print("STAGE3_NAV2_LIFECYCLE_" +
                  name.lstrip("/") + "=unconfigured", flush=True)
        print("STAGE3_DEPTHART_NODE=DISCOVERED_NO_IMAGES", flush=True)
        print("STAGE3_APP_SMOKE=PASS_REAL_NODES_IN_ISOLATED_GRAPH", flush=True)
        print("STAGE3_APP_SMOKE_REAL_SENSOR_DATA=NONE", flush=True)
        print("STAGE3_APP_SMOKE_MOTION=DISABLED", flush=True)
    finally:
        for client in clients:
            node.destroy_client(client)
        for sub in subs:
            node.destroy_subscription(sub)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    inspect()
