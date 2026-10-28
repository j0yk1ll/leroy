#!/usr/bin/env python3
"""Stage 3 NAV2 no-device lifecycle Configure -> Cleanup smoke.

NEVER activates lifecycle nodes; never publishes odom/TF, actions or
motor-compatible velocity; never creates simulated sensor inputs.
This is plugin discovery, not a navigation or safety acceptance test.
"""
import os
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState

NODES = ("/controller_server", "/planner_server", "/bt_navigator")
MOTOR_TOPICS = (
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed",
    "/diff_drive_controller/cmd_vel",
)
METRIC_OUTPUT_TOPICS = ("/odom", "/tf")


def guard_environment():
    required = {
        "M23BV_STAGE3_NAV2_CONFIGURE_NO_DEVICE": "1",
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    if any(os.environ.get(k) != val for k, val in required.items()):
        raise RuntimeError("BLOCKED_ISOLATED_NAV2_ENV")
    if os.path.exists("/dev/ttyACM0") or os.path.exists("/dev/video2"):
        raise RuntimeError("BLOCKED_HARDWARE_DEVICE_VISIBLE")


def guard_graph(node):
    names = {name for name, _ in node.get_node_names_and_namespaces()}
    if any("bridge" in name or "velocity_smoother" in name
           or "controller_manager" in name for name in names):
        raise RuntimeError("BLOCKED_BRIDGE_OR_ACTUATION_NODE")
    # Nav2 is allowed to SUBSCRIBE to /odom and /tf while configuring;
    # otherwise a real controller could not load. But there must be NO
    # fake metric publisher/TF authority in this inert graph.
    for topic in METRIC_OUTPUT_TOPICS:
        if node.get_publishers_info_by_topic(topic):
            raise RuntimeError("BLOCKED_FAKE_METRIC_PUBLISHER:" + topic)
    # Actuator-compatible command topics are prohibited even if only a
    # subscriber is present. A diagnostic proposed_cmd_vel publisher is
    # remapped to its unconsumed isolated name.
    for topic in MOTOR_TOPICS:
        if (node.get_publishers_info_by_topic(topic)
                or node.get_subscriptions_info_by_topic(topic)):
            raise RuntimeError("BLOCKED_MOTOR_COMPATIBLE_TOPIC:" + topic)
    return all(name.lstrip("/") in names for name in NODES)


def call(node, service, kind, request, timeout):
    client = node.create_client(kind, service)
    try:
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("MISSING_SERVICE:" + service)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError("SERVICE_TIMEOUT:" + service)
        return future.result()
    finally:
        node.destroy_client(client)


def state(node, name):
    return call(node, name + "/get_state",
                GetState, GetState.Request(), 12).current_state.id


def change(node, name, transition):
    request = ChangeState.Request()
    request.transition.id = transition
    return bool(call(node, name + "/change_state",
                     ChangeState, request, 22).success)


def main():
    guard_environment()
    rclpy.init()
    node = rclpy.create_node("stage3_nav2_configure_only_audit")
    configured = []
    error = None
    cleanup_faults = []
    try:
        until = time.monotonic() + 15
        while time.monotonic() < until:
            if guard_graph(node):
                break
            rclpy.spin_once(node, timeout_sec=0.2)
        else:
            raise RuntimeError("MISSING_NAV2_APPLICATIONS")
        for name in NODES:
            if state(node, name) != State.PRIMARY_STATE_UNCONFIGURED:
                raise RuntimeError("NOT_UNCONFIGURED_BEFORE_TEST:" + name)
        for name in NODES:
            guard_graph(node)
            # The ONLY forward transition is Configure (id=1).
            # No ACTIVATE, goals, synthetic odom or TF are ever sent.
            if not change(node, name, Transition.TRANSITION_CONFIGURE):
                raise RuntimeError("CONFIGURE_FAILED:" + name)
            configured.append(name)
            if state(node, name) != State.PRIMARY_STATE_INACTIVE:
                raise RuntimeError("CONFIGURE_NOT_INACTIVE:" + name)
            print("STAGE3_NAV2_CONFIGURE_" + name.lstrip("/") +
                  "=INACTIVE", flush=True)
        guard_graph(node)
        # Deliberately do not activate. An inactive controller cannot
        # execute FollowPath, even in this disconnected diagnostic graph.
    except Exception as exc:
        error = str(exc)
    finally:
        # If a transition succeeded or left a node inactive despite a
        # service error, still attempt cleanup, in reverse dependency order.
        for name in reversed(NODES):
            try:
                current = state(node, name)
                if current == State.PRIMARY_STATE_INACTIVE:
                    if not change(node, name, Transition.TRANSITION_CLEANUP):
                        raise RuntimeError("CLEANUP_FAILED")
                    current = state(node, name)
                if current != State.PRIMARY_STATE_UNCONFIGURED:
                    raise RuntimeError("NODE_NOT_UNCONFIGURED:" + str(current))
                print("STAGE3_NAV2_CLEANUP_" + name.lstrip("/") +
                      "=UNCONFIGURED", flush=True)
            except Exception as exc:
                cleanup_faults.append(name + ":" + str(exc))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if cleanup_faults:
        raise RuntimeError("BLOCKED_INCOMPLETE_NAV2_CLEANUP:" +
                           ";".join(cleanup_faults))
    if error is not None:
        raise RuntimeError("BLOCKED_NAV2_CONFIGURE:" + error)
    print("STAGE3_NAV2_CONFIGURE=PASS_REAL_SERVERS_CONFIGURE_CLEANUP_NO_ACTIVATION")
    print("STAGE3_NAV2_OMIT_ODOM_TF_GOALS=CONFIRMED")
    print("STAGE3_NAV2_SENSOR_ACCESS=NONE")
    print("STAGE3_NAV2_MOTION_AUTHORIZATION=NONE")


if __name__ == "__main__":
    main()
