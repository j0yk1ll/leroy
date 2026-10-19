#!/usr/bin/env python3
"""M23P: read-only synthetic camera/rectifier DDS endpoint audit.

No publisher, subscriber, image processing, device open or robot action.
Source functions are importable without ROS for offline tests.
"""
import json
import time

from m23o_rect_qos_graph import capture_graph

REQUIRED = (
    ("/camera/image_raw", "publishers", "m23p_synthetic_calibrated_publisher"),
    ("/camera/image_raw", "subscriptions", "rectify_node"),
    ("/camera/image_raw", "subscriptions", "m23k_cpp_header_counter"),
    ("/camera/camera_info", "publishers", "m23p_synthetic_calibrated_publisher"),
    ("/camera/camera_info", "subscriptions", "rectify_node"),
    ("/camera/camera_info", "subscriptions", "m23k_cpp_header_counter"),
    ("/camera/image_rect", "publishers", "rectify_node"),
    ("/camera/image_rect", "subscriptions", "m23k_cpp_header_counter"),
)


def validate_synthetic_graph(graph):
    if not isinstance(graph, dict):
        raise ValueError("graph missing")
    if not str(graph.get("rmw_implementation", "")).startswith("rmw_"):
        raise ValueError("actual RMW identifier missing")
    topics = graph.get("topics")
    if not isinstance(topics, dict):
        raise ValueError("graph topic inventory missing")
    for topic, role, name in REQUIRED:
        node = topics.get(topic)
        if not isinstance(node, dict):
            raise ValueError("missing topic: " + topic)
        endpoints = node.get(role)
        if not isinstance(endpoints, list):
            raise ValueError("missing endpoints: " + topic)
        matches = [e for e in endpoints
                   if isinstance(e, dict) and e.get("node_name") == name]
        if len(matches) != 1:
            raise ValueError("missing/duplicate endpoint: " + topic + " " + name)
        qos = matches[0].get("qos", {})
        if (qos.get("reliability") != "RELIABLE"
                or qos.get("durability") != "VOLATILE"):
            raise ValueError("endpoint QoS is not reliable/volatile: " +
                             topic + " " + name)
    return True


def main():
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier

    rclpy.init()
    node = rclpy.create_node(
        "m23p_qos_graph_inspector",
        start_parameter_services=False, enable_rosout=False)
    graph = None
    error = "no graph snapshot"
    try:
        for _ in range(14):
            rclpy.spin_once(node, timeout_sec=0.2)
            graph = capture_graph(node, get_rmw_implementation_identifier())
            try:
                validate_synthetic_graph(graph)
                error = None
                break
            except ValueError as exc:
                error = str(exc)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("M23P_GRAPH_JSON=" + json.dumps(graph, sort_keys=True), flush=True)
    if error:
        print("M23P_GRAPH_VERDICT=BLOCKED:" + error, flush=True)
        return 2
    print("M23P_GRAPH_VERDICT=PASS", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
