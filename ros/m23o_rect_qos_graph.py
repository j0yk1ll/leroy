#!/usr/bin/env python3
"""M23O: read-only ROS graph QoS inventory for the isolated camera/rectifier.

The module is importable without ROS for offline contract tests. The live
entrypoint creates only a graph-inspection node; it never publishes,
subscribes to image payloads, opens devices or controls the robot.
"""
import json
import time

TOPICS = ("/camera/image_raw", "/camera/image_rect", "/camera/camera_info")
REQUIRED = (
    ("/camera/image_raw", "publishers", "usb_cam"),
    ("/camera/image_raw", "subscriptions", "rectify_node"),
    ("/camera/image_raw", "subscriptions", "m23k_cpp_header_counter"),
    ("/camera/image_rect", "publishers", "rectify_node"),
    ("/camera/image_rect", "subscriptions", "m23k_cpp_header_counter"),
    ("/camera/camera_info", "publishers", "usb_cam"),
    ("/camera/camera_info", "subscriptions", "rectify_node"),
)


def validate_graph(graph):
    """Fail closed on missing or nonreliable endpoints in this M23O probe."""
    if not isinstance(graph, dict):
        raise ValueError("graph must be a JSON object")
    rmw = graph.get("rmw_implementation")
    if not isinstance(rmw, str) or not rmw.startswith("rmw_"):
        raise ValueError("actual RMW implementation was not identified")
    topics = graph.get("topics")
    if not isinstance(topics, dict):
        raise ValueError("graph topics missing")
    for topic, role, node_name in REQUIRED:
        endpoints = topics.get(topic, {}).get(role, [])
        if not isinstance(endpoints, list):
            raise ValueError("invalid endpoint set: " + topic)
        matching = [e for e in endpoints if e.get("node_name") == node_name]
        if len(matching) != 1:
            raise ValueError("expected one endpoint: " + topic + " " + role +
                             " " + node_name)
        qos = matching[0].get("qos", {})
        if qos.get("reliability") != "RELIABLE":
            raise ValueError("nonreliable endpoint: " + topic + " " + node_name)
        if qos.get("durability") != "VOLATILE":
            raise ValueError("unexpected durability: " + topic + " " + node_name)
    return True


def capture_graph(node, rmw):
    def snapshot_endpoint(info):
        q = info.qos_profile
        return {
            "node_name": info.node_name,
            "node_namespace": info.node_namespace,
            "qos": {
                "reliability": q.reliability.name,
                "durability": q.durability.name,
                "history": q.history.name,
                "depth": q.depth,
            },
        }

    return {
        "rmw_implementation": rmw,
        "topics": {
            topic: {
                "publishers": [
                    snapshot_endpoint(e) for e in
                    node.get_publishers_info_by_topic(topic)
                ],
                "subscriptions": [
                    snapshot_endpoint(e) for e in
                    node.get_subscriptions_info_by_topic(topic)
                ],
            }
            for topic in TOPICS
        },
    }


def main():
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier

    rclpy.init()
    node = rclpy.create_node(
        "m23o_qos_graph_inspector",
        start_parameter_services=False,
        enable_rosout=False,
    )
    graph = None
    error = "no endpoint snapshot"
    try:
        for _ in range(12):
            rclpy.spin_once(node, timeout_sec=0.25)
            graph = capture_graph(node, get_rmw_implementation_identifier())
            try:
                validate_graph(graph)
                error = None
                break
            except ValueError as exc:
                error = str(exc)
            time.sleep(0.25)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("M23O_GRAPH_JSON=" + json.dumps(graph, sort_keys=True), flush=True)
    if error:
        print("M23O_GRAPH_VERDICT=BLOCKED:" + error, flush=True)
        return 2
    print("M23O_GRAPH_VERDICT=PASS", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
