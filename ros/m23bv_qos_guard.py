#!/usr/bin/env python3
"""Fail-closed, READ-ONLY ROS graph QoS audit; run BEFORE MCU serial open.

Requires live camera, rectifier, and rosbag recorder endpoints to match the
intended image transport. No subscriptions to payloads, device opens,
requests for actuation, or publishers. Import works without ROS for tests.
"""
import json
import sys
import time

IMAGE = "/camera/image_rect"
RAW = "/camera/image_raw"
INFO = "/camera/camera_info"


def _endpoints(graph, topic, role, node):
    rows = graph.get("topics", {}).get(topic, {}).get(role, [])
    return [e for e in rows if e.get("node_name") == node]


def _qos_ok(endpoint, *, reliability):
    q = endpoint.get("qos", {})
    # Depth may be UNKNOWN/0 in actual Jazzy endpoint inventories; don't
    # claim verified depth from an unknown. Reliability/durability are gates.
    return q.get("reliability") == reliability and q.get("durability") == "VOLATILE"


def validate_camera_graph(graph):
    if not str(graph.get("rmw_implementation", "")).startswith("rmw_"):
        raise ValueError("BLOCKED_UNKNOWN_RMW")
    exact = ((RAW, "publishers", "usb_cam"),
             (RAW, "subscriptions", "rectify_node"),
             (IMAGE, "publishers", "rectify_node"))
    for topic, role, name in exact:
        matches = _endpoints(graph, topic, role, name)
        if len(matches) != 1 or not _qos_ok(matches[0], reliability="RELIABLE"):
            raise ValueError("BLOCKED_RELIABLE_ENDPOINT:" + topic + ":" + role + ":" + name)
    # rosbag2's subscriber must actually request reliable, not merely have
    # a YAML file with reliable set. Other benign graph subscribers ignored.
    recorder = [e for e in graph.get("topics", {}).get(IMAGE, {}).get("subscriptions", [])
                if "rosbag2_recorder" in e.get("node_name", "")]
    if len(recorder) != 1 or not _qos_ok(recorder[0], reliability="RELIABLE"):
        raise ValueError("BLOCKED_RECORDER_IMAGE_QOS")
    # CameraInfo may intentionally use best-effort subscriptions, but the
    # camera publishes reliable; check each known endpoint and compatibility.
    info_pub = _endpoints(graph, INFO, "publishers", "usb_cam")
    info_rect = _endpoints(graph, INFO, "subscriptions", "rectify_node")
    if len(info_pub) != 1 or not _qos_ok(info_pub[0], reliability="RELIABLE"):
        raise ValueError("BLOCKED_CAMERAINFO_PUBLISHER")
    if len(info_rect) != 1 or info_rect[0].get("qos", {}).get("reliability") not in (
            "RELIABLE", "BEST_EFFORT") or info_rect[0].get("qos", {}).get("durability") != "VOLATILE":
        raise ValueError("BLOCKED_CAMERAINFO_RECTIFIER")
    return True


def snapshot(node, rmw):
    def conv(info):
        q = info.qos_profile
        return {"node_name": info.node_name,
                "node_namespace": info.node_namespace,
                "qos": {"reliability": q.reliability.name,
                        "durability": q.durability.name,
                        "history": q.history.name, "depth": q.depth}}
    return {"rmw_implementation": rmw,
            "topics": {name: {
                "publishers": [conv(x) for x in node.get_publishers_info_by_topic(name)],
                "subscriptions": [conv(x) for x in node.get_subscriptions_info_by_topic(name)]}
                for name in (RAW, IMAGE, INFO)}}


def main():
    import rclpy
    from rclpy.utilities import get_rmw_implementation_identifier
    rclpy.init(args=[])
    node = rclpy.create_node("m23bv_camera_qos_guard",
                            start_parameter_services=False, enable_rosout=False)
    latest = None
    error = "BLOCKED_GRAPH_NOT_READY"
    try:
        for _ in range(16):
            rclpy.spin_once(node, timeout_sec=0.15)
            latest = snapshot(node, get_rmw_implementation_identifier())
            try:
                validate_camera_graph(latest)
                error = None
                break
            except ValueError as exc:
                error = str(exc)
            time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("M23BV_CAMERA_ENDPOINTS_JSON=" + json.dumps(latest, sort_keys=True), flush=True)
    print("M23BV_CAMERA_QOS_GATE=" + ("PASS" if error is None else error), flush=True)
    print("M23BV_CAMERA_GUARD_DEVICE_ACCESS=NONE", flush=True)
    return 0 if error is None else 2


if __name__ == "__main__":
    sys.exit(main())
