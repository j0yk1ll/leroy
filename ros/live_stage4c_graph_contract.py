"""Pure Stage 4C capture graph policy; no ROS, hardware or I/O imports."""
from live_stage4c_capture_contract import CaptureError, METADATA_TOPIC, TOPICS

REQUIRED_NODES = {
    "usb_cam", "rectify_node", "depthart", "depth_obstacles",
    "costmap", "leroy_xrslam", "m18_imu_source",
}
FORBIDDEN_NODES = {
    "leroy_bridge", "leroy_bridge_node", "controller_server",
    "planner_server", "bt_navigator", "behavior_server",
    "velocity_smoother", "collision_monitor", "motor_bridge",
}
FORBIDDEN_TOPICS = {
    "/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed",
    "/motor/command", "/motor/enable", "/joint_commands",
}


def graph_ready(node_names, topic_names, publishers_by_topic, valid, map_valid):
    """Check the same graph ownership policy before accepting passive capture.

    publishers_by_topic is a mapping of topic to ROS-like publisher endpoint
    objects with topic_type, node_name and node_namespace attributes.
    A missing required node or publisher returns False while an unexpected
    publisher or forbidden graph raises CaptureError.
    """
    if set(node_names) & FORBIDDEN_NODES or set(topic_names) & FORBIDDEN_TOPICS:
        raise CaptureError("forbidden_motor_or_navigation_graph")
    if not REQUIRED_NODES <= set(node_names):
        return False
    for topic, (typ, _) in TOPICS.items():
        publishers = publishers_by_topic.get(topic, ())
        if len(publishers) != 1:
            return False
        pub = publishers[0]
        expected = "m18_imu_source" if topic in (
            "/imu/data_raw", METADATA_TOPIC) else None
        if pub.topic_type != typ or (
            expected and (pub.node_name != expected
                          or pub.node_namespace != "/")
        ):
            raise CaptureError("unexpected_publisher_" + topic)
    return valid is False and map_valid is False
