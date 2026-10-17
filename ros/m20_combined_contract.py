"""M20 pure contract check for simultaneous actual camera + MCU IMU ROS.

Message existence, freshness, provenance and receive overlap are *not*
physical camera/IMU temporal calibration or extrinsics verification.
"""

DURATION_S = 120
MIN_IMU_MESSAGES = 5700
MIN_IMAGE_MESSAGES = 400
MIN_INFO_MESSAGES = 400
MIN_OVERLAP_S = 90.0

EXPECTED_PUBLISHERS = {
    "camera_image": {
        "node": "rectify_node", "namespace": "/camera",
        "type": "sensor_msgs/msg/Image",
    },
    "camera_info": {
        "node": "usb_cam", "namespace": "/camera",
        "type": "sensor_msgs/msg/CameraInfo",
    },
    "imu": {
        "node": "m18_imu_source", "namespace": "/",
        "type": "sensor_msgs/msg/Imu",
    },
}


def evaluate(*, completed, start_ack, stop_ack, source, observer,
             overlap_s, forbidden_graph_seen, observer_thread_error):
    """Return True only when all independently checked contracts pass."""
    if (not completed or not start_ack or not stop_ack
            or forbidden_graph_seen or observer_thread_error
            or overlap_s is None or overlap_s < MIN_OVERLAP_S):
        return False
    counts = source["counts"]
    if (counts["published_imu_messages"] < MIN_IMU_MESSAGES
            or source["terminal_fault"] is not None
            or source["clock_rate_estimate_ready"] is not True
            or source["measured_publication_rate_hz"] is None
            or not 48.0 <= source["measured_publication_rate_hz"] <= 52.0
            or any(counts[k] for k in (
                "malformed_imu_packets", "sequence_faults",
                "sequence_gap_samples", "timestamp_faults",
                "stale_or_future_withheld", "calibration_withheld",
                "invalid_ascii_lines", "unexpected_serial_lines",
            ))):
        return False
    topics = observer["topics"]
    for name, minimum, frame in (
        ("camera_image", MIN_IMAGE_MESSAGES, "camera_optical_frame"),
        ("camera_info", MIN_INFO_MESSAGES, "camera_optical_frame"),
        ("imu", MIN_IMU_MESSAGES - 100, "imu_link"),
    ):
        st = topics[name]
        if (st["accepted"] < minimum or st["invalid"] != 0
                or st["received"] != st["accepted"]
                or st["frame"] != frame
                or st["fresh_valid_receipt"] is not True
                or st["publishers"] != [EXPECTED_PUBLISHERS[name]]):
            return False
    if topics["camera_image"].get("encoding") != "rgb8":
        return False
    if topics["camera_image"].get("dimensions") != [640, 480]:
        return False
    if topics["camera_info"].get("dimensions") != [640, 480]:
        return False
    # Other M16 streams are intentionally absent, not readiness evidence.
    for name in (
        "depth_image", "xrslam_tracking", "xrslam_valid",
        "odom", "tf", "obstacle_cloud",
    ):
        if topics[name]["publisher_count"] or topics[name]["accepted"]:
            return False
    return True
