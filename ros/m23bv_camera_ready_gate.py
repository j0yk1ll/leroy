#!/usr/bin/env python3
"""M23BV candidate ONLY: fail-closed physical camera DDS readiness gate.

Read-only ROS subscription process, no video/serial device open, no motion.
A zero exit means source data/QoS is ready to TRY the rectifier, NOT physical
capture approval, camera/IMU clock calibration, or any motor authorization.
"""
import json
import os
import sys
import time
from collections import OrderedDict

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

RAW = "/camera/image_raw"
INFO = "/camera/camera_info"
EXPECTED_ENV = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23BV_CAMERA_READY_CANDIDATE": "1",
}
WAIT_SECONDS = 15.0
PAIR_THRESHOLD = 10
MAX_PENDING = 64
VALID_ENCODINGS = {"mono8": 1, "rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4}


def stamp_key(msg):
    h = msg.header
    t = h.stamp
    if (type(t.sec) is not int or t.sec <= 0 or
            type(t.nanosec) is not int or not 0 <= t.nanosec < 1_000_000_000 or
            h.frame_id != "camera_optical_frame"):
        return None
    if (type(msg.width) is not int or type(msg.height) is not int or
            not 0 < msg.width <= 4096 or not 0 < msg.height <= 4096):
        return None
    return (t.sec * 1_000_000_000 + t.nanosec,
            h.frame_id, msg.width, msg.height)


def offered_reliable(node, topic):
    publishers = [
        ep for ep in node.get_publishers_info_by_topic(topic)
        if ep.node_name == "usb_cam" and ep.node_namespace == "/camera"
    ]
    return (len(publishers) == 1 and
            publishers[0].qos_profile.reliability == ReliabilityPolicy.RELIABLE and
            publishers[0].qos_profile.durability == DurabilityPolicy.VOLATILE)


def check_environment():
    missing = [k for k, expected in EXPECTED_ENV.items()
               if os.environ.get(k) != expected]
    if missing:
        raise RuntimeError("BLOCKED_CAMERA_READY_ISOLATION_ENV:" + ",".join(missing))


def run():
    check_environment()
    rclpy.init(args=[])
    node = rclpy.create_node("m23bv_camera_ready_gate",
                             start_parameter_services=False, enable_rosout=False)
    reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    images, infos = OrderedDict(), OrderedDict()
    matched = set()
    freshest = [0.0]

    def bounded_add(store, key):
        if key is None:
            return
        store[key] = time.monotonic()
        store.move_to_end(key)
        while len(store) > MAX_PENDING:
            store.popitem(last=False)
        if key in images and key in infos:
            matched.add(key)
            freshest[0] = time.monotonic()

    def receive_image(msg):
        key = stamp_key(msg)
        channels = VALID_ENCODINGS.get(msg.encoding)
        if (key is None or channels is None or
                type(msg.step) is not int or msg.step < msg.width * channels or
                len(msg.data) < msg.step * msg.height):
            return
        bounded_add(images, key)

    def receive_info(msg):
        key = stamp_key(msg)
        if (key is None or len(msg.p) != 12 or
                not (msg.p[0] > 0 and msg.p[5] > 0) or
                msg.roi.do_rectify):
            return
        bounded_add(infos, key)

    node.create_subscription(Image, RAW, receive_image, reliable)
    node.create_subscription(CameraInfo, INFO, receive_info, reliable)
    deadline = time.monotonic() + WAIT_SECONDS
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if (offered_reliable(node, RAW) and offered_reliable(node, INFO)
                    and len(matched) >= PAIR_THRESHOLD
                    and now - freshest[0] <= 1.0):
                print("M23BV_CANDIDATE_CAMERA_QOS=RELIABLE_VOLATILE", flush=True)
                print("M23BV_CANDIDATE_MATCHED_CAMERA_FRAMES=" + str(len(matched)), flush=True)
                print("M23BV_CANDIDATE_CAMERA_READINESS=PASS", flush=True)
                print("M23BV_READY_NOT_CAPTURE_AUTHORIZATION=TRUE", flush=True)
                return 0
        print("M23BV_CANDIDATE_CAMERA_READINESS=BLOCKED_TIMEOUT_OR_QOS", flush=True)
        return 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        raise SystemExit(run())
    except (RuntimeError, ValueError, OSError) as error:
        print("M23BV_CANDIDATE_CAMERA_READINESS=BLOCKED:" +
              type(error).__name__ + ":" + str(error), file=sys.stderr, flush=True)
        raise SystemExit(2)
