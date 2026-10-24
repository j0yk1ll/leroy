#!/usr/bin/env python3
"""M23BV Stage 1: bounded, CAMERA-ONLY V3 physical observer.

It has no device API, M20 import, bag recorder, motor or motion publisher.
A separate operator-approved host wrapper is the only intended entrypoint.
"""
import json
import re
import os
import signal
import subprocess
import sys
import threading
import time

import yaml

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from rclpy.utilities import get_rmw_implementation_identifier

EXPECTED = {
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "M23BV_CAMERA_READY_CANDIDATE": "1",
    "M23BV_RELIABLE_RECTIFIER_PROTO_V3": "1",
}
LAUNCH = "/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py"
ALLOWED_TOPICS = {
    "/camera/image_raw", "/camera/camera_info", "/camera/image_rect",
    # Observed on the real usb_cam node: image_transport advertises these
    # exact optional raw-image transports. No wildcard or other ROS topics.
    "/camera/image_raw/compressed", "/camera/image_raw/compressedDepth",
    "/camera/image_raw/theora", "/camera/image_raw/zstd",
    "/parameter_events", "/rosout",
}
ALLOWED_NODES = {
    ("/camera", "usb_cam"), ("/camera", "rectify_node"),
    ("/", "m23bv_camera_ready_gate"), ("/", "m23bv_v3_stage1_observer"),
}
WIDTH, HEIGHT, FRAME_ID, ENCODING = 640, 480, "camera_optical_frame", "rgb8"
MAX_OBSERVE_SECONDS = 48
MIN_RECT_FRAMES = 90


def fail(reason):
    raise RuntimeError("BLOCKED_" + reason)


def key(msg):
    st = msg.header.stamp
    if (st.sec <= 0 or st.nanosec < 0 or st.nanosec >= 1_000_000_000 or
            msg.header.frame_id != FRAME_ID or
            msg.width != WIDTH or msg.height != HEIGHT):
        fail("STAMP_FRAME_OR_DIMENSION")
    return (st.sec, st.nanosec)


def valid_image(msg):
    k = key(msg)
    if (msg.encoding != ENCODING or msg.step < WIDTH * 3 or
            len(msg.data) < msg.step * HEIGHT):
        fail("IMAGE_ENCODING_STRIDE_OR_PAYLOAD")
    return k


def valid_info(msg):
    k = key(msg)
    if (msg.distortion_model != "plumb_bob" or len(msg.d) < 5 or
            len(msg.k) != 9 or len(msg.p) != 12 or
            msg.k[0] <= 0 or msg.k[4] <= 0 or
            msg.p[0] <= 0 or msg.p[5] <= 0 or msg.roi.do_rectify):
        fail("UNCALIBRATED_OR_UNEXPECTED_CAMERAINFO")
    return k



def calibration_from_file(path="/config/camera/front_camera.yaml"):
    """Validate pinned calibration payload; compare with real CameraInfo later."""
    try:
        with open(path, encoding="utf-8") as stream:
            calibration = yaml.safe_load(stream)
        if not isinstance(calibration, dict):
            fail("CALIBRATION_FORMAT")
        if (calibration.get("image_width") != WIDTH or
                calibration.get("image_height") != HEIGHT or
                calibration.get("camera_name") != "front_camera" or
                calibration.get("distortion_model") != "plumb_bob"):
            fail("CALIBRATION_IDENTITY_OR_SHAPE")
        arrays = {}
        for key, length in (("camera_matrix", 9), ("distortion_coefficients", 5),
                            ("rectification_matrix", 9), ("projection_matrix", 12)):
            values = calibration[key]["data"]
            if len(values) != length:
                fail("CALIBRATION_VECTOR_LENGTH:" + key)
            arrays[key] = tuple(float(x) for x in values)
        if arrays["camera_matrix"][0] <= 0 or arrays["camera_matrix"][4] <= 0:
            fail("CALIBRATION_FOCAL_LENGTH")
        return arrays
    except (OSError, KeyError, TypeError, ValueError) as exc:
        fail("CALIBRATION_READ_OR_PARSE:" + type(exc).__name__)


def info_matches_calibration(msg, expected):
    pairs = ((msg.k, expected["camera_matrix"]),
             (msg.d, expected["distortion_coefficients"]),
             (msg.r, expected["rectification_matrix"]),
             (msg.p, expected["projection_matrix"]))
    return all(len(actual) == len(reference) and
               all(abs(a - b) <= 1e-8 for a, b in zip(actual, reference))
               for actual, reference in pairs)

def qos_is(ep, reliability):
    qos = ep.qos_profile
    return qos.reliability == reliability and qos.durability == DurabilityPolicy.VOLATILE


def endpoints(node, topic, publisher, name):
    method = node.get_publishers_info_by_topic if publisher else node.get_subscriptions_info_by_topic
    return [ep for ep in method(topic)
            if ep.node_namespace == "/camera" and ep.node_name == name]


def check_graph(node):
    topics = {topic for topic, _types in node.get_topic_names_and_types()}
    nodes = {(namespace, name) for name, namespace in node.get_node_names_and_namespaces()}
    if topics - ALLOWED_TOPICS:
        fail("UNEXPECTED_TOPICS:" + ",".join(sorted(topics - ALLOWED_TOPICS)))
    if nodes - ALLOWED_NODES:
        fail("UNEXPECTED_NODES:" + repr(sorted(nodes - ALLOWED_NODES)))
    if not {("/camera", "usb_cam"), ("/camera", "rectify_node")}.issubset(nodes):
        return False
    for topic, pub, name, reliability in (
        ("/camera/image_raw", True, "usb_cam", ReliabilityPolicy.RELIABLE),
        ("/camera/image_raw", False, "rectify_node", ReliabilityPolicy.RELIABLE),
        ("/camera/camera_info", True, "usb_cam", ReliabilityPolicy.RELIABLE),
        ("/camera/image_rect", True, "rectify_node", ReliabilityPolicy.RELIABLE),
    ):
        eps = endpoints(node, topic, pub, name)
        if len(eps) != 1 or not qos_is(eps[0], reliability):
            return False
    info_sub = endpoints(node, "/camera/camera_info", False, "rectify_node")
    if len(info_sub) != 1 or not qos_is(info_sub[0], info_sub[0].qos_profile.reliability) or (
            info_sub[0].qos_profile.reliability not in
            (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT)):
        return False
    return True


def reap(proc):
    """Interrupt the isolated launch process group; escalate, never hang."""
    try:
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=10)
        return True
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=4)
        except (subprocess.TimeoutExpired, ProcessLookupError):
            try:
                os.killpg(proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            proc.wait()
        return False


def run():
    for k, required in EXPECTED.items():
        if os.environ.get(k) != required:
            fail("ISOLATION_ENV:" + k)
    if not os.path.isfile(LAUNCH):
        fail("V3_LAUNCH_MISSING")

    expected_cal = calibration_from_file()
    # No serial, recorder, Nav2, XRSLAM or actuation process is started.
    proc = subprocess.Popen(
        ["ros2", "launch", LAUNCH],
        start_new_session=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        text=True, bufsize=1,
    )
    marker = {"ready": False, "matched": False}
    abnormal_children = []
    def relay():
        for line in proc.stdout:
            print(line, end="", flush=True)
            if "M23BV_CANDIDATE_CAMERA_READINESS=PASS" in line:
                marker["ready"] = True
            match = re.search(r"M23BV_CANDIDATE_MATCHED_CAMERA_FRAMES=(\d+)", line)
            if match:
                marker["matched"] = int(match.group(1)) >= 10
            # ros2 launch can itself return 0 even if a managed camera or
            # rectifier process died abnormally during shutdown. Never call
            # that a clean Stage 1 child shutdown.
            if ("process has died" in line and
                    ("[usb_cam_node_exe-" in line or
                     "[m23bv_rectify_reliable_node-" in line)):
                abnormal_children.append(line.rstrip())
    relay_thread = threading.Thread(target=relay, daemon=True)
    relay_thread.start()
    node = None
    success = False
    reason = "BLOCKED_UNCLASSIFIED"
    try:
        rclpy.init(args=[])
        rmw_id = get_rmw_implementation_identifier()
        if not rmw_id.startswith("rmw_"):
            fail("UNKNOWN_RMW_IMPLEMENTATION")
        node = rclpy.create_node("m23bv_v3_stage1_observer",
                                  start_parameter_services=False, enable_rosout=False)
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        raw, info, rect = {}, {}, {}
        errors = []
        def remember(store, msg, check):
            try:
                k = check(msg)
                if k not in store:
                    store[k] = time.monotonic()
                # Small bounded stamp inventory; never store frame buffers.
                if len(store) > 200:
                    for old in list(store)[:50]:
                        del store[old]
            except RuntimeError as exc:
                errors.append(str(exc))
        node.create_subscription(Image, "/camera/image_raw",
                                 lambda m: remember(raw, m, valid_image), q)
        def inspect_info(msg):
            stamp = valid_info(msg)
            if not info_matches_calibration(msg, expected_cal):
                fail("CAMERAINFO_CALIBRATION_MISMATCH")
            return stamp
        node.create_subscription(CameraInfo, "/camera/camera_info",
                                 lambda m: remember(info, m, inspect_info), q)
        node.create_subscription(Image, "/camera/image_rect",
                                 lambda m: remember(rect, m, valid_image), q)
        deadline = time.monotonic() + MAX_OBSERVE_SECONDS
        stable_start = None
        graph_ok = False
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                fail("LAUNCH_EARLY_EXIT:" + str(proc.returncode))
            rclpy.spin_once(node, timeout_sec=0.1)
            if errors:
                fail("INVALID_PAYLOAD:" + errors[0])
            graph_ok = check_graph(node)
            if graph_ok and marker["ready"] and marker["matched"]:
                stamps = sorted(set(raw) & set(info) & set(rect))
                if len(stamps) >= MIN_RECT_FRAMES:
                    if stable_start is None:
                        stable_start = time.monotonic()
                    if time.monotonic() - stable_start >= 1.0:
                        sample = stamps[-MIN_RECT_FRAMES:]
                        span = (sample[-1][0] - sample[0][0]) + (
                            sample[-1][1] - sample[0][1]) / 1_000_000_000
                        gaps = [
                            (b[0] - a[0]) + (b[1] - a[1]) / 1_000_000_000
                            for a, b in zip(sample, sample[1:])
                        ]
                        rate = (MIN_RECT_FRAMES - 1) / span if span > 0 else 0
                        if not (10.0 <= rate <= 65.0) or max(gaps) > 0.25:
                            fail("UNEXPECTED_FRAME_RATE_OR_STAMPS")
                        print("M23BV_V3_STAGE1_GRAPH=" + json.dumps({
                            "rmw_implementation": rmw_id,
                            "raw_publisher": "usb_cam:RELIABLE/VOLATILE",
                            "raw_rectifier_reader": "rectify_node:RELIABLE/VOLATILE",
                            "rect_publisher": "rectify_node:RELIABLE/VOLATILE",
                            "matched_rect_frames": len(stamps),
                            "frame_dimensions": [WIDTH, HEIGHT],
                            "encoding": ENCODING, "sample_span_seconds": round(span, 3),
                            "observed_hz": round(rate, 2),
                            "max_stamp_gap_seconds": round(max(gaps), 3),
                        }, sort_keys=True), flush=True)
                        print("M23BV_V3_STAGE1_QOS=PASS", flush=True)
                        print("M23BV_V3_STAGE1_FRAMES=PASS", flush=True)
                        success = True
                        break
        if not success:
            fail("TIMEOUT_CAMERA_OR_QOS_OR_FRAMES")
    except (RuntimeError, KeyboardInterrupt) as exc:
        reason = str(exc) if isinstance(exc, RuntimeError) else "BLOCKED_INTERRUPT"
    finally:
        launch_reaped = reap(proc)
        # A clean SIGINT-managed launch must exit zero and close its output.
        # A nonzero launch exit or a surviving output reader cannot be PASS.
        print("M23BV_V3_STAGE1_LAUNCH_EXIT=" + str(proc.returncode), flush=True)
        relay_thread.join(timeout=2)
        launch_reaped = (launch_reaped and proc.returncode == 0
                         and not relay_thread.is_alive())
        if abnormal_children:
            print("M23BV_V3_STAGE1_CHILD_EXIT=BLOCKED_ABNORMAL", flush=True)
        launch_reaped = launch_reaped and not abnormal_children
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("M23BV_V3_STAGE1_LAUNCH_REAPED=" +
              ("PASS" if launch_reaped else "BLOCKED_LAUNCH_OR_CHILD_EXIT"), flush=True)
    if success and launch_reaped:
        print("M23BV_V3_STAGE1_RESULT=PASS", flush=True)
        print("M23BV_V3_STAGE1_CAPTURE_OR_MCU=NOT_AUTHORIZED", flush=True)
        return 0
    print("M23BV_V3_STAGE1_RESULT=" + reason, file=sys.stderr, flush=True)
    return 2


if __name__ == "__main__":
    try:
        sys.exit(run())
    except Exception as exc:
        print("M23BV_V3_STAGE1_RESULT=BLOCKED_UNHANDLED:" + repr(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
