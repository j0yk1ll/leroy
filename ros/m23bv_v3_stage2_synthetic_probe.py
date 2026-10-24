#!/usr/bin/env python3
"""Stage 2A ONLY: isolated real Jazzy DDS with synthetic camera and IMU.

Run via m23bv_v3_stage2_synthetic_preflight.sh in pinned immutable V3 image.
NEVER imports serial, maps devices, starts rosbag or uses physical M20 runner.
"""
import ast
import hashlib
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import textwrap
import time
import json

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import Image, CameraInfo, Imu

SOURCE = Path("/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py")
SOURCE_SHA = "a0ae75a62da39235b113e3d2c324efda5fcfc6e1"
CPP = Path("/opt/m23bv_reliable_rectifier_proto/ws/src/m23bv_reliable_rectifier/src/rectify.cpp")
CPP_SHA = "7e5d199e633f7cf0dac266a656ae70849df07f71"
CHECKER = Path("/opt/m23bv/m23bv_camera_ready_gate.py")
CHECKER_SHA = "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"
CASE = os.environ.get("M23BV_V3_STAGE2A_CASE")
CASES = ("healthy", "missing_imu", "imu_qos_reliable")
ALLOWED_TOPICS = {
    "/camera/image_raw", "/camera/camera_info", "/camera/image_rect",
    "/camera/image_raw/compressed", "/camera/image_raw/compressedDepth",
    "/camera/image_raw/theora", "/camera/image_raw/zstd",
    "/camera/image_rect/compressed", "/camera/image_rect/compressedDepth",
    "/camera/image_rect/theora", "/camera/image_rect/zstd",
    "/imu/data_raw", "/parameter_events", "/rosout",
}
ALLOWED_NODES = {
    ("/camera", "usb_cam"), ("/camera", "rectify_node"),
    ("/", "m23bv_camera_ready_gate"),
    ("/", "m23bv_stage2a_fake_imu"), ("/", "m23bv_stage2a_probe"),
}

FAKE_CAMERA = r'''import time
import signal
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
running = True
def stop(_signum, _frame):
    global running
    running = False
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
rclpy.init(args=[])
node = rclpy.create_node("usb_cam", namespace="/camera", enable_rosout=False,
                         start_parameter_services=False)
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                 reliability=ReliabilityPolicy.RELIABLE,
                 durability=DurabilityPolicy.VOLATILE)
raw = node.create_publisher(Image, "/camera/image_raw", qos)
info = node.create_publisher(CameraInfo, "/camera/camera_info", qos)
sent = 0
try:
    while running and rclpy.ok():
        stamp = time.time_ns()
        sec, nsec = divmod(stamp, 1000000000)
        msg = Image()
        msg.header.stamp.sec, msg.header.stamp.nanosec = sec, nsec
        msg.header.frame_id = "camera_optical_frame"
        msg.width, msg.height = 16, 16
        msg.encoding, msg.is_bigendian, msg.step = "mono8", 0, 16
        msg.data = bytes([sent % 180 + 30] * 256)
        ci = CameraInfo()
        ci.header = msg.header
        ci.width, ci.height = 16, 16
        ci.distortion_model = "plumb_bob"
        ci.d = [0.03, -0.01, 0.0, 0.0, 0.0]
        ci.k = [100.0, 0., 8., 0., 100., 8., 0., 0., 1.]
        ci.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
        ci.p = [100., 0., 8., 0., 0., 100., 8., 0., 0., 0., 1., 0.]
        info.publish(ci)
        raw.publish(msg)
        sent += 1
        time.sleep(1 / 30)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("M23BV_V3_STAGE2A_FAKE_CAMERA_STOP=PASS", flush=True)
'''

FAKE_IMU = r'''import os
import signal
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
running = True
def stop(_signum, _frame):
    global running
    running = False
signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
case = os.environ["M23BV_V3_STAGE2A_CASE"]
rclpy.init(args=[])
node = rclpy.create_node("m23bv_stage2a_fake_imu", enable_rosout=False,
                         start_parameter_services=False)
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                 reliability=(ReliabilityPolicy.RELIABLE if case == "imu_qos_reliable"
                              else ReliabilityPolicy.BEST_EFFORT),
                 durability=DurabilityPolicy.VOLATILE)
publisher = (None if case == "missing_imu" else
             node.create_publisher(Imu, "/imu/data_raw", qos))
try:
    while running and rclpy.ok():
        if publisher is not None:
            msg = Imu()
            stamp = time.time_ns()
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1000000000)
            msg.header.frame_id = "imu_link"
            msg.orientation_covariance[0] = -1.
            msg.angular_velocity.z = 0.001
            msg.linear_acceleration.z = 9.81
            publisher.publish(msg)
        time.sleep(1 / 50)
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    print("M23BV_V3_STAGE2A_FAKE_IMU_STOP=PASS", flush=True)
'''


def require(ok, why):
    if not ok:
        raise RuntimeError("BLOCKED_" + why)


def blob(payload):
    return hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()


def pinned(path, sha):
    require(path.is_file() and blob(path.read_bytes()) == sha,
            "IMMUTABLE_SOURCE_PIN:" + str(path))


def fixture(folder):
    # Copy only the exact AST-defined V3 rectifier/checker/lifecycle handlers.
    # NEVER import or execute the physical camera candidate launch itself.
    tree = ast.parse(SOURCE.read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == "_camera_only")
    assigns = {}
    funcs = {}
    returns = []
    for expr in fn.body:
        if isinstance(expr, ast.Assign) and len(expr.targets) == 1 and isinstance(expr.targets[0], ast.Name):
            assigns[expr.targets[0].id] = expr.value
        elif isinstance(expr, ast.FunctionDef):
            funcs[expr.name] = expr
        elif isinstance(expr, ast.Return):
            returns.append(expr.value)
    require({"camera", "rectifier", "ready"} <= set(assigns)
            and set(funcs) == {"ready_exited", "camera_exited"}
            and len(returns) == 1, "CANDIDATE_AST_LAYOUT")
    original = ast.unparse(returns[0])
    require(original.startswith("[") and original.endswith("]")
            and "target_action=ready" in original
            and "target_action=camera" in original
            and "return [rectifier]" in ast.unparse(funcs["ready_exited"]),
            "CANDIDATE_HANDLER_CONTRACT")
    cam = folder / "stage2_fake_camera.py"
    imu = folder / "stage2_fake_imu.py"
    cam.write_text(FAKE_CAMERA)
    imu.write_text(FAKE_IMU)
    callbacks = "\n".join(textwrap.indent(ast.unparse(funcs[name]), "    ")
                          for name in ("ready_exited", "camera_exited"))
    launch = (
        "import os,sys\nfrom launch import LaunchDescription\n"
        "from launch.actions import ExecuteProcess,RegisterEventHandler,EmitEvent\n"
        "from launch.event_handlers import OnProcessExit\n"
        "from launch.events import Shutdown\nfrom launch_ros.actions import Node\n"
        "ALT_OPENCV='/opt/m23aj-opencv/lib'\n"
        "def generate_launch_description():\n"
        f"    camera=ExecuteProcess(cmd=[sys.executable,'-u',{str(cam)!r}],"
        " name='stage2a_camera', output='screen')\n"
        f"    rectifier={ast.unparse(assigns['rectifier'])}\n"
        f"    ready={ast.unparse(assigns['ready'])}\n"
        f"    imu=ExecuteProcess(cmd=[sys.executable,'-u',{str(imu)!r}],"
        " name='stage2a_imu', output='screen')\n"
        + callbacks + "\n"
        + f"    return LaunchDescription({original[:-1]},imu])\n"
    )
    ast.parse(launch)
    location = folder / "stage2a_synthetic.launch.py"
    location.write_text(launch)
    print("M23BV_V3_STAGE2A_V3_PINNED_AST_FIXTURE=PASS", flush=True)
    return location


def qos(reliability):
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                      reliability=reliability, durability=DurabilityPolicy.VOLATILE)


def endpoints(node, topic, publisher, name, namespace):
    method = node.get_publishers_info_by_topic if publisher else node.get_subscriptions_info_by_topic
    return [x for x in method(topic)
            if x.node_name == name and x.node_namespace == namespace]


def correct_qos(ep, reliability):
    return (ep.qos_profile.reliability == reliability and
            ep.qos_profile.durability == DurabilityPolicy.VOLATILE)


def evaluate_graph(node):
    topics = {n for n, _types in node.get_topic_names_and_types()}
    nodes = {(ns, name) for name, ns in node.get_node_names_and_namespaces()}
    require(not (topics - ALLOWED_TOPICS),
            "UNKNOWN_TOPICS:" + repr(sorted(topics - ALLOWED_TOPICS)))
    require(not (nodes - ALLOWED_NODES),
            "UNKNOWN_NODES:" + repr(sorted(nodes - ALLOWED_NODES)))
    imu_pub = endpoints(node, "/imu/data_raw", True, "m23bv_stage2a_fake_imu", "/")
    if imu_pub and (len(imu_pub) != 1 or
                    not correct_qos(imu_pub[0], ReliabilityPolicy.BEST_EFFORT)):
        return "BLOCKED_IMU_OFFERED_QOS"
    expected = (
        ("/camera/image_raw", True, "usb_cam", "/camera", ReliabilityPolicy.RELIABLE),
        ("/camera/image_raw", False, "rectify_node", "/camera", ReliabilityPolicy.RELIABLE),
        ("/camera/camera_info", True, "usb_cam", "/camera", ReliabilityPolicy.RELIABLE),
        ("/camera/image_rect", True, "rectify_node", "/camera", ReliabilityPolicy.RELIABLE),
        ("/imu/data_raw", True, "m23bv_stage2a_fake_imu", "/", ReliabilityPolicy.BEST_EFFORT),
        ("/imu/data_raw", False, "m23bv_stage2a_probe", "/", ReliabilityPolicy.BEST_EFFORT),
    )
    if not {("/camera", "usb_cam"), ("/camera", "rectify_node"),
            ("/", "m23bv_stage2a_fake_imu")}.issubset(nodes):
        return "WAIT"
    for topic, pub, name, namespace, reliability in expected:
        eps = endpoints(node, topic, pub, name, namespace)
        if len(eps) != 1 or not correct_qos(eps[0], reliability):
            return "WAIT"
    ci = endpoints(node, "/camera/camera_info", False, "rectify_node", "/camera")
    if len(ci) != 1 or ci[0].qos_profile.reliability not in (
            ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT):
        return "WAIT"
    return "PASS"


def stamp(message):
    return message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec


def inspect_image(msg):
    require(msg.header.frame_id == "camera_optical_frame" and
            msg.width == 16 and msg.height == 16 and
            msg.encoding == "mono8" and msg.step == 16 and
            len(msg.data) == 256 and stamp(msg) > 0,
            "SYNTHETIC_IMAGE_LAYOUT")
    return stamp(msg)


def inspect_info(msg):
    require(msg.header.frame_id == "camera_optical_frame" and
            msg.width == 16 and msg.height == 16 and
            msg.distortion_model == "plumb_bob" and
            len(msg.k) == 9 and msg.k[0] == 100. and
            len(msg.d) == 5 and len(msg.p) == 12 and stamp(msg) > 0,
            "SYNTHETIC_CAMERA_INFO")
    return stamp(msg)


def inspect_imu(msg):
    require(msg.header.frame_id == "imu_link" and stamp(msg) > 0 and
            msg.orientation_covariance[0] == -1.0,
            "SYNTHETIC_IMU_LAYOUT")
    return stamp(msg)


def stop(proc):
    forced = False
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        rc = proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        forced = True
        proc.terminate()
        try:
            rc = proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()
            rc = proc.wait(timeout=4)
    return rc, forced


def main():
    require(CASE in CASES, "UNKNOWN_CASE")
    for key, value in {
        "ROS_DOMAIN_ID": "170",
        "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
        "LEROY_M20_SENSOR_ONLY": "1",
        "LEROY_NAV2_DRY_RUN": "1",
        "M23BV_CAMERA_READY_CANDIDATE": "1",
        "M23BV_RELIABLE_RECTIFIER_PROTO_V3": "1",
    }.items():
        require(os.environ.get(key) == value, "ISOLATION_ENV:" + key)
    for path, sha in ((SOURCE, SOURCE_SHA), (CPP, CPP_SHA), (CHECKER, CHECKER_SHA)):
        pinned(path, sha)
    require(not any(Path(s).exists() for s in
                    ("/dev/video2", "/dev/ttyACM0", "/dev/video0", "/dev/ttyACM1")),
            "DEVICE_VISIBLE")
    require(get_rmw_implementation_identifier() == "rmw_fastrtps_cpp",
            "RMW_DIFFERENT_FROM_STAGE1")
    print("M23BV_V3_STAGE2A_HARDWARE_DEVICE_ACCESS=NONE", flush=True)
    print("M23BV_V3_STAGE2A_CPP_AND_LAUNCH_PINS=PASS", flush=True)

    with tempfile.TemporaryDirectory(prefix="m23bv-stage2a-", dir="/tmp") as directory:
        folder = Path(directory)
        launch = fixture(folder)
        logpath = folder / "launch.log"
        proc = None
        rclpy.init(args=[])
        observer = rclpy.create_node("m23bv_stage2a_probe",
                                     enable_rosout=False, start_parameter_services=False)
        raw, info, rect, imus = {}, {}, {}, {}
        faults = []
        def keep(target, message, validator):
            try:
                st = validator(message)
                if st not in target:
                    target[st] = time.monotonic()
                if len(target) > 600:
                    for old in list(target)[:100]:
                        del target[old]
            except RuntimeError as exc:
                faults.append(str(exc))

        observer.create_subscription(Image, "/camera/image_raw",
                                     lambda m: keep(raw, m, inspect_image),
                                     qos(ReliabilityPolicy.RELIABLE))
        observer.create_subscription(CameraInfo, "/camera/camera_info",
                                     lambda m: keep(info, m, inspect_info),
                                     qos(ReliabilityPolicy.RELIABLE))
        observer.create_subscription(Image, "/camera/image_rect",
                                     lambda m: keep(rect, m, inspect_image),
                                     qos(ReliabilityPolicy.RELIABLE))
        observer.create_subscription(Imu, "/imu/data_raw",
                                     lambda m: keep(imus, m, inspect_imu),
                                     qos(ReliabilityPolicy.BEST_EFFORT))
        ok = False
        blocked_qos = False
        last_graph = "WAIT"
        rc = None
        forced = True
        output = ""
        try:
            with logpath.open("w+") as stream:
                proc = subprocess.Popen(["ros2", "launch", str(launch)],
                                        stdout=stream, stderr=subprocess.STDOUT,
                                        start_new_session=True)
                deadline = time.monotonic() + (28 if CASE == "healthy" else 13)
                while time.monotonic() < deadline:
                    require(proc.poll() is None, "LAUNCH_EARLY_EXIT")
                    rclpy.spin_once(observer, timeout_sec=0.005)
                    require(not faults, "INVALID_MESSAGE:" + repr(faults))
                    last_graph = evaluate_graph(observer)
                    if last_graph == "BLOCKED_IMU_OFFERED_QOS":
                        blocked_qos = True
                        break
                    joint = set(raw) & set(info) & set(rect)
                    if (CASE == "healthy" and last_graph == "PASS" and
                            len(joint) >= 100 and len(imus) >= 300):
                        t0 = max(raw[min(joint)], imus[min(imus)])
                        t1 = min(raw[max(joint)], imus[max(imus)])
                        if t1 - t0 >= 3.0:
                            print("M23BV_V3_STAGE2A_SYNTHETIC_GRAPH=" +
                                  json.dumps({
                                      "matched_image_stamps": len(joint),
                                      "imu_messages": len(imus),
                                      "receipt_overlap_seconds": round(t1-t0, 3),
                                      "raw_rectifier_subscriber": "RELIABLE/VOLATILE",
                                      "rect_image_publisher": "RELIABLE/VOLATILE",
                                      "imu_source_publisher": "BEST_EFFORT/VOLATILE",
                                      "rmw": get_rmw_implementation_identifier()
                                  }, sort_keys=True), flush=True)
                            ok = True
                            break
                if CASE == "imu_qos_reliable":
                    require(blocked_qos, "NEGATIVE_IMU_QOS_NOT_REJECTED")
                    print("M23BV_V3_STAGE2A_NEGATIVE_IMU_QOS=PASS", flush=True)
                elif CASE == "missing_imu":
                    require(not imus and last_graph != "PASS" and
                            len(set(raw) & set(info) & set(rect)) >= 30,
                            "NEGATIVE_MISSING_IMU_NOT_REJECTED")
                    print("M23BV_V3_STAGE2A_NEGATIVE_MISSING_IMU=PASS", flush=True)
                else:
                    require(ok, "HEALTHY_NO_QOS_FRAME_OR_OVERLAP")
                rc, forced = stop(proc)
                stream.flush()
                stream.seek(0)
                output = stream.read()
        finally:
            if proc is not None and proc.poll() is None:
                rc, forced = stop(proc)
            observer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        print("M23BV_V3_STAGE2A_LAUNCH_EXIT=" + str(rc), flush=True)
        require(rc == 0 and not forced, "LAUNCH_FORCED_OR_NONZERO")
        require("Traceback (most recent call last):" not in output and
                "process has died" not in output, "ABNORMAL_CHILD_EXIT")
        child_started = set(re.findall(r"process started with pid \[(\d+)\]", output))
        child_clean = set(re.findall(r"process has finished cleanly \[pid (\d+)\]", output))
        require(len(child_started) == 4 and child_clean == child_started,
                "CHILD_PROCESS_REAP_OR_START_COUNT:" +
                repr((sorted(child_started), sorted(child_clean))))
        require(not any(Path("/proc/" + pid).exists() for pid in child_started),
                "CHILD_PROCESS_REMAINS")
        require("M23BV_V3_STAGE2A_FAKE_CAMERA_STOP=PASS" in output and
                "M23BV_V3_STAGE2A_FAKE_IMU_STOP=PASS" in output,
                "SYNTHETIC_CHILD_STOP_MARKER")
        require("M23BV_CANDIDATE_CAMERA_READINESS=PASS" in output,
                "REAL_V3_READINESS_NOT_PASS")
        print("M23BV_V3_STAGE2A_CHILDREN_REAPED=PASS", flush=True)
        print("M23BV_V3_STAGE2A_CAMERA_AND_IMU=SYNTHETIC_ONLY", flush=True)
        print("M23BV_V3_STAGE2A_MCU_SERIAL_OR_ROSBAG=NOT_USED", flush=True)
        print("M23BV_V3_STAGE2A_CASE_" + CASE.upper() + "=PASS", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("M23BV_V3_STAGE2A_RESULT=BLOCKED:" +
              type(exc).__name__ + ":" + str(exc), file=sys.stderr, flush=True)
        sys.exit(2)
