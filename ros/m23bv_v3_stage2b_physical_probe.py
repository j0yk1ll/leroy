#!/usr/bin/env python3
"""M23BV Stage 2B: supervised V3 camera + unchanged M20 physical IMU.

Real MCU serial is motor-capable. Invoke ONLY using the separately gated
operator-host wrapper with motor rails physically disconnected.
NO recording, bridge, motion, Nav2 or XRSLAM. Never promoted to production.
"""
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from rclpy.utilities import get_rmw_implementation_identifier
import m23bv_v3_stage1_camera_observer as stage1

V3_LAUNCH = "/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py"
M20_OBSERVER = "/opt/m20/m20_combined_observer.py"
EXPECTED = {
    "ROS_DOMAIN_ID": "170",
    "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
    "LEROY_M20_SENSOR_ONLY": "1",
    "LEROY_NAV2_DRY_RUN": "1",
    "M23BV_CAMERA_READY_CANDIDATE": "1",
    "M23BV_RELIABLE_RECTIFIER_PROTO_V3": "1",
    "M23BV_V3_STAGE2B_SENSOR_ONLY": "YES",
}
PRE_NODES = {
    ("/camera", "usb_cam"), ("/camera", "rectify_node"),
    ("/", "m23bv_camera_ready_gate"), ("/", "m23bv_v3_stage2b_probe"),
}
PRE_REQUIRED = {
    ("/camera", "usb_cam"), ("/camera", "rectify_node"),
    ("/", "m23bv_v3_stage2b_probe"),
}
CAMERA_START_MAX_S = 38
M20_MAX_S = 155
M20_TERM_GRACE_S = 28
MIN_PRECAM_STAMPS = 90

def fail(reason):
    raise RuntimeError("BLOCKED_" + reason)

def git_blob(path):
    data = Path(path).read_bytes()
    return hashlib.sha1(b"blob " + str(len(data)).encode() + b"\0" + data).hexdigest()

def check_env():
    for name, expected in EXPECTED.items():
        if os.environ.get(name) != expected:
            fail("ENV_" + name)
    if get_rmw_implementation_identifier() != "rmw_fastrtps_cpp":
        fail("RMW_NOT_PINNED")
    # All source files inside the immutable V3 image or read-only host mount.
    for path, sha in (
        (V3_LAUNCH, "a0ae75a62da39235b113e3d2c324efda5fcfc6e1"),
        ("/opt/m20/m20_combined_observer.py", "5cf2f88cb2f79be80821888401b57a6393ee5e3b"),
        ("/opt/m20/m20_combined_contract.py", "66111d8da486dc1e0c78057c1948f5e1aad9a06e"),
        ("/opt/m18/m18_imu_stream_source.py", "3c5fb37ca291533e2702a9159f3e8db11b076d22"),
        ("/opt/m18/m18_imu_stream_contract.py", "443aa82066e88d50cb88859304e9197e3f459ee0"),
        ("/opt/m23bv/m23bv_camera_ready_gate.py", "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"),
        ("/opt/m23bv/m23bv_v3_stage1_camera_observer.py", "93320b985c5dda3fdf0d1f8def001ba06480e7b6"),
        ("/opt/m23bv_reliable_rectifier_proto/ws/src/m23bv_reliable_rectifier/src/rectify.cpp",
         "7e5d199e633f7cf0dac266a656ae70849df07f71"),
    ):
        if git_blob(path) != sha:
            fail("SOURCE_PIN:" + path)
    # Import M20 dependencies, without serial opening or sensor operation.
    import serial
    import m20_combined_observer
    from m18_imu_stream_source import send_only_whitelisted_serial_command
    from m18_imu_stream_contract import START, STOP, PORT
    if (PORT != "/dev/ttyACM0" or START != b"set_imu_stream_hz 50\r\n"
            or STOP != b"set_imu_stream_hz 0\r\n"):
        fail("MCU_COMMAND_ALLOWLIST")
    if not (serial and m20_combined_observer and send_only_whitelisted_serial_command):
        fail("M20_DEPENDENCIES")
    print("M23BV_V3_STAGE2B_RUNTIME_PINS=PASS", flush=True)

def qos():
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=40,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)

def graph_pre_serial(node):
    topics = {name for name, _ in node.get_topic_names_and_types()}
    nodes = {(ns, name) for name, ns in node.get_node_names_and_namespaces()}
    if topics - stage1.ALLOWED_TOPICS:
        fail("PRE_SERIAL_UNKNOWN_TOPICS:" + repr(sorted(topics - stage1.ALLOWED_TOPICS)))
    if nodes - PRE_NODES:
        fail("PRE_SERIAL_UNKNOWN_NODES:" + repr(sorted(nodes - PRE_NODES)))
    if not PRE_REQUIRED.issubset(nodes):
        return False
    for topic, pub, name, reliability in (
        ("/camera/image_raw", True, "usb_cam", ReliabilityPolicy.RELIABLE),
        ("/camera/image_raw", False, "rectify_node", ReliabilityPolicy.RELIABLE),
        ("/camera/camera_info", True, "usb_cam", ReliabilityPolicy.RELIABLE),
        ("/camera/image_rect", True, "rectify_node", ReliabilityPolicy.RELIABLE),
    ):
        eps = stage1.endpoints(node, topic, pub, name)
        if len(eps) != 1 or not stage1.qos_is(eps[0], reliability):
            return False
    ci_reader = stage1.endpoints(node, "/camera/camera_info", False, "rectify_node")
    if (len(ci_reader) != 1 or
            not stage1.qos_is(ci_reader[0], ci_reader[0].qos_profile.reliability) or
            ci_reader[0].qos_profile.reliability not in
            (ReliabilityPolicy.RELIABLE, ReliabilityPolicy.BEST_EFFORT)):
        return False
    return True

class Relay:
    def __init__(self, proc, prefix):
        self.lines = []
        self.proc = proc
        self.prefix = prefix
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()

    def read(self):
        for line in self.proc.stdout:
            self.lines.append(line.rstrip("\n"))
            if len(self.lines) > 1200:
                self.lines = self.lines[-900:]
            print(self.prefix + line, end="", flush=True)

    def join(self):
        self.thread.join(timeout=4)
        if self.thread.is_alive():
            fail(self.prefix + "STDOUT_NOT_CLOSED")

def stop_camera(proc):
    if proc is None:
        return False
    if proc.poll() is None:
        os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=16)
        return proc.returncode == 0
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
            proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait()
        return False

def run():
    check_env()
    camera_proc = m20_proc = None
    camera_relay = m20_relay = None
    camera_clean = False
    node = None
    complete = False
    error = ""
    pre_serial_pass = False
    m20_exited = False
    try:
        rclpy.init(args=[])
        node = rclpy.create_node("m23bv_v3_stage2b_probe",
                                 enable_rosout=False, start_parameter_services=False)
        expected_cal = stage1.calibration_from_file()
        raw, info, rect = set(), set(), set()
        errors = []
        def take(store, msg, check):
            try:
                store.add(check(msg))
                if len(store) > 800:
                    # Preserve bounded memory without frame buffers.
                    store.clear()
            except RuntimeError as exc:
                errors.append(str(exc))
        node.create_subscription(Image, "/camera/image_raw",
                                 lambda m: take(raw, m, stage1.valid_image), qos())
        def info_check(msg):
            k = stage1.valid_info(msg)
            if not stage1.info_matches_calibration(msg, expected_cal):
                fail("CAMERA_CALIBRATION_MISMATCH")
            return k
        node.create_subscription(CameraInfo, "/camera/camera_info",
                                 lambda m: take(info, m, info_check), qos())
        node.create_subscription(Image, "/camera/image_rect",
                                 lambda m: take(rect, m, stage1.valid_image), qos())

        camera_proc = subprocess.Popen(["ros2", "launch", V3_LAUNCH],
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT, text=True,
                                       bufsize=1, start_new_session=True)
        camera_relay = Relay(camera_proc, "")
        deadline = time.monotonic() + CAMERA_START_MAX_S
        while time.monotonic() < deadline:
            if camera_proc.poll() is not None:
                fail("CAMERA_LAUNCH_EARLY_EXIT")
            rclpy.spin_once(node, timeout_sec=0.06)
            if errors:
                fail("BAD_CAMERA_PAYLOAD:" + errors[0])
            graph_ok = graph_pre_serial(node)
            lines = camera_relay.lines
            ready = any("M23BV_CANDIDATE_CAMERA_READINESS=PASS" in s for s in lines)
            joint = raw & info & rect
            if ready and graph_ok and len(joint) >= MIN_PRECAM_STAMPS:
                pre_serial_pass = True
                print("M23BV_V3_STAGE2B_PRE_SERIAL_GRAPH_QOS=PASS", flush=True)
                print("M23BV_V3_STAGE2B_PRE_SERIAL_MATCHED_FRAMES=" +
                      str(len(joint)), flush=True)
                break
        if not pre_serial_pass:
            fail("PRE_SERIAL_CAMERA_QOS_OR_FRAMES_TIMEOUT")
        # Previous live guard has passed. MCU port is opened ONLY by the
        # exact unchanged M20 observer; it must send its fixed START/STOP ACK.
        print("M23BV_V3_STAGE2B_MCU_OBSERVER=STARTING_UNCHANGED_M20", flush=True)
        m20_proc = subprocess.Popen(["python3", "-u", M20_OBSERVER],
                                    stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT, text=True,
                                    bufsize=1, start_new_session=True)
        m20_relay = Relay(m20_proc, "")
        deadline = time.monotonic() + M20_MAX_S
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if errors:
                fail("CAMERA_MESSAGE_INVALID_DURING_M20:" + errors[0])
            if camera_proc.poll() is not None:
                fail("CAMERA_EXIT_DURING_M20")
            # M20 checks forbidden ROS publishers/nodes itself every 5s.
            if m20_proc.poll() is not None:
                m20_exited = True
                break
        if not m20_exited:
            fail("M20_EXCEEDED_155_SECOND_DEADLINE")
        m20_relay.join()
        lines = "\n".join(m20_relay.lines)
        for marker in (
            "M20_IMU_STREAM_START_ACK=PASS",
            "M20_IMU_STREAM_STOP_ACK=PASS",
            "M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT",
        ):
            if marker not in lines:
                fail("M20_MISSING_REQUIRED_EVIDENCE:" + marker)
        if m20_proc.returncode != 0:
            fail("M20_NONZERO:" + str(m20_proc.returncode))
        # The camera stayed alive until the exact original M20 STOP+ACK.
        complete = True
    except (RuntimeError, OSError, KeyboardInterrupt) as exc:
        error = str(exc)
    finally:
        # REQUIRED ordering: terminate/wait M20 first to allow STOP/ACK,
        # then camera. Do not SIGKILL a stuck M20; operator must power isolate
        # and treat MCU STOP state as unknown.
        if m20_proc is not None and m20_proc.poll() is None:
            m20_proc.send_signal(signal.SIGTERM)
            try:
                m20_proc.wait(timeout=M20_TERM_GRACE_S)
            except subprocess.TimeoutExpired:
                error = error + ";BLOCKED_M20_STOP_UNVERIFIED_MANUAL_POWER_ISOLATION"
                print("M23BV_V3_STAGE2B_MCU_STOP=UNVERIFIED", flush=True)
        if m20_proc is not None and m20_proc.poll() is not None and m20_relay is not None:
            m20_relay.join()
        if camera_proc is not None:
            camera_clean = stop_camera(camera_proc)
        if camera_relay is not None:
            camera_relay.join()
            output = "\n".join(camera_relay.lines)
            if "process has died" in output or "Traceback (most recent call last):" in output:
                camera_clean = False
            # A zero ros2 launch exit is insufficient: require exactly
            # the three known camera/readiness/rectifier children to have
            # exited cleanly, with no surviving child process IDs.
            started = set(re.findall(r"process started with pid \\[(\\d+)\\]", output))
            finished = set(re.findall(r"process has finished cleanly \\[pid (\\d+)\\]", output))
            if (len(started) != 3 or finished != started or
                    any(Path("/proc/" + pid).exists() for pid in started)):
                camera_clean = False
                print("M23BV_V3_STAGE2B_CAMERA_CHILD_REAP=BLOCKED", flush=True)
            else:
                print("M23BV_V3_STAGE2B_CAMERA_CHILD_REAP=PASS", flush=True)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("M23BV_V3_STAGE2B_CAMERA_LAUNCH_REAPED=" +
              ("PASS" if camera_clean else "BLOCKED"), flush=True)
    if (complete and camera_clean and not error and
            m20_proc is not None and m20_proc.returncode == 0):
        print("M23BV_V3_STAGE2B_RESULT=PASS", flush=True)
        print("M23BV_V3_STAGE2B_RECORDING_OR_MOTION=NOT_AUTHORIZED", flush=True)
        return 0
    print("M23BV_V3_STAGE2B_RESULT=BLOCKED:" + (error or "UNVERIFIED"),
          file=sys.stderr, flush=True)
    return 2

if __name__ == "__main__":
    try:
        sys.exit(run())
    except Exception as exc:
        print("M23BV_V3_STAGE2B_RESULT=BLOCKED_UNHANDLED:" +
              type(exc).__name__ + ":" + str(exc), file=sys.stderr, flush=True)
        sys.exit(2)
