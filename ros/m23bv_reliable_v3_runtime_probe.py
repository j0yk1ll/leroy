#!/usr/bin/env python3
"""V3 explicit-RELIABLE rectifier prototype Jazzy synthetic acceptance.

A temporary test-only launch substitutes only the physical camera Node with
an in-memory ROS 2 publisher. The ACTUAL candidate checker, rectifier Node,
and OnProcessExit event handlers are copied by AST from the pinned image.
No production or physical launch is ever imported/called/executed.
"""
import ast
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import textwrap
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import Image

from m23bv_qos_guard import RAW, IMAGE, INFO, _endpoints, snapshot

SOURCE = Path("/opt/m23bv/m23bv_reliable_rectifier_v3_candidate.launch.py")
EXPECTED_BLOB = "a0ae75a62da39235b113e3d2c324efda5fcfc6e1"
CPP_SOURCE = Path("/opt/m23bv_reliable_rectifier_proto/ws/src/m23bv_reliable_rectifier/src/rectify.cpp")
EXPECTED_CPP_BLOB = "7e5d199e633f7cf0dac266a656ae70849df07f71"
CHECKER = Path("/opt/m23bv/m23bv_camera_ready_gate.py")
EXPECTED_CHECKER_BLOB = "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"
CASES = ("healthy", "best_effort", "missing_info", "camera_exit")
DOMAIN = "170"
QOS_ORDER_MODES = ("as_pinned", "rectifier_first")

CAMERA_PROGRAM = r'''#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo

CASE = os.environ["M23BV_READY_TEST_CASE"]
def main():
    rclpy.init(args=[])
    n = rclpy.create_node("usb_cam", namespace="/camera",
                          enable_rosout=False, start_parameter_services=False)
    def qos(reliability):
        return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                          reliability=reliability, durability=DurabilityPolicy.VOLATILE)
    reliability = (ReliabilityPolicy.BEST_EFFORT
                   if CASE == "best_effort" else ReliabilityPolicy.RELIABLE)
    raw = n.create_publisher(Image, "/camera/image_raw", qos(reliability))
    info = (None if CASE == "missing_info" else
            n.create_publisher(CameraInfo, "/camera/camera_info",
                               qos(ReliabilityPolicy.RELIABLE)))
    print("M23BV_READY_TEST_CAMERA_STARTED=" + CASE, flush=True)
    sent = 0
    try:
        while rclpy.ok():
            if CASE == "camera_exit" and sent >= 4:
                print("M23BV_READY_TEST_CAMERA_EARLY_EXIT=PASS", flush=True)
                return 3
            ns = time.time_ns()
            sec, nanosec = divmod(ns, 1_000_000_000)
            msg = Image()
            msg.header.stamp.sec = sec
            msg.header.stamp.nanosec = nanosec
            msg.header.frame_id = "camera_optical_frame"
            msg.height, msg.width = 16, 16
            msg.encoding, msg.is_bigendian, msg.step = "mono8", 0, 16
            msg.data = bytes([sent % 256] * 256)
            if info is not None:
                ci = CameraInfo()
                ci.header = msg.header
                ci.height, ci.width = 16, 16
                ci.distortion_model = "plumb_bob"
                ci.d = [0.03, -0.01, 0., 0., 0.]
                ci.k = [100., 0., 8., 0., 100., 8., 0., 0., 0., 1.]
                ci.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
                ci.p = [100., 0., 8., 0., 0., 100., 8., 0., 0., 0., 1., 0.]
                info.publish(ci)
            raw.publish(msg)
            sent += 1
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("M23BV_READY_TEST_CAMERA_STOP=PASS", flush=True)
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
'''


def git_blob(payload):
    return hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()


def create_test_launch(folder, qos_order_mode):
    if not SOURCE.is_file():
        raise RuntimeError("BLOCKED_CANDIDATE_SOURCE_ABSENT")
    payload = SOURCE.read_bytes()
    if git_blob(payload) != EXPECTED_BLOB:
        raise RuntimeError("BLOCKED_CANDIDATE_SOURCE_BLOB_MISMATCH")
    tree = ast.parse(payload, filename=str(SOURCE))
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == "_camera_only")
    assigns = {}
    funcs = {}
    returned = []
    for node in fn.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
            assigns[node.targets[0].id] = node.value
        elif isinstance(node, ast.FunctionDef):
            funcs[node.name] = node
        elif isinstance(node, ast.Return):
            returned.append(node.value)
    for name in ("camera", "rectifier", "ready"):
        if name not in assigns:
            raise RuntimeError("BLOCKED_CANDIDATE_MISSING_NODE:" + name)
    if set(funcs) != {"ready_exited", "camera_exited"} or len(returned) != 1:
        raise RuntimeError("BLOCKED_CANDIDATE_EVENT_HANDLERS")
    rectifier = ast.unparse(assigns["rectifier"])
    checker = ast.unparse(assigns["ready"])
    handlers = {name: ast.unparse(funcs[name])
                for name in ("ready_exited", "camera_exited")}
    if qos_order_mode == "as_pinned":
        print("M23BV_V3_CANDIDATE_HANDLER_EXACT=TRUE", flush=True)
    elif qos_order_mode == "rectifier_first":
        # Diagnostic ONLY, camera is launched after the rectifier's
        # downstream consumer has been present for four seconds.
        # No actual candidate readiness-handler acceptance may be claimed.
        print("M23BV_V3_DIAGNOSTIC_EARLY_RECTIFIER=TRUE", flush=True)
        print("M23BV_V3_CANDIDATE_HANDLER_EXACT=FALSE", flush=True)
    else:
        raise RuntimeError("BLOCKED_V3_STARTUP_MODE")
    callbacks = "\n".join(textwrap.indent(handlers[name], "    ")
                          for name in ("ready_exited", "camera_exited"))
    returned_expr = ast.unparse(returned[0])
    if ("OnProcessExit" not in returned_expr or
            "target_action=ready" not in returned_expr or
            "target_action=camera" not in returned_expr or
            "return [rectifier]" not in ast.unparse(funcs["ready_exited"])):
        raise RuntimeError("BLOCKED_CANDIDATE_SOURCE_HANDLER_CHANGED")
    if qos_order_mode == "rectifier_first":
        # This modifies ONLY a disposable synthetic launch. A graph sample
        # must prove the RELIABLE raw reader existed before any source.
        returned_expr = "[rectifier, TimerAction(period=4.0, actions=[camera])]"
    camera_path = folder / "m23bv_fake_cam.py"
    camera_path.write_text(CAMERA_PROGRAM)
    launch_script = (
        "import os, sys\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import ExecuteProcess, RegisterEventHandler, EmitEvent, TimerAction\n"
        "from launch.event_handlers import OnProcessExit\n"
        "from launch.events import Shutdown\n"
        "from launch_ros.actions import Node\n"
        "ALT_OPENCV = '/opt/m23aj-opencv/lib'\n"
        "def generate_launch_description():\n"
        f"    camera = ExecuteProcess(cmd=[sys.executable, '-u', {str(camera_path)!r}], "
        "name='m23bv_test_camera', output='screen')\n"
        f"    rectifier = {rectifier}\n"
        f"    ready = {checker}\n"
        + callbacks + "\n"
        # The original _camera_only() callback correctly returns a list to
        # OpaqueFunction. This synthetic launch generates a TOP-LEVEL
        # generate_launch_description(): it must return LaunchDescription.
        f"    return LaunchDescription({returned_expr})\n"
    )
    script_path = folder / "m23bv_ready_candidate_synthetic.launch.py"
    parsed_launch = ast.parse(launch_script, filename=str(script_path))
    generated = next(n for n in parsed_launch.body
                     if isinstance(n, ast.FunctionDef)
                     and n.name == "generate_launch_description")
    terminal = generated.body[-1]
    if (not isinstance(terminal, ast.Return) or
            not isinstance(terminal.value, ast.Call) or
            not isinstance(terminal.value.func, ast.Name) or
            terminal.value.func.id != "LaunchDescription" or
            len(terminal.value.args) != 1 or
            not isinstance(terminal.value.args[0], ast.List)):
        raise RuntimeError("BLOCKED_INVALID_SYNTHETIC_LAUNCH_RETURN_TYPE")
    print("M23BV_CANDIDATE_SYNTHETIC_LAUNCH_DESCRIPTION=PASS", flush=True)
    script_path.write_text(launch_script)
    if qos_order_mode == "as_pinned":
        print("M23BV_CANDIDATE_EVENT_HANDLERS_FROM_PINNED_SOURCE=PASS", flush=True)
    else:
        print("M23BV_V3_EARLY_RECTIFIER_HANDLER=TEST_ONLY", flush=True)
    print("M23BV_CANDIDATE_RECTIFIER_FROM_PINNED_SOURCE=PASS", flush=True)
    return script_path


def rect_graph(node):
    graph = snapshot(node, get_rmw_implementation_identifier())
    for topic, role, name, reliabilities in (
        (RAW, "publishers", "usb_cam", ("RELIABLE",)),
        (RAW, "subscriptions", "rectify_node", ("RELIABLE",)),
        (IMAGE, "publishers", "rectify_node", ("RELIABLE",)),
        (INFO, "publishers", "usb_cam", ("RELIABLE",)),
        (INFO, "subscriptions", "rectify_node", ("RELIABLE", "BEST_EFFORT")),
    ):
        endpoints = _endpoints(graph, topic, role, name)
        if len(endpoints) != 1:
            return False
        qos = endpoints[0]["qos"]
        if qos["reliability"] not in reliabilities or qos["durability"] != "VOLATILE":
            return False
    return True


def stop_parent(proc):
    if proc.poll() is None:
        proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=24)
    except subprocess.TimeoutExpired as exc:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        raise RuntimeError("BLOCKED_TEST_LAUNCH_PARENT_TIMEOUT") from exc


def main():
    case = os.environ.get("M23BV_READY_TEST_CASE")
    if case not in CASES:
        raise RuntimeError("BLOCKED_UNSUPPORTED_TEST_CASE")
    qos_order_mode = os.environ.get("M23BV_V3_STARTUP_MODE", "as_pinned")
    if qos_order_mode not in QOS_ORDER_MODES:
        raise RuntimeError("BLOCKED_QOS_ORDER_MODE")
    if qos_order_mode != "as_pinned" and case != "healthy":
        raise RuntimeError("BLOCKED_QOS_ORDER_NEGATIVE_MODE")
    print("M23BV_V3_STARTUP_MODE=" + qos_order_mode, flush=True)
    required = {"ROS_DOMAIN_ID": DOMAIN,
                "ROS_AUTOMATIC_DISCOVERY_RANGE": "LOCALHOST",
                "LEROY_M20_SENSOR_ONLY": "1",
                "LEROY_NAV2_DRY_RUN": "1",
                "M23BV_CAMERA_READY_CANDIDATE": "1",
                "M23BV_RELIABLE_RECTIFIER_PROTO_V3": "1"}
    if any(os.environ.get(k) != v for k, v in required.items()):
        raise RuntimeError("BLOCKED_TEST_ENV")
    if not CHECKER.is_file() or git_blob(CHECKER.read_bytes()) != EXPECTED_CHECKER_BLOB:
        raise RuntimeError("BLOCKED_SIGINT_V2_CHECKER_SOURCE_MISMATCH")
    print("M23BV_SIGINT_V2_CHECKER_SOURCE_PIN=PASS", flush=True)
    if not CPP_SOURCE.is_file() or git_blob(CPP_SOURCE.read_bytes()) != EXPECTED_CPP_BLOB:
        raise RuntimeError("BLOCKED_V3_COMPILED_SOURCE_PIN")
    print("M23BV_V3_CPP_SOURCE_PIN=PASS", flush=True)
    if any(Path(d).exists() for d in ("/dev/video2", "/dev/ttyACM0", "/dev/video0", "/dev/ttyACM1")):
        raise RuntimeError("BLOCKED_DEVICE_PRESENT")
    print("M23BV_READY_RUNTIME_TEST_STARTED=" + case, flush=True)
    with tempfile.TemporaryDirectory(prefix="m23bv-ready-test-", dir="/tmp") as scratch:
        scratch = Path(scratch)
        launch_path = create_test_launch(scratch, qos_order_mode)
        received = []
        rclpy.init(args=[])
        observer = rclpy.create_node("m23bv_ready_test_observer",
                                     start_parameter_services=False, enable_rosout=False)
        profile = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                             reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.VOLATILE)
        observer.create_subscription(
            Image, IMAGE, lambda m: received.append(m.header.stamp.sec * 1_000_000_000 +
                                                    m.header.stamp.nanosec), profile)
        proc = None
        out = ""
        log = scratch / "launch.log"
        try:
            with log.open("w+") as file:
                try:
                    proc = subprocess.Popen(
                        ["ros2", "launch", str(launch_path)],
                        stdout=file, stderr=subprocess.STDOUT)
                    end = time.monotonic() + (19 if case == "healthy" else 25)
                    success = False
                    pre_source_reader_reliable = False
                    while time.monotonic() < end:
                        rclpy.spin_once(observer, timeout_sec=0.04)
                        if qos_order_mode == "rectifier_first" and not pre_source_reader_reliable:
                            early_graph = snapshot(observer, get_rmw_implementation_identifier())
                            no_source = len(_endpoints(early_graph, RAW, "publishers", "usb_cam")) == 0
                            readers = _endpoints(early_graph, RAW, "subscriptions", "rectify_node")
                            if no_source and len(readers) == 1:
                                qos = readers[0]["qos"]
                                if qos["reliability"] != "RELIABLE" or qos["durability"] != "VOLATILE":
                                    raise RuntimeError("BLOCKED_V3_EARLY_READER_UNRELIABLE")
                                pre_source_reader_reliable = True
                                print("M23BV_V3_RELIABLE_BEFORE_CAMERA=PASS", flush=True)
                        if case == "healthy":
                            if proc.poll() is not None:
                                raise RuntimeError("BLOCKED_HEALTHY_EARLY_LAUNCH_EXIT")
                            if (len(received) >= 90 and rect_graph(observer)
                                    and (qos_order_mode != "rectifier_first"
                                         or pre_source_reader_reliable)):
                                success = True
                                break
                        elif proc.poll() is not None:
                            success = True
                            break
                    if not success:
                        # Fail CLOSED on the same original 19s healthy threshold.
                        # Report both independent predicates BEFORE teardown
                        # to distinguish missed frames from actual DDS QoS.
                        if case == "healthy":
                            graph = snapshot(observer, get_rmw_implementation_identifier())
                            graph_ok = rect_graph(observer)
                            print("M23BV_SIGINT_V2_HEALTHY_RECT_FRAMES=" +
                                  str(len(received)), flush=True)
                            print("M23BV_SIGINT_V2_HEALTHY_GRAPH_OK=" +
                                  ("TRUE" if graph_ok else "FALSE"), flush=True)
                            print("M23BV_SIGINT_V2_HEALTHY_GRAPH_JSON=" +
                                  json.dumps(graph, sort_keys=True), flush=True)
                            print("M23BV_SIGINT_V2_HEALTHY_LAUNCH_PARENT_ACTIVE=" +
                                  str(proc.poll() is None).upper(), flush=True)
                        raise RuntimeError("BLOCKED_EXPECTED_LAUNCH_RESULT_TIMEOUT")
                    parent_rc = stop_parent(proc)
                    print("M23BV_READY_TEST_PARENT_EXIT=" + str(parent_rc), flush=True)
                    if case == "healthy" and parent_rc != 0:
                        raise RuntimeError("BLOCKED_HEALTHY_PARENT_NONZERO")
                finally:
                    if proc is not None and proc.poll() is None:
                        stop_parent(proc)
                    file.flush()
                    file.seek(0)
                    out = file.read()
                    print("M23BV_READY_TEST_LAUNCH_LOG_BEGIN", flush=True)
                    print(out[-16000:], flush=True)
                    print("M23BV_READY_TEST_LAUNCH_LOG_END", flush=True)
        finally:
            observer.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        children = set(re.findall(r"process started with pid \[(\d+)\]", out))
        expected = 3 if case == "healthy" and qos_order_mode == "as_pinned" else 2
        if len(children) != expected:
            raise RuntimeError("BLOCKED_TEST_CHILD_START_COUNT:" + str((len(children), expected)))
        remaining = [p for p in children if Path("/proc/" + p).exists()]
        if remaining:
            raise RuntimeError("BLOCKED_TEST_CHILDREN_NOT_REAPED:" + repr(remaining))
        if "M23BV_READY_TEST_CAMERA_STOP=PASS" not in out:
            raise RuntimeError("BLOCKED_TEST_CAMERA_STOP_MISSING")
        if "Traceback (most recent call last):" in out:
            raise RuntimeError("BLOCKED_SIGINT_V2_READINESS_TRACEBACK")
        print("M23BV_SIGINT_V2_NO_TRACEBACK=PASS", flush=True)
        if case == "healthy":
            if qos_order_mode == "as_pinned":
                for marker in ("M23BV_CANDIDATE_CAMERA_READINESS=PASS",
                               "M23BV_CANDIDATE_CAMERA_QOS=RELIABLE_VOLATILE"):
                    if marker not in out:
                        raise RuntimeError("BLOCKED_HEALTHY_CHECKER_PASS_MISSING:" + marker)
            elif not pre_source_reader_reliable:
                raise RuntimeError("BLOCKED_V3_MISSING_PRE_CAMERA_RELIABLE_READER")
            print("M23BV_READY_TEST_RECTIFIED_FRAMES=" + str(len(received)), flush=True)
            print("M23BV_READY_TEST_ACTUAL_CANDIDATE_QOS=PASS", flush=True)
            if qos_order_mode == "as_pinned":
                print("M23BV_READY_TEST_HEALTHY=PASS", flush=True)
            else:
                print("M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY", flush=True)
        else:
            if re.search(r"\[rectify_node-\d+\]: process started with pid", out):
                raise RuntimeError("BLOCKED_RECTIFIER_STARTED_AFTER_FAILED_READINESS")
            if "M23BV_CANDIDATE_CAMERA_READINESS=PASS" in out:
                raise RuntimeError("BLOCKED_INVALID_CAMERA_ACCEPTED")
            if case == "camera_exit":
                if "M23BV_READY_TEST_CAMERA_EARLY_EXIT=PASS" not in out:
                    raise RuntimeError("BLOCKED_EARLY_EXIT_TEST_INCOMPLETE")
                if "M23BV_CANDIDATE_CAMERA_READINESS=INTERRUPTED_SIGINT" not in out:
                    raise RuntimeError("BLOCKED_SIGINT_V2_INTERRUPT_MARKER_MISSING")
                if not re.search(
                        r"\[m23bv_camera_ready-\d+\]: process has died "
                        r"\[pid \d+, exit code 130,", out):
                    raise RuntimeError("BLOCKED_SIGINT_V2_NONZERO_EXIT_130_MISSING")
                print("M23BV_SIGINT_V2_INTERRUPTED_NONZERO=PASS", flush=True)
            elif "M23BV_CANDIDATE_CAMERA_READINESS=BLOCKED_TIMEOUT_OR_QOS" not in out:
                raise RuntimeError("BLOCKED_MISSING_CHECKER_FAILURE_EVIDENCE")
            print("M23BV_READY_TEST_NEGATIVE_NO_RECTIFIER=PASS", flush=True)
            print("M23BV_READY_TEST_FAIL_CLOSED_" + case.upper() + "=PASS", flush=True)
        print("M23BV_READY_TEST_CHILDREN_REAPED=PASS", flush=True)
        if qos_order_mode == "as_pinned":
            print("M23BV_READY_CANDIDATE_RUNTIME_" + case.upper() + "=PASS", flush=True)
            print("M23BV_V3_CANDIDATE_RUNTIME_" +
                  case.upper() + "=PASS", flush=True)
        else:
            print("M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY", flush=True)
            print("M23BV_V3_AS_PINNED_ACCEPTANCE=NOT_ESTABLISHED_BY_DIAGNOSTIC", flush=True)
        print("M23BV_ACTUAL_DEVICE_CAMERA_OR_MCU=NOT_TESTED", flush=True)
        print("M23BV_PHYSICAL_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("M23BV_READY_CANDIDATE_RUNTIME=BLOCKED:" +
              type(exc).__name__ + ":" + str(exc), flush=True)
        sys.exit(2)
