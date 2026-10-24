#!/usr/bin/env python3
"""No-device ROS 2 launch-managed camera/rectifier lifetime probe.

Constructs a disposable synthetic launch from the EXACT rectifier Node AST
in the pinned M23BV physical launch source, without executing that source's
camera/device setup or changing its physical precondition checks.
Two real launch-managed children: a synthetic ROS camera publisher and the
real Jazzy image_proc rectifier. Never opens any camera, serial port or MCU.
"""
import ast
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import get_rmw_implementation_identifier
from sensor_msgs.msg import Image

from m23bv_qos_guard import IMAGE, INFO, RAW, _endpoints, snapshot

CAMERA_SOURCE = Path("/opt/m23bv/m23bv_camera_only.launch.py")
DOMAIN = "179"
STARTUP_MODES = ("as_is", "camera_first_3s")

CAMERA_PROGRAM = r'''#!/usr/bin/env python3
import time
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo

def main():
    rclpy.init(args=[])
    node = rclpy.create_node("usb_cam", namespace="/camera",
                             start_parameter_services=False, enable_rosout=False)
    reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    raw = node.create_publisher(Image, "/camera/image_raw", reliable)
    meta = node.create_publisher(CameraInfo, "/camera/camera_info", reliable)
    print("M23BV_SYNTH_LAUNCH_CAMERA_STARTED=PASS", flush=True)
    count = 0
    try:
        while rclpy.ok():
            ns = time.time_ns()
            stamp_s, stamp_ns = divmod(ns, 1_000_000_000)
            image = Image()
            image.header.stamp.sec = stamp_s
            image.header.stamp.nanosec = stamp_ns
            image.header.frame_id = "camera_optical_frame"
            image.height = 16
            image.width = 16
            image.encoding = "mono8"
            image.step = 16
            image.data = bytes([count % 256] * 256)
            calibration = CameraInfo()
            calibration.header = image.header
            calibration.height = 16
            calibration.width = 16
            calibration.distortion_model = "plumb_bob"
            calibration.d = [0.03, -0.01, 0.0, 0.0, 0.0]
            calibration.k = [100., 0., 8., 0., 100., 8., 0., 0., 1.]
            calibration.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
            calibration.p = [100., 0., 8., 0., 0., 100., 8., 0., 0., 0., 1., 0.]
            meta.publish(calibration)
            raw.publish(image)
            count += 1
            time.sleep(1.0 / 30.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("M23BV_SYNTH_LAUNCH_CAMERA_STOP=PASS", flush=True)

if __name__ == "__main__":
    main()
'''


def rectifier_expression():
    tree = ast.parse(CAMERA_SOURCE.read_text(), filename=str(CAMERA_SOURCE))
    fun = next(n for n in tree.body if isinstance(n, ast.FunctionDef)
               and n.name == "_camera_only")
    entries = [n.value for n in fun.body
               if isinstance(n, ast.Assign)
               and len(n.targets) == 1
               and isinstance(n.targets[0], ast.Name)
               and n.targets[0].id == "rectifier"]
    if len(entries) != 1 or not isinstance(entries[0], ast.Call):
        raise RuntimeError("BLOCKED_RECTIFIER_SOURCE_AST")
    call = entries[0]
    if not isinstance(call.func, ast.Name) or call.func.id != "Node":
        raise RuntimeError("BLOCKED_RECTIFIER_NOT_STANDALONE_NODE")
    args = {k.arg: k.value for k in call.keywords}
    for name, required in (("package", "image_proc"), ("executable", "rectify_node"),
                           ("namespace", "camera"), ("name", "rectify_node")):
        if name not in args or not isinstance(args[name], ast.Constant) or args[name].value != required:
            raise RuntimeError("BLOCKED_RECTIFIER_SOURCE_CHANGED:" + name)
    for mandatory in ("parameters", "remappings", "additional_env"):
        if mandatory not in args:
            raise RuntimeError("BLOCKED_RECTIFIER_SOURCE_MISSING:" + mandatory)
    return ast.unparse(call)


def prepare_launch(folder):
    camera = folder / "synthetic_usb_cam.py"
    camera.write_text(CAMERA_PROGRAM)
    launch_path = folder / "m23bv_synthetic_tree.launch.py"
    rectifier = rectifier_expression()
    # The *unchanged physical* camera guard is deliberately NOT called:
    # this is a distinct synthetic launch, not a bypass for physical capture.
    startup = os.environ.get("M23BV_LAUNCH_STARTUP_MODE", "as_is")
    if startup not in STARTUP_MODES:
        raise RuntimeError("BLOCKED_INVALID_STARTUP_MODE")
    launch_src = (
        "import os, sys\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import ExecuteProcess, TimerAction\n"
        "from launch_ros.actions import Node\n"
        "ALT_OPENCV = '/opt/m23aj-opencv/lib'\n"
        "def generate_launch_description():\n"
        f"    camera = ExecuteProcess(cmd=[sys.executable, '-u', {str(camera)!r}], "
        "name='m23bv_synthetic_camera', output='screen')\n"
        f"    rectifier = {rectifier}\n"
        ("    return LaunchDescription([camera, rectifier])\n"
         if startup == "as_is" else
         "    return LaunchDescription([camera, TimerAction(period=3.0, actions=[rectifier])])\n")
    )
    ast.parse(launch_src, filename=str(launch_path))
    launch_path.write_text(launch_src)
    print("M23BV_LAUNCH_STARTUP_MODE=" + startup, flush=True)
    print("M23BV_PRODUCTION_LAUNCH_TIMING_REPRODUCED=" +
          ("TRUE" if startup == "as_is" else "FALSE"), flush=True)
    print("M23BV_RECTIFIER_LAUNCH_AST_FROM_PINNED_SOURCE=PASS", flush=True)
    return launch_path


def required_graph(node):
    graph = snapshot(node, get_rmw_implementation_identifier())
    for topic, role, name, reliabilities in (
        (RAW, "publishers", "usb_cam", ("RELIABLE",)),
        (RAW, "subscriptions", "rectify_node", ("RELIABLE",)),
        (IMAGE, "publishers", "rectify_node", ("RELIABLE",)),
        (INFO, "publishers", "usb_cam", ("RELIABLE",)),
        (INFO, "subscriptions", "rectify_node", ("RELIABLE", "BEST_EFFORT")),
    ):
        matches = _endpoints(graph, topic, role, name)
        if len(matches) != 1:
            raise ValueError("BLOCKED_LAUNCH_ENDPOINT:" + topic + role + name)
        qos = matches[0]["qos"]
        if qos["reliability"] not in reliabilities or qos["durability"] != "VOLATILE":
            raise ValueError("BLOCKED_LAUNCH_QOS:" + topic + role + name)
    return True


def clean_launch(proc):
    if proc.poll() is None:
        # ONLY target the isolated test-only launch parent, NEVER a sensor
        # process group or real physical M20/STM32 subprocess.
        proc.send_signal(signal.SIGINT)
    try:
        return proc.wait(timeout=25)
    except subprocess.TimeoutExpired as exc:
        proc.terminate()
        try:
            proc.wait(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=4)
        raise RuntimeError("BLOCKED_SYNTHETIC_LAUNCH_TREE_TIMEOUT") from exc


def main():
    print("M23BV_LAUNCH_TREE_SCRIPT_STARTED=PASS", flush=True)
    if (os.environ.get("ROS_DOMAIN_ID") != DOMAIN or
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"):
        raise RuntimeError("BLOCKED_LAUNCH_ISOLATION_ENV")
    for bad in ("/dev/video2", "/dev/ttyACM0", "/dev/video0", "/dev/ttyACM1"):
        if Path(bad).exists():
            raise RuntimeError("BLOCKED_PHYSICAL_DEVICE_PRESENT:" + bad)
    if not CAMERA_SOURCE.is_file():
        raise RuntimeError("BLOCKED_PINNED_LAUNCH_SOURCE_MISSING")

    with tempfile.TemporaryDirectory(prefix="m23bv-launch-tree-", dir="/tmp") as td:
        folder = Path(td)
        launch_path = prepare_launch(folder)
        proc = None
        received = []
        graph_pass = False
        last_error = "BLOCKED_GRAPH_NOT_READY"
        rclpy.init(args=[])
        node = rclpy.create_node("m23bv_synthetic_tree_observer",
                                 start_parameter_services=False,
                                 enable_rosout=False)
        reliable = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                              reliability=ReliabilityPolicy.RELIABLE,
                              durability=DurabilityPolicy.VOLATILE)
        node.create_subscription(
            Image, IMAGE,
            lambda msg: received.append(
                msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec),
            reliable)
        log_path = folder / "launch.log"
        try:
            with log_path.open("w+") as logfile:
                try:
                    proc = subprocess.Popen(
                        ["ros2", "launch", str(launch_path)],
                        stdout=logfile, stderr=subprocess.STDOUT)
                    deadline = time.monotonic() + 20.0
                    while time.monotonic() < deadline:
                        if proc.poll() is not None:
                            raise RuntimeError(
                                "BLOCKED_LAUNCH_EARLY_EXIT:" + str(proc.returncode))
                        rclpy.spin_once(node, timeout_sec=0.02)
                        try:
                            if required_graph(node):
                                graph_pass = True
                                break
                        except ValueError as exc:
                            last_error = str(exc)
                    if not graph_pass:
                        print("M23BV_LAUNCH_GRAPH_JSON=" +
                              json.dumps(snapshot(node, get_rmw_implementation_identifier()),
                                         sort_keys=True), flush=True)
                        raise RuntimeError("BLOCKED_LAUNCH_GRAPH:" + last_error)
                    print("M23BV_LAUNCH_TREE_GRAPH_QOS=PASS", flush=True)

                    delivery_deadline = time.monotonic() + 8.0
                    while time.monotonic() < delivery_deadline and len(received) < 90:
                        if proc.poll() is not None:
                            raise RuntimeError(
                                "BLOCKED_LAUNCH_PROCESS_DIED_WHILE_STREAMING")
                        rclpy.spin_once(node, timeout_sec=0.03)
                    if len(received) < 90:
                        raise RuntimeError(
                            "BLOCKED_LAUNCH_RECTIFICATION_DELIVERY:" + str(len(received)))
                    print("M23BV_LAUNCH_TREE_RECT_FRAMES=" + str(len(received)), flush=True)

                    rc = clean_launch(proc)
                    print("M23BV_LAUNCH_TREE_PARENT_EXIT=" + str(rc), flush=True)
                    if rc != 0:
                        raise RuntimeError("BLOCKED_LAUNCH_PARENT_NONZERO")
                finally:
                    if proc is not None and proc.poll() is None:
                        clean_launch(proc)
                    logfile.flush()
                    logfile.seek(0)
                    output = logfile.read()
                    print("M23BV_LAUNCH_TREE_CHILD_LOG_BEGIN", flush=True)
                    print(output[-12000:], flush=True)
                    print("M23BV_LAUNCH_TREE_CHILD_LOG_END", flush=True)
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

        started = re.findall(r"process started with pid \[(\d+)\]", output)
        if len(set(started)) < 2:
            raise RuntimeError("BLOCKED_LAUNCH_CHILD_PID_EVIDENCE_MISSING")
        if "M23BV_SYNTH_LAUNCH_CAMERA_STARTED=PASS" not in output:
            raise RuntimeError("BLOCKED_SYNTHETIC_CAMERA_START_UNVERIFIED")
        if "M23BV_SYNTH_LAUNCH_CAMERA_STOP=PASS" not in output:
            raise RuntimeError("BLOCKED_SYNTHETIC_CAMERA_STOP_UNVERIFIED")
        surviving = [pid for pid in set(started) if Path("/proc/" + pid).exists()]
        if surviving:
            raise RuntimeError("BLOCKED_LAUNCH_CHILDREN_SURVIVE:" + repr(surviving))
        print("M23BV_LAUNCH_TREE_CHILD_COUNT=" + str(len(set(started))), flush=True)
        print("M23BV_LAUNCH_TREE_CHILDREN_REAPED=PASS", flush=True)
        print("M23BV_LAUNCH_TREE_CLEAN_SHUTDOWN=PASS", flush=True)
    print("M23BV_SYNTHETIC_LAUNCH_LIFECYCLE=PASS", flush=True)
    print("M23BV_PHYSICAL_USB_CAM_QOS=NOT_TESTED", flush=True)
    print("M23BV_REAL_M20_SENSOR_LAUNCH=NOT_TESTED", flush=True)
    print("M23BV_PHYSICAL_ACCESS=NONE", flush=True)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print("M23BV_SYNTHETIC_LAUNCH_LIFECYCLE=BLOCKED:" +
              type(exc).__name__ + ":" + str(exc), flush=True)
        sys.exit(2)
