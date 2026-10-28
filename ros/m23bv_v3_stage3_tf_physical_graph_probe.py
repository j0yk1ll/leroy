#!/usr/bin/env python3
"""Fail-closed Stage 3 physical-host TF sidecar DDS pre/postflight.

No hardware APIs. Executed only within pinned, device-free, rootless
Leroy TF candidate containers in an existing sensor DDS namespace.
"""
import importlib.util
import math
import os
from pathlib import Path
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def emit(key, result):
    print("STAGE3_TF_DEPLOY_" + key + "=" + result, flush=True)


def quaternion(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll/2), math.sin(roll/2)
    cp, sp = math.cos(pitch/2), math.sin(pitch/2)
    cy, sy = math.cos(yaw/2), math.sin(yaw/2)
    return (
        sr*cp*cy - cr*sp*sy,
        cr*sp*cy + sr*cp*sy,
        cr*cp*sy - sr*sp*cy,
        cr*cp*cy + sr*sp*sy,
    )


def expected_frames():
    source = Path(get_package_share_directory("leroy_description")) / "launch/description.launch.py"
    spec = importlib.util.spec_from_file_location("leroy_tf_physical_probe", source)
    if spec is None or spec.loader is None:
        raise RuntimeError("TF_LAUNCH_UNAVAILABLE")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    cfg = Path(get_package_share_directory("leroy_description")) / "config/sensor_mounts.yaml"
    return {child: (parent, xyz, quaternion(rpy))
            for parent, child, xyz, rpy in mod.load_sensor_transforms(str(cfg))}


def check(mode):
    if mode not in ("preflight", "postflight"):
        raise RuntimeError("BAD_PROBE_MODE")
    if (os.environ.get("ROS_DOMAIN_ID") != "170" or
            os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST" or
            os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"):
        raise RuntimeError("DDS_ENV_UNSAFE")
    if any(Path(p).exists() for p in
           ("/dev/video2", "/dev/ttyACM0", "/dev/ttyUSB0")):
        raise RuntimeError("PHYSICAL_DEVICE_EXPOSED")
    expected = expected_frames()
    if len(expected) != 6:
        raise RuntimeError("INVALID_EXPECTED_GRAPH")

    received = set()
    errors = set()
    rclpy.init()
    node = rclpy.create_node("leroy_stage3_tf_deploy_" + mode)
    try:
        qos = QoSProfile(depth=100)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        def callback(msg):
            for transform in msg.transforms:
                child = transform.child_frame_id
                if mode == "preflight":
                    if child in expected:
                        errors.add("EXISTING_SENSOR_CHILD:" + child)
                    continue
                if child not in expected:
                    errors.add("UNEXPECTED_STATIC_CHILD:" + child)
                    continue
                parent, xyz, q = expected[child]
                t = transform.transform
                v = (t.translation.x, t.translation.y, t.translation.z)
                got_q = (t.rotation.x, t.rotation.y, t.rotation.z,
                         t.rotation.w)
                if transform.header.frame_id != parent:
                    errors.add("PARENT_MISMATCH:" + child)
                elif (not all(math.isfinite(x) for x in v + got_q) or
                      any(abs(a-b) > 1e-8 for a, b in zip(v, xyz)) or
                      abs(sum(a*b for a, b in zip(got_q, got_q))-1) > 1e-6 or
                      abs(abs(sum(a*b for a, b in zip(got_q, q)))-1) > 1e-7):
                    errors.add("VALUE_MISMATCH:" + child)
                else:
                    received.add(child)

        sub = node.create_subscription(TFMessage, "/tf_static", callback, qos)
        end = time.monotonic() + (5 if mode == "preflight" else 20)
        stable = None
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            nodes = [name for name, _ in node.get_node_names_and_namespaces()]
            if any("leroy_bridge" in name or
                   "robot_state_publisher" in name for name in nodes):
                errors.add("UNSAFE_BRIDGE_OR_LEGACY_TF_OWNER_RUNNING")
            count = sub.get_publisher_count()
            if mode == "preflight":
                # Strict zero-writer preflight: never compete with an existing
                # static TF tree. Other /tf_static publishers require review.
                if count:
                    errors.add("EXISTING_TF_STATIC_WRITER_COUNT:" + str(count))
            else:
                if count != 6:
                    stable = None
                elif len(received) == 6:
                    if stable is None:
                        stable = time.monotonic()
                    if time.monotonic() - stable >= 1.0:
                        break
            if errors:
                raise RuntimeError(",".join(sorted(errors)))
        if mode == "preflight":
            emit("GRAPH_PREFLIGHT", "PASS_NO_EXISTING_STATIC_TF_OR_BRIDGE")
        elif len(received) == 6 and sub.get_publisher_count() == 6:
            emit("GRAPH_POSTFLIGHT", "PASS_SIX_PUBLISHERS_SIX_CORRECT_FRAMES")
        else:
            raise RuntimeError("POSTFLIGHT_TIMEOUT:" +
                               ",".join(sorted(received)))
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    try:
        check(sys.argv[1] if len(sys.argv) == 2 else "")
    except Exception as exc:
        print("STAGE3_TF_DEPLOY=BLOCKED_" + repr(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
