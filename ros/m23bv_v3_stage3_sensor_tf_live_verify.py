#!/usr/bin/env python3
"""No-device integration observer for the *real* installed Leroy static TF launch.

Runs only inside the isolated rootless Podman test. No device, bridge,
camera, motor, recording, navigation, pose or calibration operations.
"""
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage
import yaml


INSTALL = Path("/tmp/leroy_static_tf_ws/install")
SOURCE = Path("/opt/leroy_tf_source")
EXPECTED = (
    ("mounts", "imu", "base_link", "imu_link",
     (0.095, -0.095, 0.0), (0.0, 0.0, 0.0)),
    ("mounts", "front_camera", "base_link", "front_camera_link",
     (0.0, 0.0, 0.0), (0.0, 0.5235987755982988, 0.0)),
    ("fixed_frames", "front_camera_optical", "front_camera_link",
     "front_camera_optical_frame", (0.0, 0.0, 0.0),
     (-math.pi / 2, 0.0, -math.pi / 2)),
    ("fixed_frames", "camera_optical_alias", "front_camera_optical_frame",
     "camera_optical_frame", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)),
    ("mounts", "workspace_camera", "base_link", "workspace_camera_link",
     (0.0, 0.0, 0.0), (0.0, 1.0471975511965976, 0.0)),
    ("fixed_frames", "workspace_camera_optical", "workspace_camera_link",
     "workspace_camera_optical_frame", (0.0, 0.0, 0.0),
     (-math.pi / 2, 0.0, -math.pi / 2)),
)
TIMEOUT_S = 30.0


def emit(key, value):
    print(f"STAGE3_SENSOR_TF_INTEGRATION_{key}={value}", flush=True)


def close3(a, b, tolerance=1e-8):
    return (len(a) == len(b) == 3
            and all(math.isfinite(x) and abs(x - y) <= tolerance
                    for x, y in zip(a, b)))


def quat_rpy(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def check_install():
    share = Path(get_package_share_directory("leroy_description"))
    if share != INSTALL / "share/leroy_description":
        raise RuntimeError("PACKAGE_NOT_FROM_EPHEMERAL_INSTALL:" + str(share))
    files = (
        ("config/sensor_mounts.yaml", SOURCE / "config/sensor_mounts.yaml"),
        ("launch/description.launch.py", SOURCE / "launch/description.launch.py"),
    )
    for rel, source in files:
        if (share / rel).read_bytes() != source.read_bytes():
            raise RuntimeError("INSTALLED_SOURCE_BYTES_MISMATCH:" + rel)
    if (share / "urdf").exists():
        raise RuntimeError("OBSOLETE_URDF_INSTALLED")
    cfg = yaml.safe_load((share / "config/sensor_mounts.yaml").read_text())
    if cfg["status"] != "provisional":
        raise RuntimeError("INVALID_GEOMETRY_ACCEPTANCE")
    if len(EXPECTED) != 6:
        raise RuntimeError("EXPECTED_GRAPH_INVALID")
    for section, name, _, _, xyz, rpy in EXPECTED:
        entry = cfg[section][name]
        if not close3(entry["xyz"], xyz, tolerance=1e-12):
            raise RuntimeError("CHANGED_MOUNT_XYZ:" + name)
        if not close3(entry["rpy"], rpy, tolerance=1e-12):
            raise RuntimeError("CHANGED_MOUNT_RPY:" + name)
    emit("INSTALLED_PACKAGE", "PASS_NEW_SOURCE_BUILD_AND_BYTE_PARITY")
    emit("GEOMETRY", "PROVISIONAL_NOT_CALIBRATED")


class LaunchLogs:
    def __init__(self, stream):
        self.messages = []
        self.reader = threading.Thread(target=self.run, args=(stream,), daemon=True)
        self.reader.start()

    def run(self, stream):
        try:
            for line in stream:
                line = line.rstrip()
                if len(self.messages) >= 250:
                    del self.messages[:100]
                self.messages.append(line)
                print("STAGE3_SENSOR_TF_LAUNCH " + line, flush=True)
        except (OSError, ValueError):
            pass


def remaining_in_group(group):
    """Ignore zombies; only live processes mean cleanup is incomplete."""
    scan = subprocess.run(["ps", "-e", "-o", "pgid=,stat="],
                          text=True, capture_output=True, check=True)
    for row in scan.stdout.splitlines():
        fields = row.split()
        if len(fields) >= 2 and fields[0].isdigit():
            if int(fields[0]) == group and not fields[1].startswith("Z"):
                return True
    return False


def stop_group(proc):
    if proc is None:
        return True
    # ros2 launch and its static publishers share this isolated process group.
    if proc.poll() is None or remaining_in_group(proc.pid):
        try:
            os.killpg(proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(proc.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGKILL)
            proc.wait(timeout=3)
            return False
        return False

    # Even if the launcher exited, no static publisher should linger.
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        if not remaining_in_group(proc.pid):
            return proc.returncode == 0
        time.sleep(0.2)
    try:
        os.killpg(proc.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    return False


def check_actual_tf():
    received = {}
    errors = []
    unexpected = set()
    publishers = 0
    expected = {item[3]: item for item in EXPECTED}
    proc = None
    reader = None
    node = None
    rclpy_initialized = False
    verified = False
    clean = False
    try:
        proc = subprocess.Popen(
            ["ros2", "launch", "leroy_description", "description.launch.py"],
            stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, errors="replace",
            bufsize=1, start_new_session=True,
        )
        reader = LaunchLogs(proc.stdout)
        rclpy.init(args=None)
        rclpy_initialized = True
        node = rclpy.create_node("leroy_stage3_tf_integration_observer")
        qos = QoSProfile(depth=100)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = ReliabilityPolicy.RELIABLE

        def callback(msg):
            for transform in msg.transforms:
                child = transform.child_frame_id
                if child not in expected:
                    unexpected.add(child)
                    continue
                _, _, parent, _, xyz, rpy = expected[child]
                if transform.header.frame_id != parent:
                    errors.append("WRONG_PARENT:" + child)
                    continue
                vec = transform.transform.translation
                if not close3((vec.x, vec.y, vec.z), xyz):
                    errors.append("WRONG_TRANSLATION:" + child)
                    continue
                quat = transform.transform.rotation
                q = (quat.x, quat.y, quat.z, quat.w)
                target = quat_rpy(rpy)
                if (any(not math.isfinite(x) for x in q)
                        or abs(sum(a * b for a, b in zip(q, q)) - 1.0) > 1e-6
                        or abs(abs(sum(a * b for a, b in zip(q, target))) - 1.0)
                        > 1e-7):
                    errors.append("WRONG_QUATERNION:" + child)
                    continue
                received[child] = True

        sub = node.create_subscription(TFMessage, "/tf_static", callback, qos)
        deadline = time.monotonic() + TIMEOUT_S
        settled_at = None
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise RuntimeError("LAUNCH_EXITED_EARLY:" + str(proc.returncode))
            rclpy.spin_once(node, timeout_sec=0.25)
            if errors or unexpected:
                raise RuntimeError("BAD_TF_DATA:" +
                                   ",".join(errors + sorted(unexpected)))
            # Require six CURRENT DDS writers, not a historic maximum.
            publishers = sub.get_publisher_count()
            if len(received) == 6 and publishers == 6:
                if settled_at is None:
                    settled_at = time.monotonic()
                # Allow one extra second for a contradictory late message.
                if time.monotonic() - settled_at >= 1.0:
                    break
            else:
                settled_at = None
        else:
            raise RuntimeError("TF_TIMEOUT:received=" +
                               ",".join(sorted(received)) +
                               ",publishers=" + str(publishers))
        if len(received) != 6 or publishers != 6:
            raise RuntimeError("INCOMPLETE_FRAME_GRAPH")
        verified = True
        emit("TF_STATIC", "PASS_SIX_REAL_EDGES")
        emit("TRANSLATIONS_AND_QUATERNIONS", "PASS")
        emit("PUBLISHERS", "PASS_SIX_DISTINCT_DDS_WRITERS")
        emit("FRAME_OWNERSHIP", "PASS_EXACT_CHILDREN_AND_PARENTS")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy_initialized and rclpy.ok():
            rclpy.shutdown()
        clean = stop_group(proc)
        if reader is not None:
            reader.reader.join(timeout=3)
        emit("SHUTDOWN", "PASS" if clean else "BLOCKED")
    if not verified or not clean:
        raise RuntimeError("INCOMPLETE_TF_OR_UNCLEAN_SHUTDOWN")


def main():
    if (os.environ.get("ROS_DOMAIN_ID") != "170"
            or os.environ.get("ROS_AUTOMATIC_DISCOVERY_RANGE") != "LOCALHOST"
            or os.environ.get("RMW_IMPLEMENTATION") != "rmw_fastrtps_cpp"):
        raise RuntimeError("NOT_ISOLATED_ROS_CONFIGURATION")
    if (Path("/dev/video2").exists()
            or Path("/dev/ttyACM0").exists()):
        raise RuntimeError("PHYSICAL_DEVICE_VISIBLE")
    check_install()
    check_actual_tf()
    emit("REAL_NODES", "PASS_SIX_STANDARD_STATIC_TF_PUBLISHERS")
    emit("MCU_CAMERA_MOTOR_ACCESS", "NONE")
    emit("MOTION_OR_RECORDING", "NOT_AUTHORIZED")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print("STAGE3_SENSOR_TF_INTEGRATION=BLOCKED_" + repr(exc),
              file=sys.stderr, flush=True)
        sys.exit(2)
