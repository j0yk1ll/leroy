#!/usr/bin/env bash
# M23BV: no-device Jazzy storage and standalone rectifier lifecycle smoke test.
# No camera/MCU mappings, no production ROS, no motion or sensor launch.
set -Eeuo pipefail
umask 077

IMAGE=localhost/leroy-sensors:m23bv-capture-prep
EXPECTED_ID=57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec
: "${M23BV_LOG_DIR:?Set an existing owner-private log directory}"
if [[ "${M23BV_LOG_DIR}" != /* || ! -d "${M23BV_LOG_DIR}" ||
      -L "${M23BV_LOG_DIR}" || ! -w "${M23BV_LOG_DIR}" ]]; then
  echo "M23BV_JAZZY_PREFLIGHT=BLOCKED_LOG_DIR" >&2
  exit 2
fi
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo "M23BV_JAZZY_PREFLIGHT=BLOCKED_IMAGE_ID" >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo "M23BV_JAZZY_PREFLIGHT=BLOCKED_PRODUCTION_RUNNING" >&2
  exit 2
fi

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_jazzy_XXXXXXXX.log)"
set +e
# Keep stdin attached: the container's bash -s reads the heredoc below.
podman run --rm -i --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e ROS_DOMAIN_ID=177 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --entrypoint /bin/bash "$IMAGE" -s >"$log" 2>&1 <<'IN_CONTAINER'
set -Eeuo pipefail
echo 'M23BV_JAZZY_CONTAINER_SCRIPT_STARTED=PASS'
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M23BV_DEVICE_ISOLATION=BLOCKED"
  exit 2
fi
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
export ROS_LOG_DIR=/tmp/m23bv-jazzy-logs
mkdir -p "$ROS_LOG_DIR"
rect="$(ros2 pkg prefix image_proc)/lib/image_proc/rectify_node"
test -x "$rect"
test -d /opt/m23aj-opencv/lib

# ELF dependency resolution for the rectifier's private library environment.
env LD_LIBRARY_PATH="/opt/m23aj-opencv/lib:${LD_LIBRARY_PATH:-}" \
  ldd "$rect" > /tmp/m23bv-rect-ldd.txt
cat /tmp/m23bv-rect-ldd.txt
if grep -q 'not found' /tmp/m23bv-rect-ldd.txt; then
  echo "M23BV_RECTIFIER_ELF=BLOCKED_MISSING_LIBRARY"
  exit 2
fi
echo "M23BV_RECTIFIER_ELF=DEPENDENCIES_RESOLVED"

python3 - "$rect" <<'PY'
import os
import signal
import sqlite3
import subprocess
import sys
import tempfile
import time
from pathlib import Path
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier
from rclpy.serialization import serialize_message, deserialize_message
import rosbag2_py
from sensor_msgs.msg import Imu

print("M23BV_JAZZY_RMW=" + get_rmw_implementation_identifier(), flush=True)
if not callable(getattr(rosbag2_py.SequentialWriter, "close", None)):
    raise RuntimeError("M23BV_ROSBAG_WRITER_CLOSE_UNAVAILABLE")

with tempfile.TemporaryDirectory(prefix="m23bv-jazzy-", dir="/tmp") as td:
    dest = Path(td) / "synthetic"
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(dest), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    meta_kwargs = {"name": "/imu/data_raw", "type": "sensor_msgs/msg/Imu",
                   "serialization_format": "cdr"}
    try:
        topic = rosbag2_py.TopicMetadata(**meta_kwargs, offered_qos_profiles="")
    except TypeError:
        topic = rosbag2_py.TopicMetadata(**meta_kwargs)
    writer.create_topic(topic)
    imu = Imu()
    imu.header.frame_id = "imu_link"
    imu.header.stamp.sec = 1
    stamp = 1_000_000_000
    payload = serialize_message(imu)
    writer.write("/imu/data_raw", payload, stamp)
    writer.close()
    del writer
    if not (dest / "metadata.yaml").is_file():
        raise RuntimeError("M23BV_ROSBAG_METADATA_MISSING")
    dbs = list(dest.glob("*.db3"))
    if len(dbs) != 1:
        raise RuntimeError("M23BV_ROSBAG_EXPECTED_ONE_DB")
    with sqlite3.connect("file:" + str(dbs[0]) + "?mode=ro", uri=True) as conn:
        if conn.execute("PRAGMA quick_check").fetchone()[0] != "ok":
            raise RuntimeError("M23BV_ROSBAG_SQLITE_INTEGRITY")
        rows = conn.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
        if rows != 1:
            raise RuntimeError("M23BV_ROSBAG_MESSAGE_COUNT")
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(dest), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("cdr", "cdr"))
    name, actual_payload, actual_stamp = reader.read_next()
    if name != "/imu/data_raw" or actual_payload != payload or actual_stamp != stamp:
        raise RuntimeError("M23BV_ROSBAG_PAYLOAD_OR_TIMESTAMP_CHANGED")
    if deserialize_message(actual_payload, Imu).header.frame_id != "imu_link":
        raise RuntimeError("M23BV_ROSBAG_CDR_ROUNDTRIP")
    if reader.has_next():
        raise RuntimeError("M23BV_ROSBAG_UNEXPECTED_EXTRA_MESSAGE")
    if callable(getattr(reader, "close", None)):
        reader.close()
    del reader
    print("M23BV_JAZZY_SQLITE_CDR_FINALIZATION=PASS", flush=True)

# Standalone rectifier only. No source publisher, no video/serial, no Nav2.
env = os.environ.copy()
env["LD_LIBRARY_PATH"] = "/opt/m23aj-opencv/lib:" + env.get("LD_LIBRARY_PATH", "")
rect = sys.argv[1]
with tempfile.TemporaryFile(mode="w+t") as log:
    proc = subprocess.Popen(
        [rect, "--ros-args", "-r", "__node:=m23bv_rectifier_smoke",
         "-r", "__ns:=/m23bv_smoke"],
        env=env, stdout=log, stderr=subprocess.STDOUT)
    try:
        time.sleep(2.5)
        if proc.poll() is not None:
            raise RuntimeError(f"M23BV_RECTIFIER_EARLY_EXIT={proc.returncode}")
        proc.send_signal(signal.SIGINT)
        rc = proc.wait(timeout=15)
        if rc != 0:
            raise RuntimeError(f"M23BV_RECTIFIER_SHUTDOWN_EXIT={rc}")
        print("M23BV_RECTIFIER_CLEAN_SHUTDOWN=PASS", flush=True)
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=5)
        log.seek(0)
        print("M23BV_RECTIFIER_PROCESS_LOG_BEGIN", flush=True)
        print(log.read()[-12000:], flush=True)
        print("M23BV_RECTIFIER_PROCESS_LOG_END", flush=True)

print("M23BV_NO_DEVICE_JAZZY_PREFLIGHT=PASS", flush=True)
print("M23BV_SYNTHETIC_DDS_CAMERA_TRANSPORT=NOT_TESTED", flush=True)
print("M23BV_PHYSICAL_ACCESS=NONE", flush=True)
PY
IN_CONTAINER
rc=$?
set -e
echo "M23BV_JAZZY_PREFLIGHT_EXIT=$rc"
echo "M23BV_JAZZY_PREFLIGHT_LOG=$log"
sha256sum "$log"
tail -n 45 "$log"
if [[ "$rc" != 0 ]] || ! grep -qx 'M23BV_NO_DEVICE_JAZZY_PREFLIGHT=PASS' "$log"; then
  echo "M23BV_JAZZY_PREFLIGHT=BLOCKED" >&2
  exit 2
fi
echo "M23BV_JAZZY_PREFLIGHT=PASS"
echo "M23BV_CAMERA_OR_MCU_ACCESS=NONE"
