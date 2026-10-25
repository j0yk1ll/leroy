#!/usr/bin/env bash
# Stage 3 VIO: reviewable NO-DEVICE source and pinned-image import preflight.
# It never opens cameras/MCU serial, runs ROS nodes, connects to host DDS,
# records payloads, launches motors, or authorizes the physical trial.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_VIO_PREFLIGHT=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_WORKTREE
bash -n m23bv_v3_stage3_vio_host.sh || fail HOST_SHELL_SYNTAX
python3 -B - <<'PY'
import ast
from pathlib import Path
for source in (
    "m23bv_v3_stage3_vio_imu_sender.py",
    "m23bv_v3_stage3_vio_audit.py",
    "m23bv_v3_stage3_xrslam_vio_diag.launch.py",
):
    ast.parse(Path(source).read_text(), filename=source)
    print("STAGE3_VIO_AST_PASS=" + source, flush=True)
PY
command -v podman >/dev/null || fail PODMAN
[[ "$(id -u)" != 0 ]] || fail ROOT_DISALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail NOT_ROOTLESS
[[ -z "$(podman ps --format '{{.Names}}')" ]] || fail CONTAINERS_RUNNING

check_image() {
  local observed
  observed="$(podman image inspect "$1" --format '{{.Id}}')" ||
    fail IMAGE_MISSING
  [[ "$(echo "$observed" | sed 's/^sha256://')" == "$2" ]] ||
    fail IMAGE_ID_DRIFT
}
check_image localhost/leroy-sensors:m23bv-reliable-rectifier-v3 \
  f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
check_image localhost/leroy-ros:xrslam \
  20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a
check_image localhost/leroy-nav2:jazzy \
  89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4

# Unprivileged source inspection: simple fail-closed invariants, not a
# substitute for a peer code review or an operator's motor-power inspection.
python3 -B - <<'PY'
from pathlib import Path
host = Path("m23bv_v3_stage3_vio_host.sh").read_text()
sender = Path("m23bv_v3_stage3_vio_imu_sender.py").read_text()
launch = Path("m23bv_v3_stage3_xrslam_vio_diag.launch.py").read_text()
required_host = (
    '--network none',
    '--device "$cam:/dev/video2:rw"',
    '--device "$mcu:/dev/ttyACM0:rw"',
    '--group-add keep-groups',
    '--network "container:$parent"',
    '--ipc "container:$parent"',
    'STAGE3_VIO_IMU_STOP_ACK=PASS',
    'M23BV_STAGE3_VIO_AUDIT=PASS_REAL_CAMERA_IMU_GRAPH_NO_ODOM',
    'Submitted [0-9]+ camera frames to XRSLAM',
)
required_sender = (
    'send_only_whitelisted_serial_command(device, START)',
    'send_only_whitelisted_serial_command(device, STOP)',
    'stop_ack = wait_ack(device, timeout_s=2.0)',
    'STAGE3_VIO_IMU_STOP_ACK=',
    'graph_guard(src, True)',
)
for token in required_host:
    assert token in host, "missing host barrier: " + token
for token in required_sender:
    assert token in sender, "missing MCU barrier: " + token
assert '--network host' not in host
assert '--privileged' not in host
assert 'pose_convention_validated": False' in launch
assert 'enable_map_tf": False' in launch
assert '"camera_imu_offset_sec": 0.0' in launch
assert "set_wheels" not in sender
print("STAGE3_VIO_SOURCE_BOUNDARIES=PASS", flush=True)
PY

# A V3 image with NO device maps. Import exactly protected M18 source and
# the sender in a rootless, read-only, isolated container; never call run().
podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=96m \
  --mount "type=bind,src=$PWD/m18_imu_stream_source.py,dst=/opt/m18/m18_imu_stream_source.py,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_vio_imu_sender.py,dst=/opt/stage3_imu_sender.py,readonly" \
  --entrypoint /bin/bash localhost/leroy-sensors:m23bv-reliable-rectifier-v3 -c '
set -e
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
export PYTHONPATH="/opt/m18:/opt/m18/monitor:$PYTHONPATH"
python3 -B - <<PY
import importlib.util
from pathlib import Path
assert not Path("/dev/video2").exists()
assert not Path("/dev/ttyACM0").exists()
from m18_imu_stream_contract import START, STOP, PORT
assert (START, STOP, PORT) == (
    b"set_imu_stream_hz 50\r\n",
    b"set_imu_stream_hz 0\r\n",
    "/dev/ttyACM0")
spec=importlib.util.spec_from_file_location(
    "stage3_vio_imu_sender", "/opt/stage3_imu_sender.py")
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
assert module.ImuStreamSource.__name__ == "ImuStreamSource"
print("STAGE3_VIO_M18_IMPORT_NO_DEVICE=PASS", flush=True)
PY
' || fail MCU_SOURCE_IMPORT

podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=96m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_vio_diag.yaml,dst=/opt/leroy_stage3_slam.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml,dst=/opt/leroy_stage3_device.yaml,readonly" \
  --entrypoint /bin/bash localhost/leroy-ros:xrslam -c '
set -e
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
ros2 pkg prefix leroy_xrslam >/dev/null
ros2 pkg prefix leroy_description >/dev/null
python3 -B - <<PY
from pathlib import Path
import yaml
slam=yaml.safe_load(Path("/opt/leroy_stage3_slam.yaml").read_text())
dev=yaml.safe_load(Path("/opt/leroy_stage3_device.yaml").read_text())
assert slam["sensor"]["mode"] == "monocular_imu"
assert slam["initializer"]["refine_imu"] is True
assert "cam0" in dev and "imu" in dev
assert "noise" in dev["cam0"] and "noise" in dev["imu"]
print("STAGE3_VIO_XRSLAM_YAML_NO_DEVICE=PASS", flush=True)
PY
' || fail XRSLAM_CONFIG

echo STAGE3_VIO_PREFLIGHT=PASS_NO_DEVICE_SOURCE_AND_IMPORTS
echo STAGE3_VIO_MCU_CDC=NOT_OPENED
echo STAGE3_VIO_MOTION_AND_RECORDING=NOT_AUTHORIZED
