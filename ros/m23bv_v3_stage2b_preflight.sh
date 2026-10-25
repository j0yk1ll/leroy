#!/usr/bin/env bash
# Necessary Stage 2B PRE-FLIGHT ONLY: source and immutable image imports.
# NO CAMERA, MCU SERIAL, ROS STREAMING, NETWORK, RECORDER OR DEVICES.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
fail() { echo "M23BV_V3_STAGE2B_PRECHECK=BLOCKED_$1" >&2; exit 2; }
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_WORKTREE
[[ "$(git branch --show-current)" == 'feature/m23bv-v3-stage2b-physical-sensors-optin' ]] ||
  fail WRONG_BRANCH
python3 -B m23bv_v3_stage2b_static_gate.py || fail STATIC_CONTRACT
bash -n m23bv_v3_stage2b_host_run.sh || fail HOST_SYNTAX
python3 -B - <<'PY'
import ast
from pathlib import Path
ast.parse(Path("m23bv_v3_stage2b_physical_probe.py").read_text())
print("M23BV_V3_STAGE2B_PROBE_AST=PASS")
PY
IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
PIN='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')" ||
  fail V3_IMAGE_ABSENT
[[ "${actual#sha256:}" == "$PIN" ]] || fail V3_IMAGE_CHANGED
[[ -z "$(podman ps --format '{{.Names}}')" ]] || fail OTHER_CONTAINER_RUNNING

# Source-only check of imported M20/M18 stack IN EXACT V3 runtime image.
# No --device, --mount, ros2 launch or physical access.
podman run --rm -i --pull=never --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export PYTHONPATH="/opt/m23bv:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"; exec python3 -B -' <<'PY'
import hashlib
from pathlib import Path
for name, pin in (
    ("/opt/m20/m20_combined_observer.py", "5cf2f88cb2f79be80821888401b57a6393ee5e3b"),
    ("/opt/m20/m20_combined_contract.py", "66111d8da486dc1e0c78057c1948f5e1aad9a06e"),
    ("/opt/m18/m18_imu_stream_source.py", "3c5fb37ca291533e2702a9159f3e8db11b076d22"),
    ("/opt/m18/m18_imu_stream_contract.py", "443aa82066e88d50cb88859304e9197e3f459ee0"),
    ("/opt/m23bv/m23bv_camera_ready_gate.py", "777db7cbeb0ce6c3669a837f1530f7b1499dd9dd"),
):
    payload = Path(name).read_bytes()
    sha = hashlib.sha1(b"blob " + str(len(payload)).encode() + b"\0" + payload).hexdigest()
    if sha != pin:
        raise SystemExit("M23BV_V3_STAGE2B_IMAGE_IMPORT=BLOCKED_PIN:" + name)
import rclpy, serial, m20_combined_observer, m18_imu_stream_source
from m18_imu_stream_contract import START, STOP, PORT
if (START != b"set_imu_stream_hz 50\r\n" or
        STOP != b"set_imu_stream_hz 0\r\n" or PORT != "/dev/ttyACM0"):
    raise SystemExit("M23BV_V3_STAGE2B_IMAGE_IMPORT=BLOCKED_MCU_ALLOWLIST")
for blocked in ("/dev/video2", "/dev/ttyACM0"):
    if Path(blocked).exists():
        raise SystemExit("M23BV_V3_STAGE2B_IMAGE_IMPORT=BLOCKED_DEVICE_VISIBLE:" + blocked)
print("M23BV_V3_STAGE2B_IMAGE_IMPORTS=PASS")
print("M23BV_V3_STAGE2B_PREFLIGHT_DEVICE_ACCESS=NONE")
PY
echo M23BV_V3_STAGE2B_PRECHECK=PASS
echo M23BV_V3_STAGE2B_PHYSICAL_MCU_ACCESS=NOT_STARTED
