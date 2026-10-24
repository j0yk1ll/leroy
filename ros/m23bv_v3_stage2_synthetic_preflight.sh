#!/usr/bin/env bash
# Stage 2A real ROS 2 Jazzy DDS / V3 C++ rectifier, SYNTHETIC INPUTS ONLY.
# No USB device mapping, MCU, recorder, production runner, XRSLAM or motion.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"

CASE="${M23BV_V3_STAGE2A_CASE:-healthy}"
case "$CASE" in
  healthy|missing_imu|imu_qos_reliable) ;;
  *) echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_CASE' >&2; exit 2 ;;
esac
IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
PIN='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
LOGDIR="${M23BV_LOG_DIR:-}"
[[ "$LOGDIR" == /* && -d "$LOGDIR" && ! -L "$LOGDIR" && -w "$LOGDIR" ]] || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_PRIVATE_LOGDIR' >&2; exit 2;
}
git rev-parse --show-toplevel >/dev/null || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_NO_WORKTREE' >&2; exit 2;
}
[[ -z "$(git status --porcelain)" ]] || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_DIRTY_TREE' >&2; exit 2;
}
python3 -B m23bv_v3_stage2_source_gate.py
python3 -B - <<'PY'
import ast
from pathlib import Path
ast.parse(Path("m23bv_v3_stage2_synthetic_probe.py").read_text())
PY
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null)" || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_IMAGE_MISSING' >&2; exit 2;
}
[[ "${actual#sha256:}" == "$PIN" ]] || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_IMAGE_ID_MISMATCH' >&2; exit 2;
}
running="$(podman ps --format '{{.Names}}')" || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_INVENTORY' >&2; exit 2;
}
[[ -z "$running" ]] || {
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_RUNNING_CONTAINERS' >&2; exit 2;
}

log="$(mktemp --tmpdir="$LOGDIR" m23bv_v3_stage2a_${CASE}_XXXXXXXX.log)"
name="m23bv-stage2a-${CASE//_/-}-$$"
cleanup() {
  podman stop --time 6 "$name" >/dev/null 2>&1 || true
  podman rm -f "$name" >/dev/null 2>&1 || true
}
trap cleanup EXIT
set +e
timeout --foreground --signal=INT --kill-after=15s 60s \
  podman run --rm -i --name "$name" --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 \
  -e M23BV_RELIABLE_RECTIFIER_PROTO_V3=1 \
  -e M23BV_V3_STAGE2A_CASE="$CASE" \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-stage2a-ros; mkdir -p "$ROS_LOG_DIR"; exec python3 -u -' \
  <m23bv_v3_stage2_synthetic_probe.py >"$log" 2>&1
rc=$?
set -e
cleanup
trap - EXIT

printf 'M23BV_V3_STAGE2A_PROCESS_EXIT=%s\n' "$rc"
printf 'M23BV_V3_STAGE2A_CASE=%s\n' "$CASE"
printf 'M23BV_V3_STAGE2A_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_V3_STAGE2A_|M23BV_CANDIDATE_CAMERA_READINESS=' "$log" | tail -n 90 || true

if [[ "$rc" != 0 ]] ||
   ! grep -qx 'M23BV_V3_STAGE2A_CPP_AND_LAUNCH_PINS=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2A_V3_PINNED_AST_FIXTURE=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2A_CHILDREN_REAPED=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2A_LAUNCH_EXIT=0' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2A_HARDWARE_DEVICE_ACCESS=NONE' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2A_MCU_SERIAL_OR_ROSBAG=NOT_USED' "$log" ||
   ! grep -qx "M23BV_V3_STAGE2A_CASE_${CASE^^}=PASS" "$log"; then
  echo 'M23BV_V3_STAGE2A_PREFLIGHT=BLOCKED_RUNTIME_OR_CLEANUP' >&2
  exit 2
fi
echo "M23BV_V3_STAGE2A_PREFLIGHT_${CASE^^}=PASS"
echo 'M23BV_V3_STAGE2A_PHYSICAL_MCU_OR_RECORDING=NOT_AUTHORIZED'
