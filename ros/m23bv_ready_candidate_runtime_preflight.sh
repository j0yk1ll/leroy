#!/usr/bin/env bash
# M23BV read-only synthetic launch test of the actual separately built candidate.
# No device, no production network, no original image changes.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"

IMAGE=localhost/leroy-sensors:m23bv-camera-ready-candidate
EXPECTED_ID=2a557fc07aead1ba680cc4024d27a5d67227cf659754114487911f6264690c40
CASE="${M23BV_READY_TEST_CASE:-healthy}"
case "$CASE" in
  healthy|best_effort|missing_info|camera_exit) ;;
  *) echo 'M23BV_READY_RUNTIME=BLOCKED_INVALID_CASE' >&2; exit 2 ;;
esac
: "${M23BV_LOG_DIR:?Set existing owner-private log directory}"
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_READY_RUNTIME=BLOCKED_LOG_DIR' >&2
  exit 2
fi
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo 'M23BV_READY_RUNTIME=BLOCKED_UNPINNED_CANDIDATE' >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo 'M23BV_READY_RUNTIME=BLOCKED_PRODUCTION_RUNNING' >&2
  exit 2
fi
python3 - <<'PY'
import ast
from pathlib import Path
ast.parse(Path("m23bv_ready_candidate_runtime_probe.py").read_text())
PY

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_ready_runtime_XXXXXXXX.log)"
set +e
podman run --rm -i --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 -e M23BV_READY_TEST_CASE="$CASE" \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-ready-runtime-ros-logs; mkdir -p "$ROS_LOG_DIR"; exec python3 -u -' \
  <m23bv_ready_candidate_runtime_probe.py >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_READY_RUNTIME_EXIT=%s\n' "$rc"
printf 'M23BV_READY_RUNTIME_CASE=%s\n' "$CASE"
printf 'M23BV_READY_RUNTIME_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|Traceback|RuntimeError|Error:|TypeError:' "$log" | tail -n 100 || true
if [[ "$rc" != 0 ]] || \
   ! grep -qx "M23BV_READY_CANDIDATE_RUNTIME_${CASE^^}=PASS" "$log" || \
   ! grep -qx 'M23BV_READY_TEST_CHILDREN_REAPED=PASS' "$log"; then
  echo 'M23BV_READY_RUNTIME_RAW_LAUNCH_OUTPUT_BEGIN' >&2
  # Outer M23BV grep hides ordinary Python/launch errors; expose the
  # bounded captured launch block verbatim, without launching again.
  sed -n '/^M23BV_READY_TEST_LAUNCH_LOG_BEGIN$/,/^M23BV_READY_TEST_LAUNCH_LOG_END$/p' "$log" \
    | tail -n 140 >&2 || true
  echo 'M23BV_READY_RUNTIME_RAW_LAUNCH_OUTPUT_END' >&2
  echo 'M23BV_READY_RUNTIME_PREFLIGHT=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_READY_RUNTIME_PREFLIGHT=PASS'
echo 'M23BV_PRODUCTION_OR_PHYSICAL_CAPTURE=NOT_AUTHORIZED'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
