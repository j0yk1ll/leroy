#!/usr/bin/env bash
# M23BV SIGINT-v2 synthetic-only test of independently versioned readiness candidate.
# No device, no production network, no original image changes.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"

IMAGE=localhost/leroy-sensors:m23bv-camera-ready-sigint-v2
: "${M23BV_SIGINT_V2_EXPECTED_IMAGE_ID:?Pin v2 image ID from offline build output}"
EXPECTED_ID="${M23BV_SIGINT_V2_EXPECTED_IMAGE_ID}"
if [[ ! "$EXPECTED_ID" =~ ^[[:xdigit:]]{64}$ ]]; then
  echo 'M23BV_SIGINT_V2_RUNTIME=BLOCKED_EXPECTED_ID_FORMAT' >&2
  exit 2
fi
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
ast.parse(Path("m23bv_sigint_v2_runtime_probe.py").read_text())
PY

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_sigint_v2_runtime_XXXXXXXX.log)"
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
  <m23bv_sigint_v2_runtime_probe.py >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_SIGINT_V2_RUNTIME_EXIT=%s\n' "$rc"
printf 'M23BV_SIGINT_V2_RUNTIME_CASE=%s\n' "$CASE"
printf 'M23BV_SIGINT_V2_RUNTIME_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|Traceback|RuntimeError|Error:|TypeError:' "$log" | tail -n 100 || true
if [[ "$rc" != 0 ]] || \
   ! grep -qx "M23BV_SIGINT_V2_CANDIDATE_RUNTIME_${CASE^^}=PASS" "$log" || \
   ! grep -qx 'M23BV_READY_TEST_CHILDREN_REAPED=PASS' "$log" || \
   ! grep -qx 'M23BV_SIGINT_V2_NO_TRACEBACK=PASS' "$log" || \
   { [[ "$CASE" == camera_exit ]] && ! grep -qx 'M23BV_SIGINT_V2_INTERRUPTED_NONZERO=PASS' "$log"; }; then
  echo 'M23BV_SIGINT_V2_RUNTIME_RAW_LAUNCH_OUTPUT_BEGIN' >&2
  # Outer M23BV grep hides ordinary Python/launch errors; expose the
  # bounded captured launch block verbatim, without launching again.
  sed -n '/^M23BV_READY_TEST_LAUNCH_LOG_BEGIN$/,/^M23BV_READY_TEST_LAUNCH_LOG_END$/p' "$log" \
    | tail -n 140 >&2 || true
  echo 'M23BV_SIGINT_V2_RUNTIME_RAW_LAUNCH_OUTPUT_END' >&2
  echo 'M23BV_SIGINT_V2_RUNTIME_PREFLIGHT=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_SIGINT_V2_RUNTIME_PREFLIGHT=PASS'
echo 'M23BV_PRODUCTION_OR_PHYSICAL_CAPTURE=NOT_AUTHORIZED'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
