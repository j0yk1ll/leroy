#!/usr/bin/env bash
# M23BV V3 explicit RELIABLE rectifier: isolated synthetic-only tests.
# No device, no production network, no original image changes.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"

IMAGE=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
: "${M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID:?Pin new V3 image ID from the V3 offline build output}"
EXPECTED_ID="${M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID}"
if [[ ! "$EXPECTED_ID" =~ ^[[:xdigit:]]{64}$ ]]; then
  echo 'M23BV_V3_RUNTIME=BLOCKED_EXPECTED_ID_FORMAT' >&2
  exit 2
fi
CASE="${M23BV_READY_TEST_CASE:-healthy}"
ORDER="${M23BV_V3_STARTUP_MODE:-as_pinned}"
case "$ORDER" in
  as_pinned|rectifier_first) ;;
  *) echo 'M23BV_V3_RUNTIME=BLOCKED_ORDER' >&2; exit 2 ;;
esac
if [[ "$ORDER" == rectifier_first && "$CASE" != healthy ]]; then
  echo 'M23BV_V3_RUNTIME=BLOCKED_ORDER_CASE' >&2
  exit 2
fi
case "$CASE" in
  healthy|best_effort|missing_info|camera_exit) ;;
  *) echo 'M23BV_V3_RUNTIME=BLOCKED_INVALID_CASE' >&2; exit 2 ;;
esac
: "${M23BV_LOG_DIR:?Set existing owner-private log directory}"
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_V3_RUNTIME=BLOCKED_LOG_DIR' >&2
  exit 2
fi
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo 'M23BV_V3_RUNTIME=BLOCKED_UNPINNED_CANDIDATE' >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo 'M23BV_V3_RUNTIME=BLOCKED_PRODUCTION_RUNNING' >&2
  exit 2
fi
python3 - <<'PY'
import ast
from pathlib import Path
ast.parse(Path("m23bv_reliable_v3_runtime_probe.py").read_text())
PY

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_reliable_v3_runtime_XXXXXXXX.log)"
set +e
podman run --rm -i --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 -e M23BV_READY_TEST_CASE="$CASE" \
  -e M23BV_RELIABLE_RECTIFIER_PROTO_V3=1 -e M23BV_V3_STARTUP_MODE="$ORDER" \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-ready-runtime-ros-logs; mkdir -p "$ROS_LOG_DIR"; exec python3 -u -' \
  <m23bv_reliable_v3_runtime_probe.py >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_RELIABLE_V3_RUNTIME_EXIT=%s\n' "$rc"
printf 'M23BV_RELIABLE_V3_RUNTIME_CASE=%s\n' "$CASE"
printf 'M23BV_RELIABLE_V3_RUNTIME_ORDER=%s\n' "$ORDER"
printf 'M23BV_RELIABLE_V3_RUNTIME_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|Traceback|RuntimeError|Error:|TypeError:' "$log" | tail -n 100 || true
if [[ "$rc" != 0 ]] || \
   ! grep -qx 'M23BV_READY_TEST_CHILDREN_REAPED=PASS' "$log" || \
   ! grep -qx 'M23BV_SIGINT_V2_NO_TRACEBACK=PASS' "$log" || \
   ! grep -qx 'M23BV_V3_CPP_SOURCE_PIN=PASS' "$log" || \
   { [[ "$ORDER" == as_pinned ]] && ! grep -qx "M23BV_V3_CANDIDATE_RUNTIME_${CASE^^}=PASS" "$log"; } || \
   { [[ "$ORDER" == rectifier_first ]] && \
     { ! grep -qx 'M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY' "$log" || \
       ! grep -qx 'M23BV_V3_RELIABLE_BEFORE_CAMERA=PASS' "$log"; }; } || \
   { [[ "$CASE" == camera_exit ]] && ! grep -qx 'M23BV_SIGINT_V2_INTERRUPTED_NONZERO=PASS' "$log"; }; then
  echo 'M23BV_RELIABLE_V3_RUNTIME_RAW_LAUNCH_OUTPUT_BEGIN' >&2
  # Outer M23BV grep hides ordinary Python/launch errors; expose the
  # bounded captured launch block verbatim, without launching again.
  sed -n '/^M23BV_READY_TEST_LAUNCH_LOG_BEGIN$/,/^M23BV_READY_TEST_LAUNCH_LOG_END$/p' "$log" \
    | tail -n 140 >&2 || true
  echo 'M23BV_RELIABLE_V3_RUNTIME_RAW_LAUNCH_OUTPUT_END' >&2
  echo 'M23BV_RELIABLE_V3_RUNTIME_PREFLIGHT=BLOCKED' >&2
  exit 2
fi
if [[ "$ORDER" == as_pinned ]]; then
  echo 'M23BV_RELIABLE_V3_RUNTIME_PREFLIGHT=PASS'
else
  echo 'M23BV_V3_EARLY_RECTIFIER_DIAGNOSTIC=PASS_ONLY'
  echo 'M23BV_V3_AS_PINNED_ACCEPTANCE=NOT_ESTABLISHED_BY_DIAGNOSTIC'
fi
echo 'M23BV_PRODUCTION_OR_PHYSICAL_CAPTURE=NOT_AUTHORIZED'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
