#!/usr/bin/env bash
# No-device V3 RELIABLE rectifier closed-bag CDR proof (separate).
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
IMAGE=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
: "${M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID:?Require independently built V3 image ID}"
EXPECTED_ID="${M23BV_RELIABLE_V3_EXPECTED_IMAGE_ID}"
if [[ ! "$EXPECTED_ID" =~ ^[[:xdigit:]]{64}$ ]]; then
  echo 'M23BV_V3_BAG=BLOCKED_PIN_FORMAT' >&2
  exit 2
fi
: "${M23BV_LOG_DIR:?Set the existing owner-private log directory}"
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_V3_BAG=BLOCKED_LOG_DIR' >&2
  exit 2
fi
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo 'M23BV_V3_BAG=BLOCKED_UNPINNED_IMAGE' >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo 'M23BV_V3_BAG=BLOCKED_PRODUCTION_CONTAINER_RUNNING' >&2
  exit 2
fi
python3 - <<'PY'
import ast
from pathlib import Path
ast.parse(Path('m23bv_reliable_v3_bag_probe.py').read_text())
PY

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_reliable_v3_bag_XXXXXXXX.log)"
# -i IS MANDATORY: the read-only Python test program is streamed over stdin.
# No extra images, network, cameras, MCU devices, process privileges, or motors.
set +e
# Stream an exact M23I-corrected auditor module before the DDS main test.
# No host path/volume/device is passed into Podman; both sources arrive on stdin.
{
  printf '%s\n' 'import base64, sys, types'
  printf '%s\n' 'audit_module = types.ModuleType("m23bv_m23i_independent_preflight")'
  printf '%s\n' 'sys.modules[audit_module.__name__] = audit_module'
  printf '%s' 'exec(compile(base64.b64decode("'
  base64 -w0 m23bv_m23i_independent_preflight.py
  printf '%s\n' '"), "m23bv_m23i_independent_preflight.py", "exec"), audit_module.__dict__)'
  cat m23bv_reliable_v3_bag_probe.py
} | podman run --rm -i --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  -e ROS_DOMAIN_ID=178 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-ros-logs; mkdir -p "$ROS_LOG_DIR"; exec python3 -u -' \
  >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_V3_BAG_PREFLIGHT_EXIT=%s\n' "$rc"
printf 'M23BV_V3_BAG_PREFLIGHT_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|Traceback|RuntimeError|Error:|TypeError:' "$log" | tail -n 85 || true
if [[ "$rc" != 0 ]] || \
   ! grep -qx 'M23BV_V3_BAG_CPP_SOURCE_PIN=PASS' "$log" || \
   ! grep -qx 'M23BV_DDS_GRAPH_QOS=PASS' "$log" || \
   ! grep -qx 'M23BV_V3_BAG_STAMP_INFO_LAYOUT_AND_CENTER_PIXEL=PASS' "$log" || \
   ! grep -qx 'M23BV_DERIVED_CDR_AND_RECEIPT_EXACT_MATCH=PASS' "$log" || \
   ! grep -qx 'M23BV_M23I_INDEPENDENT_DERIVED_AUDIT=PASS' "$log" || \
   ! grep -qx 'M23BV_V3_BAG_CLOSED_SQLITE_CDR=PASS' "$log" || \
   ! grep -qx 'M23BV_V3_BAG_DATA_INTEGRITY=PASS' "$log"; then
  echo 'M23BV_V3_BAG_PREFLIGHT=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_V3_BAG_PREFLIGHT=PASS'
echo 'M23BV_V3_BAG_CDR_AND_FINALIZATION=PASS'
echo 'M23BV_V3_BAG_INDEPENDENT_M23I=PASS'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
echo 'M23BV_PRODUCTION_CAPTURE_OR_CALIBRATION=NOT_AUTHORIZED'
echo 'M23BV_V3_PHYSICAL_GEOMETRIC_RECTIFICATION=UNVERIFIED'
