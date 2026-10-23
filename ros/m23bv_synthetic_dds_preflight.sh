#!/usr/bin/env bash
# No-device M23BV DDS integration, pinned to the accepted derived image.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
IMAGE=localhost/leroy-sensors:m23bv-capture-prep
EXPECTED_ID=57d886bf0b903e08ff1dc9f766d3daea28f44a20bcf9fd34733e7d3eab3029ec
: "${M23BV_LOG_DIR:?Set the existing owner-private log directory}"
if [[ "$M23BV_LOG_DIR" != /* || ! -d "$M23BV_LOG_DIR" ||
      -L "$M23BV_LOG_DIR" || ! -w "$M23BV_LOG_DIR" ]]; then
  echo 'M23BV_DDS=BLOCKED_LOG_DIR' >&2
  exit 2
fi
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}')"
actual="${actual#sha256:}"
if [[ "$actual" != "$EXPECTED_ID" ]]; then
  echo 'M23BV_DDS=BLOCKED_DERIVED_IMAGE_ID' >&2
  exit 2
fi
status="$(podman inspect leroy-ros --format '{{.State.Status}}' 2>/dev/null || true)"
if [[ "$status" == running ]]; then
  echo 'M23BV_DDS=BLOCKED_PRODUCTION_CONTAINER_RUNNING' >&2
  exit 2
fi
python3 - <<'PY'
import ast
from pathlib import Path
ast.parse(Path('m23bv_synthetic_dds_probe.py').read_text())
PY

log="$(mktemp --tmpdir="$M23BV_LOG_DIR" m23bv_dds_XXXXXXXX.log)"
# -i IS MANDATORY: the read-only Python test program is streamed over stdin.
# No extra images, network, cameras, MCU devices, process privileges, or motors.
set +e
podman run --rm -i --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  -e ROS_DOMAIN_ID=178 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-ros-logs; mkdir -p "$ROS_LOG_DIR"; exec python3 -u -' \
  <m23bv_synthetic_dds_probe.py >"$log" 2>&1
rc=$?
set -e
printf 'M23BV_DDS_PREFLIGHT_EXIT=%s\n' "$rc"
printf 'M23BV_DDS_PREFLIGHT_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_|Traceback|RuntimeError|Error:|TypeError:' "$log" | tail -n 85 || true
if [[ "$rc" != 0 ]] || ! grep -qx 'M23BV_SYNTHETIC_DDS_CAMERA_TRANSPORT=PASS' "$log"; then
  echo 'M23BV_DDS_PREFLIGHT=BLOCKED' >&2
  exit 2
fi
echo 'M23BV_DDS_PREFLIGHT=PASS'
echo 'M23BV_PHYSICAL_ACCESS=NONE'
echo 'M23BV_PRODUCTION_CAPTURE_OR_CALIBRATION=NOT_AUTHORIZED'
