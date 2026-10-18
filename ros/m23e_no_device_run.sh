#!/usr/bin/env bash
# Synthetic-only fixture by default; optional read-only existing bag data.
# No ROS nodes, hardware access, physical cameras, STM32, or robot bridge.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23E_OFFLINE_EVIDENCE_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "175" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23E_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23E_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23e:/opt/m23d:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23e-ros-logs
export XDG_CACHE_HOME=/tmp/m23e-cache
mkdir -p "$ROS_LOG_DIR"

if [[ -n "${M23E_BAG_PATH:-}" ]]; then
  if [[ "${M23E_BAG_PATH}" != "/input/bag" ]]; then
    echo "M23E_PRECONDITION=UNEXPECTED_DATA_MOUNT" >&2
    exit 2
  fi
  test -f /input/bag/metadata.yaml
  echo "M23E: read-only existing bag evidence; provenance UNAUTHENTICATED"
  exec python3 -u /opt/m23e/m23e_offline_evidence_snapshot.py \
    --bag /input/bag --origin-label user_declared_recorded
fi

echo "M23E: isolated synthetic CDR evidence test; NO real bag or calibration"
exec python3 -u /opt/m23e/m23e_synthetic_cdr_evidence_selftest.py
