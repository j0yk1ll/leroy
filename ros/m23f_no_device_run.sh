#!/usr/bin/env bash
# M23F: isolated SYNTHETIC smoke or explicit read-only existing data folder.
# Physical CDC open can reset STM32 and energize motors. Do not attach.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23F_EXISTING_DATA_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "176" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23F_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23F_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23f:/opt/m23e:/opt/m23d:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23f-ros-logs
export XDG_CACHE_HOME=/tmp/m23f-cache
mkdir -p "$ROS_LOG_DIR"

if [[ -n "${M23F_DATA_ROOT:-}" ]]; then
  if [[ "${M23F_DATA_ROOT}" != "/input/recordings" ]]; then
    echo "M23F_PRECONDITION=UNEXPECTED_DATA_ROOT" >&2
    exit 2
  fi
  test -d /input/recordings
  echo "M23F: inspecting explicit pre-existing read-only bag directory ONLY"
  exec python3 -u /opt/m23f/m23f_existing_recording_inventory.py \
    --root /input/recordings --audit-cdr
fi

echo "M23F: SYNTHETIC inventory fixtures ONLY, no real bag or hardware"
exec python3 -u /opt/m23f/m23f_synthetic_inventory_selftest.py
