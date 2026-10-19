#!/usr/bin/env bash
# M23I default is NO DEVICE / synthetically sized in-memory tests.
# Optional explicit closed existing bag data is mounted read-only.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${M23I_FORENSICS_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "177" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23I_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23I_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23i:/opt/m23f:/opt/m23e:/opt/m23d:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23i-ros-logs
export XDG_CACHE_HOME=/tmp/m23i-cache
mkdir -p "$ROS_LOG_DIR"

if [[ -n "${M23I_DATA_ROOT:-}" ]]; then
  if [[ "${M23I_DATA_ROOT}" != "/input/recordings" ]]; then
    echo "M23I_PRECONDITION=UNEXPECTED_DATA_ROOT" >&2
    exit 2
  fi
  test -d /input/recordings
  echo "M23I: immutable first real-bag read-only M23A fault forensics"
  exec python3 -u /opt/m23f/m23f_existing_recording_inventory.py \
    --root /input/recordings --audit-cdr
fi

python3 -u /opt/m23i/m23i_long_bag_diagnostics_test.py
echo "M23I_SYNTHETIC_LONG_BAG_SELFTEST=PASS"
echo "M23I_DEVICE_ACCESS=NONE"
