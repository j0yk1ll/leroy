#!/usr/bin/env bash
# Read-only offline bag chronology, NO sensor opening or ROS node launch.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${M23J_CLOSED_BAG_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "178" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23J_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23J_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23j:/opt/m23i:/opt/m23f:/opt/m23e:/opt/m23d:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23j-ros-logs
export XDG_CACHE_HOME=/tmp/m23j-cache
mkdir -p "$ROS_LOG_DIR"

if [[ -n "${M23J_BAG:-}" ]]; then
  if [[ "${M23J_BAG}" != "/input/recordings" ||
        ! -f /input/recordings/metadata.yaml ]]; then
    echo "M23J_PRECONDITION=EXACT_CLOSED_BAG_MOUNT_REQUIRED" >&2
    exit 2
  fi
  echo "M23J: reading ONLY the explicitly mounted closed bag"
  exec python3 -u /opt/m23j/m23j_real_bag_timeline.py \
    --bag /input/recordings
fi

python3 -u /opt/m23j/m23j_real_bag_timeline_test.py
echo "M23J_SYNTHETIC_TIMELINE_SELFTEST=PASS"
echo "M23J_DEVICE_ACCESS=NONE"
