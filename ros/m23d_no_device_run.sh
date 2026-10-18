#!/usr/bin/env bash
# M23D: synthetic rosbag smoke or read-only candidate bag audit. NO devices.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23D_OFFLINE_BAG_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "174" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23D_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23D_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23d:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23d-ros-logs
export XDG_CACHE_HOME=/tmp/m23d-cache
mkdir -p "$ROS_LOG_DIR"

if [[ -n "${M23D_BAG_PATH:-}" ]]; then
  if [[ "${M23D_BAG_PATH}" != "/input/bag" ]]; then
    echo "M23D_PRECONDITION=UNEXPECTED_BAG_MOUNT" >&2
    exit 2
  fi
  test -d /input/bag
  echo "M23D: read-only supplied bag profile; recording provenance UNVERIFIED"
  exec python3 -u /opt/m23d/m23d_recorded_bag_readiness.py \
    --bag /input/bag --declared-origin user_declared_recorded
fi

echo "M23D: synthetic ROS bag profile only; no real capture or calibration"
exec python3 -u /opt/m23d/m23d_synthetic_rosbag_profile_selftest.py
