#!/usr/bin/env bash
# M23A: synthetic ROS2 rosbag roundtrip, no ROS nodes or physical devices.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23A_BAG_REPLAY_PREFLIGHT_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "171" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23A_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23A_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23a-ros-logs
export XDG_CACHE_HOME=/tmp/m23a-cache
mkdir -p "$ROS_LOG_DIR"
echo "M23A: read-only bag input validation using SYNTHETIC ROS Jazzy CDR only"
exec python3 -u /opt/m23a/m23a_rosbag_roundtrip_selftest.py
