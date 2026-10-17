#!/usr/bin/env bash
# M21L: synthetic camera ROS publishers and in-memory IMU source only.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21L_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21L_PRECONDITION=PHYSICAL_SENSOR_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m21l:/opt/m21h:/opt/m21g:/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21l-ros-logs
export XDG_CACHE_HOME=/tmp/m21l-cache
mkdir -p "$ROS_LOG_DIR"
echo "M21L: simulated ROS producers and IMU metadata only, no hardware"
exec python3 -u /opt/m21l/m21l_ros_executor_quiescence_selftest.py
