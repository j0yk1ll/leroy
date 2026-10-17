#!/usr/bin/env bash
# M21E pure ROS simulator: no camera / MCU device access and no motors.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21E_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi

if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21E_PRECONDITION=UNEXPECTED_PHYSICAL_DEVICE" >&2
  exit 2
fi

export PYTHONPATH="/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21e-ros-logs
export XDG_CACHE_HOME=/tmp/m21e-cache
mkdir -p "$ROS_LOG_DIR"

echo "M21E: synthetic publishers only, without USB or MCU access"
exec python3 -u /opt/m21e/m21e_ros_adapter_selftest.py
