#!/usr/bin/env bash
# M18 isolated timestamped MCU IMU stream; DO NOT RUN ROBOT BRIDGE.
set -Eeuo pipefail

# Installed ROS and ament shell setup scripts are not nounset-safe.
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_M18_IMU_STREAM:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "169" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo 'M18_PRECONDITION=ISOLATION_ENV_FAILED' >&2
  exit 1
fi
export PYTHONPATH="/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m18-ros-logs
export XDG_CACHE_HOME=/tmp/m18-cache
mkdir -p "$ROS_LOG_DIR"

if [[ "${LEROY_M18_SELFTEST:-0}" == "1" ]]; then
  exec python3 -u /opt/m18/m18_imu_stream_source.py --self-test
fi

if [[ ! -c /dev/ttyACM0 ]]; then
  echo 'M18_PRECONDITION=MCU_SERIAL_DEVICE_MISSING' >&2
  exit 1
fi
exec python3 -u /opt/m18/m18_imu_stream_source.py
