#!/usr/bin/env bash
# M19 stationary 120-second IMU clock-rate audit, never the motor bridge.
set -Eeuo pipefail

# Installed ROS/ament setup scripts reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_M18_IMU_STREAM:-}" != "1" ||
      "${LEROY_M19_CLOCK_AUDIT:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "169" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo 'M19_PRECONDITION=ISOLATION_ENV_FAILED' >&2
  exit 1
fi
export PYTHONPATH="/opt/m19:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m19-ros-logs
export XDG_CACHE_HOME=/tmp/m19-cache
mkdir -p "$ROS_LOG_DIR"

if [[ "${LEROY_M19_SELFTEST:-0}" == "1" ]]; then
  exec python3 -u /opt/m19/m19_imu_clock_audit.py
fi

if [[ ! -c /dev/ttyACM0 ]]; then
  echo 'M19_PRECONDITION=MCU_SERIAL_DEVICE_MISSING' >&2
  exit 1
fi
exec python3 -u /opt/m19/m19_imu_clock_audit.py
