#!/usr/bin/env bash
# M21F: in-memory fake MCU telemetry -> real M18 ROS publisher -> metadata.
# Absolutely no MCU tty, V4L2 or motor-capable production service.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21F_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21F_PRECONDITION=UNEXPECTED_PHYSICAL_DEVICE" >&2
  exit 2
fi
export PYTHONPATH="/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21f-ros-logs
export XDG_CACHE_HOME=/tmp/m21f-cache
mkdir -p "$ROS_LOG_DIR"
echo "M21F: synthetic MCU telemetry only; no serial or camera hardware"
exec python3 -u /opt/m21f/m21f_imu_source_hook_selftest.py
