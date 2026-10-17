#!/usr/bin/env bash
# M21H simulated CDC cleanup: ONLY a mock port; no physical devices.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21H_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21H_PRECONDITION=UNEXPECTED_PHYSICAL_DEVICE" >&2
  exit 2
fi
export PYTHONPATH="/opt/m21h:/opt/m21g:/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21h-ros-logs
export XDG_CACHE_HOME=/tmp/m21h-cache
mkdir -p "$ROS_LOG_DIR"
echo "M21H: simulated port fault injection only; no hardware access"
exec python3 -u /opt/m21h/m21h_fake_cdc_cleanup_test.py
