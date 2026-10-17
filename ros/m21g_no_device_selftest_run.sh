#!/usr/bin/env bash
# M21G guarded source-latch tests: ONLY in-memory telemetry and no devices.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21G_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21G_PRECONDITION=UNEXPECTED_PHYSICAL_DEVICE" >&2
  exit 2
fi
export PYTHONPATH="/opt/m21g:/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21g-ros-logs
export XDG_CACHE_HOME=/tmp/m21g-cache
mkdir -p "$ROS_LOG_DIR"
echo "M21G: simulated M18 fail-latch tests only; no serial, USB or camera"
exec python3 -u /opt/m21g/m21g_fault_latch_selftest.py
