#!/usr/bin/env bash
# M22K: signed diagnostic permission only; real serial never opened.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M22F_FAKE_SERIAL_ONLY:-}" != "1" ||
      "${M22K_DIAGNOSTIC_SUPERVISOR_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M22K_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M22K_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m22k:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m22k-ros-logs
export XDG_CACHE_HOME=/tmp/m22k-cache
mkdir -p "$ROS_LOG_DIR"
echo "M22K: actual ROS bridge diagnostic supervisor, injected FAKE CDC only"
exec python3 -u /opt/m22k/m22k_real_bridge_supervisor_ros_selftest.py
