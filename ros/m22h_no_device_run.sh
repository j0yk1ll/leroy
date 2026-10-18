#!/usr/bin/env bash
# M22H: strict isolated ROS signed diagnostic integration, fake MCU only.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M22F_FAKE_SERIAL_ONLY:-}" != "1" ||
      "${M22H_FAKE_SIGNED_ROS_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M22H_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M22H_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m22h:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m22h-ros-logs
export XDG_CACHE_HOME=/tmp/m22h-cache
mkdir -p "$ROS_LOG_DIR"
echo "M22H: HMAC-signed ROS diagnostics beside REAL bridge, both fake-only"
exec python3 -u /opt/m22h/m22h_signed_ros_fake_bridge_selftest.py
