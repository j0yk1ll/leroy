#!/usr/bin/env bash
# M22J: no real serial, cameras, motor devices or production bringup.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M22F_FAKE_SERIAL_ONLY:-}" != "1" ||
      "${M22J_BRIDGE_AUDIT_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M22J_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M22J_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m22j:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m22j-ros-logs
export XDG_CACHE_HOME=/tmp/m22j-cache
mkdir -p "$ROS_LOG_DIR"
echo "M22J: real ROS bridge owns non-actuating audit method; fake CDC only"
exec python3 -u /opt/m22j/m22j_real_bridge_signed_audit_ros_selftest.py
