#!/usr/bin/env bash
# Only runs in a no-device isolated rootless Podman ROS 2 image.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" || \
      "${M22F_FAKE_SERIAL_ONLY:-}" != "1" || \
      "${ROS_DOMAIN_ID:-}" != "170" || \
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M22F_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M22F_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m22f:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m22f-ros-logs
export XDG_CACHE_HOME=/tmp/m22f-cache
mkdir -p "$ROS_LOG_DIR"
echo "M22F: ACTUAL ROS bridge node on injected fake CDC; physical device access disabled"
exec python3 -u /opt/m22f/m22f_fake_bridge_ros_selftest.py
