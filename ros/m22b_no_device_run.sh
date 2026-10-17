#!/usr/bin/env bash
# M22B isolated synthetic ROS authority. NO camera, USB, or motor path.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M22B_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M22B_PRECONDITION=PHYSICAL_SENSOR_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m22b:/opt/m21m:/opt/m21l:/opt/m21h:/opt/m21g:/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m22b-ros-logs
export XDG_CACHE_HOME=/tmp/m22b-cache
mkdir -p "$ROS_LOG_DIR"
echo "M22B: isolated ROS software-authority diagnostics only, no hardware"
python3 -u /opt/m22b/m22a_diagnostic_authority_test.py
exec python3 -u /opt/m22b/m22b_isolated_ros_authority_selftest.py
