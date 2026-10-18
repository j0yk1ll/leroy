#!/usr/bin/env bash
# Synthetic camera/IMU CDR to actual XRSLAM. NO hardware or production ROS.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23C_SYNTHETIC_VIO_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "173" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23C_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23C_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23c:/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23c-ros-logs
export XDG_CACHE_HOME=/tmp/m23c-cache
mkdir -p "$ROS_LOG_DIR"
echo "M23C: real XRSLAM monocular_imu with SYNTHETIC camera+IMU; no hardware"
exec python3 -u /opt/m23c/m23c_real_xrslam_synthetic_vio_selftest.py
