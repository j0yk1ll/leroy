#!/usr/bin/env bash
# M23L: synthetic in-memory 640x480 Image DDS only.
# No /dev/video* camera or /dev/ttyACM* STM32 mappings permitted.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${M23L_SYNTHETIC_DDS_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23L_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23L_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23l:/opt/m23k:/opt/m20:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23l-ros-logs
export XDG_CACHE_HOME=/tmp/m23l-cache
mkdir -p "$ROS_LOG_DIR"
echo 'M23L: isolated synthetic full-size Image DDS publisher/subscriber ONLY'
exec python3 -u /opt/m23l/m23l_synthetic_dds_baseline.py
