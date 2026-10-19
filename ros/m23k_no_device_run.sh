#!/usr/bin/env bash
# M23K defaults to no-device tests. Live mode uses camera ONLY, no MCU.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
export PYTHONPATH="/opt/m23k:/opt/m20:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23k-ros-logs
export XDG_CACHE_HOME=/tmp/m23k-cache
mkdir -p "$ROS_LOG_DIR"

if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23K_PRECONDITION=WRONG_ROS_ENV" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ]]; then
  echo "M23K_PRECONDITION=MCU_DEVICE_MAPPING_PROHIBITED" >&2
  exit 2
fi

if [[ "${M23K_PROBE_MODE:-selftest}" != "live" ]]; then
  if [[ -e /dev/video2 ]]; then
    echo "M23K_PRECONDITION=CAMERA_MAPPING_IN_SELFTEST" >&2
    exit 2
  fi
  python3 -u /opt/m23k/m23k_camera_path_probe_test.py
  echo "M23K_NO_DEVICE_SELFTEST=PASS"
  echo "M23K_DEVICE_ACCESS=NONE"
  exit 0
fi

if [[ "${M23K_OPERATOR_CAMERA_ONLY_APPROVED:-}" != "YES" ||
      ! -c /dev/video2 ]]; then
  echo "M23K_PRECONDITION=NO_APPROVAL_OR_CAMERA_MAPPING" >&2
  exit 2
fi
echo "M23K: bounded camera-only raw-versus-rect probe (no STM32)"
exec python3 -u /opt/m23k/m23k_camera_path_probe.py
