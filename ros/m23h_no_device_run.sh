#!/usr/bin/env bash
# Defaults to hardware-free tests. Physical capture requires explicit opt-in.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
export PYTHONPATH="/opt/m23h:/opt/m23g:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23h-ros-logs
export XDG_CACHE_HOME=/tmp/m23h-cache
mkdir -p "$ROS_LOG_DIR"

if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23H_PRECONDITION=UNSAFE_ROS_ENV" >&2
  exit 2
fi

if [[ "${M23H_SELFTEST:-1}" == "1" ]]; then
  if [[ -e /dev/ttyACM0 || -e /dev/video2 ]]; then
    echo "M23H_SELFTEST_REFUSES_DEVICE_MAPPINGS" >&2
    exit 2
  fi
  python3 -u /opt/m23h/m23h_bounded_capture_test.py
  python3 -u /opt/m23h/m23h_synthetic_cli_recorder_selftest.py
  echo "M23H_SYNTHETIC_CLI_RECORDER_SELFTEST=PASS"
  echo "M23H_SIM_EXIT_CODE=0"
  exit 0
fi

if [[ "${M23H_SELFTEST}" != "0" ||
      "${M23H_OPERATOR_CAPTURE_APPROVED:-}" != "YES" ||
      "${M23H_SESSION_ID:-}" == "" ]]; then
  echo "M23H_PRECONDITION=CAPTURE_NOT_OPERATOR_APPROVED" >&2
  exit 2
fi
if [[ ! -c /dev/video2 || ! -c /dev/ttyACM0 ]]; then
  echo "M23H_PRECONDITION=SENSOR_DEVICES_ABSENT" >&2
  exit 2
fi
if [[ ! -d /recordings || ! -w /recordings ]]; then
  echo "M23H_PRECONDITION=WRITABLE_OFFLINE_BAG_MOUNT_ABSENT" >&2
  exit 2
fi

echo "M23H: operator-approved sensor-only M20 + exact-topic sqlite recorder"
exec python3 -u /opt/m23h/m23h_bounded_capture.py \
  --output-root /recordings --session-id "$M23H_SESSION_ID"
