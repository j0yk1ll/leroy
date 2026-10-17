#!/usr/bin/env bash
# M21M: simulated producer-fault rejection and ROS executor success path.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u
if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M21M_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/video2 || -e /dev/ttyACM0 ]]; then
  echo "M21M_PRECONDITION=PHYSICAL_SENSOR_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m21m:/opt/m21l:/opt/m21h:/opt/m21g:/opt/m21f:/opt/m21e:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m21m-ros-logs
export XDG_CACHE_HOME=/tmp/m21m-cache
mkdir -p "$ROS_LOG_DIR"
echo "M21M: synthetic producer fault matrix and ROS joins; no hardware"
python3 -u /opt/m21m/m21m_producer_fault_test.py
python3 -u /opt/m21m/m21l_ros_executor_quiescence_selftest.py
echo "M21M_ROS_FAULT_INTEGRATION_SELFTEST=PASS"
echo "M21M_DEVICE_ACCESS=NONE"
