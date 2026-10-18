#!/usr/bin/env bash
# M23B: strictly synthetic monocular frames to the actual XRSLAM C++ node.
# The device-free container must be --network none with no device mapping.
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${M23B_SYNTHETIC_XRSLAM_ONLY:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "172" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo "M23B_PRECONDITION=ISOLATION_ENV_FAILED" >&2
  exit 2
fi
if [[ -e /dev/ttyACM0 || -e /dev/ttyACM1 ||
      -e /dev/video2 || -e /dev/video8 ]]; then
  echo "M23B_PRECONDITION=PHYSICAL_DEVICE_PRESENT" >&2
  exit 2
fi
export PYTHONPATH="/opt/m23b:/opt/m23a:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m23b-ros-logs
export XDG_CACHE_HOME=/tmp/m23b-cache
mkdir -p "$ROS_LOG_DIR"
echo "M23B: real XRSLAM executable, synthetic monocular bag ONLY, no odometry or motors"
exec python3 -u /opt/m23b/m23b_real_xrslam_monocular_replay_selftest.py
