#!/usr/bin/env bash
# M20 combined real front-camera + MCU-IMU sensors ONLY. Never motor bridge.
set -Eeuo pipefail

# ROS setup scripts may reference unset variables; contain nounset change.
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_M20_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "170" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo 'M20_PRECONDITION=ISOLATION_ENV_FAILED' >&2
  exit 1
fi

export PYTHONPATH="/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR=/tmp/m20-ros-logs
export XDG_CACHE_HOME=/tmp/m20-cache
mkdir -p "$ROS_LOG_DIR"

if [[ "${LEROY_M20_SELFTEST:-0}" == "1" ]]; then
  exec python3 -u /opt/m20/m20_combined_observer.py
fi

if [[ ! -c /dev/video2 || ! -c /dev/ttyACM0 ||
      ! -r /config/camera/front_camera.yaml ]]; then
  echo 'M20_PRECONDITION=CAMERA_MCU_OR_CALIBRATION_MISSING' >&2
  exit 1
fi

camera_pid=""
imu_pid=""
forced_shutdown=0
cleanup() {
  trap - EXIT INT TERM
  local pid name i
  for name in imu camera; do
    if [[ "$name" == "imu" ]]; then pid="$imu_pid"; else pid="$camera_pid"; fi
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      echo "M20_SHUTDOWN: sending SIGTERM to $name"
      kill -TERM "$pid" 2>/dev/null || true
      for i in {1..75}; do
        if ! kill -0 "$pid" 2>/dev/null; then break; fi
        sleep 0.2
      done
      if kill -0 "$pid" 2>/dev/null; then
        echo "M20_SHUTDOWN=FORCED_SIGKILL_$name" >&2
        kill -KILL "$pid" 2>/dev/null || true
        forced_shutdown=1
      fi
    fi
    if [[ -n "$pid" ]]; then wait "$pid" 2>/dev/null || true; fi
  done
  if [[ "$forced_shutdown" != "0" ]]; then exit 1; fi
  echo 'M20_SHUTDOWN=SENSOR_PROCESSES_EXITED'
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

echo 'M20: starting only usb_cam + image_proc rectifier (NO DepthART)'
ros2 launch /opt/m20/m20_camera_only.launch.py &
camera_pid=$!

sleep 9
if ! kill -0 "$camera_pid" 2>/dev/null; then
  echo 'M20_FAIL=CAMERA_LAUNCH_EARLY_EXIT' >&2
  exit 1
fi

# Local DDS graph guard BEFORE opening serial. Source script repeats checks.
if ros2 topic list --no-daemon | grep -Eq '^(/cmd_vel|/odom|/tf|/depthart/depth)$'; then
  echo 'M20_FAIL=FORBIDDEN_ROS_TOPICS' >&2
  exit 1
fi
if ros2 node list --no-daemon | grep -Eq '^/(leroy_bridge|leroy_xrslam|controller_server|planner_server|bt_navigator|depthart)$'; then
  echo 'M20_FAIL=FORBIDDEN_ROS_NODES' >&2
  exit 1
fi

echo 'M20: starting bounded physical stamped IMU and concurrent passive observer'
python3 -u /opt/m20/m20_combined_observer.py &
imu_pid=$!
wait "$imu_pid"
imu_pid=""
echo 'M20: combined observation complete; camera launch cleanup follows'
