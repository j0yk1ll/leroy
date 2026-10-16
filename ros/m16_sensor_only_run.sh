#!/usr/bin/env bash
# M16 only: no bridge, no serial device, no motors, no Nav2 control.
set -Eeuo pipefail

# ROS/ament setup scripts are not nounset-safe: they test unset variables
# (e.g. AMENT_TRACE_SETUP_FILES). Keep error checking but temporarily
# disable nounset ONLY for sourcing the trusted, installed ROS overlays.
set +u
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
set -u

if [[ "${LEROY_M16_SENSOR_ONLY:-}" != "1" ||
      "${LEROY_NAV2_DRY_RUN:-}" != "1" ||
      "${ROS_DOMAIN_ID:-}" != "168" ||
      "${ROS_AUTOMATIC_DISCOVERY_RANGE:-}" != "LOCALHOST" ]]; then
  echo 'FAIL: M16 isolated sensor-only environment not set' >&2
  exit 1
fi
# Build-time entrypoint regression: exercise this exact ROS setup sequence
# without needing a camera device or starting any ROS nodes. Not a sensor PASS.
if [[ "${LEROY_M16_ENV_SELFTEST:-0}" == "1" ]]; then
  PYTHONPATH="/opt/m16/monitor:${PYTHONPATH:-}" python3 -c \
    'import rclpy; import leroy_nav2_integration.m16_passive_observer; print("M16_ROS_ENV_SELFTEST=PASS")'
  exit 0
fi

if [[ "${M16_VIDEO_DEVICE:-}" != "/dev/video2" ]]; then
  echo 'FAIL: M16 only permits explicitly mapped /dev/video2' >&2
  exit 1
fi
if [[ ! -c /dev/video2 || ! -r /config/camera/front_camera.yaml ]]; then
  echo 'FAIL: camera device or checked-in calibration asset unavailable' >&2
  exit 1
fi

mkdir -p /tmp/m16-ros-logs
export ROS_LOG_DIR=/tmp/m16-ros-logs
export PYTHONPATH=/opt/m16/monitor:"${PYTHONPATH:-}"
export PYTHONDONTWRITEBYTECODE=1
export XDG_CACHE_HOME=/tmp/m16-cache

echo 'M16 SENSOR-ONLY: starting patched USB camera, rectifier and CPU DepthART'
ros2 launch /opt/m16/launch/m16_sensor_only.launch.py \
  video_device:=/dev/video2 \
  camera_info_file:=/config/camera/front_camera.yaml &
launch_pid=$!

cleanup() {
  trap - EXIT INT TERM
  echo 'M16 SENSOR-ONLY: stopping sensor launch'
  kill -INT "$launch_pid" 2>/dev/null || true
  local count
  for count in {1..60}; do
    if ! kill -0 "$launch_pid" 2>/dev/null; then
      wait "$launch_pid" 2>/dev/null || true
      return
    fi
    sleep .2
  done
  echo 'M16 SENSOR-ONLY: SIGINT cleanup timed out; terminating launch'
  kill -TERM "$launch_pid" 2>/dev/null || true
  wait "$launch_pid" 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT TERM

# CPU DepthART may need startup time; fail if launch process already exited.
sleep 12
if ! kill -0 "$launch_pid" 2>/dev/null; then
  echo 'FAIL: M16 sensor-only launch exited before observation' >&2
  exit 1
fi

echo 'M16 SENSOR-ONLY: verifying no motor/odom/bridge ROS graph'
if ros2 topic list --no-daemon | grep -Eq '^(/cmd_vel|/odom)$'; then
  echo 'FAIL: forbidden motor-compatible or odometry topic present' >&2
  exit 1
fi
if ros2 node list --no-daemon | grep -Eq '^/(leroy_bridge|controller_server|bt_navigator|planner_server|leroy_xrslam)$'; then
  echo 'FAIL: unexpected control/bridge/localization node in sensor-only domain' >&2
  exit 1
fi

echo 'M16 SENSOR-ONLY: begin bounded passive real-camera observation'
python3 -m leroy_nav2_integration.m16_passive_observer \
  | tee /tmp/m16-passive-report.log

python3 - <<'PY'
import json
from pathlib import Path
lines = Path('/tmp/m16-passive-report.log').read_text().splitlines()
prefix = 'M16_PASSIVE_REPORT_JSON='
reports = [json.loads(line[len(prefix):]) for line in lines
           if line.startswith(prefix)]
if len(reports) != 1:
    raise SystemExit('FAIL: exactly one completed M16 JSON report required')
report = reports[0]
streams = report['topics']
camera = streams['camera_image']
info = streams['camera_info']
depth = streams['depth_image']
source = depth['publishers']
acceptable = (
    all(streams[k]['accepted'] >= 2 and streams[k]['fresh_valid_receipt']
        for k in ('camera_image', 'camera_info', 'depth_image'))
    and report['depth_camera_info_stamp_frame_matches'] >= 1
    and len(source) == 1
    and source[0]['node'] == 'depthart'
    and source[0]['namespace'] == '/'
    and source[0]['type'] == 'sensor_msgs/msg/Image'
)
print('M16_CAMERA_DEPTH_VERDICT=' + (
    'OBSERVED_CAMERA_DEPTH_MESSAGE_CONTRACT'
    if acceptable else 'INCOMPLETE_OR_UNVERIFIED_CAMERA_DEPTH'), flush=True)
PY

echo 'M16 SENSOR-ONLY: report complete; cleanup follows'
