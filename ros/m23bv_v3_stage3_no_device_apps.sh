#!/usr/bin/env bash
# Stage 3: real DepthART + XRSLAM + Nav2 application startup, NO DEVICES.
set -Eeuo pipefail
SENSOR_IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
XRSLAM_IMAGE='localhost/leroy-ros:xrslam'
NAV2_IMAGE='localhost/leroy-nav2:jazzy'
SENSOR_ID='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
XRSLAM_ID='20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a'
NAV2_ID='89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4'
fail() { echo "M23BV_V3_STAGE3_APPS=BLOCKED_$1" >&2; exit 2; }
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || fail NOT_ROOTLESS
check_image() {
  local name="$1" pin="$2" role="$3" got
  podman image exists "$name" || fail "IMAGE_MISSING_$role"
  got="$(podman image inspect "$name" --format '{{.Id}}')" || fail IMAGE_INSPECTION
  got="$(printf '%s' "$got" | sed 's/^sha256://')"
  [[ "$got" == "$pin" ]] || fail "IMAGE_ID_DRIFT_$role"
  echo "STAGE3_APPS_IMAGE_$role=$got"
}
check_image "$SENSOR_IMAGE" "$SENSOR_ID" SENSOR
check_image "$XRSLAM_IMAGE" "$XRSLAM_ID" XRSLAM
check_image "$NAV2_IMAGE" "$NAV2_ID" NAV2
src="$(cd -- "$(dirname -- "$0")" && pwd -P)"
slam="$src/m23bv_v3_stage3_smoke_monocular.yaml"
device="$src/m23bv_v3_stage3_smoke_device.yaml"
launch="$src/m23bv_v3_stage3_nav2_smoke.launch.py"
audit="$src/m23bv_v3_stage3_app_smoke_audit.py"
for file in "$slam" "$device" "$launch" "$audit"; do
  [[ -r "$file" && -f "$file" ]] || fail MISSING_SOURCE
done
suffix="$$-$(date +%s)"
sensor="leroy-stage3-app-sensor-$suffix"
xrslam="leroy-stage3-app-xrslam-$suffix"
nav2="leroy-stage3-app-nav2-$suffix"
names=("$sensor" "$xrslam" "$nav2")
for name in "$sensor" "$xrslam" "$nav2"; do
  if podman container exists "$name"; then fail NAME_COLLISION; fi
done
created=()
cleanup() {
  trap - EXIT INT TERM
  local name
  for name in "${created[@]}"; do
    podman stop --time 7 "$name" >/dev/null 2>&1 || true
  done
  for name in "${created[@]}"; do
    podman rm -f "$name" >/dev/null 2>&1 || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
common=(
  --read-only --cap-drop all --security-opt no-new-privileges
  --tmpfs /tmp:rw,nosuid,nodev,size=128m
  --entrypoint /bin/bash
  --env M23BV_STAGE3_NO_DEVICE_APP_SMOKE=1
  --env ROS_DOMAIN_ID=170 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  --env ROS_LOG_DIR=/tmp/stage3-ros-logs
  --env XDG_CACHE_HOME=/tmp/stage3-cache
  --env PYTHONDONTWRITEBYTECODE=1
)
bootstrap='
set -e
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
test ! -e /dev/ttyACM0
test ! -e /dev/video2
exec "$@"
'
show_logs() {
  local name
  for name in "$sensor" "$xrslam" "$nav2"; do
    echo "--- STAGE3 LOGS $name ---" >&2
    podman logs --tail 50 "$name" >&2 || true
  done
}
created+=("$sensor")
podman run --detach --name "$sensor" \
  --network none --ipc shareable --pid private \
  "${common[@]}" "$SENSOR_IMAGE" \
  -c "$bootstrap" stage3 ros2 launch leroy_depthart depthart.launch.py \
  >/dev/null || fail DEPTHART_CONTAINER_START
[[ "$(podman inspect "$sensor" --format '{{.HostConfig.NetworkMode}}')" == none ]] ||
  fail PARENT_NETWORK_NOT_NONE
[[ "$(podman inspect "$sensor" --format '{{.State.Running}}')" == true ]] ||
  fail DEPTHART_CONTAINER_EARLY_EXIT
echo 'STAGE3_APPS_PARENT_NETWORK=NONE'
created+=("$xrslam")
podman run --detach --name "$xrslam" \
  --network "container:$sensor" --ipc "container:$sensor" --pid private \
  "${common[@]}" \
  --volume "$slam:/opt/leroy_stage3_slam.yaml:ro" \
  --volume "$device:/opt/leroy_stage3_device.yaml:ro" \
  "$XRSLAM_IMAGE" -c "$bootstrap" stage3 \
  ros2 run leroy_xrslam leroy_xrslam_node --ros-args \
    -p slam_yaml:=/opt/leroy_stage3_slam.yaml \
    -p device_template_yaml:=/opt/leroy_stage3_device.yaml \
    -p image_topic:=/camera/image_rect \
    -p camera_info_topic:=/camera/camera_info \
    -p camera_optical_frame:=camera_optical_frame \
    -p pose_convention_validated:=false \
    -p enable_map_tf:=false >/dev/null || fail XRSLAM_CONTAINER_START
created+=("$nav2")
podman run --detach --name "$nav2" \
  --network "container:$sensor" --ipc "container:$sensor" --pid private \
  "${common[@]}" \
  --volume "$launch:/opt/leroy_stage3_nav2_smoke.launch.py:ro" \
  --volume "$audit:/opt/leroy_stage3_app_smoke_audit.py:ro" \
  "$NAV2_IMAGE" -c "$bootstrap" stage3 \
  ros2 launch /opt/leroy_stage3_nav2_smoke.launch.py \
  >/dev/null || fail NAV2_CONTAINER_START
for name in "$sensor" "$xrslam" "$nav2"; do
  if [[ "$(podman inspect "$name" --format '{{.State.Running}}')" != true ]]; then
    show_logs
    fail EARLY_APP_EXIT
  fi
done
echo 'STAGE3_APPS_REAL_PROCESSES=DEPTHART_XRSLAM_NAV2'
if ! podman exec "$nav2" /bin/bash -c '
set -e
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
exec python3 /opt/leroy_stage3_app_smoke_audit.py
'; then
  show_logs
  fail REAL_APP_AUDIT_FAILED
fi
for name in "$sensor" "$xrslam" "$nav2"; do
  if [[ "$(podman inspect "$name" --format '{{.State.Running}}')" != true ]]; then
    show_logs
    fail APP_EXITED_DURING_AUDIT
  fi
done
printf '%s\n' \
  'M23BV_V3_STAGE3_APPS=PASS_REAL_APP_STARTUP_NO_DEVICE' \
  'M23BV_V3_STAGE3_APPS_SENSOR_INPUT=NOT_SUPPLIED' \
  'M23BV_V3_STAGE3_APPS_MCU_OPEN=NONE' \
  'M23BV_V3_STAGE3_APPS_MOTION_AND_RECORDING=NOT_AUTHORIZED'
