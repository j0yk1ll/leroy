#!/usr/bin/env bash
# Live Stage 3 yaw observer. Uses the existing motor-isolated M18 Stage 4A
# IMU source: no MCU, camera, GPIO or wheel device mapped to this process.
set -Eeuo pipefail
cd "$(dirname "$0")"

fail() { printf 'BOOT_LIVE_YAW_LAUNCH=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGUMENTS
[[ "$(id -u)" != 0 ]] || fail ROOT_HOST
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED

parent=leroy-live-costmap
source=leroy-stage4-imu
sidecar=leroy-live-boot-yaw
image=localhost/leroy-ros:stage4c-capture-offline

[[ "$(podman inspect -f '{{.State.Running}}' "$parent" 2>/dev/null)" == true ]] ||
  fail STAGE3_PARENT_NOT_RUNNING
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
  fail NOT_NETWORK_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.IpcMode}}' "$parent")" == shareable ]] ||
  fail IPC_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.PidMode}}' "$parent")" == private ]] ||
  fail PID_NOT_PRIVATE
[[ "$(podman inspect -f '{{.State.Running}}' "$source" 2>/dev/null)" == true ]] ||
  fail REAL_STAGE4A_M18_IMU_SOURCE_NOT_RUNNING
podman exec "$parent" test ! -e /dev/ttyACM0 ||
  fail MOTOR_CAPABLE_MCU_VISIBLE_TO_PARENT
podman image exists "$image" || fail ROS_IMAGE_MISSING
podman container exists "$sidecar" && fail SIDECAR_NAME_OCCUPIED
for file in boot_yaw_core.py live_boot_yaw.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail PYTHON_SOURCE_MISSING_OR_SYMLINK
done

echo 'BOOT_LIVE_YAW_LAUNCH=STARTING_REAL_MCU_GYRO_DDS_ONLY'
exec podman run --rm --pull=never --name "$sidecar" \
  --network "container:$parent" \
  --ipc "container:$parent" \
  --pid "container:$parent" \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/boot_yaw_core.py,dst=/opt/leroy_boot/boot_yaw_core.py,readonly" \
  --mount "type=bind,src=$PWD/live_boot_yaw.py,dst=/opt/leroy_boot/live_boot_yaw.py,readonly" \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_BOOT_IMU_YAW_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/leroy-boot-yaw-ros \
  -e XDG_CACHE_HOME=/tmp/leroy-boot-yaw-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    test ! -e /dev/ttyACM0
    test ! -e /dev/video2
    test ! -e /dev/gpiochip0
    exec python3 -B -u /opt/leroy_boot/live_boot_yaw.py
  '
