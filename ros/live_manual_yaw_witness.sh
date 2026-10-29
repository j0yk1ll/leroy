#!/usr/bin/env bash
# Run ONLY a real, unpowered, read-only full startup-yaw witness.
# Joins the already-live Stage 3 ROS graph; no devices or motor authority.
set -Eeuo pipefail
cd "$(dirname "$0")"
fail() { printf 'MANUAL_YAW_LAUNCH=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGS
[[ "$(id -u)" != 0 ]] || fail ROOT_HOST
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail NOT_ROOTLESS
parent=leroy-live-costmap
source=leroy-boot-imu-source
yaw=leroy-live-boot-yaw-persistent
observer=leroy-manual-yaw-witness
image=localhost/leroy-ros:stage4c-capture-offline

for name in "$parent" "$source" "$yaw"; do
  [[ "$(podman inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" == true ]] ||
    fail REQUIRED_LIVE_SESSION_NOT_RUNNING
done
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
  fail STAGE3_NETWORK_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.IpcMode}}' "$parent")" == shareable ]] ||
  fail STAGE3_IPC_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.PidMode}}' "$parent")" == private ]] ||
  fail STAGE3_PID_NOT_PRIVATE
podman exec "$parent" test ! -e /dev/ttyACM0 ||
  fail MOTOR_CAPABLE_MCU_VISIBLE_IN_PARENT
podman image exists "$image" || fail SOURCE_IMAGE_MISSING
podman container exists "$observer" && fail OBSERVER_ALREADY_RUNNING
for file in live_manual_yaw_witness.py boot_yaw_tracking_core.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail WITNESS_SOURCE_MISSING
done

printf '%s\n' \
  'This tool never controls wheels. It observes the current 50Hz MCU-derived' \
  'relative gyro heading while the chassis is manually and safely rotated.' \
  'Target sequence relative to start: -90 deg, +90 deg, back to 0 deg.' \
  'The diagnostic angular-rate proposal is NOT a motor command.'
echo 'MANUAL_YAW_LAUNCH=STARTING_REAL_READ_ONLY_WITNESS'
exec podman run --rm --pull=never --name "$observer" \
  --network "container:$parent" --ipc "container:$parent" \
  --pid "container:$parent" \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/live_manual_yaw_witness.py,dst=/opt/leroy_boot/live_manual_yaw_witness.py,readonly" \
  --mount "type=bind,src=$PWD/boot_yaw_tracking_core.py,dst=/opt/leroy_boot/boot_yaw_tracking_core.py,readonly" \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_MANUAL_SWEEP_READ_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/leroy-manual-yaw-witness \
  -e XDG_CACHE_HOME=/tmp/leroy-manual-yaw-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    test ! -e /dev/ttyACM0
    test ! -e /dev/video2
    test ! -e /dev/gpiochip0
    exec python3 -B -u /opt/leroy_boot/live_manual_yaw_witness.py
  '
