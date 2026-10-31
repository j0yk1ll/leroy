#!/usr/bin/env bash
# One bounded, non-actuating observation of the EXISTING live Stage 3 ROS
# graph. No camera, MCU, GPIO or motor device is mapped into this sidecar.
set -Eeuo pipefail
cd "$(dirname "$0")/../../.."

fail() { echo "LIVE_BOOT_LAUNCH=BLOCKED_$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGUMENTS
[[ "$(id -u)" -ne 0 ]] || fail ROOT_USER
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED

parent=leroy-live-costmap
xrslam=leroy-live-xrslam
sidecar=leroy-live-boot-readiness
image=localhost/leroy-ros:stage4c-capture-offline

for name in "$parent" "$xrslam"; do
  [[ "$(podman inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" == true ]] ||
    fail STAGE3_NOT_RUNNING
done
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
  fail PARENT_NETWORK_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.IpcMode}}' "$parent")" == shareable ]] ||
  fail PARENT_IPC_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.PidMode}}' "$parent")" == private ]] ||
  fail PARENT_PID_NOT_PRIVATE
podman exec "$parent" test ! -e /dev/ttyACM0 ||
  fail MCU_VISIBLE_IN_LIVE_PARENT
podman image exists "$image" || fail COMMISSIONED_ROS_IMAGE_MISSING
podman container exists "$sidecar" && fail SIDECAR_NAME_ALREADY_USED
for f in src/leroy_bringup/leroy_bringup/readiness.py src/leroy_localization/leroy_localization/localization_core.py; do
  [[ -f "$f" && ! -L "$f" ]] || fail SOURCE_MISSING_OR_SYMLINK
done

echo 'LIVE_BOOT_LAUNCH=STARTING_REAL_READ_ONLY_INPUTS'
exec podman run --rm --pull=never --name "$sidecar" \
  --network "container:$parent" \
  --ipc "container:$parent" \
  --pid "container:$parent" \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/src/leroy_bringup/leroy_bringup/readiness.py,dst=/opt/stage4/live_boot_readiness.py,readonly" \
  --mount "type=bind,src=$PWD/src/leroy_localization/leroy_localization,dst=/opt/stage4/leroy_localization,readonly" \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_LIVE_BOOT_READ_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/leroy-live-boot-ros \
  -e XDG_CACHE_HOME=/tmp/leroy-live-boot-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    test ! -e /dev/ttyACM0
    test ! -e /dev/video2
    test ! -e /dev/gpiochip0
    exec python3 -B -u /opt/stage4/live_boot_readiness.py
  '
