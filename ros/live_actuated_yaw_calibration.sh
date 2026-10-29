#!/usr/bin/env bash
# ACTUATING: sends signed ROS yaw commands only; signed bridge owns MCU.
# An independent authenticated safety monitor MUST already be commissioned.
set -Eeuo pipefail
cd "$(dirname "$0")"
fail() { printf 'ACTUATED_YAW_LAUNCH=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 && "$(id -u)" != 0 ]] || fail INVALID_ARGS_OR_ROOT
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || fail NOT_ROOTLESS
parent=leroy-live-costmap
bridge=leroy-signed-bridge
monitor=leroy-physical-safety-monitor
controller=leroy-actuated-yaw-calibration
image=localhost/leroy-ros:stage4c-capture-offline
for name in "$parent" "$bridge" "$monitor"; do
  [[ "$(podman inspect -f '{{.State.Running}}' "$name" 2>/dev/null)" == true ]] ||
    fail REQUIRED_REAL_HARDWARE_SERVICE_NOT_RUNNING
done
for name in leroy-boot-imu-source leroy-stage4-imu leroy-live-boot-yaw-persistent "$controller"; do
  podman container exists "$name" && fail CONFLICTING_SERIAL_OWNER_OR_CONTROLLER
done
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
  fail NONISOLATED_PARENT
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$bridge")" == "container:$parent" ]] ||
  fail BRIDGE_NOT_IN_PROTECTED_STAGE3_DDS
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$monitor")" == "container:$parent" ]] ||
  fail MONITOR_NOT_IN_PROTECTED_STAGE3_DDS
podman exec "$parent" test ! -e /dev/ttyACM0 || fail PARENT_MCU_VISIBLE
podman exec "$monitor" test ! -e /dev/ttyACM0 || fail MONITOR_MCU_VISIBLE
podman image exists "$image" || fail IMAGE_MISSING
for file in autonomous_motor_yaw_core.py live_actuated_yaw_calibration.py boot_yaw_core.py boot_auto_calibration_core.py boot_yaw_tracking_core.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail CODE_MISSING_OR_SYMLINK
done

validate_key() {
  local file
  file="${!1:-}"
  [[ "$file" == /* && -f "$file" && ! -L "$file" ]] ||
    fail PROTECTED_KEY_MISSING
  [[ "$(stat -Lc '%u' "$file")" == "$(id -u)" &&
     "$(stat -Lc '%a' "$file")" == 600 &&
     "$(stat -Lc '%s' "$file")" == 32 &&
     "$(stat -Lc '%h' "$file")" == 1 ]] ||
    fail PROTECTED_KEY_OWNER_PERMISSIONS_SIZE
}
validate_key LEROY_MOTOR_MOTION_KEY_PATH
validate_key LEROY_MOTOR_OPERATOR_KEY_PATH
validate_key LEROY_SAFETY_MONITOR_VERIFY_KEY_PATH
[[ "$LEROY_MOTOR_MOTION_KEY_PATH" != "$LEROY_MOTOR_OPERATOR_KEY_PATH" &&
   "$LEROY_MOTOR_MOTION_KEY_PATH" != "$LEROY_SAFETY_MONITOR_VERIFY_KEY_PATH" &&
   "$LEROY_MOTOR_OPERATOR_KEY_PATH" != "$LEROY_SAFETY_MONITOR_VERIFY_KEY_PATH" ]] ||
  fail DISTINCT_KEY_ROLES_REQUIRED

printf '%s\n' \
 'This application CAN sign nonzero motor yaw commands.' \
 'An independently commissioned whole-swept-volume safety monitor and' \
 'real bridge motor interlocks must already be operational.' \
 'No software flag substitutes for hardwired stopping.' \
 'Ctrl-C revokes this session; no automatic rearm after power loss.'
read -r -p 'Type ARM_PHYSICAL_AUTONOMOUS_YAW for this session: ' approval
[[ "$approval" == ARM_PHYSICAL_AUTONOMOUS_YAW ]] ||
  fail OPERATOR_APPROVAL_MISSING
echo ACTUATED_YAW_LAUNCH=STARTING_SIGNED_CONTROLLER

exec podman run --rm --pull=never --name "$controller" \
  --userns=keep-id --network "container:$parent" \
  --ipc "container:$parent" --pid "container:$parent" \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=96m \
  --mount "type=bind,src=$LEROY_MOTOR_MOTION_KEY_PATH,dst=/run/secrets/motor-motion,readonly" \
  --mount "type=bind,src=$LEROY_MOTOR_OPERATOR_KEY_PATH,dst=/run/secrets/motor-operator,readonly" \
  --mount "type=bind,src=$LEROY_SAFETY_MONITOR_VERIFY_KEY_PATH,dst=/run/secrets/safety-verifier,readonly" \
  --mount "type=bind,src=$PWD/live_actuated_yaw_calibration.py,dst=/opt/leroy_boot/live_actuated_yaw_calibration.py,readonly" \
  --mount "type=bind,src=$PWD/autonomous_motor_yaw_core.py,dst=/opt/leroy_boot/autonomous_motor_yaw_core.py,readonly" \
  --mount "type=bind,src=$PWD/boot_yaw_tracking_core.py,dst=/opt/leroy_boot/boot_yaw_tracking_core.py,readonly" \
  --mount "type=bind,src=$PWD/boot_yaw_core.py,dst=/opt/leroy_boot/boot_yaw_core.py,readonly" \
  --mount "type=bind,src=$PWD/boot_auto_calibration_core.py,dst=/opt/leroy_boot/boot_auto_calibration_core.py,readonly" \
  -e LEROY_MOTION_KEY=/run/secrets/motor-motion \
  -e LEROY_OPERATOR_KEY=/run/secrets/motor-operator \
  -e LEROY_SAFETY_MONITOR_KEY=/run/secrets/safety-verifier \
  -e LEROY_SIGNED_YAW_CONTROLLER=1 \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/leroy-actuated-yaw \
  -e XDG_CACHE_HOME=/tmp/leroy-actuated-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    export PYTHONPATH="/opt/leroy_boot:${PYTHONPATH:-}"
    test ! -e /dev/ttyACM0
    test ! -e /dev/video2
    test ! -e /dev/gpiochip0
    exec python3 -B -u /opt/leroy_boot/live_actuated_yaw_calibration.py
  '
