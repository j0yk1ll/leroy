#!/usr/bin/env bash
# One supervised, persistent, SENSOR-ONLY session: MCU/M18 -> ROS IMU ->
# gyro relative yaw. Use the existing isolated Stage 3 graph. NO motors.
# Press Ctrl-C to end; always attempt MCU IMU STOP with ACK.
set -Eeuo pipefail
cd "$(dirname "$0")"

fail() { printf 'BOOT_LIVE_SESSION=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# == 0 ]] || fail UNEXPECTED_ARGUMENTS
[[ "$(id -u)" != 0 ]] || fail ROOT_HOST
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED
[[ "${LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED:-}" == YES ]] ||
  fail MOTOR_RAILS_NOT_PHYSICALLY_ISOLATED
[[ "${LEROY_STAGE4_MCU_BY_ID:-}" == /dev/serial/by-id/* ]] ||
  fail APPROVED_MCU_BY_ID_REQUIRED
[[ -L "$LEROY_STAGE4_MCU_BY_ID" ]] || fail MCU_ID_SYMLINK_MISSING
mcu="$(readlink -f -- "$LEROY_STAGE4_MCU_BY_ID")"
[[ "$mcu" == /dev/ttyACM0 && -c "$mcu" ]] ||
  fail MCU_DEVICE_ID_CHANGED
device_id="$(stat -Lc '%t:%T' "$mcu")"

parent=leroy-live-costmap
xrslam=leroy-live-xrslam
source=leroy-boot-imu-source
image=localhost/leroy-ros:stage4b-imu

[[ "$(podman inspect -f '{{.State.Running}}' "$parent" 2>/dev/null)" == true ]] ||
  fail STAGE3_PARENT_NOT_RUNNING
[[ "$(podman inspect -f '{{.State.Running}}' "$xrslam" 2>/dev/null)" == true ]] ||
  fail STAGE3_XRSLAM_NOT_RUNNING
[[ "$(podman inspect -f '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
  fail NETWORK_NOT_ISOLATED
[[ "$(podman inspect -f '{{.HostConfig.IpcMode}}' "$parent")" == shareable ]] ||
  fail IPC_NOT_SHAREABLE
[[ "$(podman inspect -f '{{.HostConfig.PidMode}}' "$parent")" == private ]] ||
  fail PID_NOT_PRIVATE
podman exec "$parent" test ! -e /dev/ttyACM0 ||
  fail MOTOR_CAPABLE_MCU_VISIBLE_IN_PARENT
podman exec "$xrslam" test ! -e /dev/ttyACM0 ||
  fail MOTOR_CAPABLE_MCU_VISIBLE_IN_XRSLAM
podman image exists "$image" || fail ACCEPTED_STAGE4B_IMAGE_MISSING
for name in "$source" leroy-stage4-imu leroy-live-boot-yaw-persistent; do
  if podman container exists "$name"; then fail EXISTING_SOURCE_OR_YAW_CONTAINER; fi
done
for file in live_boot_imu_service.py boot_yaw_core.py live_boot_yaw.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_FILE_MISSING_OR_SYMLINK
done
[[ -L "$LEROY_STAGE4_MCU_BY_ID" &&
   "$(readlink -f -- "$LEROY_STAGE4_MCU_BY_ID")" == "$mcu" &&
   "$(stat -Lc '%t:%T' "$mcu")" == "$device_id" ]] ||
  fail MCU_CHANGED_DURING_PREFLIGHT

printf '%s\n' \
  'Leroy continuous sensor-only heading; DOES NOT enable any motor command.' \
  'The MCU port is motor-capable: motor and manipulator rails MUST remain' \
  'independently disconnected while USB powers the sensors.' \
  'A hardwired whole-robot E-stop alone cannot meet that isolation while' \
  'the MCU remains powered. No automatic motor arming or session restart.' \
  'Ctrl-C to stop; MCU IMU STOP ACK must be observed.'
read -r -p 'Type BOOT_IMU_ONLY to start this supervised session: ' consent
[[ "$consent" == BOOT_IMU_ONLY ]] || fail OPERATOR_APPROVAL_MISSING

started=0
cleanup() {
  rc=$?
  trap - EXIT INT TERM
  if [[ "$started" == 1 ]]; then
    podman stop -t 7 "$source" >/dev/null 2>&1 || true
    # Deliberately omit --rm above so STOP evidence remains readable.
    logs="$(podman logs "$source" 2>&1 || true)"
    printf '%s\n' "$logs" | grep -E '^BOOT_IMU_SOURCE_(START_ACK|STOP_ACK|STOP_STATE|RESULT|FAULT)=' || true
    if ! grep -Fxq 'BOOT_IMU_SOURCE_STOP_ACK=PASS' <<<"$logs"; then
      echo 'BOOT_LIVE_SESSION=BLOCKED_SOURCE_STOP_ACK_UNVERIFIED' >&2
      rc=2
    fi
    podman rm -f "$source" >/dev/null 2>&1 || true
  fi
  if [[ "$rc" == 0 || "$rc" == 130 ]]; then
    echo 'BOOT_LIVE_SESSION=STOPPED_NO_MOTION_AUTHORITY'
    exit 0
  fi
  echo 'BOOT_LIVE_SESSION=BLOCKED_OR_INTERRUPTED' >&2
  exit 2
}
trap cleanup EXIT

printf 'BOOT_LIVE_SESSION=STARTING_M18_SOURCE\n'
podman run --detach --pull=never --name "$source" \
  --network "container:$parent" --ipc "container:$parent" \
  --pid "container:$parent" \
  --device="$mcu:/dev/ttyACM0:rw" --group-add keep-groups \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/live_boot_imu_service.py,dst=/opt/stage4/live_boot_imu_service.py,readonly" \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_BOOT_PERSISTENT_IMU_ONLY=1 \
  -e ROS_LOG_DIR=/tmp/leroy-boot-imu-source \
  -e XDG_CACHE_HOME=/tmp/leroy-boot-imu-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    test -c /dev/ttyACM0
    test ! -e /dev/video2
    test ! -e /dev/gpiochip0
    exec python3 -B -u /opt/stage4/live_boot_imu_service.py
  ' >/dev/null
started=1

# The source may need up to 8s for DDS preflight, plus serial-open
# reset and a 2s START acknowledgement. Allow 20s, not a 10s race.
ready=0
for _attempt in $(seq 1 200); do
  if podman logs "$source" 2>&1 | grep -Fxq 'BOOT_IMU_SOURCE_START_ACK=PASS'; then
    ready=1
    break
  fi
  [[ "$(podman inspect -f '{{.State.Running}}' "$source" 2>/dev/null)" == true ]] ||
    break
  sleep 0.1
done
[[ "$ready" == 1 ]] || fail M18_SOURCE_DID_NOT_START
printf 'BOOT_LIVE_SESSION=IMU_STREAM_READY\n'
# This foreground observer uses the existing, real, exclusive M18 source
# and publishes ONLY /leroy_boot/relative_yaw_status.
set +e
LEROY_BOOT_YAW_PERSISTENT=1 bash ./live_boot_yaw.sh
yaw_rc=$?
set -e
exit "$yaw_rc"
