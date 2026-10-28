#!/usr/bin/env bash
# Stage 4A: explicitly supervised, bounded MCU IMU + existing Stage 3 graph.
# IMU device is accessible ONLY in this disposable sidecar, not the camera,
# XRSLAM, DepthART or costmap containers. No navigation or VIO mode switch.
set -Eeuo pipefail
cd "$(dirname "$0")"

fail() { printf 'STAGE4A=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGUMENTS
[[ "$(id -u)" -ne 0 ]] || fail NOT_ROOTLESS_USER
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
    fail PODMAN_NOT_ROOTLESS
[[ "${LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED-}" == YES ]] ||
    fail MOTOR_RAILS_PHYSICAL_DISCONNECTION_NOT_ATTESTED
[[ "${LEROY_STAGE4_MCU_BY_ID-}" == /dev/serial/by-id/* ]] ||
    fail STABLE_MCU_BY_ID_REQUIRED
[[ -L "$LEROY_STAGE4_MCU_BY_ID" ]] || fail MCU_STABLE_ID_SYMLINK_MISSING
mcu="$(readlink -f -- "$LEROY_STAGE4_MCU_BY_ID")" || fail MCU_BY_ID_UNRESOLVED
[[ "$mcu" == /dev/ttyACM0 && -c "$mcu" ]] || fail MCU_BY_ID_NOT_APPROVED_DEVICE
mcu_rdev="$(stat -Lc '%t:%T' -- "$mcu")" || fail MCU_RDEV_MISSING

parent=leroy-live-costmap
sidecar=leroy-stage4-imu
image=localhost/leroy-ros:stage4-imu
podman image exists "$image" || fail STAGE4A_IMAGE_NOT_BUILT
[[ "$(podman inspect --format '{{.State.Running}}' "$parent" 2>/dev/null)" == true ]] ||
    fail STAGE3_PARENT_NOT_RUNNING
[[ "$(podman inspect --format '{{.HostConfig.NetworkMode}}' "$parent")" == none ]] ||
    fail NETWORK_ISOLATION_REQUIRED
[[ "$(podman inspect --format '{{.HostConfig.IpcMode}}' "$parent")" == shareable ]] ||
    fail SHAREABLE_PARENT_IPC_REQUIRED
[[ "$(podman inspect --format '{{.HostConfig.PidMode}}' "$parent")" == private ]] ||
    fail PRIVATE_PARENT_PID_REQUIRED
podman exec "$parent" test ! -e /dev/ttyACM0 || fail MOTOR_CAPABLE_MCU_VISIBLE_TO_PARENT
[[ "$(podman inspect --format '{{.State.Running}}' leroy-live-xrslam 2>/dev/null)" == true ]] ||
    fail MONOCULAR_STAGE3_SIDECAR_REQUIRED
podman exec leroy-live-xrslam test ! -e /dev/ttyACM0 ||
    fail MOTOR_CAPABLE_MCU_VISIBLE_TO_XRSLAM
podman container exists "$sidecar" && fail SIDECAR_NAME_IN_USE

printf '%s\n' \
  'Stage 4A: standalone 75s IMU ROS overlap test, NO visual-inertial XRSLAM.' \
  'The STM32 CDC interface can address motor hardware even if this code only' \
  'writes two whitelisted IMU-stream commands. STOP ACK MUST be observed.' \
  'Motor-driver and manipulator rails must be physically disconnected,' \
  'chassis secured, E-stop verified and operator present at a hard cutoff.' \
  'Measured camera-IMU offset, IMU noise and camera/IMU extrinsics remain UNKNOWN.' \
  'No odometry, dynamic TF, planner, controller, motor command, MCU flashing,' \
  'or physical movement is authorized.'
read -r -p 'Type STAGE4_IMU_ONLY to authorize this bounded serial session: ' approval
[[ "$approval" == STAGE4_IMU_ONLY ]] || fail OPERATOR_CONSENT_NOT_GIVEN
[[ -L "$LEROY_STAGE4_MCU_BY_ID" &&
   "$(readlink -f -- "$LEROY_STAGE4_MCU_BY_ID")" == "$mcu" &&
   "$(stat -Lc '%t:%T' "$mcu")" == "$mcu_rdev" ]] ||
    fail MCU_CHANGED_AFTER_APPROVAL

echo 'STAGE4A=STARTING_PHYSICAL_IMU_ONLY'
# Shared container-private network+IPC+PID avoids Fast DDS shared-memory
# namespace collisions; no host namespace, no camera or bridge device.
set +e
timeout --foreground --signal=INT --kill-after=30s 130s \
  podman run --rm -it --pull=never --name "$sidecar" \
  --network "container:$parent" --ipc "container:$parent" \
  --pid "container:$parent" \
  --device="$mcu:/dev/ttyACM0:rw" --group-add keep-groups \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_STAGE4_IMU_ONLY=1 \
  -e LEROY_NAV2_DRY_RUN=1 \
  -e ROS_LOG_DIR=/tmp/stage4a-ros \
  -e XDG_CACHE_HOME=/tmp/stage4a-cache \
  --entrypoint bash "$image" -lc '
    set -e
    source /opt/ros/jazzy/setup.bash
    source /leroy_ws/install/setup.bash
    test -c /dev/ttyACM0
    test ! -e /dev/video2
    exec python3 -u /opt/stage4/live_stage4_imu_overlap.py
  '
rc=$?
set -e
printf 'STAGE4A_PROCESS_RC=%s\n' "$rc"
if [[ "$rc" -ne 0 ]]; then
    echo 'STAGE4A=BLOCKED_NO_ACCEPTANCE_VERIFY_MCU_STOP_PHYSICALLY' >&2
    exit 2
fi
echo 'STAGE4A=PASS_TRANSPORT_ONLY_NOT_VIO_OR_CALIBRATION'
