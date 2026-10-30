#!/usr/bin/env bash
# TODO(AUTONOMY) #038 [runtime supervisor]: Replace this attended, sensor-only
# wrapper with a lifecycle supervisor that starts every commissioned node in
# dependency order, checks real readiness and stops children in reverse order.
# Distinguish sensor diagnostics, attended motor calibration, and unattended
# navigation as separate authority states; do not infer a permit from consent.
# Require a single hardware/ROS serial owner across modes and latch faults
# for sensor loss, reboots, bad TF, and unexpected motion.
# Acceptance: cold boot, interrupted startup, Ctrl-C, SIGTERM, watchdog
# expiration and power recovery leave no active motion authority; verify
# on the physical machine before enabling any automatic actuation.
# One operator-attended, non-actuating Leroy sensor runtime.
# Camera+DepthART+XRSLAM and MCU IMU/yaw share an isolated ROS graph.
# Separate containers preserve exclusive camera and MCU ownership.
set -Eeuo pipefail
cd "$(dirname "$0")"

usage() {
  cat <<'HELP'
Usage: bash ./leroy.sh {check|build|run|status|stop|help}
  check   No-device checkout and regression tests
  build   Build every current Leroy container image
  run     Start camera, depth, XRSLAM, STM32 IMU and yaw in one session
          Ctrl-C: supervised IMU STOP/ACK, then reverse-order graph shutdown
  status  Read current container states
  stop    Stop the existing sensor-only session; require IMU STOP acknowledgement

For run: disconnect motor/actuator rails independently of STM32 USB, then:
  LEROY_MOTOR_RAILS_DISCONNECTED=YES
  LEROY_MCU_BY_ID=/dev/serial/by-id/<real STM32 ID>
The operator must also type BOOT_IMU_ONLY on the MCU session prompt.
The environment variable and typed consent are attestations, not sensors.
This session never enables motors, navigation, or metric odometry.
HELP
}
fail() { printf 'LEROY_RUNTIME=BLOCKED_%s\n' "$1" >&2; exit 2; }
rootless() {
  [[ "$(id -u)" != 0 ]] || fail ROOT_USER
  command -v podman >/dev/null || fail PODMAN_MISSING
  [[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
    fail NOT_ROOTLESS_PODMAN
}
command="${1:-help}"
[[ $# -le 1 ]] || fail INVALID_ARGUMENTS
case "$command" in
  help|--help|-h) usage; exit 0 ;;
  check) exec bash ./check_current_checkout.sh ;;
  build) exec bash ./build_current_images.sh --build ;;
  status)
    rootless
    for name in leroy-live-costmap leroy-live-xrslam \
                leroy-boot-imu-source leroy-live-boot-yaw-persistent; do
      if podman container exists "$name"; then
        state="$(podman inspect -f '{{.State.Status}}' "$name")" ||
          fail STATUS_INSPECTION_FAILED
      else
        state=absent
      fi
      printf 'LEROY_CONTAINER=%s:%s\n' "$name" "$state"
    done
    exit 0 ;;
  stop)
    rootless
    rc=0
    # Only stop known sensor-only containers. Never touch an unrelated
    # motor-capable owner. Preserve the IMU process's normal STOP/ACK path.
    source=leroy-boot-imu-source
    if podman container exists "$source"; then
      podman stop -t 10 "$source" >/dev/null || rc=2
      logs="$(podman logs "$source" 2>&1)" || rc=2
      if ! grep -Fxq 'BOOT_IMU_SOURCE_STOP_ACK=PASS' <<<"$logs"; then
        echo 'LEROY_RUNTIME=IMU_STOP_UNVERIFIED_DISCONNECT_POWER' >&2
        rc=2
      fi
      # Preserve a failed container for postmortem investigation.
      if [[ "$rc" == 0 ]]; then
        podman rm "$source" >/dev/null || rc=2
      fi
    fi
    # Reverse the established startup dependency order.
    for name in leroy-live-boot-yaw-persistent leroy-live-xrslam leroy-live-costmap; do
      if podman container exists "$name"; then
        podman stop -t 10 "$name" >/dev/null || rc=2
      fi
    done
    if [[ "$rc" != 0 ]]; then
      echo 'LEROY_RUNTIME=STOP_FAILED_OR_IMU_UNVERIFIED' >&2
      exit 2
    fi
    echo 'LEROY_RUNTIME=STOPPED_SENSOR_ONLY_NO_MOTION_AUTHORITY'
    exit 0 ;;
  run) ;;
  *) usage >&2; fail UNKNOWN_COMMAND ;;
esac

rootless
[[ "${LEROY_MOTOR_RAILS_DISCONNECTED:-}" == YES ]] ||
  fail PHYSICAL_MOTOR_RAIL_ISOLATION_NOT_ATTESTED
mcu="${LEROY_MCU_BY_ID:-}"
[[ "$mcu" == /dev/serial/by-id/* && -L "$mcu" ]] ||
  fail STABLE_STM32_DEVICE_REQUIRED
resolved="$(readlink -f -- "$mcu")" || fail STABLE_DEVICE_UNRESOLVED
[[ "$resolved" == /dev/ttyACM0 && -c "$resolved" ]] ||
  fail STM32_DEVICE_ID_CHANGED

# Refuse existing or stopped sessions and any known competing MCU writer.
for name in leroy-live-costmap leroy-live-xrslam \
            leroy-boot-imu-source leroy-live-boot-yaw-persistent \
            leroy-stage4-imu leroy-live-boot-yaw \
            leroy-signed-bridge leroy-actuated-yaw-calibration; do
  if podman container exists "$name"; then
    printf 'LEROY_RUNTIME=EXISTING_CONTAINER:%s\n' "$name" >&2
    fail EXISTING_SESSION_OR_SERIAL_OWNER
  fi
done
for image in costmap xrslam stage4b-imu stage4c-capture-offline; do
  podman image exists "localhost/leroy-ros:$image" ||
    fail MISSING_CURRENT_IMAGE
done

camera_graph_started=0
cleanup() {
  rc=$?
  trap - EXIT INT TERM
  # The nested source normally completes its own IMU STOP/ACK and exits.
  # Never report a clean exit if a motor-capable serial owner remains.
  if [[ "$camera_graph_started" == 1 ]]; then
    if podman container exists leroy-boot-imu-source; then
      echo 'LEROY_RUNTIME=UNEXPECTED_IMU_SOURCE_STILL_PRESENT' >&2
      podman stop -t 7 leroy-boot-imu-source >/dev/null 2>&1 || true
      podman rm -f leroy-boot-imu-source >/dev/null 2>&1 || true
      rc=2
    fi
    for name in leroy-live-xrslam leroy-live-costmap; do
      if podman container exists "$name"; then
        podman stop -t 10 "$name" >/dev/null || rc=2
      fi
    done
  fi
  if [[ "$rc" == 0 ]]; then
    echo 'LEROY_RUNTIME=STOPPED_SENSOR_ONLY_NO_MOTION_AUTHORITY'
  else
    printf 'LEROY_RUNTIME=FAILED_OR_INTERRUPTED_EXIT_%d\n' "$rc" >&2
  fi
  exit "$rc"
}
trap cleanup EXIT
# The child owns its MCU START/STOP lifecycle. It handles Ctrl-C,
# performs MCU stream STOP/ACK, and reports its exit result. Do not
# replace a successful child STOP with a synthetic 130 return code.
trap ':' INT
trap 'exit 143' TERM

echo 'LEROY_RUNTIME=STARTING_UNIFIED_SENSOR_GRAPH'
LEROY_MOTOR_POWER_DISCONNECTED=1 bash ./live_stage3_start.sh
camera_graph_started=1
echo 'LEROY_RUNTIME=CAMERA_DEPTH_XRSLAM_READY'
# Preserve the proven session-local consent and MCU STOP ACK implementation.
# Legacy STAGE4 environment names are internal only, not public workflow.
LEROY_STAGE4_MCU_BY_ID="$mcu" \
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
bash ./live_boot_session.sh
