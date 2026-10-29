#!/usr/bin/env bash
# Stage 4A: explicitly supervised, bounded MCU IMU + existing Stage 3 graph.
# IMU device is accessible ONLY in this disposable sidecar, not the camera,
# XRSLAM, DepthART or costmap containers. No navigation or VIO mode switch.
set -Eeuo pipefail
cd "$(dirname "$0")"

fail() { printf 'STAGE4A=BLOCKED_%s\n' "$1" >&2; exit 2; }
[[ $# -eq 0 ]] || fail UNEXPECTED_ARGUMENTS
record_mode="${LEROY_STAGE4C_RECORD:-0}"
[[ "$record_mode" == 0 || "$record_mode" == 1 ]] || fail INVALID_RECORD_MODE
if [[ "$record_mode" == 1 ]]; then
    [[ "${LEROY_STAGE4C_SOURCE_META:-0}" == 1 &&
       "${LEROY_STAGE4_IMU_IMAGE:-}" == localhost/leroy-ros:stage4c-capture-offline ]] ||
        fail RECORDER_REQUIRES_SOURCE_METADATA_IMAGE
    [[ "${LEROY_STAGE4C_CAPTURE_REVIEWED:-}" == YES ]] ||
        fail CAPTURE_REVIEW_ATTESTATION_REQUIRED
    capture_name="${LEROY_STAGE4C_CAPTURE_NAME:-}"
    [[ "$capture_name" =~ ^stage4c-[a-zA-Z0-9_-]{6,50}$ ]] ||
        fail INVALID_CAPTURE_NAME
    capture_root="${LEROY_STAGE4C_CAPTURE_ROOT:-}"
    [[ "$capture_root" == /* && -d "$capture_root" &&
       ! -L "$capture_root" && -w "$capture_root" ]] ||
        fail INVALID_CAPTURE_ROOT
    [[ "$(readlink -f -- "$capture_root")" == "$capture_root" &&
       "$(stat -c '%u' -- "$capture_root")" == "$(id -u)" ]] ||
        fail CAPTURE_ROOT_MUST_BE_OWNED_REAL_PATH
    [[ ! -e "$capture_root/$capture_name" &&
       ! -L "$capture_root/$capture_name" &&
       ! -e "$capture_root/$capture_name-capture-summary.json" &&
       ! -e "$capture_root/$capture_name-armed" &&
       ! -L "$capture_root/$capture_name-armed" &&
       ! -e "$capture_root/$capture_name-source-done" &&
       ! -L "$capture_root/$capture_name-source-done" ]] ||
        fail CAPTURE_OUTPUT_ALREADY_EXISTS
    free_kib="$(df -Pk -- "$capture_root" | awk 'END {print $4}')"
    [[ "$free_kib" =~ ^[0-9]+$ && "$free_kib" -ge 1500000 ]] ||
        fail CAPTURE_DISK_SPACE_INSUFFICIENT
fi
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
# Preserve the accepted Stage 4A image by default. Stage 4B must be
# selected deliberately and ONLY from this exact locally built image tag.
case "${LEROY_STAGE4_IMU_IMAGE:-localhost/leroy-ros:stage4-imu}" in
    localhost/leroy-ros:stage4-imu)
        image=localhost/leroy-ros:stage4-imu ;;
    localhost/leroy-ros:stage4b-imu)
        image=localhost/leroy-ros:stage4b-imu ;;
    localhost/leroy-ros:stage4c-capture-offline)
        [[ "${LEROY_STAGE4C_SOURCE_META-}" == 1 ]] || fail SOURCE_METADATA_OPT_IN_REQUIRED
        image=localhost/leroy-ros:stage4c-capture-offline ;;
    *) fail UNAPPROVED_STAGE4_IMAGE ;;
esac
if [[ "${LEROY_STAGE4C_SOURCE_META:-0}" != 0 ]]; then
    [[ "${LEROY_STAGE4C_SOURCE_META}" == 1 && "$image" == localhost/leroy-ros:stage4c-capture-offline ]] || fail SOURCE_METADATA_IMAGE_MISMATCH
fi
podman image exists "$image" || fail STAGE4A_IMAGE_NOT_BUILT
# Read-only, zero-device preflight for the Stage 4B image, BEFORE opening
# the motor-capable MCU or prompting for a new physical attempt.
if [[ "$image" == localhost/leroy-ros:stage4b-imu ]]; then
    podman run --rm --pull=never --network=none \
      --read-only --cap-drop=all --security-opt=no-new-privileges \
      --entrypoint /bin/sh "$image" -c '
        test -s /opt/stage4/live_stage4b_delivery_trace.py
        test -s /opt/stage4/live_stage4b_spin_budget.py
        grep -Fq "stage4b_delivery_trace" /opt/stage4/live_stage4_imu_overlap.py
        grep -Fq "service_callbacks(" /opt/stage4/live_stage4_imu_overlap.py
      ' || fail STAGE4B_IMAGE_TRACE_NOT_INSTALLED
    echo 'STAGE4B_IMAGE_TRACE=VERIFIED_OFFLINE'
fi
if [[ "$image" == localhost/leroy-ros:stage4c-capture-offline ]]; then
    podman run --rm --pull=never --network=none \
      --read-only --cap-drop=all --security-opt=no-new-privileges \
      -e LEROY_STAGE4C_RECORD="$record_mode" \
      --entrypoint /bin/sh "$image" -c '
        test -s /opt/stage4c/live_stage4c_source_meta_ros.py
        test -s /opt/stage4c/live_stage4c_source_meta.py
        grep -Fq "SourceMetadataRosAdapter(source)" /opt/stage4/live_stage4_imu_overlap.py
        grep -Fq "source_metadata_requires_capture_only_session" /opt/stage4/live_stage4_imu_overlap.py
        grep -Fq "if not LIVE_CAPTURE_COMMISSIONED:" /opt/stage4c/live_stage4c_passive_recorder.py
        if [ "$LEROY_STAGE4C_RECORD" = 1 ]; then
          grep -Fqx "LIVE_CAPTURE_COMMISSIONED = True" /opt/stage4c/live_stage4c_passive_recorder.py
        else
          grep -Fqx "LIVE_CAPTURE_COMMISSIONED = False" /opt/stage4c/live_stage4c_passive_recorder.py
        fi
      ' || fail SOURCE_METADATA_IMAGE_NOT_COMMISSIONED
    echo 'STAGE4C_M18_METADATA_IMAGE=VERIFIED_OFFLINE'
fi
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
capture_container=leroy-stage4c-recorder
if [[ "$record_mode" == 1 ]]; then
    podman container exists "$capture_container" && fail RECORDER_NAME_IN_USE
fi

# Before touching the motor-capable serial port, establish a FRESH read-only
# real camera/depth/diagnostic costmap acceptance in the existing parent.
timeout 100s podman exec "$parent" bash -lc '
  set -e
  source /opt/ros/jazzy/setup.bash
  source /leroy_ws/install/setup.bash
  ros2 run leroy_nav2_perception live_costmap_probe
' || fail LIVE_STAGE3_COSTMAP_NOT_ACCEPTED

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
if [[ "$record_mode" == 1 ]]; then
    printf '%s\n' \
      'This is a new REAL sensor recording, not the accepted Stage 4B session.' \
      'Only proceed under an independently reviewed stationary/rigid-rig' \
      'physical procedure: no wheel/manipulator power, secured chassis,' \
      'checked E-stop, immediate hard cutoff and operator supervision.' \
      'Camera/IMU rig motion requires separate procedural approval.' \
      'Recorder cannot reach STM32, camera or motor devices.'
    read -r -p 'Type RECORD_REAL_SENSORS to additionally authorize capture: ' capture_approval
    [[ "$capture_approval" == RECORD_REAL_SENSORS ]] ||
        fail CAPTURE_OPERATOR_CONSENT_MISSING
fi
[[ -L "$LEROY_STAGE4_MCU_BY_ID" &&
   "$(readlink -f -- "$LEROY_STAGE4_MCU_BY_ID")" == "$mcu" &&
   "$(stat -Lc '%t:%T' "$mcu")" == "$mcu_rdev" ]] ||
    fail MCU_CHANGED_AFTER_APPROVAL

# Recorder runs without /dev passthrough, on the accepted isolated DDS
# graph. It must be started BEFORE M18 so its four subscriptions can
# observe the entire bounded live session.
capture_started=0
cleanup_capture() {
    if [[ "$capture_started" == 1 ]]; then
        podman stop -t 5 "$capture_container" >/dev/null 2>&1 || true
        podman rm -f "$capture_container" >/dev/null 2>&1 || true
    fi
}
if [[ "$record_mode" == 1 ]]; then
    trap cleanup_capture EXIT
    podman run -d --pull=never --name "$capture_container" \
      --network "container:$parent" --ipc "container:$parent" \
      --pid "container:$parent" \
      --read-only --cap-drop=all --security-opt=no-new-privileges \
      --tmpfs /tmp:rw,nosuid,nodev,size=256m \
      -v "$capture_root:/capture:rw" \
      -e ROS_DOMAIN_ID=170 \
      -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
      -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
      -e LEROY_NAV2_DRY_RUN=1 \
      -e LEROY_STAGE4C_CAPTURE_ONLY=1 \
      -e LEROY_STAGE4C_OUTPUT_NAME="$capture_name" \
      -e ROS_LOG_DIR=/tmp/stage4c-ros \
      -e XDG_CACHE_HOME=/tmp/stage4c-cache \
      --entrypoint bash "$image" -lc '
        set -e
        source /opt/ros/jazzy/setup.bash
        source /leroy_ws/install/setup.bash
        test ! -e /dev/ttyACM0
        test ! -e /dev/video2
        test ! -e /dev/gpiochip0
        exec python3 -u /opt/stage4c/live_stage4c_passive_recorder.py \
          --output-name "$LEROY_STAGE4C_OUTPUT_NAME"
      ' >/dev/null || fail RECORDER_COULD_NOT_START
    capture_started=1
    sleep 1
    [[ "$(podman inspect --format '{{.State.Running}}' "$capture_container" 2>/dev/null)" == true ]] ||
        fail RECORDER_FAILED_BEFORE_MCU_OPEN
fi
# Only the capture-mode IMU sidecar gets a read-only view of the
# recorder's one-run armed marker; the recorder retains its own
# device-free read/write output mount.
record_mount=()
if [[ "$record_mode" == 1 ]]; then
    record_mount=(-v "$capture_root:/capture:ro")
fi
printf 'STAGE4A_SELECTED_IMAGE=%s\n' "$image"
echo 'STAGE4A=STARTING_PHYSICAL_IMU_ONLY'
# Shared container-private network+IPC+PID avoids Fast DDS shared-memory
# namespace collisions; no host namespace, no camera or bridge device.
set +e
timeout --foreground --signal=INT --kill-after=30s 130s \
  podman run --rm -it --pull=never --name "$sidecar" \
  --network "container:$parent" --ipc "container:$parent" \
  --pid "container:$parent" \
  --device="$mcu:/dev/ttyACM0:rw" --group-add keep-groups \
  "${record_mount[@]}" \
  --read-only --cap-drop=all --security-opt=no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  -e ROS_DOMAIN_ID=170 \
  -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e LEROY_STAGE4_IMU_ONLY=1 \
  -e LEROY_STAGE4C_SOURCE_META="${LEROY_STAGE4C_SOURCE_META:-0}" \
  -e LEROY_STAGE4C_CAPTURE_ONLY="${LEROY_STAGE4C_SOURCE_META:-0}" \
  -e LEROY_STAGE4C_RECORD="$record_mode" \
  -e LEROY_STAGE4C_CAPTURE_NAME="${capture_name:-}" \
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
if [[ "$record_mode" == 1 ]]; then
    # The source has exited with PASS (which requires STOP ACK and no
    # terminal fault). Publish an exclusive host-only completion receipt:
    # recorder MUST see this before it can accept the bag.
    source_done="$capture_root/$capture_name-source-done"
    ( set -C; umask 077
      printf 'STAGE4C_SOURCE_DONE=STOP_ACK_AND_TRANSPORT_PASS\\n' > "$source_done"
    ) || fail SOURCE_COMPLETION_MARKER_FAILED
    if ! capture_rc="$(timeout 110s podman wait "$capture_container")"; then
        fail RECORDER_DID_NOT_FINISH
    fi
    podman logs "$capture_container" || fail RECORDER_LOGS_UNAVAILABLE
    [[ "$capture_rc" == 0 ]] || fail RECORDER_FAILED
    summary="$capture_root/$capture_name-capture-summary.json"
    python3 - "$summary" <<'PY' || fail RECORDER_SUMMARY_INVALID
import json
import sys
from pathlib import Path
path = Path(sys.argv[1])
if not path.is_file() or path.is_symlink():
    raise SystemExit("missing_or_symlinked_manifest")
data = json.loads(path.read_text(encoding="utf-8"))
c = data["counts"]
imu = c["/imu/data_raw"]
good = (data["error"] is None
        and data["readiness"] == "INPUT_FORMAT_COMPLETE_UNVERIFIED"
        and c["/camera/image_rect"] >= 100
        and c["/camera/camera_info"] >= 100
        and imu >= 500
        and c["/stage4c/imu_source_meta"] == imu
        and data["matched_imu_metadata_stamps"] == imu
        and data["physical_camera_imu_offset_validated"] is False
        and "sha256_files" in data)
if not good:
    raise SystemExit("incomplete_or_invalid_capture")
print("STAGE4C_REAL_FOUR_TOPIC_RECORDING=COMPLETE_UNCALIBRATED", flush=True)
PY
    echo 'STAGE4C_REAL_CAPTURE=FORMAT_COMPLETE_NOT_YET_CALIBRATED'
else
    echo 'STAGE4A=PASS_TRANSPORT_ONLY_NOT_VIO_OR_CALIBRATION'
fi
