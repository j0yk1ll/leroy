#!/usr/bin/env bash
# Stage 3 camera+STM32 IMU -> XRSLAM VIO; no actuation or metric odom.
# The motor-capable MCU opens ONLY inside the fourth container with M18 START/STOP.
set -Eeuo pipefail
umask 077
cd "$(dirname "$0")"
fail() { echo "STAGE3_VIO=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] || fail BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY
git merge-base --is-ancestor 93a399fe0b9b0572d389d4c5139f092f4423ceb1 HEAD || fail STACK
for f in m23bv_v3_stage3_vio_imu_sender.py m23bv_v3_stage3_physical_camera_supervisor.py m23bv_v3_stage3_vio_audit.py m23bv_v3_stage3_nav2_smoke.launch.py m23bv_v3_stage3_xrslam_vio_diag.launch.py m23bv_v3_stage3_xrslam_vio_diag.yaml m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml; do
  [[ -f "$f" && ! -L "$f" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$f")" == "$(git rev-parse "HEAD:ros/$f")" ]] || fail SOURCE_DRIFT
done
[[ "$(git rev-parse HEAD:ros/m23bv_reliable_rectifier_v3_candidate.launch.py)" == a0ae75a62da39235b113e3d2c324efda5fcfc6e1 ]] || fail V3_SOURCE_CHANGED
[[ "$(id -u)" != 0 ]] || fail NO_ROOT
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] || fail NOT_ROOTLESS
[[ -z "$(podman ps --format '{{.Names}}')" ]] || fail CONTAINERS_ALREADY_RUNNING
sensor=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
xrimage=localhost/leroy-ros:xrslam
navimage=localhost/leroy-nav2:jazzy
pin() {
  local got
  got="$(podman image inspect "$1" --format '{{.Id}}')" || fail IMAGE_ABSENT
  got="$(echo "$got" | sed 's/^sha256://')"
  [[ "$got" == "$2" ]] || fail IMAGE_DRIFT
}
pin "$sensor" f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
pin "$xrimage" 20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a
pin "$navimage" 89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4
link=/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3.4.2:1.0-video-index0
[[ -L "$link" ]] || fail CAMERA_LINK
cam="$(readlink -f "$link")" || fail CAMERA_RESOLVE
[[ "$cam" =~ ^/dev/video[0-9]+$ && -c "$cam" ]] || fail CAMERA_CHARACTER_DEVICE
[[ -v M23BV_STAGE3_CAMERA_VISUALLY_VERIFIED_NODE ]] || fail VISUAL_ATTESTATION_MISSING
[[ "$M23BV_STAGE3_CAMERA_VISUALLY_VERIFIED_NODE" == "$cam" ]] || fail WRONG_VISUAL_CAMERA
udev="$(udevadm info --query=property --name="$cam")" || fail CAMERA_UDEV
[[ "$(sed -n 's/^ID_SERIAL=//p' <<<"$udev")" == Wed_Camera_Wed_Camera_20240420112206 ]] || fail CAMERA_SERIAL
[[ "$(sed -n 's/^ID_PATH=//p' <<<"$udev")" == pci-0000:00:14.0-usb-0:3.4.2:1.0 ]] || fail CAMERA_PHYSICAL_PORT
[[ "$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$udev")" == 32e6 && "$(sed -n 's/^ID_MODEL_ID=//p' <<<"$udev")" == 9221 ]] || fail CAMERA_VENDOR
[[ "$(sed -n 's/^ID_V4L_CAPABILITIES=//p' <<<"$udev")" == *:capture:* ]] || fail CAMERA_CAPABILITIES
rdev="$(stat -Lc '%t:%T' "$cam")" || fail CAMERA_RDEV
root="$(git rev-parse --show-toplevel)" || fail REPO
[[ "$(git rev-parse HEAD:ros/m18_imu_stream_source.py)" == 3c5fb37ca291533e2702a9159f3e8db11b076d22 &&
   "$(git hash-object "$root/ros/m18_imu_stream_source.py")" == 3c5fb37ca291533e2702a9159f3e8db11b076d22 &&
   "$(git rev-parse HEAD:ros/m18_imu_stream_contract.py)" == 443aa82066e88d50cb88859304e9197e3f459ee0 &&
   "$(git hash-object "$root/ros/m18_imu_stream_contract.py")" == 443aa82066e88d50cb88859304e9197e3f459ee0 ]] ||
  fail M18_SOURCE_DRIFT
cal="$root/ros/config/camera/front_camera.yaml"
[[ -f "$cal" && ! -L "$cal" ]] || fail CAL_FILE
[[ "$(sha256sum "$cal" | cut -d' ' -f1)" == f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138 ]] || fail CAL_SHA
[[ -v M23BV_LOG_DIR ]] || fail LOG_DIR
[[ "$M23BV_LOG_DIR" == /* && -d "$M23BV_LOG_DIR" && ! -L "$M23BV_LOG_DIR" && -w "$M23BV_LOG_DIR" ]] || fail LOG_DIR
[[ "$(stat -c '%a' "$M23BV_LOG_DIR")" == 700 ]] || fail PRIVATE_LOG_DIR
mcu_link=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00
[[ -L "$mcu_link" && -n "$M23BV_STAGE3_MCU_ID_SERIAL" ]] || fail MCU_ID_REQUIRED
mcu="$(readlink -f "$mcu_link")" || fail MCU_LINK
[[ "$mcu" =~ ^/dev/ttyACM[0-9]+$ && -c "$mcu" ]] || fail MCU_CHAR_DEVICE
mcu_props="$(udevadm info --query=property --name="$mcu")" || fail MCU_UDEV
[[ "$(sed -n 's/^ID_SERIAL=//p' <<<"$mcu_props")" == "$M23BV_STAGE3_MCU_ID_SERIAL" ]] ||
  fail MCU_ID_SERIAL_MISMATCH
mcu_rdev="$(stat -Lc '%t:%T' "$mcu")" || fail MCU_RDEV
mcu_gid="$(stat -Lc '%g' "$mcu")" || fail MCU_GID
[[ " $(id -G) " == *" $mcu_gid "* && -r "$mcu" && -w "$mcu" ]] ||
  fail MCU_GROUP_PERMISSIONS
[[ "$(podman info --format '{{.Host.OCIRuntime.Name}}')" == crun ]] ||
  fail MCU_NEEDS_CRUN_KEEP_GROUPS
[[ -r /dev/tty && -w /dev/tty ]] || fail INTERACTIVE_TTY
printf '%s\n' \
  'Stage 3: real front camera + motor-capable MCU IMU for XRSLAM VIO.' \
  'Both motor-power rails must be physically disconnected; chassis immobilized.' \
  'E-stop tested; operator present for immediate power disconnect.' \
  'Camera feed and pinned calibration have been visually verified just now.' \
  'URDF camera extrinsics and EuRoC SLAM tuning are PROVISIONAL diagnostics only.' \
  'Native XRSLAM camera+IMU processing uses PROVISIONAL 0.0s offset.' \
  'Opening MCU CDC/DTR may reset or energize motor-capable firmware.' \
  'Only M18 IMU START 50 and STOP 0 serial requests allowed.' \
  'Missing STOP ACK means UNKNOWN MCU STATE: physical intervention.' \
  'Metric odometry and map TF remain OFF; Nav2 stays unconfigured.' \
  'No motor commands, recording, metric odometry, navigation goals, or movement.' \
  'Type exactly YES to authorize this ONE supervised camera+MCU VIO test:' >&2
IFS= read -r answer </dev/tty || fail NO_REPLY
[[ "$answer" == YES ]] || fail DECLINED
[[ "$(readlink -f "$link")" == "$cam" && "$(stat -Lc '%t:%T' "$cam")" == "$rdev" ]] || fail CAMERA_HOTPLUG
[[ -L "$mcu_link" && "$(readlink -f "$mcu_link")" == "$mcu" &&
   "$(stat -Lc '%t:%T' "$mcu")" == "$mcu_rdev" ]] || fail MCU_CHANGED_AFTER_YES
mcu_check="$(udevadm info --query=property --name="$mcu")" || fail MCU_UDEV_RECHECK
[[ "$(sed -n 's/^ID_SERIAL=//p' <<<"$mcu_check")" == "$M23BV_STAGE3_MCU_ID_SERIAL" ]] ||
  fail MCU_SERIAL_CHANGED_AFTER_YES
again="$(udevadm info --query=property --name="$cam")" || fail POST_ATTESTATION_UDEV
[[ "$(sed -n 's/^ID_PATH=//p' <<<"$again")" == pci-0000:00:14.0-usb-0:3.4.2:1.0 &&
   "$(sed -n 's/^ID_SERIAL=//p' <<<"$again")" == Wed_Camera_Wed_Camera_20240420112206 &&
   "$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$again")" == 32e6 &&
   "$(sed -n 's/^ID_MODEL_ID=//p' <<<"$again")" == 9221 &&
   "$(sed -n 's/^ID_V4L_CAPABILITIES=//p' <<<"$again")" == *:capture:* ]] ||
  fail POST_ATTESTATION_CAMERA
approval="$(mktemp --tmpdir="$M23BV_LOG_DIR" stage3_monocular_approval_XXXXXXXX.txt)"
printf 'UTC=%s HEAD=%s CAMERA=%s CAMERA_RDEV=%s MOTOR_RAILS=OPERATOR_ATTESTED_DISCONNECTED ESTOP=OPERATOR_ATTESTED IMMEDIATE_DISCONNECT=OPERATOR_ATTESTED AUTHORIZE=YES MCU=ISOLATED_IMU_CONTAINER_STOP_REQUIRED\n' "$(date -u -Is)" "$(git rev-parse HEAD)" "$cam" "$rdev" >"$approval"
log="$(mktemp --tmpdir="$M23BV_LOG_DIR" stage3_monocular_XXXXXXXX.log)"
echo "STAGE3_APPROVAL_SHA256=$(sha256sum "$approval" | cut -d' ' -f1)" >>"$log"
suffix="$$-$(date +%s)"
parent="stage3-vio-cam-$suffix"
xr="stage3-vio-xr-$suffix"
nav="stage3-vio-nav-$suffix"
imu="stage3-vio-imu-$suffix"
cleanup() {
  trap - EXIT INT TERM
  for name in "$imu" "$nav" "$xr" "$parent"; do
    if podman container exists "$name"; then
      podman stop --time 26 "$name" >/dev/null 2>&1 || true
      echo "--- $name ---" >>"$log"
      podman logs "$name" >>"$log" 2>&1 || true
      podman rm -f "$name" >/dev/null 2>&1 || true
    fi
  done
}
trap 'cleanup; echo "STAGE3_VIO_FAILURE_LOG=$log" >&2; sha256sum "$log" >&2' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
boot='set -e; . /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; . /opt/m23bv_reliable_rectifier_proto/ws/install/setup.sh; exec "$@"'
# Parent gets exactly the V3 camera; no MCU device mapping.
podman run -d --name "$parent" --pull=never --network none --ipc shareable --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=1024m \
  --device "$cam:/dev/video2:rw" \
  --mount "type=bind,src=$cal,dst=/config/camera/front_camera.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_physical_camera_supervisor.py,dst=/opt/stage3_camera_supervisor.py,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/stage3-ros -e XDG_CACHE_HOME=/tmp/stage3-cache -e PYTHONDONTWRITEBYTECODE=1 \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 -e M23BV_RELIABLE_RECTIFIER_PROTO_V3=1 \
  -e M23BV_STAGE3_PHYSICAL_CAMERA_ONLY=YES --entrypoint /bin/bash \
  "$sensor" -c "$boot" stage3 python3 -u /opt/stage3_camera_supervisor.py >/dev/null || fail CAMERA_START
[[ "$(podman inspect "$parent" --format '{{.HostConfig.NetworkMode}}')" == none ]] || fail PARENT_NETWORK_NOT_NONE
ready=0
for i in $(seq 1 225); do
  if podman logs "$parent" 2>&1 | grep -q M23BV_STAGE3_REAL_CAMERA_READY=PASS; then ready=1; break; fi
  [[ "$(podman inspect "$parent" --format '{{.State.Running}}')" == true ]] || fail CAMERA_EARLY_EXIT
  sleep 0.2
done
[[ "$ready" == 1 ]] || fail CAMERA_NOT_READY
echo 'STAGE3_VIO_PARENT_NETWORK=NONE'
side='set -e; . /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; test ! -e /dev/video2; test ! -e /dev/ttyACM0; exec "$@"'
podman run -d --name "$xr" --pull=never --network "container:$parent" --ipc "container:$parent" \
  --pid private --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_vio_diag.yaml,dst=/opt/leroy_stage3_slam.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_vio_diag.launch.py,dst=/opt/leroy_stage3_xrslam.launch.py,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_xrslam_euroc_template_UNCALIBRATED.yaml,dst=/opt/leroy_stage3_device.yaml,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/stage3-ros -e XDG_CACHE_HOME=/tmp/stage3-cache -e PYTHONDONTWRITEBYTECODE=1 \
  -e M23BV_STAGE3_XRSLAM_VIO_DIAG=YES \
  --entrypoint /bin/bash "$xrimage" -c "$side" stage3 \
  ros2 launch /opt/leroy_stage3_xrslam.launch.py >/dev/null || fail XRSLAM_START
podman run -d --name "$nav" --pull=never --network "container:$parent" --ipc "container:$parent" \
  --pid private --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_smoke.launch.py,dst=/opt/stage3_nav2.launch.py,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_vio_audit.py,dst=/opt/stage3_live_audit.py,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/stage3-ros -e XDG_CACHE_HOME=/tmp/stage3-cache -e PYTHONDONTWRITEBYTECODE=1 \
  -e M23BV_STAGE3_NO_DEVICE_APP_SMOKE=1 -e M23BV_STAGE3_CAMERA_SIDECAR=1 \
  -e M23BV_STAGE3_XRSLAM_VIO_DIAG=YES \
  --entrypoint /bin/bash "$navimage" -c "$side" stage3 \
  ros2 launch /opt/stage3_nav2.launch.py >/dev/null || fail NAV2_START
for name in "$parent" "$xr" "$nav"; do
  [[ "$(podman inspect "$name" --format '{{.State.Running}}')" == true ]] || fail APP_EARLY_EXIT
done
echo STAGE3_VIO_APPS=CAMERA_DEPTHART_XRSLAM_NAV2_STATIC_TF
# Audit concurrently; start the dedicated MCU-only process only after
# camera, XRSLAM and Nav2 containers are all established.
set +e
timeout --foreground --signal=TERM --kill-after=8s 120s \
  podman exec "$nav" /bin/bash -c \
    '. /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; exec python3 -u /opt/stage3_live_audit.py' \
  >>"$log" 2>&1 &
audit_pid=$!
set -e
set +e
timeout --foreground --signal=INT --kill-after=18s 93s \
  podman run --rm --name "$imu" --pull=never \
    --network "container:$parent" --ipc "container:$parent" --pid private \
    --group-add keep-groups --read-only --cap-drop all \
    --security-opt no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=256m \
    --device "$mcu:/dev/ttyACM0:rw" \
    --mount "type=bind,src=$PWD/m18_imu_stream_source.py,dst=/opt/m18/m18_imu_stream_source.py,readonly" \
    --mount "type=bind,src=$PWD/m23bv_v3_stage3_vio_imu_sender.py,dst=/opt/stage3_imu_sender.py,readonly" \
    -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_LOG_DIR=/tmp/stage3-vio-ros -e PYTHONDONTWRITEBYTECODE=1 \
    -e M23BV_STAGE3_VIO_IMU_ONLY=YES \
    --entrypoint /bin/bash "$sensor" -c \
      'set -e; . /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; export PYTHONPATH="/opt/m18:/opt/m18/monitor:$PYTHONPATH"; exec python3 -u /opt/stage3_imu_sender.py' \
  >>"$log" 2>&1
imu_rc=$?
wait "$audit_pid"
audit_rc=$?
set -e
echo "STAGE3_VIO_IMU_PROCESS_RC=$imu_rc" >>"$log"
echo "STAGE3_VIO_AUDIT_PROCESS_RC=$audit_rc" >>"$log"
rc=0
if [[ "$imu_rc" != 0 || "$audit_rc" != 0 ]]; then rc=2; fi
for name in "$parent" "$xr" "$nav"; do
  if [[ "$(podman inspect "$name" --format '{{.State.Running}}')" != true ]]; then
    echo "STAGE3_PREMATURE_PROCESS_EXIT=$name" >>"$log"
    rc=2
  fi
done
cleanup
trap - EXIT
echo "STAGE3_AUDIT_RC=$rc" >>"$log"
echo "STAGE3_LOG=$log"
sha256sum "$log"
grep -E 'M23BV_STAGE3_VIO_AUDIT=|STAGE3_VIO_IMU_(START|STOP)_ACK=|STAGE3_VIO_IMU_VERDICT=|M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=|Submitted [0-9]+ camera frames to XRSLAM' "$log" | tail -n 30 || true
if ! grep -qx STAGE3_VIO_IMU_STOP_ACK=PASS "$log"; then
  echo STAGE3_VIO_MCU_STATE=UNKNOWN_OPERATOR_PHYSICAL_INTERVENTION_NO_RETRY >&2
  fail STOP_ACK_NOT_VERIFIED
fi
[[ "$rc" == 0 ]] || fail VIO_RUNTIME
grep -qx STAGE3_VIO_IMU_START_ACK=PASS "$log" || fail START_ACK
grep -qx STAGE3_VIO_IMU_VERDICT=PASS_DIAGNOSTIC_STREAM "$log" || fail IMU_VERDICT
grep -qx M23BV_STAGE3_VIO_AUDIT=PASS_REAL_CAMERA_IMU_GRAPH_NO_ODOM "$log" || fail AUDIT_VERDICT
grep -qE 'Submitted [0-9]+ camera frames to XRSLAM \(monocular_imu\)' "$log" ||
  fail NO_NATIVE_VIO_FRAME_SUBMISSION
grep -q M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS "$log" || fail CAMERA_SHUTDOWN
echo STAGE3_VIO=PASS_REAL_CAMERA_IMU_NATIVE_FRAMES_NO_ODOM
echo STAGE3_VIO_METRIC_POSE_NAV2_MOTION_RECORDING=NOT_AUTHORIZED
