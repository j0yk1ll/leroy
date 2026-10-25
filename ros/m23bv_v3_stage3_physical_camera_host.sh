#!/usr/bin/env bash
# Stage 3 real front camera + DepthART + diagnostic XRSLAM/Nav2.
# Deliberately no MCU serial, bridge, rosbag, goals, or motor commands.
set -Eeuo pipefail
umask 077
cd "$(dirname "$0")"
fail() { echo "STAGE3_CAMERA=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] || fail BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY
git merge-base --is-ancestor 93a399fe0b9b0572d389d4c5139f092f4423ceb1 HEAD || fail STACK
for f in m23bv_v3_stage3_physical_camera_supervisor.py m23bv_v3_stage3_live_camera_audit.py m23bv_v3_stage3_nav2_smoke.launch.py m23bv_v3_stage3_smoke_monocular.yaml m23bv_v3_stage3_smoke_device.yaml; do
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
cal="$root/ros/config/camera/front_camera.yaml"
[[ -f "$cal" && ! -L "$cal" ]] || fail CAL_FILE
[[ "$(sha256sum "$cal" | cut -d' ' -f1)" == f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138 ]] || fail CAL_SHA
[[ -v M23BV_LOG_DIR ]] || fail LOG_DIR
[[ "$M23BV_LOG_DIR" == /* && -d "$M23BV_LOG_DIR" && ! -L "$M23BV_LOG_DIR" && -w "$M23BV_LOG_DIR" ]] || fail LOG_DIR
[[ "$(stat -c '%a' "$M23BV_LOG_DIR")" == 700 ]] || fail PRIVATE_LOG_DIR
[[ -r /dev/tty && -w /dev/tty ]] || fail INTERACTIVE_TTY
printf '%s\n' \
  'Stage 3: real front camera and diagnostic DepthART/XRSLAM/Nav2. NO MCU serial.' \
  'Both motor-power rails must be physically disconnected; chassis immobilized.' \
  'E-stop tested; operator present for immediate power disconnect.' \
  'Camera feed and pinned calibration have been visually verified just now.' \
  'No motor commands, recording, metric odometry, navigation goals, or movement.' \
  'Type exactly YES to authorize this ONE camera-only session:' >&2
IFS= read -r answer </dev/tty || fail NO_REPLY
[[ "$answer" == YES ]] || fail DECLINED
[[ "$(readlink -f "$link")" == "$cam" && "$(stat -Lc '%t:%T' "$cam")" == "$rdev" ]] || fail CAMERA_HOTPLUG
again="$(udevadm info --query=property --name="$cam")" || fail POST_ATTESTATION_UDEV
[[ "$(sed -n 's/^ID_PATH=//p' <<<"$again")" == pci-0000:00:14.0-usb-0:3.4.2:1.0 &&
   "$(sed -n 's/^ID_SERIAL=//p' <<<"$again")" == Wed_Camera_Wed_Camera_20240420112206 &&
   "$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$again")" == 32e6 &&
   "$(sed -n 's/^ID_MODEL_ID=//p' <<<"$again")" == 9221 &&
   "$(sed -n 's/^ID_V4L_CAPABILITIES=//p' <<<"$again")" == *:capture:* ]] ||
  fail POST_ATTESTATION_CAMERA
approval="$(mktemp --tmpdir="$M23BV_LOG_DIR" stage3_camera_approval_XXXXXXXX.txt)"
printf 'UTC=%s HEAD=%s CAMERA=%s CAMERA_RDEV=%s MOTOR_RAILS=OPERATOR_ATTESTED_DISCONNECTED ESTOP=OPERATOR_ATTESTED IMMEDIATE_DISCONNECT=OPERATOR_ATTESTED AUTHORIZE=YES MCU=NONE\n' "$(date -u -Is)" "$(git rev-parse HEAD)" "$cam" "$rdev" >"$approval"
log="$(mktemp --tmpdir="$M23BV_LOG_DIR" stage3_camera_XXXXXXXX.log)"
echo "STAGE3_APPROVAL_SHA256=$(sha256sum "$approval" | cut -d' ' -f1)" >>"$log"
suffix="$$-$(date +%s)"
parent="stage3-camera-$suffix"
xr="stage3-xrslam-$suffix"
nav="stage3-nav2-$suffix"
cleanup() {
  trap - EXIT INT TERM
  for name in "$nav" "$xr" "$parent"; do
    if podman container exists "$name"; then
      podman stop --time 26 "$name" >/dev/null 2>&1 || true
      echo "--- $name ---" >>"$log"
      podman logs "$name" >>"$log" 2>&1 || true
      podman rm -f "$name" >/dev/null 2>&1 || true
    fi
  done
}
trap 'cleanup; echo "STAGE3_CAMERA_FAILURE_LOG=$log" >&2; sha256sum "$log" >&2' EXIT
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
  -e ROS_LOG_DIR=/tmp/stage3-ros -e PYTHONDONTWRITEBYTECODE=1 \
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
echo 'STAGE3_LIVE_CAMERA_PARENT_NETWORK=NONE'
side='set -e; . /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; test ! -e /dev/video2; test ! -e /dev/ttyACM0; exec "$@"'
podman run -d --name "$xr" --pull=never --network "container:$parent" --ipc "container:$parent" \
  --pid private --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_smoke_monocular.yaml,dst=/opt/stage3_slam.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_smoke_device.yaml,dst=/opt/stage3_device.yaml,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/stage3-ros -e PYTHONDONTWRITEBYTECODE=1 \
  --entrypoint /bin/bash "$xrimage" -c "$side" stage3 \
  ros2 run leroy_xrslam leroy_xrslam_node --ros-args \
    -p slam_yaml:=/opt/stage3_slam.yaml -p device_template_yaml:=/opt/stage3_device.yaml \
    -p image_topic:=/camera/image_rect -p camera_info_topic:=/camera/camera_info \
    -p camera_optical_frame:=camera_optical_frame \
    -p pose_convention_validated:=false -p enable_map_tf:=false >/dev/null || fail XRSLAM_START
podman run -d --name "$nav" --pull=never --network "container:$parent" --ipc "container:$parent" \
  --pid private --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=128m \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_nav2_smoke.launch.py,dst=/opt/stage3_nav2.launch.py,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage3_live_camera_audit.py,dst=/opt/stage3_live_audit.py,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e ROS_LOG_DIR=/tmp/stage3-ros -e PYTHONDONTWRITEBYTECODE=1 \
  -e M23BV_STAGE3_NO_DEVICE_APP_SMOKE=1 -e M23BV_STAGE3_CAMERA_SIDECAR=1 \
  --entrypoint /bin/bash "$navimage" -c "$side" stage3 \
  ros2 launch /opt/stage3_nav2.launch.py >/dev/null || fail NAV2_START
for name in "$parent" "$xr" "$nav"; do
  [[ "$(podman inspect "$name" --format '{{.State.Running}}')" == true ]] || fail APP_EARLY_EXIT
done
echo STAGE3_LIVE_CAMERA_APPS=CAMERA_DEPTHART_XRSLAM_NAV2
set +e
timeout --foreground --signal=TERM --kill-after=8s 133s \
  podman exec "$nav" /bin/bash -c \
    '. /opt/ros/jazzy/setup.sh; . /leroy_ws/install/setup.sh; exec python3 -u /opt/stage3_live_audit.py' >>"$log" 2>&1
rc=$?
set -e
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
grep -E 'M23BV_STAGE3_REAL_CAMERA_AUDIT=|M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=|M23BV_STAGE3_REAL_CAMERA_READY=' "$log" | tail -n 15 || true
[[ "$rc" == 0 ]] || fail AUDIT_RUNTIME
grep -qx M23BV_STAGE3_REAL_CAMERA_AUDIT=PASS "$log" || fail AUDIT_VERDICT
grep -qx M23BV_STAGE3_REAL_CAMERA_SHUTDOWN=PASS "$log" || fail SHUTDOWN
echo STAGE3_LIVE_CAMERA=PASS_DIAGNOSTICS_ONLY
echo STAGE3_IMU_RECORDING_MOTION=NOT_AUTHORIZED
