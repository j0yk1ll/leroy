#!/usr/bin/env bash
# M23BV Stage 1 CAMERA-ONLY V3 trial. NEVER called by selected production runners.
# This script opens the camera ONLY after separately documented authorization.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
fail() { printf 'M23BV_V3_STAGE1=BLOCKED_%s\n' "$1" >&2; exit 2; }
IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
IMAGE_ID='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
BRANCH='feature/m23bv-v3-stage1-camera-only-optin'

# Values are a recording mechanism, NEVER evidence that isolation is physically real.
[[ "${M23BV_STAGE1_CAMERA_ONLY_APPROVED:-}" == YES &&
   "${M23BV_STAGE1_MOTOR_RAILS_ISOLATED:-}" == YES &&
   "${M23BV_STAGE1_ESTOP_VERIFIED:-}" == YES &&
   "${M23BV_STAGE1_CHASSIS_SECURED:-}" == YES ]] || fail OPERATOR_SIGNOFF_MISSING
[[ -n "${M23BV_STAGE1_OPERATOR:-}" &&
   "${M23BV_STAGE1_APPROVAL_RECORD:-}" == /* &&
   -f "$M23BV_STAGE1_APPROVAL_RECORD" &&
   ! -L "$M23BV_STAGE1_APPROVAL_RECORD" &&
   -r "$M23BV_STAGE1_APPROVAL_RECORD" ]] ||
  fail OPERATOR_ID_OR_APPROVAL_RECORD
[[ -r /dev/tty ]] || fail INTERACTIVE_TTY_REQUIRED
printf '%s\n' 'Camera-only V3 physical access request. Both motor rails disconnected,' \
  'chassis secured, E-stop checked, human supervising. No MCU or recording.' >&2
printf '%s\n' 'Type exactly: AUTHORIZE V3 CAMERA ONLY' >&2
IFS= read -r confirmation </dev/tty || fail INTERACTIVE_CONFIRMATION
[[ "$confirmation" == 'AUTHORIZE V3 CAMERA ONLY' ]] || fail INTERACTIVE_CONFIRMATION

# Before any Podman device mapping: source and camera identity checks.
repo_root="$(git rev-parse --show-toplevel)" || fail NOT_A_WORKTREE
[[ "$(git branch --show-current)" == "$BRANCH" ]] || fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_WORKTREE
while read -r path expected; do
  actual="$(git rev-parse "HEAD:$path" 2>/dev/null)" || fail MISSING_SOURCE_PIN
  [[ "$actual" == "$expected" ]] || fail SOURCE_PIN_CHANGED
  [[ "$(git hash-object "$repo_root/$path")" == "$expected" ]] || fail WORKTREE_SOURCE_DRIFT
done <<'PINS'
ros/m23bv_sensor_only_run.sh 8e0db7a1f223fc992ba7704039e9ceb5a48465a4
ros/m23bv_camera_only.launch.py 7ea893b9050da690b7163d508cd536617f3e4a41
ros/m23bv_reliable_rectifier_v3_candidate.launch.py a0ae75a62da39235b113e3d2c324efda5fcfc6e1
ros/m23bv_camera_ready_gate_sigint_v2.py 777db7cbeb0ce6c3669a837f1530f7b1499dd9dd
ros/m23bv_reliable_rectifier_proto/src/rectify.cpp 7e5d199e633f7cf0dac266a656ae70849df07f71
PINS

[[ "${M23BV_STAGE1_CAMERA_ID_PATH:-}" == /dev/v4l/by-id/* &&
   -L "$M23BV_STAGE1_CAMERA_ID_PATH" ]] || fail STABLE_CAMERA_ID_MISSING
camera_dev="$(readlink -f -- "$M23BV_STAGE1_CAMERA_ID_PATH")" ||
  fail CAMERA_ID_NOT_RESOLVABLE
# The operator-confirmed host device is /dev/video8.  The immutable V3 launch
# intentionally continues to see /dev/video2 ONLY inside the container.
[[ "$camera_dev" == /dev/video8 && -c "$camera_dev" ]] || fail CAMERA_ID_MISMATCH
cal="${M23BV_STAGE1_CALIBRATION_FILE:-}"
hash="${M23BV_STAGE1_CALIBRATION_SHA256:-}"
[[ "$cal" == /* && -f "$cal" && ! -L "$cal" && -r "$cal" &&
   "$hash" =~ ^[[:xdigit:]]{64}$ ]] || fail CALIBRATION_PROVENANCE
[[ "$(sha256sum -- "$cal" | awk '{print $1}')" == "$hash" ]] ||
  fail CALIBRATION_HASH_MISMATCH
logdir="${M23BV_LOG_DIR:-}"
[[ "$logdir" == /* && -d "$logdir" && ! -L "$logdir" &&
   -w "$logdir" ]] || fail PRIVATE_LOG_DIR
[[ -f m23bv_v3_stage1_camera_observer.py ]] || fail OBSERVER_MISSING
actual="$(podman image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null)" ||
  fail IMAGE_NOT_PRESENT
[[ "${actual#sha256:}" == "$IMAGE_ID" ]] || fail IMMUTABLE_IMAGE_MISMATCH
running="$(podman ps --format '{{.Names}}')" || fail CONTAINER_INVENTORY
[[ -z "$running" ]] || fail OTHER_CONTAINER_RUNNING

# Never use the selected sensor-only parent, M20, Nav2, XRSLAM or a recorder.
log="$(mktemp --tmpdir="$logdir" m23bv_v3_stage1_XXXXXXXX.log)"
name="m23bv-v3-stage1-$$"
echo "M23BV_V3_STAGE1_OPERATOR=$M23BV_STAGE1_OPERATOR" >>"$log"
echo "M23BV_V3_STAGE1_APPROVAL_RECORD=$M23BV_STAGE1_APPROVAL_RECORD" >>"$log"
printf 'M23BV_V3_STAGE1_APPROVAL_SHA256=%s\n' \
  "$(sha256sum -- "$M23BV_STAGE1_APPROVAL_RECORD" | awk '{print $1}')" >>"$log"
echo "M23BV_V3_STAGE1_IMAGE_ID=$IMAGE_ID" >>"$log"
echo "M23BV_V3_STAGE1_CALIBRATION_SHA256=$hash" >>"$log"
echo "M23BV_V3_STAGE1_CAMERA_SYMLINK=$M23BV_STAGE1_CAMERA_ID_PATH" >>"$log"
echo "M23BV_V3_STAGE1_CAMERA_HOST_DEVICE=$camera_dev" >>"$log"
echo "M23BV_V3_STAGE1_CAMERA_CONTAINER_DEVICE=/dev/video2" >>"$log"
echo 'M23BV_V3_STAGE1_SOURCE_PINS=PASS' >>"$log"
cleanup() {
  # Treat unexpected host exit as an abort. No automatic re-run.
  podman stop --time 8 "$name" >/dev/null 2>&1 || true
  podman rm -f "$name" >/dev/null 2>&1 || true
}
trap 'cleanup; if [[ -f "$log" ]]; then printf "M23BV_V3_STAGE1_ABORT_LOG_SHA256=" >&2; sha256sum "$log" >&2; fi' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
set +e
timeout --foreground --signal=INT --kill-after=15s 75s \
  podman run --rm --name "$name" --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  --device "$camera_dev:/dev/video2:rw" \
  --mount "type=bind,src=$cal,dst=/config/camera/front_camera.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage1_camera_observer.py,dst=/opt/m23bv/m23bv_v3_stage1_camera_observer.py,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 \
  -e M23BV_RELIABLE_RECTIFIER_PROTO_V3=1 \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-stage1-ros; mkdir -p "$ROS_LOG_DIR"; exec python3 -u /opt/m23bv/m23bv_v3_stage1_camera_observer.py' \
  >>"$log" 2>&1
rc=$?
set -e
cleanup
trap - EXIT
printf 'M23BV_V3_STAGE1_PROCESS_EXIT=%s\n' "$rc" >>"$log"
printf 'M23BV_V3_STAGE1_LOG=%s\n' "$log"
sha256sum "$log"
if [[ "$rc" != 0 ]] ||
   ! grep -qx 'M23BV_V3_STAGE1_QOS=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE1_FRAMES=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE1_LAUNCH_REAPED=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE1_RESULT=PASS' "$log"; then
  echo 'M23BV_V3_STAGE1=BLOCKED_RUNTIME_OR_CLEANUP' >&2
  exit 2
fi
echo 'M23BV_V3_STAGE1=CAMERA_ONLY_PASS'
echo 'M23BV_V3_STAGE2_MCU_OR_RECORDING=NOT_AUTHORIZED'
