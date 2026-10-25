#!/usr/bin/env bash
# STAGE 2B PHYSICAL CAMERA + MCU IMU, sensor-only and human-supervised.
# No recording, motor commands, M20 bridge, Nav2, XRSLAM or production changes.
# Serial CDC open may RESET motor-capable firmware. MOTOR RAILS MUST BE DISCONNECTED.
# Two additional protected read-only source bind mounts supply the later
# M20 STOP-finally fix and M18 terminal-fault latch absent in inherited V3 image.
# They are exact Git-blobs verified below; V3 camera binaries/image are unchanged.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "${BASH_SOURCE[0]}")"
fail() { echo "M23BV_V3_STAGE2B=BLOCKED_$1" >&2; exit 2; }

IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
IMAGE_ID='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
BRANCH='feature/m23bv-v3-stage2b-physical-sensors-optin'
# Must be an explicitly selected, visually verified capture interface.
# The prior by-id ...video-index0 pointed to WRONG /dev/video6 even
# though its ID_SERIAL matched; operator ffplay identified /dev/video8.
# Never silently choose the first index or any /dev/videoN by itself.
CAMERA_BY_ID="${M23BV_STAGE2B_CAMERA_STABLE_LINK:-}"
CAMERA_VERIFIED_NODE="${M23BV_STAGE2B_CAMERA_VISUALLY_VERIFIED_NODE:-}"
CAMERA_SERIAL='Wed_Camera_Wed_Camera_20240420112206'
# Two Wed Camera interfaces have the SAME ID_SERIAL and model.
# ONLY USB port 3.4.2 was visually verified with ffplay as front camera.
CAMERA_APPROVED_LINK='/dev/v4l/by-path/pci-0000:00:14.0-usb-0:3.4.2:1.0-video-index0'
CAMERA_APPROVED_ID_PATH='pci-0000:00:14.0-usb-0:3.4.2:1.0'
CAMERA_VENDOR_ID='32e6'
CAMERA_MODEL_ID='9221'
CAL_SHA='f1c99cb8d4e4f3c77373dcabbe683dee9ffbc050614396fcd2964d61e965a138'

root="$(git rev-parse --show-toplevel)" || fail WORKTREE
[[ "$(git branch --show-current)" == "$BRANCH" ]] || fail BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_CHECKOUT
git merge-base --is-ancestor 2feee11f0043454a9b618521c737ac0468f0712e HEAD ||
  fail STACKED_BASE_CHANGED
for source in m23bv_v3_stage2b_physical_probe.py m23bv_v3_stage1_camera_observer.py; do
  [[ -f "$source" && ! -L "$source" ]] || fail SCRIPT_MISSING
  [[ "$(git hash-object "$source")" == "$(git rev-parse "HEAD:ros/$source")" ]] ||
    fail LOCAL_SCRIPT_DRIFT
done
while read -r source sha; do
  [[ "$(git rev-parse "HEAD:$source")" == "$sha" &&
     "$(git hash-object "$root/$source")" == "$sha" ]] || fail PROTECTED_SOURCE_DRIFT
done <<'PINS'
ros/m23bv_reliable_rectifier_v3_candidate.launch.py a0ae75a62da39235b113e3d2c324efda5fcfc6e1
ros/m23bv_v3_stage1_camera_observer.py 93320b985c5dda3fdf0d1f8def001ba06480e7b6
ros/m23bv_camera_ready_gate_sigint_v2.py 777db7cbeb0ce6c3669a837f1530f7b1499dd9dd
ros/m23bv_reliable_rectifier_proto/src/rectify.cpp 7e5d199e633f7cf0dac266a656ae70849df07f71
ros/m20_combined_observer.py 5cf2f88cb2f79be80821888401b57a6393ee5e3b
ros/m20_combined_contract.py 66111d8da486dc1e0c78057c1948f5e1aad9a06e
ros/m18_imu_stream_source.py 3c5fb37ca291533e2702a9159f3e8db11b076d22
ros/m18_imu_stream_contract.py 443aa82066e88d50cb88859304e9197e3f459ee0
PINS

logdir="${M23BV_LOG_DIR:-}"
[[ "$logdir" == /* && -d "$logdir" && ! -L "$logdir" &&
   -w "$logdir" ]] || fail OWNER_PRIVATE_LOGDIR
[[ "$(stat -c %a "$logdir")" == 700 ]] || fail LOGDIR_MODE
[[ -r /dev/tty && -w /dev/tty ]] || fail INTERACTIVE_OPERATOR_REQUIRED

# Bind the exact real camera stream established visually by the operator
# to a stable V4L by-id or by-path symlink. No implicit default.
[[ "$CAMERA_BY_ID" == "$CAMERA_APPROVED_LINK" ]] ||
  fail CAMERA_APPROVED_USB_PORT_LINK_REQUIRED
[[ "$CAMERA_VERIFIED_NODE" =~ ^/dev/video[0-9]+$ ]] ||
  fail CAMERA_VISUAL_CONFIRMATION_REQUIRED
[[ -L "$CAMERA_BY_ID" ]] || fail CAMERA_STABLE_ID_MISSING
cam="$(readlink -f -- "$CAMERA_BY_ID")" || fail CAMERA_RESOLUTION
[[ "$cam" =~ ^/dev/video[0-9]+$ && -c "$cam" ]] ||
  fail CAMERA_DEVICE_NOT_VIDEO_CAPTURE_NODE
[[ "$cam" == "$CAMERA_VERIFIED_NODE" ]] ||
  fail CAMERA_LINK_NOT_VISUALLY_VERIFIED_STREAM
cam_properties="$(udevadm info --query=property --name="$cam")" ||
  fail CAMERA_UDEV_INFO
cam_serial="$(sed -n 's/^ID_SERIAL=//p' <<<"$cam_properties")"
[[ "$cam_serial" == "$CAMERA_SERIAL" ]] || fail CAMERA_SERIAL_CHANGED
cam_id_path="$(sed -n 's/^ID_PATH=//p' <<<"$cam_properties")"
cam_vendor="$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$cam_properties")"
cam_model="$(sed -n 's/^ID_MODEL_ID=//p' <<<"$cam_properties")"
[[ "$cam_id_path" == "$CAMERA_APPROVED_ID_PATH" ]] ||
  fail CAMERA_USB_PORT_CHANGED
[[ "$cam_vendor" == "$CAMERA_VENDOR_ID" &&
   "$cam_model" == "$CAMERA_MODEL_ID" ]] ||
  fail CAMERA_USB_VENDOR_MODEL_CHANGED
cam_v4l_caps="$(sed -n 's/^ID_V4L_CAPABILITIES=//p' <<<"$cam_properties")"
[[ "$cam_v4l_caps" == *:capture:* ]] || fail CAMERA_NOT_CAPTURE_CAPABLE
# Pin character-device major:minor to detect hotplug replacement after
# operator approval. Never accept a generic /dev/video* without by-id+serial.
cam_rdev="$(stat -Lc '%t:%T' -- "$cam")" || fail CAMERA_RDEV_MISSING

mcu_by_id="${M23BV_STAGE2B_MCU_BY_ID:-}"
mcu_id="${M23BV_STAGE2B_MCU_ID_SERIAL:-}"
[[ "$mcu_by_id" == /dev/serial/by-id/* && -L "$mcu_by_id" &&
   -n "$mcu_id" ]] || fail MCU_STABLE_ID_NOT_PINNED
mcu="$(readlink -f "$mcu_by_id")" || fail MCU_LINK_RESOLUTION
[[ "$mcu" =~ ^/dev/ttyACM[0-9]+$ && -c "$mcu" ]] || fail MCU_DEVICE_NOT_ACM
actual_mcu_id="$(udevadm info --query=property --name="$mcu" |
  sed -n 's/^ID_SERIAL=//p')" || fail MCU_UDEV_INFO
[[ "$actual_mcu_id" == "$mcu_id" ]] || fail MCU_SERIAL_ID_MISMATCH
[[ "$cam" != "$mcu" ]] || fail DEVICES_IDENTICAL
mcu_rdev="$(stat -Lc '%t:%T' -- "$mcu")" || fail MCU_RDEV_MISSING

cal="$root/ros/config/camera/front_camera.yaml"
[[ -f "$cal" && ! -L "$cal" &&
   "$(sha256sum "$cal" | awk '{print $1}')" == "$CAL_SHA" ]] ||
  fail CAMERA_CALIBRATION_CHANGED

actual="$(podman image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null)" ||
  fail PINNED_IMAGE_MISSING
[[ "${actual#sha256:}" == "$IMAGE_ID" ]] || fail IMAGE_ID_CHANGED
[[ -z "$(podman ps --format '{{.Names}}')" ]] || fail CONTAINER_ALREADY_RUNNING

printf '%s\n' \
  'STAGE 2B MOTOR-CAPABLE MCU SERIAL ACCESS: motor rails BOTH physically' \
  'DISCONNECTED (electrical isolation), chassis secured, emergency stop' \
  'tested, supervisor present with immediate physical power disconnect.' \
  'No wheels or manipulators may be energized. Camera calibration must' \
  'belong to the pinned camera serial. No recording, Nav2 or XRSLAM.' \
  'Opening USB CDC can reset the MCU EVEN BEFORE START/STOP requests.' \
  'Type exactly: MOTOR RAILS OFF MCU SENSOR ONLY' >&2
IFS= read -r attestation </dev/tty || fail NO_PHYSICAL_ATTESTATION
[[ "$attestation" == 'MOTOR RAILS OFF MCU SENSOR ONLY' ]] ||
  fail SAFETY_ATTESTATION_DECLINED
printf '%s\n' "Operator-verified physical front-camera stream MUST be $cam." \
  "The exact link $CAMERA_BY_ID currently resolves to that node." \
  "Independently confirm THIS is the same visually checked camera." >&2
printf 'Type exactly: CAMERA STREAM VERIFIED %s\n' "$cam" >&2
IFS= read -r stream_attestation </dev/tty || fail NO_CAMERA_STREAM_ATTESTATION
[[ "$stream_attestation" == "CAMERA STREAM VERIFIED $cam" ]] ||
  fail CAMERA_STREAM_ATTESTATION_DECLINED
printf '%s\n' 'Type exactly: CALIBRATION VERIFIED FOR CAMERA' >&2
IFS= read -r cal_attestation </dev/tty || fail NO_CALIBRATION_ATTESTATION
[[ "$cal_attestation" == 'CALIBRATION VERIFIED FOR CAMERA' ]] ||
  fail CALIBRATION_UNVERIFIED
printf '%s\n' \
  'Type exactly: AUTHORIZE V3 CAMERA AND M20 IMU NO RECORDING' >&2
IFS= read -r authorization </dev/tty || fail NO_HUMAN_OPTIN
[[ "$authorization" == 'AUTHORIZE V3 CAMERA AND M20 IMU NO RECORDING' ]] ||
  fail OPTIN_DECLINED

# Recheck BOTH stable symlinks and USB IDs after physical attestations,
# before creating the approval record and before any Podman device mapping.
# Hotplug or unrelated process takeover must block the whole attempt.
[[ -L "$CAMERA_BY_ID" && "$(readlink -f -- "$CAMERA_BY_ID")" == "$cam" &&
   "$cam" == "$CAMERA_VERIFIED_NODE" &&
   -c "$cam" && "$(stat -Lc '%t:%T' -- "$cam")" == "$cam_rdev" ]] ||
  fail CAMERA_CHANGED_AFTER_SIGNOFF
cam_recheck="$(udevadm info --query=property --name="$cam")" ||
  fail CAMERA_RECHECK_UDEV
[[ "$(sed -n 's/^ID_SERIAL=//p' <<<"$cam_recheck")" == "$CAMERA_SERIAL" ]] ||
  fail CAMERA_SERIAL_CHANGED_AFTER_SIGNOFF
[[ "$(sed -n 's/^ID_PATH=//p' <<<"$cam_recheck")" == "$CAMERA_APPROVED_ID_PATH" ]] ||
  fail CAMERA_USB_PORT_CHANGED_AFTER_SIGNOFF
[[ "$(sed -n 's/^ID_VENDOR_ID=//p' <<<"$cam_recheck")" == "$CAMERA_VENDOR_ID" &&
   "$(sed -n 's/^ID_MODEL_ID=//p' <<<"$cam_recheck")" == "$CAMERA_MODEL_ID" ]] ||
  fail CAMERA_USB_VENDOR_MODEL_CHANGED_AFTER_SIGNOFF
[[ "$(sed -n 's/^ID_V4L_CAPABILITIES=//p' <<<"$cam_recheck")" == *:capture:* ]] ||
  fail CAMERA_CAPABILITY_CHANGED_AFTER_SIGNOFF
[[ -L "$mcu_by_id" && "$(readlink -f -- "$mcu_by_id")" == "$mcu" &&
   -c "$mcu" && "$(stat -Lc '%t:%T' -- "$mcu")" == "$mcu_rdev" ]] ||
  fail MCU_CHANGED_AFTER_SIGNOFF
mcu_recheck="$(udevadm info --query=property --name="$mcu")" ||
  fail MCU_RECHECK_UDEV
[[ "$(sed -n 's/^ID_SERIAL=//p' <<<"$mcu_recheck")" == "$mcu_id" ]] ||
  fail MCU_SERIAL_CHANGED_AFTER_SIGNOFF

# Create a NEW private approval record for THIS attempt only.
approval="$(mktemp --tmpdir="$logdir" m23bv_v3_stage2b_approval_XXXXXXXX.txt)"
cat >"$approval" <<EOF
M23BV V3 STAGE 2B CAMERA + MCU SENSOR-ONLY PHYSICAL ATTEMPT
Timestamp: $(date -Is)
Operator: $(id -un)
Review HEAD: $(git rev-parse HEAD)
Motor rails physically disconnected: OPERATOR ATTESTED
Chassis secured: OPERATOR ATTESTED
E-stop verified: OPERATOR ATTESTED
Immediate power disconnect supervised: OPERATOR ATTESTED
Camera calibration provenance: OPERATOR ATTESTED
Camera stable ID: $CAMERA_BY_ID
Camera visually verified host node: $CAMERA_VERIFIED_NODE
Camera observed stream: OPERATOR ATTESTED
Camera device: $cam
Camera USB serial: $cam_serial
Camera verified USB port: $cam_id_path
Camera USB vendor:model: $cam_vendor:$cam_model
MCU stable ID: $mcu_by_id
MCU device: $mcu
MCU ID_SERIAL: $actual_mcu_id
V3 image ID: $IMAGE_ID
Calibration SHA256: $CAL_SHA
MCU serial command whitelist: set_imu_stream_hz 50 / set_imu_stream_hz 0
Recording, motor actuation, Nav2, XRSLAM: NOT AUTHORIZED
EOF
chmod 0600 "$approval"

log="$(mktemp --tmpdir="$logdir" m23bv_v3_stage2b_XXXXXXXX.log)"
name="m23bv-v3-stage2b-$$"
{
  echo "M23BV_V3_STAGE2B_APPROVAL_RECORD=$approval"
  echo "M23BV_V3_STAGE2B_APPROVAL_SHA256=$(sha256sum "$approval" | awk '{print $1}')"
  echo "M23BV_V3_STAGE2B_GIT_HEAD=$(git rev-parse HEAD)"
  echo "M23BV_V3_STAGE2B_CAMERA_STABLE_ID=$CAMERA_BY_ID"
  echo "M23BV_V3_STAGE2B_CAMERA_VISUALLY_VERIFIED_NODE=$CAMERA_VERIFIED_NODE"
  echo "M23BV_V3_STAGE2B_CAMERA_DEVICE=$cam"
  echo "M23BV_V3_STAGE2B_CAMERA_ID_PATH=$cam_id_path"
  echo "M23BV_V3_STAGE2B_CAMERA_VENDOR_MODEL=$cam_vendor:$cam_model"
  echo "M23BV_V3_STAGE2B_CAMERA_RDEV=$cam_rdev"
  echo "M23BV_V3_STAGE2B_MCU_DEVICE=$mcu"
  echo "M23BV_V3_STAGE2B_MCU_ID_SERIAL=$actual_mcu_id"
  echo "M23BV_V3_STAGE2B_IMAGE_ID=$IMAGE_ID"
  echo "M23BV_V3_STAGE2B_CALIBRATION_SHA256=$CAL_SHA"
} >>"$log"
cleanup() {
  podman stop --time 25 "$name" >/dev/null 2>&1 || true
  podman rm -f "$name" >/dev/null 2>&1 || true
}
trap 'cleanup; echo M23BV_V3_STAGE2B_ABORT_OR_COMPLETION_LOG="$log" >&2' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

set +e
timeout --foreground --signal=INT --kill-after=45s 260s \
  podman run --rm --name "$name" --pull=never \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges --read-only \
  --tmpfs /tmp:rw,nosuid,nodev,size=768m \
  --device "$cam:/dev/video2:rw" \
  --device "$mcu:/dev/ttyACM0:rw" \
  --mount "type=bind,src=$cal,dst=/config/camera/front_camera.yaml,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage1_camera_observer.py,dst=/opt/m23bv/m23bv_v3_stage1_camera_observer.py,readonly" \
  --mount "type=bind,src=$PWD/m23bv_v3_stage2b_physical_probe.py,dst=/opt/m23bv/m23bv_v3_stage2b_physical_probe.py,readonly" \
  --mount "type=bind,src=$PWD/m20_combined_observer.py,dst=/opt/m20/m20_combined_observer.py,readonly" \
  --mount "type=bind,src=$PWD/m18_imu_stream_source.py,dst=/opt/m18/m18_imu_stream_source.py,readonly" \
  -e ROS_DOMAIN_ID=170 -e ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  -e LEROY_M20_SENSOR_ONLY=1 -e LEROY_NAV2_DRY_RUN=1 \
  -e M23BV_CAMERA_READY_CANDIDATE=1 \
  -e M23BV_RELIABLE_RECTIFIER_PROTO_V3=1 \
  -e M23BV_V3_STAGE2B_SENSOR_ONLY=YES \
  --entrypoint /bin/bash "$IMAGE" -c \
  'set -Eeuo pipefail; set +u; source /opt/ros/jazzy/setup.bash; source /leroy_ws/install/setup.bash; source /opt/m23bv_reliable_rectifier_proto/ws/install/setup.bash; set -u; export ROS_LOG_DIR=/tmp/m23bv-stage2b-ros; mkdir -p "$ROS_LOG_DIR"; export PYTHONPATH="/opt/m23bv:/opt/m20:/opt/m18:/opt/m18/monitor:${PYTHONPATH:-}"; exec python3 -u /opt/m23bv/m23bv_v3_stage2b_physical_probe.py' \
  >>"$log" 2>&1
rc=$?
set -e
cleanup
trap - EXIT

printf 'M23BV_V3_STAGE2B_PROCESS_EXIT=%s\n' "$rc" >>"$log"
printf 'M23BV_V3_STAGE2B_PROCESS_EXIT=%s\n' "$rc"
printf 'M23BV_V3_STAGE2B_LOG=%s\n' "$log"
sha256sum "$log"
grep -E 'M23BV_V3_STAGE2B_|M20_IMU_STREAM_.*ACK=|M20_COMBINED_VERDICT=' "$log" | tail -n 90 || true
if [[ "$rc" != 0 ]] ||
   ! grep -qx 'M23BV_V3_STAGE2B_PRE_SERIAL_GRAPH_QOS=PASS' "$log" ||
   ! grep -qx 'M20_IMU_STREAM_START_ACK=PASS' "$log" ||
   ! grep -qx 'M20_IMU_STREAM_STOP_ACK=PASS' "$log" ||
   ! grep -qx 'M20_COMBINED_VERDICT=OBSERVED_CONCURRENT_CAMERA_IMU_ROS_CONTRACT' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2B_CAMERA_LAUNCH_REAPED=PASS' "$log" ||
   ! grep -qx 'M23BV_V3_STAGE2B_RESULT=PASS' "$log"; then
  echo 'M23BV_V3_STAGE2B=BLOCKED_REAL_RUNTIME_OR_STOP_ACK' >&2
  echo 'M23BV_V3_STAGE2B_MCU_STATE=VERIFY_PHYSICALLY_IF_STOP_NOT_ACKED' >&2
  exit 2
fi
echo 'M23BV_V3_STAGE2B=PHYSICAL_CAMERA_IMU_PASS'
echo 'M23BV_V3_STAGE2B_RECORDING_AND_MOTION=NOT_AUTHORIZED'
