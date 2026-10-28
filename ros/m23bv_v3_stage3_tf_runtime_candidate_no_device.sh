#!/usr/bin/env bash
# Stage 3: install YAML-only sensor TF into a DISTINCT candidate of the
# real Leroy XRSLAM runtime, then verify real six-edge /tf_static.
# Never replaces accepted images, runs bringup/bridge, maps devices or moves.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_TF_RUNTIME=BLOCKED_$1" >&2; exit 2; }

[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
inputs=(
  Dockerfile.stage3-tf-runtime-candidate
  m23bv_v3_stage3_tf_runtime_env.sh
  m23bv_v3_stage3_tf_runtime_candidate_verify.py
  m23bv_v3_stage3_sensor_tf_live_verify.py
  src/leroy_description/CMakeLists.txt
  src/leroy_description/package.xml
  src/leroy_description/config/sensor_mounts.yaml
  src/leroy_description/launch/description.launch.py
)
for file in "${inputs[@]}"; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] ||
    fail SOURCE_DRIFT
done
[[ ! -e src/leroy_description/urdf/leroy.urdf.xacro ]] ||
  fail LEGACY_URDF_PRESENT
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
command -v podman >/dev/null || fail PODMAN_MISSING
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED

base=localhost/leroy-ros:xrslam
base_id=20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a
got="$(podman image inspect "$base" --format '{{.Id}}')" || fail XRSLAM_RUNTIME_BASE_MISSING
got="$(printf '%s' "$got" | sed 's/^sha256://')"
[[ "$got" == "$base_id" ]] || fail XRSLAM_RUNTIME_BASE_ID_DRIFT
echo "STAGE3_TF_RUNTIME_PINNED_XRSLAM_IMAGE=$got"

# Content-addressed, distinct CANDIDATE tag; never overwrite an accepted
# sensor, XRSLAM, Nav2 or normal Leroy runtime tag. Ignore unrelated local
# worktree deletions, since they are not copied by this build recipe.
source_key="$(
  for file in "${inputs[@]}"; do
    printf '%s %s\n' "$file" "$(git rev-parse "HEAD:ros/$file")"
  done | sha256sum | cut -d' ' -f1
)"
tag="localhost/leroy-ros:stage3-yaml-tf-${source_key:0:16}"
label_source="io.leroy.stage3-tf.source-key"
label_base="io.leroy.stage3-tf.base-id"
echo "STAGE3_TF_RUNTIME_CANDIDATE_TAG=$tag"

if podman image exists "$tag"; then
  existing_source="$(podman image inspect "$tag" --format '{{index .Labels "io.leroy.stage3-tf.source-key"}}')" ||
    fail CANDIDATE_INSPECT
  existing_base="$(podman image inspect "$tag" --format '{{index .Labels "io.leroy.stage3-tf.base-id"}}')" ||
    fail CANDIDATE_INSPECT
  [[ "$existing_source" == "$source_key" && "$existing_base" == "$base_id" ]] ||
    fail CANDIDATE_TAG_PROVENANCE_MISMATCH
  echo STAGE3_TF_RUNTIME_CANDIDATE_BUILD=REUSED_SAME_PROVENANCE
else
  # Offline build using only the already-pinned XRSLAM runtime and checked-in
  # TF package. This writes a NEW rootless local image, not the robot runtime.
  podman build --pull=never --network none --no-cache \
    --file Dockerfile.stage3-tf-runtime-candidate \
    --build-arg "STAGE3_TF_XRSLAM_BASE=$base" \
    --label "$label_source=$source_key" --label "$label_base=$base_id" \
    --tag "$tag" . || fail OFFLINE_CANDIDATE_BUILD
  echo STAGE3_TF_RUNTIME_CANDIDATE_BUILD=PASS_OFFLINE_SEPARATE_IMAGE
fi
candidate_id="$(podman image inspect "$tag" --format '{{.Id}}')" ||
  fail CANDIDATE_IMAGE_INSPECT
echo "STAGE3_TF_RUNTIME_CANDIDATE_IMAGE_ID=$candidate_id"
default_cmd="$(podman image inspect "$tag" --format '{{json .Config.Cmd}}')" ||
  fail CANDIDATE_CMD_INSPECT
[[ "$default_cmd" == *STAGE3_TF_RUNTIME_CANDIDATE=INERT_NO_AUTOMATIC_ROBOT_BRINGUP* ]] ||
  fail CANDIDATE_DEFAULT_ROBOT_STARTUP_UNSAFE
echo STAGE3_TF_RUNTIME_CANDIDATE_DEFAULT=PASS_INERT_NO_ROBOT_BRINGUP
[[ "$(podman image inspect "$base" --format '{{.Id}}' | sed 's/^sha256://')" == "$base_id" ]] ||
  fail BASE_MUTATED

# No inherited robot entrypoint: launch only inert Bash. Only two
# pre-audited read-only verification scripts are mounted in the
# network-free, no-devices container. Normal bringup is NEVER launched.
podman run --rm --interactive --pull=never \
  --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=256m \
  --env PYTHONDONTWRITEBYTECODE=1 \
  --env ROS_DOMAIN_ID=170 \
  --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env ROS_LOG_DIR=/tmp/leroy-tf-runtime-logs \
  --env ROS_HOME=/tmp/leroy-tf-runtime-ros-home \
  --env XDG_CACHE_HOME=/tmp/leroy-tf-runtime-cache \
  --mount type=bind,src="$PWD/m23bv_v3_stage3_tf_runtime_candidate_verify.py",dst=/opt/leroy_tf_runtime_verify.py,readonly \
  --mount type=bind,src="$PWD/m23bv_v3_stage3_sensor_tf_live_verify.py",dst=/opt/leroy_tf_live_verify.py,readonly \
  --entrypoint /bin/bash "$tag" -s <<'LEROY_STAGE3_TF_RUNTIME_CONTAINER' || fail CANDIDATE_LIVE_TF_VERIFICATION
set -Eeuo pipefail
shopt -s nullglob
for dev in /dev/video* /dev/ttyACM* /dev/ttyUSB*; do
  echo "STAGE3_TF_RUNTIME=BLOCKED_PHYSICAL_DEVICE_VISIBLE:$dev" >&2
  exit 2
done
test "$ROS_DOMAIN_ID" = 170
test "$ROS_AUTOMATIC_DISCOVERY_RANGE" = LOCALHOST
test "$RMW_IMPLEMENTATION" = rmw_fastrtps_cpp
test "$ROS_LOG_DIR" = /tmp/leroy-tf-runtime-logs
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME" "$XDG_CACHE_HOME"
# Source the REAL installed overlay in its full Leroy runtime underlay.
# The helper bounds nounset deactivation to the trusted ROS setup scripts.
. /usr/local/bin/leroy-stage3-tf-runtime-env
test "$(ros2 pkg prefix leroy_description)" = /opt/leroy_stage3_tf_ws/install
echo STAGE3_TF_RUNTIME_SOURCE_OVERLAY=PASS_FULL_RUNTIME_SELECTED
python3 -B /opt/leroy_tf_runtime_verify.py
echo STAGE3_TF_RUNTIME_CONTAINER_DEVICES=NONE
LEROY_STAGE3_TF_RUNTIME_CONTAINER

echo STAGE3_TF_RUNTIME=PASS_CANDIDATE_REAL_TF_NO_DEVICE
echo STAGE3_TF_RUNTIME_BASE_IMAGE=UNCHANGED
echo STAGE3_TF_RUNTIME_ACTIVE_ROBOT=NOT_DEPLOYED
echo STAGE3_TF_RUNTIME_GEOMETRY_CALIBRATION=PROVISIONAL
echo STAGE3_TF_RUNTIME_MCU_CAMERA_MOTORS=NOT_AUTHORIZED
