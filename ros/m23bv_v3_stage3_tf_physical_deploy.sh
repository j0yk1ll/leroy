#!/usr/bin/env bash
# Physical-host opt-in, ONLY six static TF nodes in isolated Stage 3 DDS.
# No camera/serial/bridge/odometry/Nav2/motor launch or device mappings.
set -Eeuo pipefail
umask 077
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_TF_DEPLOY=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
for file in m23bv_v3_stage3_tf_physical_graph_probe.py \
            src/leroy_description/config/sensor_mounts.yaml \
            src/leroy_description/launch/description.launch.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] ||
    fail SOURCE_DRIFT
done
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail ROOTLESS_REQUIRED
[[ -r /dev/tty && -w /dev/tty ]] || fail INTERACTIVE_TTY_REQUIRED

image_id=9985c05cfcd5d9774b64b790c6f5c1b39c29f989d7ace6dca4e341f5f511841d
actual="$(podman image inspect "$image_id" --format '{{.Id}}')" ||
  fail CANDIDATE_IMAGE_MISSING
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$image_id" ]] || fail CANDIDATE_IMAGE_ID_DRIFT
default_cmd="$(podman image inspect "$image_id" --format '{{json .Config.Cmd}}')" ||
  fail CANDIDATE_CMD_INSPECT
[[ "$default_cmd" == *STAGE3_TF_RUNTIME_CANDIDATE=INERT_NO_AUTOMATIC_ROBOT_BRINGUP* ]] ||
  fail INERT_CANDIDATE_DEFAULT_MISSING
echo "STAGE3_TF_DEPLOY_PINNED_CANDIDATE=$actual"

# Explicit running parent: only known accepted sensor image, network=none.
parent="${LEROY_STAGE3_TF_DDS_PARENT:-}"
[[ "$parent" =~ ^[A-Za-z0-9][A-Za-z0-9_.-]{0,90}$ ]] ||
  fail EXPLICIT_SENSOR_DDS_PARENT_REQUIRED
[[ "$parent" != leroy-stage3-tf-only ]] || fail BAD_PARENT
[[ "$(podman inspect "$parent" --format '{{.State.Running}}')" == true ]] ||
  fail SENSOR_PARENT_NOT_RUNNING
parent_image="$(podman inspect "$parent" --format '{{.Image}}')" ||
  fail PARENT_INSPECT
parent_image="${parent_image#sha256:}"
[[ "$parent_image" == f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c ]] ||
  fail SENSOR_PARENT_IMAGE_NOT_ACCEPTED
[[ "$(podman inspect "$parent" --format '{{.HostConfig.NetworkMode}}')" == none ]] ||
  fail SENSOR_PARENT_NETWORK_NOT_ISOLATED
[[ "$(podman inspect "$parent" --format '{{.HostConfig.IpcMode}}')" == shareable ]] ||
  fail SENSOR_PARENT_IPC_NOT_SHAREABLE

name=leroy-stage3-tf-only
if podman container exists "$name"; then fail DEPLOYMENT_ALREADY_EXISTS; fi
probe="$PWD/m23bv_v3_stage3_tf_physical_graph_probe.py"
common=(
  --pull=never
  --network "container:$parent" --ipc "container:$parent"
  --pid private --read-only --cap-drop all
  --security-opt no-new-privileges
  --tmpfs /tmp:rw,nosuid,nodev,size=96m
  --env ROS_DOMAIN_ID=170 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  --env ROS_LOG_DIR=/tmp/leroy-stage3-physical-tf-logs
  --env ROS_HOME=/tmp/leroy-stage3-physical-tf-home
  --env PYTHONDONTWRITEBYTECODE=1
  --mount "type=bind,src=$probe,dst=/opt/leroy_stage3_tf_graph_probe.py,readonly"
  --entrypoint /bin/bash
)

echo STAGE3_TF_DEPLOY_SCOPE=ONLY_SIX_STATIC_TF_NO_HARDWARE_MAPPING
echo "STAGE3_TF_DEPLOY_PARENT=$parent"
printf '%s\n' \
  'The TF sidecar joins ONLY the existing isolated sensor DDS namespace.' \
  'It cannot map camera/MCU, launch the motor bridge or enable Nav2.' \
  'Static mount coordinates are PROVISIONAL, not measured calibration.' \
  'Physically secure the chassis, disconnect BOTH motor-power rails,' \
  'and verify the emergency stop before this supervised TF-only session.' \
  'Type exactly DEPLOY_TF_ONLY to authorize the session:' >&2
IFS= read -r approval </dev/tty || fail NO_OPERATOR_REPLY
[[ "$approval" == DEPLOY_TF_ONLY ]] || fail OPERATOR_DECLINED

# Preflight must see ZERO existing /tf_static writers and NO bridge.
podman run --rm "${common[@]}" "$image_id" \
  -c 'set -e; . /usr/local/bin/leroy-stage3-tf-runtime-env; exec python3 -B /opt/leroy_stage3_tf_graph_probe.py preflight' ||
    fail PREEXISTING_TF_OR_BRIDGE
[[ "$(podman inspect "$parent" --format '{{.State.Running}}')" == true ]] ||
  fail PARENT_STOPPED_AFTER_PREFLIGHT

owned=0
cleanup() {
  trap - EXIT INT TERM
  if [[ "$owned" == 1 ]]; then
    podman stop --time 12 "$name" >/dev/null 2>&1 || true
    podman rm -f "$name" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
owned=1
podman run --detach --name "$name" --stop-signal SIGINT \
  "${common[@]}" "$image_id" \
  -c 'set -e; . /usr/local/bin/leroy-stage3-tf-runtime-env; exec ros2 launch leroy_description description.launch.py' \
  >/dev/null || fail TF_ONLY_SIDECAR_START
[[ "$(podman inspect "$name" --format '{{.State.Running}}')" == true ]] ||
  fail TF_ONLY_SIDECAR_EARLY_EXIT

podman exec "$name" /bin/bash -c \
  'set -e; . /usr/local/bin/leroy-stage3-tf-runtime-env; exec python3 -B /opt/leroy_stage3_tf_graph_probe.py postflight' ||
    fail TF_ONLY_DDS_POSTFLIGHT
[[ "$(podman inspect "$parent" --format '{{.State.Running}}')" == true ]] ||
  fail SENSOR_PARENT_STOPPED
[[ "$(podman inspect "$name" --format '{{.State.Running}}')" == true ]] ||
  fail TF_SIDECAR_STOPPED
owned=0
trap - EXIT INT TERM
echo STAGE3_TF_DEPLOY=PASS_PHYSICAL_HOST_TF_ONLY_SIDECAR
echo STAGE3_TF_DEPLOY_MCU_CAMERA_MOTOR_OPEN=NONE
echo STAGE3_TF_DEPLOY_MOTION_AND_ODOMETRY=NOT_AUTHORIZED
echo "STAGE3_TF_DEPLOY_STOP_COMMAND=podman stop $name"
echo STAGE3_TF_DEPLOY_ACTIVE_ROBOT_BRINGUP=NOT_STARTED
