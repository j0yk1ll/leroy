#!/usr/bin/env bash
# No-device ROS DDS smoke: pinned V3 sensor, XRSLAM and Nav2 images.
# Only one read-only Python source file is bind-mounted into each container.
set -Eeuo pipefail

SENSOR_IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
XRSLAM_IMAGE='localhost/leroy-ros:xrslam'
NAV2_IMAGE='localhost/leroy-nav2:jazzy'
SENSOR_ID='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
XRSLAM_ID='20f9cf399ef331dda4aca093182787f8d622aa297b2d8fbcf730541a6f1cd43a'
NAV2_ID='89b3733f7aa281b6083d9078a11fe10e2f4f096f2aca5511a108d4873a3cc7c4'

fail() { printf 'M23BV_V3_STAGE3_DDS=BLOCKED_%s\n' "$1" >&2; exit 2; }
command -v podman >/dev/null 2>&1 || fail PODMAN_MISSING
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail PODMAN_NOT_ROOTLESS

check_image() {
  local name="$1" expected="$2" role="$3" actual
  podman image exists "$name" || fail "IMAGE_MISSING_$role"
  actual="$(podman image inspect "$name" --format '{{.Id}}')" ||
    fail "IMAGE_INSPECT_$role"
  actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
  [[ "$actual" == "$expected" ]] || fail "IMAGE_ID_DRIFT_$role"
  printf 'M23BV_V3_STAGE3_DDS_IMAGE_%s=%s\n' "$role" "$actual"
}
check_image "$SENSOR_IMAGE" "$SENSOR_ID" SENSOR
check_image "$XRSLAM_IMAGE" "$XRSLAM_ID" XRSLAM
check_image "$NAV2_IMAGE" "$NAV2_ID" NAV2

source_dir="$(cd -- "$(dirname -- "$0")" && pwd -P)"
probe_file="$source_dir/m23bv_v3_stage3_dds_probe.py"
[[ -f "$probe_file" && -r "$probe_file" ]] || fail PROBE_SOURCE_MISSING

publisher_name="leroy-stage3-no-device-dds-$$"
if podman container exists "$publisher_name"; then
  fail CONTAINER_NAME_COLLISION
fi
owned=0
cleanup() {
  trap - EXIT INT TERM
  if [[ "$owned" == 1 ]]; then
    podman stop --time 5 "$publisher_name" >/dev/null 2>&1 || true
    podman rm -f "$publisher_name" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

nonce="leroy-v3-dds-$$-$(date +%s)"
# Bash argv after -c: $0=stage3, $1=role, $2=nonce.
run_probe='
set -e
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
exec python3 /opt/leroy_stage3_dds_probe.py "$1" --nonce "$2"
'

# Force inert Bash instead of the image entrypoint. The parent network
# namespace is NONE, with no device mappings and no host IPC. Other test
# containers join only the parent's isolated network and IPC namespaces.
owned=1
podman run --detach --name "$publisher_name" \
  --network none --ipc shareable --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  --volume "$probe_file:/opt/leroy_stage3_dds_probe.py:ro" \
  --entrypoint /bin/bash \
  --env ROS_DOMAIN_ID=170 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env ROS_LOG_DIR=/tmp/stage3-ros-logs \
  --env PYTHONDONTWRITEBYTECODE=1 \
  "$SENSOR_IMAGE" -c "$run_probe" stage3 publish "$nonce" \
  >/dev/null || fail PUBLISHER_START

mode="$(podman inspect "$publisher_name" --format '{{.HostConfig.NetworkMode}}')" ||
  fail PUBLISHER_INSPECT
[[ "$mode" == none ]] || fail UNEXPECTED_NETWORK_MODE
[[ "$(podman inspect "$publisher_name" --format '{{.State.Running}}')" == true ]] ||
  fail PUBLISHER_EARLY_EXIT
echo 'M23BV_V3_STAGE3_DDS_PARENT_NETWORK=NONE'

receive_from() {
  local role="$1" image="$2"
  podman run --rm \
    --network "container:$publisher_name" --ipc "container:$publisher_name" \
    --pid private --read-only --cap-drop all \
    --security-opt no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=64m \
    --volume "$probe_file:/opt/leroy_stage3_dds_probe.py:ro" \
    --entrypoint /bin/bash \
    --env ROS_DOMAIN_ID=170 --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env ROS_LOG_DIR=/tmp/stage3-ros-logs \
    --env PYTHONDONTWRITEBYTECODE=1 \
    "$image" -c "$run_probe" stage3 "$role" "$nonce" ||
      fail "NO_MATCHED_DDS_FROM_$role"
}
receive_from XRSLAM "$XRSLAM_IMAGE"
receive_from NAV2 "$NAV2_IMAGE"

printf '%s\n' \
  'M23BV_V3_STAGE3_DDS=PASS_THREE_IMAGE_ISOLATED_TRANSPORT' \
  'M23BV_V3_STAGE3_DDS_SENSOR_ACCESS=NONE' \
  'M23BV_V3_STAGE3_DDS_XRSLAM_AND_NAV2_NODES=NOT_STARTED' \
  'M23BV_V3_STAGE3_DDS_MOTION_AND_RECORDING=NOT_AUTHORIZED'
