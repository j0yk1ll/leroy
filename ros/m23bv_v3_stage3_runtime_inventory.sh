#!/usr/bin/env bash
# M23BV V3 Stage 3: NO-DEVICE inspection of installed sensor/XRSLAM/Nav2 images.
# Never opens video, serial, ROS nodes, or a motor-capable MCU.
# No hardware mappings, recorder, bridge, Nav2 launch, or network access.
set -Eeuo pipefail

BASE_IMAGE='localhost/leroy-sensors:m23bv-reliable-rectifier-v3'
BASE_ID='f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c'
XRSLAM_IMAGE='localhost/leroy-ros:xrslam'
NAV2_IMAGE='localhost/leroy-nav2:jazzy'

fail() {
  printf 'M23BV_V3_STAGE3_INVENTORY=BLOCKED_%s\n' "$1" >&2
  exit 2
}
command -v podman >/dev/null 2>&1 || fail PODMAN_MISSING
[[ "$(id -u)" != 0 ]] || fail ROOT_NOT_ALLOWED

rootless="$(podman info --format '{{.Host.Security.Rootless}}')" || fail PODMAN_INFO
[[ "$rootless" == 'true' ]] || fail PODMAN_NOT_ROOTLESS

# Mandatory baseline is the exact previously accepted physical-camera image.
podman image exists "$BASE_IMAGE" || fail ACCEPTED_V3_IMAGE_MISSING
base_actual="$(podman image inspect "$BASE_IMAGE" --format '{{.Id}}')" ||
  fail ACCEPTED_V3_IMAGE_INSPECT
base_normalized="$(printf '%s' "$base_actual" | sed 's/^sha256://')"
[[ "$base_normalized" == "$BASE_ID" ]] || fail ACCEPTED_V3_IMAGE_ID_DRIFT
printf 'M23BV_V3_STAGE3_ACCEPTED_SENSOR_IMAGE_ID=%s\n' "$base_normalized"

# Force an inert shell entrypoint even if any inspected image has an
# automatic runtime entrypoint. This cannot publish to DDS or open devices.
probe_image() {
  local role="$1" image="$2" actual
  if ! podman image exists "$image"; then
    printf 'M23BV_V3_STAGE3_IMAGE_%s=ABSENT\n' "$role"
    return 0
  fi
  actual="$(podman image inspect "$image" --format '{{.Id}}')" ||
    fail IMAGE_INSPECT
  printf 'M23BV_V3_STAGE3_IMAGE_%s=%s\n' "$role" "$actual"
  podman run --rm --network none --ipc private --pid private \
    --read-only --cap-drop all --security-opt no-new-privileges \
    --tmpfs /tmp:rw,nosuid,nodev,size=64m \
    --entrypoint /bin/bash \
    -e STAGE3_PROBE_ROLE="$role" \
    "$image" -c '
set -eo pipefail
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
if [ -f /opt/m23bv_reliable_rectifier_proto/ws/install/setup.sh ]; then
  . /opt/m23bv_reliable_rectifier_proto/ws/install/setup.sh
fi
if [ -e /dev/video2 ] || [ -e /dev/ttyACM0 ]; then
  echo "M23BV_V3_STAGE3_PROBE=BLOCKED_PHYSICAL_DEVICE_EXPOSED" >&2
  exit 2
fi
for pkg in usb_cam m23bv_reliable_rectifier leroy_depthart \
    leroy_xrslam nav2_controller nav2_planner nav2_bt_navigator \
    leroy_nav2 leroy_nav2_perception leroy_nav2_integration \
    leroy_description leroy_bridge; do
  if prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null)"; then
    printf "STAGE3_PACKAGE_%s_%s=FOUND:%s\n" "$STAGE3_PROBE_ROLE" "$pkg" "$prefix"
  else
    printf "STAGE3_PACKAGE_%s_%s=MISSING\n" "$STAGE3_PROBE_ROLE" "$pkg"
  fi
done
if [ -s /opt/DepthART/checkpoints/metric/depthart_metric_indoor_s_448.pth ]; then
  echo "STAGE3_DEPTHART_CHECKPOINT=PRESENT"
else
  echo "STAGE3_DEPTHART_CHECKPOINT=ABSENT"
fi
if [ -s /opt/xrslam/lib/libxrslam.so ]; then
  echo "STAGE3_XRSLAM_LIBRARY=PRESENT"
else
  echo "STAGE3_XRSLAM_LIBRARY=ABSENT"
fi
if prefix="$(ros2 pkg prefix leroy_xrslam 2>/dev/null)"; then
  exe="$prefix/lib/leroy_xrslam/leroy_xrslam_node"
  if [ -x "$exe" ] && ! ldd "$exe" | grep -q "not found"; then
    echo "STAGE3_XRSLAM_LINKAGE=RESOLVED"
  else
    echo "STAGE3_XRSLAM_LINKAGE=UNVERIFIED"
  fi
fi
echo "STAGE3_PROBE_HARDWARE_ACCESS=NONE"
' || fail "IMAGE_PROBE_$role"
}
probe_image SENSOR_V3 "$BASE_IMAGE"
probe_image XRSLAM "$XRSLAM_IMAGE"
probe_image NAV2 "$NAV2_IMAGE"
printf '%s\n' \
  'M23BV_V3_STAGE3_INVENTORY=PASS_NO_DEVICE_PACKAGE_INVENTORY' \
  'M23BV_V3_STAGE3_FULLSTACK_RUN=NOT_EXECUTED' \
  'M23BV_V3_STAGE3_MCU_SERIAL_ACCESS=NONE' \
  'M23BV_V3_STAGE3_RECORDING_AND_MOTION=NOT_AUTHORIZED'
