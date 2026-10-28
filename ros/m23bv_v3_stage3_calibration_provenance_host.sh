#!/usr/bin/env bash
# Stage 3 hardware-FREE camera timestamp/extrinsic provenance inspection.
# Reads pinned source + inspects installed usb_cam package/binary in the
# pinned immutable sensor image. No camera, MCU, ROS nodes, bridge or logs.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_CAL_PROVENANCE=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_CHECKOUT
for file in m23bv_v3_stage3_calibration_provenance.py; do
  [[ -f "$file" && ! -L "$file" ]] || fail SOURCE_MISSING
  [[ "$(git hash-object "$file")" == "$(git rev-parse "HEAD:ros/$file")" ]] ||
    fail SOURCE_CHANGED
done
python3 -B m23bv_v3_stage3_calibration_provenance.py || fail SOURCE_AUDIT
[[ "$(id -u)" != 0 ]] || fail NO_ROOT_ALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail REQUIRE_ROOTLESS_PODMAN
image=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
expected=f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
actual="$(podman image inspect "$image" --format '{{.Id}}')" || fail IMAGE_ABSENT
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$expected" ]] || fail IMAGE_ID_CHANGED
echo "STAGE3_CAL_PINNED_SENSOR_IMAGE_ID=$expected"
podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=32m \
  --entrypoint /bin/bash "$image" -c '
set -eo pipefail
test ! -e /dev/video2 && test ! -e /dev/ttyACM0
package_details="$(dpkg-query -W ros-jazzy-usb-cam)" || exit 2
read -r package version <<<"$package_details"
test "$package" = ros-jazzy-usb-cam && test -n "$version" || exit 2
driver=/opt/ros/jazzy/lib/usb_cam/usb_cam_node_exe
test -x "$driver" || exit 2
digest="$(sha256sum "$driver" | cut -d" " -f1)"
printf "STAGE3_CAL_USB_CAM_INSTALLED_PACKAGE=%s\n" "$package"
printf "STAGE3_CAL_USB_CAM_INSTALLED_VERSION=%s\n" "$version"
printf "STAGE3_CAL_USB_CAM_NODE_BINARY_SHA256=%s\n" "$digest"
echo STAGE3_CAL_SENSOR_IMAGE_DEVICES=NONE
' || fail PINNED_SENSOR_IMAGE_READ_ONLY_INVENTORY
echo STAGE3_CAL_PROVENANCE=PASS_PINNED_SOURCE_AND_INSTALLED_DRIVER_INVENTORY
echo STAGE3_CAL_CAMERA_EXPOSURE_PHASE=NOT_MEASURED
echo STAGE3_CAL_EXTRINSICS=NOT_MEASURED
echo STAGE3_CAL_MOTION_RECORDING_MCU_ACCESS=NOT_AUTHORIZED
