#!/usr/bin/env bash
# Stage 3: verify EFFECTIVE usb_cam overlay selected by the accepted V3 camera
# launch environment. No physical camera or MCU mapping, no ROS nodes.
# Corrects the earlier underlay-only package inventory blind spot.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_USB_CAM_EFFECTIVE=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_WORKTREE
[[ "$(git rev-parse HEAD:ros/Dockerfile)" == "$(git hash-object Dockerfile)" ]] ||
  fail DOCKERFILE_WORKTREE_DRIFT
[[ "$(git rev-parse HEAD:ros/CAMERA_TIMESTAMP_FIX.md)" == "$(git hash-object CAMERA_TIMESTAMP_FIX.md)" ]] ||
  fail EXISTING_PATCH_DOCUMENT_DRIFT
[[ "$(id -u)" != 0 ]] || fail NO_ROOT
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail NOT_ROOTLESS
sensor=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
expected=f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
got="$(podman image inspect "$sensor" --format '{{.Id}}')" ||
  fail SENSOR_IMAGE_MISSING
got="$(printf '%s' "$got" | sed 's/^sha256://')"
[[ "$got" == "$expected" ]] || fail IMAGE_ID_DRIFT
echo "STAGE3_USB_CAM_EFFECTIVE_PINNED_SENSOR_IMAGE=$got"

# The SAME ROS overlay source-order as the accepted Stage 3 physical
# camera parent, but without starting any launch or exposing any devices.
podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=64m \
  --entrypoint /bin/bash "$sensor" -c '
set -eo pipefail
. /opt/ros/jazzy/setup.sh
. /leroy_ws/install/setup.sh
. /opt/m23bv_reliable_rectifier_proto/ws/install/setup.sh
test ! -e /dev/video2 && test ! -e /dev/ttyACM0 || exit 2

# Do not mistake the installed DEBIAN executable for the runtime ROS
# entry point. A packaged older version is intentionally still present.
underlay=/opt/ros/jazzy/lib/usb_cam/usb_cam_node_exe
test -x "$underlay" || exit 2
underlay_hash="$(sha256sum "$underlay" | cut -d" " -f1)"
test "$underlay_hash" = b760f640ecb19fdded3c4028c2f4cbd44b64ed93912246eaaad86155c5d96249 ||
  exit 2
echo "STAGE3_USB_CAM_UNDERLAY_EXE_SHA256=$underlay_hash"

effective_prefix="$(ros2 pkg prefix usb_cam)" || exit 2
printf "STAGE3_USB_CAM_EFFECTIVE_PREFIX=%s\n" "$effective_prefix"
test "$effective_prefix" = /leroy_ws/install/usb_cam || exit 2
exe="$effective_prefix/lib/usb_cam/usb_cam_node_exe"
test -x "$exe" || exit 2
echo "STAGE3_USB_CAM_EFFECTIVE_EXE_SHA256=$(sha256sum "$exe" | cut -d" " -f1)"
exe_real="$(readlink -f "$exe")" || exit 2
case "$exe_real" in
  /leroy_ws/build/usb_cam/*|/leroy_ws/install/usb_cam/*) ;;
  *) echo STAGE3_USB_CAM_EFFECTIVE=BLOCKED_EXECUTABLE_PATH >&2; exit 2 ;;
esac
echo STAGE3_USB_CAM_EXECUTABLE=OVERLAY_NOT_DEBIAN_UNDERLAY

# This source tree was cloned from a pinned upstream Git revision and
# patched by ros/Dockerfile; verify the exact edit and no other text
# substitutions are inferred from presence of an old Debian header.
src=/leroy_ws/src/usb_cam/include/usb_cam/utils.hpp
test -f "$src" || exit 2
rev="$(git -C /leroy_ws/src/usb_cam rev-parse HEAD)" || exit 2
test "$rev" = cb2ae6bc0a312de629f84fef933fb3e07dac1ef8 || exit 2
echo "STAGE3_USB_CAM_EFFECTIVE_UPSTREAM_SOURCE_REF=$rev"
python3 -B - <<PY
from pathlib import Path
p=Path("/leroy_ws/src/usb_cam/include/usb_cam/utils.hpp")
s=p.read_text()
assert "epoch_time.tv_usec / 1000.0" not in s
assert s.count("epoch_time.tv_sec * 1000000 + epoch_time.tv_usec;") == 1
print("STAGE3_USB_CAM_EFFECTIVE_SOURCE=CORRECT_MICROSECOND_EPOCH_FORMULA")
PY
echo "STAGE3_USB_CAM_EFFECTIVE_SOURCE_SHA256=$(sha256sum "$src" | cut -d" " -f1)"

# Verify what the ROS component loader links, not merely where the
# usb_cam_node_exe file is. Symlink-install may resolve into build dir.
node_lib="$effective_prefix/lib/libusb_cam_node.so"
test -f "$node_lib" || exit 2
node_lib_real="$(readlink -f "$node_lib")" || exit 2
case "$node_lib_real" in
  /leroy_ws/build/usb_cam/*|/leroy_ws/install/usb_cam/*) ;;
  *) echo STAGE3_USB_CAM_EFFECTIVE=BLOCKED_COMPONENT_PATH >&2; exit 2 ;;
esac
echo "STAGE3_USB_CAM_COMPONENT_SHA256=$(sha256sum "$node_lib" | cut -d" " -f1)"
links="$(ldd "$node_lib")" || exit 2
if printf "%s\n" "$links" | grep -q "not found"; then
  echo STAGE3_USB_CAM_EFFECTIVE=BLOCKED_LINKER_MISSING >&2; exit 2
fi
usb_line="$(printf "%s\n" "$links" | grep -E "libusb_cam[.]so[[:space:]]*=>")" ||
  exit 2
printf "STAGE3_USB_CAM_LINKED_LIBRARY=%s\n" "$usb_line"
libpath="$(printf "%s\n" "$usb_line" | awk "{print \$3}")"
case "$libpath" in
  /leroy_ws/build/usb_cam/*|/leroy_ws/install/usb_cam/lib/*) ;;
  *) echo STAGE3_USB_CAM_EFFECTIVE=BLOCKED_UNDERLAY_LIBRARY_SELECTED >&2; exit 2 ;;
esac
test -f "$libpath" || exit 2
echo "STAGE3_USB_CAM_EFFECTIVE_LIBRARY_SHA256=$(sha256sum "$libpath" | cut -d" " -f1)"
echo STAGE3_USB_CAM_EFFECTIVE_LINKAGE=OVERLAY_COMPONENT_AND_LIBRARY
echo STAGE3_USB_CAM_EFFECTIVE_CONTAINER_DEVICES=NONE
' || fail ROS_OVERLAY_OR_LINKAGE

echo STAGE3_USB_CAM_EFFECTIVE=PASS_PATCHED_OVERLAY_SELECTED_NO_DEVICE
echo STAGE3_USB_CAM_ABSOLUTE_EXPOSURE_TIME=NOT_MEASURED
echo STAGE3_USB_CAM_CAMERA_IMU_PHASE=NOT_CALIBRATED
echo STAGE3_USB_CAM_PHYSICAL_MOVEMENT_RECORDING_MCU=NOT_AUTHORIZED
