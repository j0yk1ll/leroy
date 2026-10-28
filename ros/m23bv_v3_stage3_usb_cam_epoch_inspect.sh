#!/usr/bin/env bash
# Stage 3 upstream usb_cam 0.8.1 timestamp-conversion regression inspection.
# No physical camera, MCU, ROS node, motor or recording access.
set -Eeuo pipefail
cd "$(dirname -- "$0")"
fail() { echo "STAGE3_USB_CAM_EPOCH_INSPECT=BLOCKED_$1" >&2; exit 2; }
[[ "$(git branch --show-current)" == feature/m23bv-v3-stage3-fullstack-diagnostic ]] ||
  fail WRONG_BRANCH
[[ -z "$(git status --porcelain)" ]] || fail DIRTY_SOURCE
python3 -B - <<'PY'
# Vectors for upstream usb_cam tag 0.8.1 get_epoch_time_shift_us():
# erroneous tv_usec/1000.0 instead of tv_usec (microseconds already).
# This is NOT an assertion about the installed compiled binary.
for usec in (0, 250000, 500000, 999999):
    bug_us = int(usec / 1000.0)
    error_us = bug_us - usec
    assert -1000000 < error_us <= 0
    print("STAGE3_USB_CAM_EPOCH_RELEASE_VECTOR_" + str(usec) +
          "_BIAS_US=" + str(error_us))
assert int(500000 / 1000.0) - 500000 == -499500
print("STAGE3_USB_CAM_EPOCH_SOURCE_REGRESSION=PASS_UPSTREAM_0_8_1")
PY
[[ "$(id -u)" != 0 ]] || fail ROOT_DISALLOWED
[[ "$(podman info --format '{{.Host.Security.Rootless}}')" == true ]] ||
  fail NOT_ROOTLESS
image=localhost/leroy-sensors:m23bv-reliable-rectifier-v3
expected=f43950b27ab86763acf6402afa1bd454275a9758459f6e7be373e35a9dd4089c
actual="$(podman image inspect "$image" --format '{{.Id}}')" ||
  fail PINNED_IMAGE_MISSING
actual="$(printf '%s' "$actual" | sed 's/^sha256://')"
[[ "$actual" == "$expected" ]] || fail PINNED_IMAGE_DRIFT
echo "STAGE3_USB_CAM_EPOCH_PINNED_SENSOR_IMAGE=$actual"

podman run --rm --pull=never --network none --ipc private --pid private \
  --read-only --cap-drop all --security-opt no-new-privileges \
  --tmpfs /tmp:rw,nosuid,nodev,size=32m \
  --entrypoint /bin/bash "$image" -c '
set -eo pipefail
test ! -e /dev/video2 && test ! -e /dev/ttyACM0 || exit 2
version="$(dpkg-query -W ros-jazzy-usb-cam | awk "{print \$2}")"
test "$version" = 0.8.1-1noble.20260912.190806 || exit 2
exe=/opt/ros/jazzy/lib/usb_cam/usb_cam_node_exe
test -x "$exe" || exit 2
actual="$(sha256sum "$exe" | cut -d" " -f1)"
test "$actual" = b760f640ecb19fdded3c4028c2f4cbd44b64ed93912246eaaad86155c5d96249 ||
  exit 2
echo "STAGE3_USB_CAM_EPOCH_INSTALLED_PACKAGE_VERSION=$version"
echo "STAGE3_USB_CAM_EPOCH_EXE_SHA256=$actual"
python3 -B - <<PY
from pathlib import Path
import hashlib, re
paths = sorted(set(
    p for p in Path("/opt/ros/jazzy/include").rglob("utils.hpp")
    if p.is_file() and "usb_cam" in p.parts))
if not paths:
    print("STAGE3_USB_CAM_EPOCH_INSTALLED_HEADER=NOT_AVAILABLE")
for p in paths:
    source = p.read_text(errors="replace")
    if re.search(r"epoch_time[.]tv_usec\s*/\s*1000(?:[.]0)?", source):
        result = "LEGACY_DIVIDE_1000_PRESENT"
    elif re.search(r"epoch_time[.]tv_usec\s*;", source):
        result = "CORRECT_MICROSECONDS_FORMULA"
    else:
        result = "UNRECOGNIZED"
    print("STAGE3_USB_CAM_EPOCH_INSTALLED_HEADER=" + result)
    print("STAGE3_USB_CAM_EPOCH_HEADER_SHA256=" +
          hashlib.sha256(p.read_bytes()).hexdigest())
PY
libs="$(dpkg -L ros-jazzy-usb-cam | grep -E "[.]so([.]|$)" || true)"
if test -z "$libs"; then
  echo STAGE3_USB_CAM_EPOCH_LIBRARY_IN_PACKAGE=NOT_LISTED
else
  while IFS= read -r so; do
    if test -f "$so"; then
      printf "STAGE3_USB_CAM_EPOCH_LIBRARY_SHA256=%s\n" \
        "$(sha256sum "$so" | cut -d" " -f1)"
    fi
  done <<<"$libs"
fi
echo STAGE3_USB_CAM_EPOCH_CONTAINER_DEVICES=NONE
' || fail INSTALLED_PACKAGE_AUDIT

echo STAGE3_USB_CAM_EPOCH_INSPECT=PASS_SOURCE_AND_INSTALLED_ARTIFACT_INVENTORY
echo STAGE3_USB_CAM_EPOCH_COMPILED_IMPLEMENTATION=NOT_INDEPENDENTLY_VERIFIED
echo STAGE3_USB_CAM_EXPOSURE_PHASE=NOT_MEASURED
echo STAGE3_USB_CAM_VIO_CALIBRATION=BLOCKED_TIMESTAMP_PROVENANCE
echo STAGE3_USB_CAM_MOTION_MCU_RECORDING=NOT_AUTHORIZED
