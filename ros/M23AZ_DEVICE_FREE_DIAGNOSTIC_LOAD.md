# M23AZ — device-free dynamic load of the M23AY diagnostic component

## Current evidence and limits

M23AY first and second offline builds were intentionally rejected by
colcon CLI and overstrict executable loader checks. The third offline
build finally passed, with diagnostic image ID:

`9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571`

Build-03 SHA256 `962be1ef77d0b45d61cf124fd6b6bfba9aa887b4127ed66411d39cd54ab08536`.
The image was compiled from the pinned M23AJ pthreads OpenCV parent
with M23AX acquisition/publication metadata instrumentation and a
strict `ldd -r` library closure. The component DSO resolves the new
`/opt/m23ay-install/lib/libusb_cam.so`; the Jazzy standalone executable
uses class_loader at runtime and is not required to have a direct
core DT_NEEDED.

**The instrumented component has never been loaded into a process
outside linker audit yet.** M23AZ tests that gap, but with no ROS
node instance, no camera and no V4L2 devices.

## Scope

`m23az_component_dlopen_probe.py` is INERT by default.
`--run` is allowed only with an exact, device-free approval,
isolated ROS env, empty LD_PRELOAD, camera/OpenCV loader precedence,
existing diagnostic ELF files, and no V4L2 or MCU serial device
nodes or USB bus exposed. `ctypes.CDLL(..., RTLD_NOW|RTLD_LOCAL)`
loads the *component shared library* only. It does not instantiate
`UsbCamNode`, initialize rclcpp/RMW, create an executor, publish,
subscribe, open cameras, record pixels, issue V4L2 ioctl, or load a
robot bridge.

The script checks live process `/proc/self/maps` after dynamic load
for exact isolated diagnostic `libusb_cam_node.so` + `libusb_cam.so`
and pthreads OpenCV core, refusing any mapped oneTBB, legacy camera
core/component library, or mixed system OpenCV. It reports library
maps as bounded text, not pixels.

This is **dlopen/process-mapping evidence only**. It cannot prove
safe repeated dlclose/unload, real frame timing, DDS behavior,
physical camera setup, the original duplicate's root cause, or
production acceptance. An automatic camera trial is forbidden.

## Host test procedure

Source branch `feature/m23az-device-free-component-load-smoke`
is stacked on M23AY and does not alter the pinned diagnostic image.
Use exact Git revision and image ID specified by the accompanying
instructions.

### A. No-device host unit tests

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23az-source"
  test "$(id -u)" -ne 0
  test ! -e "$DIR"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23az-device-free-component-load-smoke \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23az_component_dlopen_probe.py' \
    '/ros/m23az_source_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23az_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23az_component_dlopen_probe.py
)
~~~

Require **19 source unit tests** and inert default:
`M23AZ_STATUS=INERT_NO_ROS_NO_DEVICES`.

### B. One isolated dynamic-loader observation, no ROS or camera

Only if A passed. This container receives **no --device, no --privileged,
no host /dev, no network, no camera, no robot-volume mappings**, and a
single read-only instrumented probe source. New immutable host log.

~~~bash
(
  set -euo pipefail
  set -C
  umask 077
  test "$(id -u)" -ne 0
  SOURCE="$HOME/.local/state/leroy-m23az-source"
  LOGDIR="$HOME/.local/state/leroy-m23az"
  LOG="$LOGDIR/dlopen-01.log"
  TAG="localhost/leroy-sensors:m23ay-acquisition-diagnostic-v1"
  ID="9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571"
  test -z "$(podman ps -q)"
  test -z "$(git -C "$SOURCE" status --porcelain)"
  test ! -e "$LOG"
  test "$(podman image inspect "$TAG" --format '{{.Id}}')" = "$ID"
  mkdir -p "$LOGDIR"
  if podman run --rm --pull=never --network=none \
    --ipc=private --pid=private \
    --cap-drop=all --security-opt=no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=16m,notmpcopyup \
    --memory=1g --pids-limit=64 \
    --env M23AZ_OFFLINE_DLOPEN_APPROVED=YES_NO_DEVICES \
    --env LEROY_M20_SENSOR_ONLY=1 \
    --env LEROY_NAV2_DRY_RUN=1 \
    --env ROS_DOMAIN_ID=170 \
    --env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
    --env LD_PRELOAD= \
    --env LD_LIBRARY_PATH=/opt/m23ay-install/lib:/opt/m23aj-opencv/lib:/opt/ros/jazzy/lib \
    --volume "$SOURCE/ros/m23az_component_dlopen_probe.py:/opt/m23az-dlopen.py:ro" \
    --entrypoint /usr/bin/python3 \
    "$TAG" -B /opt/m23az-dlopen.py --run >"$LOG" 2>&1
  then RC=0; else RC=$?; fi
  echo "M23AZ_DLOPEN_EXIT=$RC"
  sed -n '1,40p' "$LOG"
  sha256sum "$LOG"
  test "$RC" -eq 0
  grep -Fxq 'M23AZ_VERDICT=OFFLINE_DIAGNOSTIC_COMPONENT_DLOPEN_PASS' "$LOG"
)
~~~

The container's default image command `/bin/false` is bypassed only
by a Python script with explicit gate. It never runs usb_cam_node_exe,
ROS components, ros2 launch, MCU bridge, or the robot stack.
The guest has no camera device mapping. If it crashes, exits nonzero,
detects unexpected library mappings or reports a blocker, stop and
preserve the exact log. No automatic retries.

The notmpcopyup option is essential: the large M23AJ OpenCV source
and build trees reside under the image's /tmp, and copying them into
a 16 MiB tmpfs previously caused OCI ENOSPC.

## Acceptance boundary

M23AZ PASS means only: *this exact immutable M23AY diagnostic
component can load once in a no-device process and its observed
camera/OpenCV/TBB maps conform to the isolated loader contract*.
It does NOT validate unloading, camera frames, capture rates,
rectified path, root cause attribution or runtime shutdown.
M23AU's raw-header timestamp fault and production acceptance remain
BLOCKED. Future physical camera-only runs need a separate, explicit
operator authorization after review of a host-preserved text-only
publisher/subscriber evidence protocol and hardware isolation.

M23AZ_CAMERA_RETEST_AUTHORIZED=FALSE
M23AZ_PRODUCTION_ACCEPTANCE=BLOCKED
