# M23AY — Disconnected instrumented usb_cam binary build

**Review-only, no device and no live camera authorization.** M23AX 15/15
source tests and pinned-image --check passed. This only permits an
offline build of the diagnostic C++ code. M23AU remains BLOCKED by a
single repeated raw-subscriber timestamp; publisher origin is unproven.

The Dockerfile inherits only the *pinned* M23AJ pthreads OpenCV image.
With no network and no device access it copies original usb_cam v0.8.1
source to /opt/m23ax-source/usb_cam, applies exactly the reviewed
three-file M23AX instrumentation **in that copy only**, and compiles
libusb_cam.so, libusb_cam_node.so and usb_cam_node_exe together against
the same changed header. The original /leroy_ws/src/usb_cam source
and /leroy_ws/install/usb_cam artifacts remain untouched. A binary
audit checks readelf architecture, ldd -r relocation closure, absence
of unresolved/tbb libraries, loading the new core from the new prefix,
and actual acquisition/publication diagnostic strings in those
binaries. ELF closure is not complete ABI certification. The new image
defaults to CMD ["/bin/false"] and cannot automatically start ROS.

## M23AY-02 corrective investigation — colcon argument parser

Operator's first offline build exited `2` **before C++ compilation**.
Source copy instrumentation printed `M23AX_SOURCE_OVERLAY_WRITTEN=ISOLATED_COPY_ONLY`,
but the installed `colcon build` CLI rejected the optional argument
`--allow-overriding usb_cam`:

```text
colcon build: error: argument --packages-select: unrecognized argument: --allow-overriding
M23AY_BUILD_EXIT=2
```

The first immutable host log is
`~/.local/state/leroy-m23ay/offline-build-01.log`,
SHA256 `622875cef115bc89b507ac640919b834318c17d4b63ea7cb0f96daed8c214436`.
Preserve it. The source copy was created **inside a failed disposable
build stage**, not on the host or pinned original image.

The revised Dockerfile removes `--allow-overriding` because it is not
part of this installed colcon CLI. It STILL compiles only the selected
`usb_cam` source package to a fresh `/opt/m23ay-install` prefix,
rebuilds both core and node against the same changed C++ header,
and requires `ldd -r` to actually resolve the core from this
new prefix rather than the original workspace. An additional
host unit test checks this command compatibility and ABI overlay
contract (13 source tests total instead of 12). No acceptance or
loader check is relaxed.

This second attempt is allowed only **offline, with no device mapping,
no network, no production process** and a fresh immutable
`offline-build-02.log`; no automatic retries if it fails again.
Operator must use the new exact Git commit and rerun source tests.

## Stage A — safe host-only source tests

Create a fresh sparse checkout, reusing neither /tmp nor production.
Pin the actual commit SHA supplied alongside the operator command.

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23ay-source"
  test "$(id -u)" -ne 0
  test ! -e "$DIR"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23ay-isolated-usb-cam-diagnostic-binary \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/Dockerfile.m23ay-usb-cam-diagnostic' \
    '/ros/m23ax_instrument_usb_cam_source.py' \
    '/ros/m23ay_binary_audit.py' \
    '/ros/m23ay_source_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ay_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ay_binary_audit.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23ax_instrument_usb_cam_source.py
)
~~~

Require **13/13** tests and inert status markers:
M23AY_STATUS=NO_ROS_OR_DEVICE_BY_DEFAULT and
M23AX_STATUS=INERT_NO_ROS_NO_DEVICE_BY_DEFAULT. The unit tests
are source-only; the corrected 13-test suite must be run anew before this attempt.

## Stage B — pinned-image, no-network no-device builder

Only AFTER Stage A passes. Use exactly one new image tag and immutable
private host log. No running Podman containers and no device mappings.

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23ay-source"
  LOGDIR="$HOME/.local/state/leroy-m23ay"
  LOG="$LOGDIR/offline-build-02.log"
  TAG="localhost/leroy-sensors:m23ay-acquisition-diagnostic-v1"
  PINNED="334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025"
  test "$(id -u)" -ne 0
  test -z "$(podman ps -q)"
  test -z "$(git -C "$DIR" status --porcelain)"
  test ! -e "$LOG"
  if podman image exists "$TAG"; then
    echo "M23AY_BLOCKED=DIAGNOSTIC_IMAGE_TAG_ALREADY_EXISTS"
    exit 2
  fi
  test "$(podman image inspect localhost/leroy-sensors:m23aj-pthreads-opencv-v2 \
    --format '{{.Id}}')" = "$PINNED"
  mkdir -p "$LOGDIR"
  cd "$DIR/ros"
  if podman build --network=none --pull=never --no-cache \
    -f Dockerfile.m23ay-usb-cam-diagnostic \
    -t "$TAG" . >"$LOG" 2>&1
  then RC=0; else RC=$?; fi
  echo "M23AY_BUILD_EXIT=$RC"
  tail -n 35 "$LOG"
  sha256sum "$LOG"
  test "$RC" -eq 0
  grep -Fxq 'M23AY_OFFLINE_INSTRUMENTED_USB_CAM_BUILD=PASS' "$LOG"
  podman image inspect "$TAG" \
    --format 'M23AY_DIAGNOSTIC_IMAGE_ID={{.Id}}'
)
~~~

No network access or device access is required. Source and toolchains
are already in the immutable parent. Build failures are blockers:
preserve both the first failed immutable log and any later fresh log; never automatically retry.
The build uses at most two CMake compilation workers.

For any subsequent **runtime** Podman self-audit, use notmpcopyup on
small /tmp mounts: the base image contains large cached OpenCV
build trees under /tmp, and default tmpcopyup previously caused
crun ENOSPC. Do not delete the base image or prune evidence.

## Boundaries

A passing disconnected build means the diagnostic source compiled
and passed selected ELF/loader checks only. It does NOT prove live
dequeue/publisher events, do any capture, collect original subscriber
samples, establish DDS-vs-publisher causality, or authorize another
real-camera trial. A future step must design a one-shot bounded
host-preserved text-only acquisition/publisher/subscriber evidence
protocol and review that separately before any new operator consent.

Never use ros/exec.sh or ros/robot.launch.py: they include serial
bridge startup and can energize actuators. No motor/manipulator
power restoration, MCU, IMU, recording, VIO, DepthART, Nav2 or
production usage authorized.

M23AY_CAMERA_RETEST_AUTHORIZED=FALSE
M23AY_PRODUCTION_ACCEPTANCE=BLOCKED
