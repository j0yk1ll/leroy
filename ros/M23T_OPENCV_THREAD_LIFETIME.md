# M23T — narrow OpenCV thread-lifetime differential (no devices)

**Draft PR stacked on M23S #113. NOT operator-tested, built or run.**
This is a diagnostic experiment, **not** a fix for the image_proc
shutdown SIGSEGV or authorization to change production.

## Evidence and exact target

M23S's accepted **one pair** of no-device synthetic trials on image
`01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2`
showed:

| Inherited synthetic profile | Rectified Images | Matched stamps | Max header gap | Child shutdown |
|---|---:|---:|---:|---|
| K>0, zero D | 900 | 900 | 33.562ms | Clean under trace |
| K>0, nonzero D | 900 | 900 | 33.521ms | SIGSEGV (-11) |

Under ROS Jazzy `image_proc/src/rectify.cpp` the nonzero D arm calls
`model_.fromCameraInfo(info_msg)`,
`cv_bridge::toCvShare(image_msg)`,
`model_.rectifyImage(image, rect, interpolation_)`,
`CvImage(...).toImageMsg()`, and publishes a *new* rectified
message. The zero-D arm publishes the original message.
Source: https://github.com/ros-perception/image_pipeline/blob/jazzy/image_proc/src/rectify.cpp

The source header `image_proc/rectify.hpp` owns an
`image_geometry::PinholeCameraModel model_` and image_transport
subscription/publisher members. Their precise runtime destruction
sequence, worker thread lifetime, TBB backend selection and faulting
instruction have **not** been established. M23R's last successful
`libtbbmalloc.so.2` close **does not** identify the faulting library.

M23T does **not** fork or rebuild `RectifyNode`. Instead it asks the
**same OpenCV library used by the original component** to run
with the default thread policy or an explicit single-thread
request, using a tiny **component-child-only** preload helper.
The question is whether this OpenCV thread API setting changes
the reproducibility of the nonzero-D shutdown SIGSEGV under otherwise
fixed conditions. It is a stronger worker/allocator-lifetime probe
than another broad `dlclose` trace.

## Exactly what differs and what does not

The pre-existing M23S single **nonzero-D** C++ publisher, its exact
`CameraInfo` D vector, 900 640x480 RGB8 images, K/R/P, graph-ready
publisher-first order, RELIABLE/VOLATILE depth100 endpoints, actual
graph audit, C++ raw/rect/CameraInfo observer, timing/continuity
thresholds, 2s drain, same ROS2 Jazzy
`image_proc::RectifyNode` factory and same `component_container`,
SIGINT teardown, and M23R scoped loader interposer are all preserved.

The two M23T arms use the **same newly built M23T image**, with
`M23S_DISTORTION_PROFILE=rectify` held fixed. Only
`M23T_OPENCV_THREAD_PROFILE` changes:

| Profile | Child OpenCV constructor action | Required observation |
|---|---|---|
| `default` | `cv::getNumThreads()` before/after, **no setter** | same reported API value before and after |
| `single` | `cv::getNumThreads()`, then `cv::setNumThreads(1)`, then `cv::getNumThreads()` | reported API value after equals 1 |

The helper is loaded only through a `SetEnvironmentVariable`
on the **component launch's child process**. M23T's runner forbids
global `LD_PRELOAD`, requires its own opt-in diagnostic guard,
and calls M23S/M23R/M23P **unchanged** rather than starting its own
ROS processes. This helper is built against the installed OpenCV
headers and linked to the local `libopencv_core` with no network.

**Important interpretive limitations:**
- `cv::getNumThreads()==1` proves an API-reported setting, **not**
  that exactly one kernel worker exists, that TBB was used, or that
  all image_geometry work is single-threaded.
- Preloading OpenCV before `image_proc` changes the module initialization
  environment relative to original M23S. Both **new** M23T arms
  share that change, but M23T default must independently
  reproduce the crash before `single` offers a meaningful comparison.
- A single clean nonzero-D run with `single` does **not** prove a
  threading root cause or justify a production configuration change.
- The separate helper itself may alter destructor/load order and
  shutdown race scheduling. Keep the exact same pinned M23T image,
  then repeat only under future, separately approved scope if warranted.

## Files and invariants

All additions only in `ros/`:
- `m23t_opencv_thread_probe.cpp` — gated OpenCV API setup in the
  exact `component_container` child, with pid/profile/before/after
  markers; no ROS, serial, camera, subprocess or writes to disk.
- `m23t_rectifier_thread_trace.launch.py` — same M23R original
  `image_proc::RectifyNode` component and event markers, with
  the additional **component-only** preload library.
- `m23t_thread_probe_runner.py` — uses M23S `run()` with just the
  M23R traced launch variable temporarily replaced, strictly
  validates the marker's PID and API values.
- `m23t_compare_thread_logs.py` — offline strict paired-log
  classifier, requires successful 900-image publication, at least
  810 raw/rect/ImageInfo receipts, >=90% matching rect stamps,
  zero unmatched rect stamps, rect gap <=250ms, actual QoS graph,
  verified M23R tracer and M23S producer calibration, genuine
  clean or child `-11` result, and the exact OpenCV setup in both.
- `m23t_thread_probe_test.py` — offline negative tests.
- `Dockerfile.m23t-opencv-thread` — inherits the verified M23S
  local diagnostic image, no installs or network downloads, builds
  only the tiny OpenCV API helper and runs source contracts.
- This runbook.

No source under existing M23S, M23R, M23P, physical driver,
production bridge/robot, firmware or M23A is modified.
The M23S and historical core/log data stay private.

## Gate A — offline source tests (NO ROS/containers)

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23t-opencv-thread-lifetime
git pull --ff-only origin feature/m23t-opencv-thread-lifetime
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23t_thread_probe_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23s_rectification_ab_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23r_shutdown_trace_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23p_no_device_rectifier_shutdown_test.py
```

Every test suite must pass; the new one prints
`M23T_NO_DEVICE_THREAD_SOURCE_CONTRACT=PASS` and
`M23T_DEVICE_ACCESS=NONE`. Retain prior M23Q offline regressions.
No source-check failure may be waived.

## Gate B — local, disconnected compilation ONLY

Require an existing **immutable validated M23S base image**, not a
rebuilt/retagged one. Operator-previously-reported M23S image ID:

```text
01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2
```

**Do not run ROS in this gate.** From `ros/`:

```bash
umask 077
DIR="$HOME/.local/state/leroy-m23t"
mkdir -p "$DIR" && chmod 700 "$DIR"

BASE=localhost/leroy-sensors:m23s-rectification-ab
BASE_EXPECTED=01a0334afd19fea39cc4b8a1dc457e65452d6b0d73d1b959ce654904e29e78c2
podman image exists "$BASE" || exit 2
BASE_BEFORE="$(podman image inspect "$BASE" --format '{{.Id}}')" || exit 2
test "${BASE_BEFORE#sha256:}" = "$BASE_EXPECTED" || {
  echo M23T_BLOCKED=M23S_IMAGE_ID_CHANGED; exit 2;
}
echo "M23T_M23S_PARENT_IMAGE_ID=$BASE_BEFORE"

LOG="$DIR/build-01.log"
test ! -e "$LOG" || { echo M23T_BLOCKED=EXISTING_BUILD_LOG; exit 2; }
BUILD_RC=0
podman build --network none --pull=never \
  -f Dockerfile.m23t-opencv-thread \
  -t localhost/leroy-sensors:m23t-opencv-thread . \
  >"$LOG" 2>&1 || BUILD_RC=$?

echo "M23T_BUILD_EXIT_CODE=$BUILD_RC"
grep -E 'M23T_NO_DEVICE_THREAD_IMAGE_CONTRACT=|error:|Error:|FAILED' \
  "$LOG" | tail -20 || true

BASE_AFTER="$(podman image inspect "$BASE" --format '{{.Id}}')" || exit 2
test "$BASE_AFTER" = "$BASE_BEFORE" || {
  echo M23T_BLOCKED=BASE_IMAGE_ID_CHANGED; exit 2;
}
test "$BUILD_RC" -eq 0 || exit 2
echo M23T_PARENT_UNCHANGED=PASS
```

Require build0, `M23T_NO_DEVICE_THREAD_IMAGE_CONTRACT=PASS`,
and unchanged parent. No `apt-get`, compiler download,
source package replacement or network pull allowed.

## Gate C — separately approved no-device ROS profiles ONLY

**Neither opening this PR nor passing Gates A/B authorizes executing
ROS.** Obtain a separate human go/no-go for each disposable
synthetic-only experiment, retain prior runtime and physical-device
safety gates, and preserve protected private host logs.

Run the **default** profile first; it must reproduce a **confirmed
child SIGSEGV -11** with valid synthetic rectified image delivery
before interpreting the `single` arm. If default does not reproduce,
classify as **baseline altered by instrumentation** and stop.
Do not rebuild between arms. Do not map any hardware.

```bash
umask 077
DIR="$HOME/.local/state/leroy-m23t"
test -d "$DIR" || exit 2
IMAGE=localhost/leroy-sensors:m23t-opencv-thread
ID="$(podman image inspect "$IMAGE" --format '{{.Id}}')" || exit 2
echo "M23T_IMAGE_ID=$ID"
LOG="$DIR/default-01.log"
test ! -e "$LOG" || { echo M23T_BLOCKED=EXISTING_LOG; exit 2; }

RC=0
podman run --rm --pull=never \
  --network none --ipc private --pid private --shm-size 512m \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  -e M23S_DISTORTION_PROFILE=rectify \
  -e M23T_OPENCV_THREAD_PROFILE=default \
  "$ID" >"$LOG" 2>&1 || RC=$?

echo "M23T_DEFAULT_EXIT_CODE=$RC"
grep -E '^(M23S_AB_VERDICT=|M23R_LIFECYCLE_VERDICT=|M23T_THREADS_VERDICT=|M23T_THREADS_JSON=|M23T_DEVICE_ACCESS=|M23T_PRODUCTION_ACCEPTANCE=)' \
  "$LOG" || true
```

Only after reviewing the complete default evidence and separately
approving the second no-device experiment, use **the same exact image
ID and Podman restrictions**, replace
`M23T_OPENCV_THREAD_PROFILE=default` with `single`,
`"$DIR/default-01.log"` with `"$DIR/single-01.log"`,
and output `M23T_SINGLE_EXIT_CODE`. Pin the same image ID
prior to the second run. Both runs remain synthetic ROS only.
`RC=2` on a confirmed child SIGSEGV is a **BLOCKED component
lifecycle**, not a successful shutdown test.

## Gate D — local read-only log comparison

After both runs, **do not** send raw cores or full process memory
to GitHub:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -u m23t_compare_thread_logs.py \
  --default-log "$DIR/default-01.log" \
  --single-log "$DIR/single-01.log"
```

`M23T_PAIR_VERDICT` is diagnostic only:
- Default SIGSEGV + single clean: OpenCV thread-setting
  **association supported, NOT causal**.
- Both SIGSEGV: requesting one OpenCV thread did **not**
  prevent the observed shutdown failure.
- Both clean: original crash **not reproduced under the helper**.
- Reverse: inconclusive; investigate timing/race sensitivity.

Any truncated/missing marker, unverified child SIGSEGV,
missing rectified stream, incorrect D, changed image, or unverified
OpenCV API setup returns `M23T_PAIR_BLOCKED`.
No outcome makes `M23T_PRODUCTION_ACCEPTANCE=BLOCKED`
change to a release PASS. One pair is not sufficient to identify a faulting
library or declare a fix.

## Strict release/safety nonclaims

M23T is no-device diagnostic preparation; no production override,
camera sensor, STM32 CDC, robot actuation, recorder, IMU, VIO or
navigation. Parent M23A physical image header-gap `>250ms`
gate is unchanged and still BLOCKED. No physical motion permission.
Keep M23T and all stacked parents draft/unmerged. Retain raw
traces and historic core dumps on private host only.
