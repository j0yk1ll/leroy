# M23AX — Acquisition-to-publication attribution, SOURCE-ONLY

**Gate:** Draft, unmerged, **not compiled in the pinned image, not exercised on
a camera, and not authorized for a device-enabled run**. This PR is stacked
after M23AW; no production launch or runtime behavior is changed.

## Why

M23AU's one approved physical-camera-only trial showed 1404 raw observer
callbacks, 1403 rect callbacks, and 1403 CameraInfo callbacks near
29.8 Hz, no >250 ms header/receipt gaps, verified reliable ROS graph/QoS,
and clean shutdown of usb_cam + stock image_proc component. One raw
nonpositive header interval nevertheless failed the strict gate.
M23AV's host-log audit proved 1404 raw observations but only 1403
distinct raw timestamps; all rect stamps matched raw unique stamps.
**The raw duplicate is not proven to originate at usb_cam.**

M23AW's actual immutable image source audit established plausible
stale-frame return paths (select EINTR, MMAP G_FMT/DQBUF EAGAIN), but
did not establish that these occurred in the original run.

## Diagnostic design (future; source change is inert by default)

M23AX adds metadata-only instrumentation to a **copy** of the pinned
usb_cam 0.8.1 source, leaving the original image and its installed
workspace untouched. A future **separately reviewed** build would
compile both usb_cam core library and ROS usb_cam node against the
same changed header in a fresh, isolated overlay. Never mix original
and diagnostic usb_cam ABI.

Each raw ROS publication would emit one bounded, structured
\`M23AX_PUBLISH\` event **before** normal
\`image_transport::CameraPublisher::publish\` with:

- monotonic **per-process publication index**;
- \`fresh=1/0\` for whether MMAP dequeued, processed, and requeued
  a V4L2 frame during the most recent \`get_image()\`;
- status \`mmap_dequeued\`, \`mmap_g_fmt_eagain\`,
  \`mmap_dqbuf_eagain\`, or \`select_eintr\` where applicable;
- kernel buffer \`v4l2_seq\`, V4L2 timeval (in microseconds),
  total successful dequeues, and the published ROS timestamp (ns).

If acquisition fails after the previous success, the last kernel
sequence/timestamp remain visible and \`fresh=0\` marks the
non-fresh attempt; no frame data/pixels/rosbags are logged. No
pub/sub filtering, timestamp clamping, or acceptance threshold
changes. C++ source limits publisher metadata to 5000 lines and
prints \`M23AX_FATAL=METADATA_LOG_LIMIT\` if exceeded; any such
truncation MUST block diagnostic acceptance. This is designed
for MMAP only, the M23AQ camera path. Other capture modes are
not qualified by this instrumentation.

A separate preserved copy of **both** original M23K/M23O subscriber
\`M23K_SAMPLE raw|rect|info STAMP RECEIPT\` sequences is required
before the disposable guest exits. Publication-side and
subscriber-side text logs must be captured to new private host
evidence without transferring image bytes. Correlate V4L2 sequence,
publisher index/header stamp, raw/rect subscriber stamp and
receipt ordering, and graph/child shutdown evidence. When stamps
collide, preserve the individual **occurrences**, not just sets.
No correct attribution follows from a single subscriber set alone.

The instrumentation changes the UsbCam C++ class layout for metadata.
A later isolated compiled-image build must rebuild BOTH usb_cam
libraries + node with the same header; validate symbol/SONAME and
loader precedence. Do not rebuild the accepted M23AJ parent in place.
This PR does not contain a live launcher, host device runner, or
instructions to approve a second camera trial.

## Source-only operator check (now safe)

Use a new persistent, sparse Git checkout and the existing
**immutable** pinned M23AJ image. No ROS nodes, Podman devices,
serial, image recording, physical motor access or changes to
installed camera library.

~~~bash
(
  set -euo pipefail
  umask 077
  test "$(id -u)" -ne 0
  DIR="$HOME/.local/state/leroy-m23ax-source"
  LOGDIR="$HOME/.local/state/leroy-m23ax"
  LOG="$LOGDIR/source-check-01.log"
  IMAGE_ID=334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025
  test ! -e "$DIR"
  mkdir -p "$LOGDIR"
  test ! -e "$LOG"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23ax-usb-cam-acquisition-attribution-source \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23ax_instrument_usb_cam_source.py' \
    '/ros/m23ax_source_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23ax_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23ax_instrument_usb_cam_source.py
  test "$(podman image inspect \
    localhost/leroy-sensors:m23aj-pthreads-opencv-v2 \
    --format '{{.Id}}')" = "$IMAGE_ID"
  if podman run --rm --pull=never \
    --network=none --ipc=private --pid=private \
    --cap-drop=all --security-opt=no-new-privileges \
    --read-only \
    --tmpfs /tmp:rw,nosuid,nodev,size=16m,notmpcopyup \
    --memory=512m --pids-limit=64 \
    --volume "$DIR/ros/m23ax_instrument_usb_cam_source.py:/opt/m23ax-audit.py:ro" \
    --entrypoint /usr/bin/python3 \
    "$IMAGE_ID" -B /opt/m23ax-audit.py --check \
    >"$LOG" 2>&1
  then
    RC=0
  else
    RC=$?
  fi
  echo "M23AX_SOURCE_AUDIT_EXIT=$RC"
  sed -n '1,40p' "$LOG"
  sha256sum "$LOG"
  test "$RC" -eq 0
  grep -Fxq 'M23AX_VERDICT=INSTRUMENTATION_SOURCE_ANCHORS_MATCH' "$LOG"
)
~~~

Require 15/15 offline tests, an inert default marker
\`M23AX_STATUS=INERT_NO_ROS_NO_DEVICE_BY_DEFAULT\`, plus
\`M23AX_SOURCE_AUDIT_EXIT=0\` and
\`M23AX_VERDICT=INSTRUMENTATION_SOURCE_ANCHORS_MATCH\` from
the actual pinned source. If any test, source check, or container
startup fails, **STOP**, preserve the log, and correct only offline.

The \`--write\` mode is **not required for this gate**. It is
explicitly gated by \`M23AX_OFFLINE_BUILD_ONLY=YES\` and permits only
a fresh \`/opt/m23ax-source/usb_cam\` copy inside an appropriately
disconnected build container, not a source edit on the host or
to the pinned original. Treat compilation, live telemetry capture,
and future safety approvals as separate gates.

## Acceptance boundaries

- M23AX source gate PASS means **source contracts matched only**;
  no instrumented binary was built/executed and no live attribution
  was obtained.
- M23AU live timestamp-integrity gate remains **FAILED/BLOCKED**.
- M23AW source hypothesis remains **PLAUSIBLE**, not proven cause.
- A new physical camera run is **not** authorized by M23AX.
- No normal \`ros/exec.sh\` or \`ros/robot.launch.py\`;
  these may open MCU CDC serial and energize actuators.
- No IMU, recording, VIO/DepthART/XRSLAM, Nav2, motor/manipulator,
  production acceptance or robot movement.

M23AX_PHYSICAL_CAMERA_AUTHORIZED=FALSE
M23AX_PRODUCTION_ACCEPTANCE=BLOCKED
