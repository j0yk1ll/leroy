# M23AW — pinned usb_cam 0.8.1 stale-header source-flow audit

## Why this gate exists

M23AU completed one 45s live camera-only pthreads test under
read-only Podman isolation and physical motor/manipulator rail
disconnection. 1404 raw, 1403 rect and 1403 info messages arrived
at the observer at ~29.8Hz, zero >250ms gaps, graph/QoS verified,
and actual usb_cam + stock component children exited cleanly.
The run **failed** its strict integrity gate on a single
`NONMONOTONIC_RAW` observation. M23AV offline log analysis (14
tests PASS) confirmed 1404 raw messages with only 1403 distinct
raw stamps, exactly matching all rect stamps. The exact raw
sample list was NOT preserved from the ephemeral original run.

M23AW is strictly an **offline control-flow hypothesis** for
the pinned usb_cam 0.8.1 source commit
`cb2ae6bc0a312de629f84fef933fb3e07dac1ef8`.
The Leroy base image at the immutable M23AJ image ID
`334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025`
contains this source under `/leroy_ws/src/usb_cam/`,
modified only for the previously accepted timeval usec -> ns
correction.

The source-level candidate path is:

1. `UsbCam::grab_image` uses `select()` before `read_frame`.
   On `EINTR`, it returns WITHOUT a new V4L2 dequeued frame.
2. `UsbCam::read_frame` under MMAP may encounter
   `VIDIOC_DQBUF` `EAGAIN` and similarly return without
   changing `m_image.stamp`.
3. `UsbCam::get_image(destination)` invokes `grab_image`
   but has `void` return: no explicit freshness status.
4. `UsbCamNode::take_and_send_image` invokes `get_image`,
   reads `get_image_timestamp()` and unconditionally
   publishes Image + CameraInfo (no check for a newly dequeued
   frame).

Thus a one-off interrupted/nonblocking acquisition **could**
republish a cached timestamp. However no such EINTR/EAGAIN event
was observed in the M23AU log; an alternative DDS/subscriber
duplication/reordering explanation is NOT eliminated. No kernel
USB/V4L2 driver defect or actual source of the one duplicate is
proven. The reported graph QoS reliability can affect DDS
delivery but does not by itself establish duplication.

## Safe source-only procedure

Run using the preserved persistent checkout of M23AV's new
child source branch, a fresh unique host audit log, and
the immutable M23AJ image. **No camera, serial, ROS launch,
Podman device mapping or recording.**

~~~bash
(
  set -euo pipefail
  set -C
  umask 077
  test "$(id -u)" -ne 0
  SOURCE="$HOME/.local/state/leroy-m23aw-source"
  LOGDIR="$HOME/.local/state/leroy-m23aw"
  LOG="$LOGDIR/source-control-flow-03.log"
  IMAGE_ID=334b9b35fe09156cfd0615be572eaecf96fdbebc3d75bf4fdd39a141d73d6025
  test ! -e "$SOURCE"
  mkdir -p "$LOGDIR"
  test ! -e "$LOG"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23aw-usb-cam-stale-frame-source-gate \
    https://github.com/j0yk1ll/leroy.git "$SOURCE"
  cd "$SOURCE"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23aw_stale_stamp_source_audit.py' \
    '/ros/m23aw_source_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23aw_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23aw_stale_stamp_source_audit.py
  test "$(podman image inspect \
    localhost/leroy-sensors:m23aj-pthreads-opencv-v2 \
    --format '{{.Id}}')" = "$IMAGE_ID"
  podman run --rm --pull=never \
    --network=none --ipc=private --pid=private \
    --cap-drop=all --security-opt=no-new-privileges \
    --read-only --tmpfs /tmp:rw,nosuid,nodev,size=16m,notmpcopyup \
    --memory=512m --pids-limit=64 \
    --volume "$SOURCE/ros/m23aw_stale_stamp_source_audit.py:/opt/m23aw-source-audit.py:ro" \
    --entrypoint /usr/bin/python3 \
    "$IMAGE_ID" -B /opt/m23aw-source-audit.py --check \
    >"$LOG" 2>&1
  grep -E '^(M23AW_RESULT_JSON=|M23AW_VERDICT=|M23AW_BLOCKED=|M23AW_CAMERA_RETEST_AUTHORIZED=)' "$LOG"
)
~~~

Expected host-side 11 source unit tests, inert default
`M23AW_STATUS=INERT_NO_DEVICE_SOURCE_ONLY`, and pinned
image source check `M23AW_VERDICT=STALE_STAMP_SOURCE_CONTROL_FLOW_PLAUSIBLE`
with all safety/source limitations explicitly `false`
for actual cause and publisher duplication proved.

### M23AW-03: OCI ENOSPC correction after host diagnosis

The first operator corrected host unit tests PASS 11/11 at commit
`988cffe280892b752ca18112af3b13492409d815`, but the pinned-image
source check did not start. An immutable 63-byte log
`source-control-flow-02.log` (SHA256
`468cdafb4ee71d7cb0c93d0277578052bec09413b4ddf5559bccc58564c19346`)
contains `Error: OCI runtime error: crun: write: No space left on device`.
Read-only host diagnostics show the Podman graphroot has 179 GB free
and ~58.5 million free inodes; the runroot, /tmp and /dev/shm have
ample free space and inodes.

**Likely specific cause:** The earlier command supplied
`--tmpfs /tmp:...size=16m`. Podman defaults to **tmpcopyup**, copying
the image's underlying /tmp into the new tmpfs at startup.
The pinned M23AJ Dockerfile creates `/tmp/m23aj-opencv-source` and
`/tmp/m23aj-opencv-build` and does not remove these large OpenCV trees.
This can overflow the isolated 16 MiB *container tmpfs* despite ample
host disk. `notmpcopyup` creates an empty tmpfs overlay, and the
source auditor does not need the image's /tmp contents.
This requires actual corrected no-device Podman inspection before PASS.
Podman docs: https://docs.podman.io/en/stable/markdown/podman-run.1.html

**Correction:** use `--tmpfs /tmp:rw,nosuid,nodev,size=16m,notmpcopyup`
and a **new immutable** `source-control-flow-03.log`. Do not delete
image/cache/evidence, don't `podman prune`, don't open the camera.
Check exact `M23AW_VERDICT=STALE_STAMP_SOURCE_CONTROL_FLOW_PLAUSIBLE`
and report Podman exit code independently. Do not overwrite -02.

The image inspection does not connect a V4L2 device. It only
reads the immutable on-image patched usb_cam C++ text. This
is NOT a run of patched behavior.

## Follow-up attribution design, NOT authorized yet

The next **separately approved** camera-only trial, if any,
must add bounded publisher-side observation of source V4L2
buffer `sequence` + kernel timestamp + acquisition result,
capture subscriber raw+rect exact stamp and monotonic receipts,
preserve those small bounded logs to a new host evidence directory
*before* removing the disposable container, and retain correct
graph/QoS and clean-shutdown checks. Instrumentation must be
reviewed and compiled separately; do not modify the original
immutable M23AJ image or source in place, or infer real
publisher behavior from subscriber samples alone. A source-only
audit does not justify replacing the existing strict
monotonicity gate with a warning.

No automatic camera retest. No serial/MCU, IMU, recording,
DepthART, XRSLAM/VIO, Nav2, motion or production.

M23AW_LIVE_ROOT_CAUSE_PROVEN=FALSE
M23AW_CAMERA_RETEST_AUTHORIZED=FALSE
M23AW_PRODUCTION_ACCEPTANCE=BLOCKED
