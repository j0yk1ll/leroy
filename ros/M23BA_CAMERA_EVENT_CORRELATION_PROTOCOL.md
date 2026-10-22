# M23BA — publisher-to-subscriber occurrence correlation (offline only)

**Status:** DRAFT source-only evidence protocol. No diagnostic camera run,
no V4L2 device, no ROS node, no robot serial/IMU, no DDS discovery, no
recording, no power restoration, no physical-test authorization.

This is stacked on [M23AZ](https://github.com/j0yk1ll/leroy/pull/142).
M23AZ's corrected device-free component `dlopen` gate PASSED in
immutable M23AY diagnostic image
`9ff2faf53109f88333e79c998b6946caa49f09fc807d2da422c1adcd47bda571`.
Host log `dlopen-02.log` SHA256
`b2fdcb0b9e47910e3bb00291f95fc911261f45865b39bc584b1253c34cab32fa`.
The original M23AU physical trial remains BLOCKED by exactly one
repeated observed raw subscriber header stamp; neither a publisher
duplicate nor a DDS error was proven.

## Grounded source formats

**Camera pre-send log:** M23AX emits `M23AX_PUBLISH` to stderr
from the diagnostic `usb_cam_node_exe` immediately **before** calling
`image_transport::CameraPublisher::publish`. Fields are
`pub_index`, `fresh`, `status`, `dequeue_count`,
`v4l2_seq`, `v4l2_timestamp_us`, `ros_stamp_ns`.

`fresh=0` and `select_eintr`, `mmap_g_fmt_eagain` or
`mmap_dqbuf_eagain` mean no successfully completed new MMAP buffer
in that `get_image` invocation. Such a pre-send entry does NOT
prove the DDS middleware actually sent a message. Likewise a fresh
MMAP frame can theoretically have a repeated ROS header stamp.

**Subscriber log:** The original M23K
`m23k_camera_header_counter.cpp` writes
`M23K_SAMPLE raw|rect|info STAMP_NS RECEIPT_STEADY_NS`.
It emits `M23K_COUNTER_VARIANT=FULL`, and the M23O reliable_100
receiver profile writes
`M23M_COUNTER_QOS_PROFILE=reliable_100`.
A clean termination emits `M23K_COUNTER_COUNTS=RAW,RECT,INFO`.
`receipt_ns` uses `steady_clock` and is comparable within
the SAME observer process, not directly to V4L2 timeval or the ROS
header time basis. Kernel V4L2 timestamp and ROS stamp have different
time bases/offsets: do not numerically compare them without separate
documented calibration.

The existing M23K supervisor writes complete per-event logs inside
the disposable container as:

- `/tmp/m23k-camera-launch.log` (includes usb_cam's pre-send lines);
- `/tmp/m23k-cpp-counter.log` (subscriber sample lines).

**Critical:** The old M23AU host log preserved only summary JSON.
A future run MUST write both full per-event files to an operator-owned
0700 *persistent host evidence directory* before the disposable
container exits. The preferred future code change is to move the
supervisor's two exclusive-create (`xb`) output files to two new,
separate files in a narrowly bound `/evidence` host directory:
the camera log and counter log are **written as events arrive**,
not copied after ephemeral `--rm` cleanup. Do not bind a home tree,
the full host /dev, a production volume, or any network resource.
Protect text evidence from overwrite; preserve hashes, complete
process exit status, launch child clean-exit markers, graph/QoS and
observer summary. Never stream pixels, rosbags or frame payloads.
Cap log sizes and terminate/fail closed if a bound is reached.

**The host-persistent capture adapter and any camera launch command
are NOT implemented or authorized in M23BA.** Their code, physical
isolation, additional process/loader mapping checks, preservation
on failures, and one-shot timing procedure require distinct review
and explicit operator permission before ever mapping a camera.

## What the offline validator does now

`ros/m23ba_event_correlation.py` is standard-library-only, inert
by default, and accepts exactly two **preexisting** regular,
non-symlinked text files (max 4,000,000 bytes each). It:
1. Refuses fatal/crash/metadata-cap markers and malformed pre-send
   events. Requires contiguous publication indexes beginning at 1,
   valid acquisition status, dequeue counts increasing only for
   successfully processed/requeued frames, and a retained stamp
   for nonfresh attempts.
2. Requires FULL reliable_100 observer selection, one finished
   `M23K_COUNTER_COUNTS` summary exactly matching **every**
   raw/rect/info `M23K_SAMPLE` occurrence, bounded per-stream
   samples, and strictly increasing steady-clock *receipt order*.
   It reports header nonmonotonicity rather than erasing it.
3. Counts **occurrences**, not merely unique header-stamp sets.
   A repeated pre-send stamp and a repeated raw subscriber stamp
   can be compared by stamp, but duplicate occurrences with the
   same stamp cannot be uniquely associated with publisher indices.
4. Reports subscriber-only repeat, pre-send-only repeat, repeat in
   both, or no repeat as **patterns**, never a conclusive
   root-cause/DDS verdict. Shows raw occurrences exceeding
   pre-send attempts, but a record may be absent simply because
   acquisition and subscriber observation windows differ.

Important remaining coverage gaps: M23AX writes a *pre-send*
entry, not a positive DDS publish acknowledgment, and does not
emit a final authoritative total count/termination marker.
The two independent log streams may have different startup/shutdown
capture windows, and ROS2 Image headers contain no publication-index
identifier. Therefore **a clean source-only correlation result is
diagnostic, not proof that the publisher caused a particular raw
callback or that DDS redelivered it.** A later source revision can
add a bounded final camera publisher summary and explicit matched
observation epochs, without altering acceptance thresholds.

Source-only correlation MUST NOT reinterpret an observed `fresh=0`
as proof that the original M23AU incident took that branch, because
the original trial did not include this instrumentation.

## Operator-safe source gate now

This stage has **no physical test**. Use an isolated sparse checkout
of the exact PR commit, run synthetic Python tests only, then check
the inert default marker. No Podman needed.

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23ba-source"
  test "$(id -u)" -ne 0
  test ! -e "$DIR"

  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23ba-camera-event-correlation-offline \
    https://github.com/j0yk1ll/leroy.git "$DIR"

  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23ba_event_correlation.py' \
    '/ros/m23ba_event_correlation_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23ba_event_correlation_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23ba_event_correlation.py
)
~~~

A future invocation with `--publisher EXISTING_FILE --observer
EXISTING_FILE` only analyzes the exact two separately captured,
private text files. **Do not point it at M23AU's summary-only host
log** or assume the already-destroyed guest `/tmp` samples
can be recovered. Do not start a camera to generate input for this
offline gate.

## Acceptance boundaries

- M23BA source tests PASS = validator's **synthetic contracts**
  PASS only, not an actual publisher/observer attribution.
- Validation result `OFFLINE_EVENT_CORRELATION_DIAGNOSTIC_ONLY`
  cannot make production acceptance PASS and does not prove DDS
  redelivery, frame send success, or a specific real-camera cause.
- All original M23AU monotonic raw headers, >250ms raw/rect continuity,
  reliable_100 QoS graph, image calibration, clean child termination
  and physical isolation conditions remain mandatory and unchanged.
- Existing M23AY/M23AZ image and pinned M23AJ source/parent unchanged.
- No further physical camera exposure, serial bridge, MCU, IMU,
  recording, XRSLAM/DepthART, Nav2, wheel or manipulator action
  is authorized by this draft protocol.

M23BA_CAMERA_RETEST_AUTHORIZED=FALSE
M23BA_PRODUCTION_ACCEPTANCE=BLOCKED
