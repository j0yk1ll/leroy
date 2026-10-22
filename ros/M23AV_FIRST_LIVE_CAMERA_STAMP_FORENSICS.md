# M23AV — NO-DEVICE audit of the first real camera-only M23AU log

## Operator evidence (one physical trial, not repeatable authorization)

One authorized 45-second Wed Camera / stock RectifyNode / pthreads
OpenCV trial ended with host exit `2`. Production and M23AU camera
acceptance remain **BLOCKED**.

The operator's M23AU log reports:

- Raw subscriber `1404` messages, rectified `1403`, CameraInfo
  `1403`. Actual receipt rates: raw `29.829Hz`, rect `29.827Hz`,
  CameraInfo `29.817Hz`.
- Reliable/volatile graph/QoS for expected `usb_cam`, `rectify_node`
  and C++ counter endpoints on `rmw_fastrtps_cpp` verified.
- All raw/rect/info **header** and callback **receipt** intervals
  have zero gaps over 250ms; rectified callback receipt maximum
  `59.439ms`; header maximum `36.14ms`.
- `shutdown_clean=true`; M23AU checked camera and standard
  component_container child process clean exit markers. No
  SIGSEGV was observed in this one live trial.
- Only reported fault `NONMONOTONIC_RAW`, with raw
  `header_intervals.nonpositive_gaps=1`. Rect and info have zero.
  The M23K Python observer rejects raw header stamps
  `<=` the previous raw stamp.
- Observed raw/rect **unique-header-stamp sets** match exactly:
  `paired=1403`, `raw_only=0`, `rect_only=0`.
  Therefore `len(set(raw))=1403`, with `1404` raw
  observations => **one repeated raw subscriber header-stamp
  observation** (one duplicate excess). The summary cannot
  distinguish a repeated publisher stamp from DDS redelivery,
  subscriber delivery order, or another source.
  It does NOT establish where or when the repeated stamp occurred.

The original M23K C++ observer emits per-message header and monotonic
receipt timestamps into `/tmp/m23k-cpp-counter.log` **inside**
the ephemeral `podman run --rm` container. The original host log
captures aggregate JSON, **not** that raw per-sample file.
Those exact sequence samples were not preserved. Do not
reconstruct or invent them.

## Safe audit (NO camera or ROS, original host evidence read-only)

This change contains an inert-by-default pure Python standard-library
parser with 14 offline tests. It validates the original saved host
output log's M23K and M23O duplicate report copies, blocked verdicts,
single timestamp anomaly, observation count/set cardinalities,
graph/QoS flags, 25Hz rate threshold, zero gaps >250ms and clean child
shutdown summary. A PASS for this audit is a classification of the
**blocked incident**; it does NOT retroactively pass M23AU.

Operator procedure from any non-root user shell:

~~~bash
(
  set -euo pipefail
  umask 077
  DIR="$HOME/.local/state/leroy-m23av-source"
  LOG="$HOME/.local/state/leroy-m23au/live-camera-01.log"
  test ! -e "$DIR"
  test -f "$LOG" && test ! -L "$LOG"
  sha256sum "$LOG"
  git clone --filter=blob:none --depth=1 --single-branch --no-checkout \
    --branch feature/m23av-single-raw-stamp-incident-audit \
    https://github.com/j0yk1ll/leroy.git "$DIR"
  cd "$DIR"
  git sparse-checkout init --no-cone
  git sparse-checkout set --no-cone \
    '/ros/m23av_log_audit.py' \
    '/ros/m23av_source_test.py'
  git checkout --detach
  test -z "$(git status --porcelain)"
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23av_source_test.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B ros/m23av_log_audit.py
  PYTHONDONTWRITEBYTECODE=1 python3 -B \
    ros/m23av_log_audit.py --log "$LOG"
)
~~~

Expected 14 offline tests, inert default,
`M23AV_VERDICT=ONE_REPEATED_OBSERVED_RAW_HEADER_STAMP`,
`M23AV_PRODUCTION_ACCEPTANCE=BLOCKED`.
A script/log mismatch is a diagnostic blocker; do not alter
original evidence to force a passing result.

## Future investigation (NOT approved here)

Preserve original host camera-run log bytes and hash. No
automatic retest is authorized from the initial M23AU consent.

If deciding to run another single camera-only experiment, first
design and source-test a controlled instrumented setup that
captures **exact** raw/rect subscriber header stamps and monotonic
arrival sequence, plus **publisher-side** source stamp/sequence
evidence, to isolate whether duplication occurs in usb_cam/V4L2
timestamping, DDS delivery or observer path. Preserve only bounded
small text samples in a fresh private host log, no image recording.
Require a new independent operator approval and verify actuator
supplies remain disconnected at that time. Never weaken the
existing strict monotonicity or 250ms image-gap gate.

Do not run `robot.launch.py` or `ros/exec.sh`, which start
the serial bridge. No IMU/MCU, recording, XRSLAM, Nav2, DepthART,
motor restoration or production use.

M23AV_CAMERA_RETEST_AUTHORIZED=FALSE
M23AV_PRODUCTION_ACCEPTANCE=BLOCKED
