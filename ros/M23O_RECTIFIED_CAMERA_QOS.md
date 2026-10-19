# M23O — isolated camera + image_proc reliable rectified image diagnostic

**DRAFT. First operator-run camera experiment completed on 2026-10-19; overall
INCOMPLETE (SIGSEGV). A post-run diagnostic patch now requires fresh offline
validation before any separately authorized physical repeat.**
Stacked on M23N draft PR #108,
which itself depends on M23M #107, M23L #106, M23K #105. No production merge.

## Why this experiment exists

The operator's physical M23N raw-only MJPEG experiment reported:
- best_effort_100: raw 277 images, 5.919 Hz, 47 header gaps >250ms, max 2347.984ms.
- reliable_100: raw 1414 images, 29.821 Hz, 0 header gaps >250ms, max 36.135ms.
- CameraInfo ~29.81Hz in both runs, both exit0 and clean, QoS verified.
- These are subscriber receipts, not independently authenticated source counts.

M23O isolates the **remaining rectifier path** using the existing M20
full_mjpeg launch (usb_cam + `image_proc::RectifyNode` component), plus
the existing M23M C++ image header receiver at `reliable_100` for raw
and rectified Image. No altered or rebuilt image_proc, no change to production.

In Jazzy, RectifyNode subscribes lazily after a downstream subscriber
appears, choosing camera input QoS by graph inspection (with a possible
sensor-data best-effort fallback when no upstream publisher is known).
M23O checks the actual discovered QoS at runtime rather than assuming
an override was effective.

## First M23O physical run — result and independent blocker

**Operator log from 2026-10-19: overall INCOMPLETE, exit 2.**

- Raw Image: **1411**, **29.812Hz**, **0 header gaps >250ms**,
  longest header interval **36.114ms**.
- Rect Image: **1410**, **29.830Hz**, **0 header gaps >250ms**,
  longest header interval **36.114ms**.
- **1410/1410** rect timestamps matched raw timestamps; one
  unmatched raw and zero unmatched rect timestamps.
- CameraInfo: **1411**, **29.812Hz**, zero large header gaps.
- ROS graph: `rmw_fastrtps_cpp`, camera/rectifier/receiver observed
  `RELIABLE/VOLATILE`; depth/history appeared as 0/UNKNOWN, not
  authenticated queue depths.
- **BLOCKER:** `component_container` child exited **-11/SIGSEGV**;
  `CAMERA_CHILD_PROCESS_CRASHED`, `shutdown_clean=false`.
- No STM32/IMU/recorder access and no motion authorization.

This establishes the observed continuity of both subscriber streams in
this one run but does **not** validate the complete camera lifecycle or
declare VIO readiness. The original `--rm` container's internal launch
log was not preserved outside the container; the available operator tee
log includes only the reported child exit.

**Post-run patch on the same M23O draft branch:** suppress duplicate
`M23N_CAMERA_QOS_JSON` / `M23N_CAMERA_QOS_VERDICT` in M23O runs,
and add bounded `M23O_CAMERA_CRASH_CONTEXT` excerpts around a future
ROS launch child failure (at most 21 lines, each line capped).
These additions are not yet operator-tested. Re-run offline Phases A–C
after pulling the new branch head. Do not immediately repeat the
physical run; any repeat requires fresh explicit authorization after
offline acceptance, robot immobilization and inhibited motor power.

## Diagnostic-only acceptance

For this experimental diagnostic only, require:
1. The C++ observer emits exactly one matching `reliable_100` marker.
2. A read-only rclpy graph snapshot confirms **RELIABLE/VOLATILE**
   for camera raw + CameraInfo publishers, rectifier raw + CameraInfo
   subscriptions, rectifier rectified publisher, and C++ raw and rectified
   subscriptions; actual `rmw_implementation` identifier is recorded.
3. Both raw and rectified observed header intervals have **zero gaps
   over the unchanged 250 ms threshold**.
4. Both raw and rect streams are observed at **>=25 Hz**; at least
   95% of rectified stamps match raw stamps observed by this independent
   C++ subscriber, with no unmatched rectified timestamps.
5. No process failure, missing stream, nonmonotonic stamps, or
   `component_container` SIGSEGV during shutdown.

If all pass, the result is `RECTIFIED_CAMERA_QOS_DIAGNOSTIC_ONLY`, **not
VIO readiness**. Missing or nonreliable actual endpoints, missing graph
output, crashed rectifier, or any unverified stream yields exit 2 and
a BLOCKED/INCOMPLETE verdict.

The raw/rect counters still deserialize complete Images through DDS,
so their counts do not prove how many camera frames were published.
The graph sample is an endpoint observation, not packet-level evidence.
The original first rosbag remains immutable. IMU coverage, camera/IMU
phase, metric calibration and XRSLAM remain BLOCKED. **NO MOTION.**

## Phase A — source regression, no devices

```bash
cd "$(git rev-parse --show-toplevel)"
git fetch origin
git switch feature/m23o-rectified-camera-qos
git pull --ff-only origin feature/m23o-rectified-camera-qos
cd ros

PYTHONDONTWRITEBYTECODE=1 python3 -u m23o_rect_camera_qos_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23n_camera_qos_probe_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23m_dds_qos_matrix_test.py
PYTHONDONTWRITEBYTECODE=1 python3 -u m23l_synthetic_dds_baseline_test.py
```

M23O expects 10/10, plus M23N 10/10, M23M 9/9, M23L 16/16
(45 total), source marker `M23O_RECT_QOS_SOURCE_CONTRACT=PASS`.
This phase never invokes ROS or opens any device.

## Phase B — build offline, no devices

The already-built local parent `m23n-real-camera-qos` image is required.

```bash
set +e
set +o pipefail
podman image exists localhost/leroy-sensors:m23n-real-camera-qos
podman build --network none --no-cache \
  -f Dockerfile.m23o-rect-camera-qos \
  -t leroy-sensors:m23o-rect-camera-qos . \
  2>&1 | tee m23o_no_device_build.log
echo "M23O_BUILD_EXIT_CODE=${PIPESTATUS[0]}"
```

Require `M23O_RECT_QOS_SOURCE_CONTRACT=PASS`,
`M23O_JAZZY_RECT_QOS_IMPORT=PASS` and exit0.
If Jazzy's rclpy graph API differs, BLOCK and fix this draft branch
before any physical camera access.

## Phase C — no-device container self-test

```bash
set +e
set +o pipefail
podman run --rm \
  --network none --ipc private --pid private \
  --cap-drop all --security-opt no-new-privileges \
  --read-only --tmpfs /tmp:rw,nosuid,nodev,size=512m \
  localhost/leroy-sensors:m23o-rect-camera-qos \
  2>&1 | tee m23o_no_device_selftest.log
echo "M23O_SELFTEST_EXIT_CODE=${PIPESTATUS[0]}"
```

Require `M23K_NO_DEVICE_SELFTEST=PASS`, exit0.
No physical action before A-C all pass.

## Phase D — ONE human-approved camera + rectifier run

Human operator must **physically immobilize robot and inhibit motor
power**, ensure `leroy-ros` is stopped and no process uses the camera,
and resolve the precise Wed USB camera by-id path. Do not open or
map `/dev/ttyACM*`. M23O changes no motor/STM32 control.

```bash
cd "$(git rev-parse --show-toplevel)/ros"
export M23K_HOST_CAMERA_BY_ID="/dev/v4l/by-id/usb-Wed_Camera_Wed_Camera_20240420112206-video-index0"
readlink -f "$M23K_HOST_CAMERA_BY_ID"
export M23O_OPERATOR_CAMERA_ONLY_APPROVED=YES

set +e
set +o pipefail
bash ./m23o_operator_camera_rect_qos.sh \
  2>&1 | tee m23o_camera_rect_reliable_100.log
echo "M23O_LIVE_EXIT_CODE=${PIPESTATUS[0]}"

unset M23O_OPERATOR_CAMERA_ONLY_APPROVED
```

Requires the interactive `AUTHORIZE_M23O` typed by the human.
Rootless Podman maps **only the selected camera** to `/dev/video2`,
uses `--network none`, independent IPC/PID, drop-all capabilities,
read-only root with bounded tmpfs. The default M23K selftest mode does
not access hardware.

Collect:
- `M23O_RECT_QOS_JSON`, `M23O_RECT_QOS_VERDICT`, live exit code.
- `graph_endpoints`, actual `rmw_implementation`, verified C++ QoS.
- Raw, rect and CameraInfo counts, observed rates, >250ms timestamp gaps,
  maximum header/receipt gaps, matching header stamps.
- `shutdown_clean`, faults and any `M23K_CAMERA_CHILD_EXIT`,
  including `M23O_CAMERA_CRASH_CONTEXT` lines if child process crashes.

If the image_proc container crashes with SIGSEGV during shutdown,
keep the run INCOMPLETE even if it had continuous image headers.
If the graph shows an unexpected best-effort rectifier subscription,
record it; do not silently force a production QoS change.

After a valid M23O diagnostic, only a **separately reviewed** recorder +
rectifier + IMU lifecycle integration can consider production use.
M23A's strict 250ms image header gap gate must not be weakened.
