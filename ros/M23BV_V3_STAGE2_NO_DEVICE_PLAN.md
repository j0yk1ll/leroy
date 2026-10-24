# M23BV V3 — Stage 2A no-device integration review

**Status (2026-10-24): DESIGN / SOURCE REVIEW ONLY.** This branch is stacked
on the Stage 1 camera-only candidate at commit
`354c05a03ba6a688669cf0f5e2f582533ae94178` (draft PR #151).
Draft PR #150 remains the older parent. **Do not merge or switch production.**

## Accepted evidence and its limits

Stage 1 bounded real camera-only trial: `M23BV_V3_STAGE1=CAMERA_ONLY_PASS`.
Operator-reported private runtime log SHA-256:
`de4b819cbd763b566c1f8be43e3bb39b082ee4cee596e6f97b806eddcabba092`.
The Stage 1 observer logged 109 matching 640x480 rgb8 raw/CameraInfo/
rectified frame stamps, actual RELIABLE/VOLATILE image endpoints under
`rmw_fastrtps_cpp`, 19.92 Hz over a selected 4.468-second stamp window
and max 52ms adjacent stamp gap. Readiness observed 10 matching raw/info
frames. Launch exited 0, camera/rectifier children finished cleanly,
and operator inventory showed no residual test container.

This validates *one bounded Stage 1 run*, not camera/IMU overlap, physical
camera-IMU time offset, extrinsics, pixel-level optical rectification accuracy,
camera disconnect handling, sustained frame-rate performance or recording.

## Why Stage 2 is NOT a simple selector change

1. **Selected M23BV parent remains legacy.**
   `ros/m23bv_sensor_only_run.sh` still invokes
   `/opt/m23bv/m23bv_camera_only.launch.py`, using stock image_proc;
   it then runs `/opt/m23bv/m23bv_qos_guard.py` before the original M20
   combined observer. Stage 1 V3 is selected only by its distinct opt-in
   camera-only wrapper, not by this parent. Changing the selected parent
   would invalidate accepted M20/M23BV evidence and is out of scope.

2. **The existing guard requires a recorder.**
   `m23bv_qos_guard.validate_camera_graph` requires exactly one
   `rosbag2_recorder` RELIABLE rectified image subscriber. A
   no-recording Stage 2 design cannot use this guard unmodified or
   suppress its requirement by treating a missing recorder as PASS.
   A separately named, strict, *read-only* no-recorder DDS guard is needed
   for any eventual V3 camera + M20 observer integration. Unknown nodes,
   unwanted MCU/motor/navigation graph, duplicate endpoints, incompatible
   durability/reliability and source changes must block.

3. **M20 MCU opening is consequential, even with motor commands excluded.**
   `m20_combined_observer.py` opens serial and uses the protected M18
   `START` / `STOP` whitelist and ACK checks. Serial-open/DTR may reset
   the MCU; this cannot be described as inert. Its STOP-in-finally and
   original lifecycle must remain unchanged. No wrapper in this branch
   maps, opens, simulates or commands any physical MCU.

4. **M20 observation thresholds exceed the Stage 1 test.**
   The original M20 contract requires a full 120-second observation,
   at least 5,700 published IMU messages, at least 400 images and
   CameraInfo messages, and at least 90 seconds of receipt overlap.
   It also checks IMU clock readiness and ~50 Hz rate, original packet
   integrity and source provenance. Stage 1's 109 camera-only matched
   stamps do not establish any of this.

5. **An immutable container ID alone is not sufficient.**
   Verify the exact V3 image contains the unchanged M20 and M18 Python
   modules, their imported dependencies (including serial), the
   correct overlay, launch, QoS implementations and the calibrated
   camera path **with no devices mapped**. The V3 Dockerfile inherits
   a previously built sensor image, but inheritance alone does not
   prove the installed runtime satisfies this next-phase contract.

6. **Recording is a separate scope.**
   The existing `m23bv_bounded_capture.py` starts an actual
   `ros2 bag record` process; its opt-in variable cannot make
   recording permissible here. The earlier synthetic V3 closed-bag
   tests are offline evidence, not authorization for physical recording.

## Proposed Stage 2A: NO-DEVICE work only

**Deliverable 1 — pinned source inventory.** Run the new
`ros/m23bv_v3_stage2_source_gate.py`. It reads pinned source files;
uses only Python standard-library text, SHA-1 Git-blob and AST checks;
does **not** import rclpy, serial, cv2 or Podman, and never launches ROS
or touches device files. A PASS means only that reviewed source
contracts have not drifted.

**Deliverable 2 — isolated synthetic composition candidate (SOURCE
IMPLEMENTED; ROS RUNTIME NOT YET TESTED).** The distinct files
`ros/m23bv_v3_stage2_synthetic_probe.py`,
`ros/m23bv_v3_stage2_synthetic_preflight.sh` and
`ros/m23bv_v3_stage2_synthetic_static_gate.py` provide a bounded,
hardware-free experiment with the *compiled pinned V3 C++ rectifier*,
original pinned camera readiness gate, synthetic camera, and synthetic
50 Hz IMU. The probe reconstructs the pinned launch's rectifier,
readiness gate and lifecycle event handlers from the exact candidate AST
but substitutes an in-memory camera and adds the fake IMU process. It
does not import/call the physical camera launch or the M20 serial
observer. The wrapper uses the unchanged immutable V3 image ID, Podman
`--network none`, read-only filesystem, private /tmp and **no device or
bind mounts**. It requires no preexisting production container.

Three source-implemented cases: `healthy` checks actual graph/RMW DDS
QoS, at least 100 matching 16x16 synthetic raw/CameraInfo/rectified
stamps, at least 300 synthetic IMU messages and at least 3 seconds of
common message-receipt overlap; `missing_imu` must reject a source
with no IMU publisher; `imu_qos_reliable` must reject a wrong
synthetic IMU offered reliability even though BEST_EFFORT subscribers
might be able to receive it. All cases require four clean child exits,
zero launch exit, verified V3 source/CPP/checker pins, a clean SIGINT
and no remaining child PIDs. Unknown graph nodes/topics also reject.
Additional negative tests for corrupted timestamps, unexpected graph
endpoints, early rectifier exit and forced shutdown remain **PENDING**.

**Crucial limit:** This is a **synthetic coexistence test**, not an
execution of the actual M20 combined observer, real MCU START/STOP/ACK,
physical IMU acquisition, physical synchronization, or metric accuracy.
Preserve the original M20 physical acceptance thresholds and require
explicit scope expansion before real MCU work.

**Deliverable 3 — no-recorder guard design and implementation (pending).**
Audit raw/image_rect/CameraInfo RELIABLE/VOLATILE endpoints, and IMU
BEST_EFFORT/VOLATILE publisher and observer compatibility, without
requiring rosbag. Reject unknown motion/TF/depth/recording publishers,
unexpected nodes or altered camera calibration. Be explicit that a
synthetic IMU has no actual MCU provenance.

**Deliverable 4 — isolated shutdown proof (pending).**
Use a finite process deadline and demonstrate all camera, rectifier,
synthetic IMU and observer children exit cleanly; nonzero child exits,
lost STOP ACK (when eventually authorized on physical serial),
stray processes, forced cleanup and missing markers must block.
A launch parent return code of 0 by itself is not sufficient.

## Scope boundary

Currently authorized and accepted: **Stage 1 bounded camera-only**.
Current work: **Stage 2A static / synthetic, NO DEVICE**.
**NOT authorized:** physical MCU serial mapping/open, M20 hardware
START/STOP/ACK, motor power or actuation, rosbag recording, Nav2,
XRSLAM, camera unplug testing, changing production selectors, or merging
draft PRs. The prior standing camera-only authorization does not
expand these Stage 2 scope boundaries.

Physical Stage 2B requires a separately reviewed MCU-capable device
design and an explicit scope change; it is not triggered by this review.
Even then, both motor-power rails physically disconnected, chassis
secured, E-stop verified, operator supervising immediate power
disconnect and a fresh owner-private record are mandatory; any failure
is a NO-GO. Do not attempt it as part of this PR.

## Source-only operator check

From a clean checkout of this Stage 2A review branch:

```bash
python3 -B ros/m23bv_v3_stage2_source_gate.py
bash ros/m23bv_offline_gate.sh
python3 -B ros/m23bv_v3_stage1_static_gate.py
git status --porcelain
```

Expected from the new source inventory:
`M23BV_V3_STAGE2_SOURCE_REVIEW=PASS` and
`M23BV_V3_STAGE2_PHYSICAL_DEVICE_ACCESS=NONE`.
There is **no Stage 2 physical acceptance marker** in this PR.

## Stage 2A synthetic Jazzy execution — ONLY no-device Podman

**Status:** source candidate committed, operator-host no-device runtime
pending. Start only from the exact review branch with clean worktree, after
source and unit checks. The test consumes no physical input and generates
synthetic messages on isolated ROS_DOMAIN_ID 170. No rosbag.

```bash
python3 -B ros/m23bv_v3_stage2_source_gate.py
python3 -B ros/m23bv_v3_stage2_synthetic_static_gate.py
bash -n ros/m23bv_v3_stage2_synthetic_preflight.sh
bash ros/m23bv_offline_gate.sh

export M23BV_LOG_DIR="$HOME/.local/state/leroy-m23bv"
# Require an existing owner-private log directory, mode 0700.
M23BV_V3_STAGE2A_CASE=healthy \
  bash ros/m23bv_v3_stage2_synthetic_preflight.sh
M23BV_V3_STAGE2A_CASE=missing_imu \
  bash ros/m23bv_v3_stage2_synthetic_preflight.sh
M23BV_V3_STAGE2A_CASE=imu_qos_reliable \
  bash ros/m23bv_v3_stage2_synthetic_preflight.sh
```

Run each case **once**, preserve its owner-private log path and SHA-256.
Only successful case-specific markers, actual graph metrics and child
shutdown markers establish a synthetic Stage 2A gate. If any blocks,
stop and diagnose the log; do not automatically repeat hardware or
start different workflows. Synthetic failure has no implications for
the already accepted Stage 1 physical camera test.

