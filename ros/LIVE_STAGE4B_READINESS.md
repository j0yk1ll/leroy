# Stage 4B — offline evidence audit and IMU delivery instrumentation

**Status:** Stage 4A physical transport PASS, Stage 4B calibration NOT READY.
This branch makes source/receiver comparisons testable without touching
hardware. It **does not** calibrate camera–IMU time, extrinsics, sensor noise,
XRSLAM pose, or authorize motion.

## What the accepted Stage 4A evidence actually proves

The supervised 2026-10-28 Stage 4A run had valid MCU START/STOP ACKs,
3,750 source IMU packets and 3,750 ROS publications over 75 s at 50.0 Hz,
1,533 passive subscriber callbacks, 727 rectified camera images,
1,490 CameraInfo callbacks, and 74.896 s concurrent receipt overlap.
There were no reported malformed/uncalibrated/sequence/timestamp/serial
faults, no actual odometry or dynamic TF, and XRSLAM remained
monocular with localization invalid.

**1,533 / 3,750 = 40.88% observed/published**, but this is **not a DDS
packet-loss measurement**. The passive observer shares execution with
the serial loop and must service multiple camera/IMU callbacks while
DepthART is running on the host. The current aggregate counters cannot
distinguish DDS queue overflow from callback starvation or scheduling.

The MCU clock-rate estimate (−31.55 ppm) measures relative clock rate,
**not** the camera-exposure-to-IMU-acquisition offset. Reported
receipt-time overlap is **not** physical timestamp synchronization.

## First supervised exact-stamp result — 2026-10-28

The operator independently ran the Stage 4B image inside the existing Stage 3
rootless isolated DDS graph; the motor/arm power isolation and physical
E-stop/chassis safety were operator-attested (not software-measured).
This is one run, **not** a calibration.

Acceptance markers: `STAGE4B_IMAGE_TRACE=VERIFIED_OFFLINE`,
`STAGE4A_PRE_SERIAL_DDS_GRAPH=PASS`,
`STAGE4A_IMU_START_ACK=PASS`, `STAGE4A_IMU_STOP_ACK=PASS`,
`STAGE4A_IMU_OVERLAP=PASS_TRANSPORT_ONLY`,
`STAGE4A_PROCESS_RC=0`. Pre-serial real DepthART/costmap diagnostic
passed. No odometry/dynamic TF and no reported motor commands.

| Observed signal | Real physical run |
| --- | ---: |
| Session and concurrent camera/IMU receipt overlap | 75.0 s / 74.916 s |
| MCU source packets / post-publish callbacks | 3,750 / 3,750 |
| Source rate / host-relative clock estimate | 50.0 Hz / −31.2 ppm |
| Passive IMU exact-stamp matches / unmatched | 1,379 / 2,371 |
| Subscriber-only unmatched stamps | 0 |
| Source matched fraction | 0.367733 |
| Rectified image callbacks / CameraInfo callbacks | 993 / 1,421 |
| Total observed sensor subscription callbacks | 3,793 (≈50.57/s) |
| Matched post-publish→subscriber delay p50 / p95 / maximum | 80.236 / 81.650 / 86.404 ms |
| Negative monotonic latency / stamp-order faults | 0 / 0 |
| MCU packet integrity, timestamp, serial faults | all 0 |
| ROS odom / dynamic TF messages | 0 / 0 |
| Camera–IMU temporal/extrinsic/noise calibration | all **UNVALIDATED** |

Full operator transcript was saved locally as
`/home/kevin/stage4b-imu-mucER3tK.log`. Only the pasted transcript was
reviewed remotely; no local file was opened by this change.

**Strong scheduling hypothesis:** `live_stage4_imu_overlap.py` originally
called **one** `rclpy.spin_once(observer, timeout_sec=0.002)` per
50 Hz serial `readline()`, while three sensor topics competed for callback
execution. The observed aggregate callback count ≈50.6/s is close to
the serial packet rate, as that design predicts. The approx. 80 ms
post-publication callback delay is also consistent with queue contention
but is not diagnostic of a particular DDS transport cause.

**Important distinction:** the 2,371 unmatched publications in this
75-second trace are not a measured DDS network packet loss count, and
a post-publication callback delay is not physical acquisition latency.
The root cause must be tested by changing callback servicing and repeating
only under separate physical authorization.

### Follow-up source-only mitigation candidate

To address the observed callback-service bottleneck, this same Stage 4B
branch now includes `live_stage4b_spin_budget.py`: the serial loop
processes the MCU line first, then attempts up to **eight** nonblocking
ROS `spin_once(timeout_sec=0.0)` operations within an **8 ms**
budget (budget checked between callbacks because individual callbacks
cannot be interrupted). The code records
`stage4b_executor_service` with spin attempts, serial iterations and
maximum callback burst. Spin attempts are *not* messages delivered.
All existing graph-fault checks and MCU STOP-finally/ACK requirements
remain intact.

This patch is **NO-DEVICE TESTED ONLY** until a new explicitly supervised
physical trial. Rebuild the Stage 4B image after fetching the new branch
commit; an image built before this patch still contains the previous
one-spin-per-MCU-line implementation. Neither the new code nor the
source-only tests authorize VIO, odom, dynamic TF, camera–IMU calibration
or motor power.

## Offline audit: NO ROS, camera, serial or motor access

`live_stage4b_evidence_audit.py` takes a bounded, existing *text transcript*
containing the single Stage 4A JSON and acceptance markers. It fails on
missing/contradictory/duplicate markers, missing STOP ACK, nonzero
integrity faults, wrong IMU publisher, unexpected odometry/dynamic TF,
or any attempt to report calibrated VIO authority.

It produces a transparent report:

```text
STAGE4B_READINESS_JSON={... "stage4b_calibration_ready": false, ...}
STAGE4B_RESULT=TRANSPORT_AUDITED_CALIBRATION_BLOCKED
```

Run only against an **existing** locally saved transcript, if available:

```bash
cd /home/kevin/Code/leroy/ros
python3 -m unittest -v live_stage4b_test
python3 live_stage4b_evidence_audit.py /path/to/existing/stage4a-console.log
```

Change the `cd` target to your Stage 4B worktree if needed.
If no full log was captured, **do not invent one**. The operator-reported
Stage 4A result is already documented in `LIVE_STAGE4A_IMU.md`, and the
unit test uses that summary as a labelled fixture. It is not a substitute
for raw matched timestamps.

The audit never estimates a camera/IMU offset from the two aggregate
ages or from header-receipt latency.

## Next repeatable evidence: exact per-message matching

`live_stage4b_delivery_trace.py` is a bounded, pure-Python in-memory
diagnostic. Its integration into `live_stage4_imu_overlap.py` uses the
existing M18 `on_imu_published` callback only **after** successful
`/imu/data_raw` publication. The passive subscriber records the same
IMU header timestamp and a local monotonic callback receipt time.
Both times come from the same host process; no new ROS topics, publishers,
MCU writes or device mappings are added.

At the end of the **next separately authorized** Stage 4A-style
physical session, `STAGE4A_RESULT_JSON` will also contain
`stage4b_delivery_trace`, with:

- source callbacks and received subscriber callbacks;
- exact matching/unmatched ROS IMU timestamp counts;
- observed matched callback-delay p50/p95/max in milliseconds;
- flags for out-of-order, duplicate or negative-delay records;
- explicit `not_proof_of_dds_packet_loss=true`,
  `not_physical_camera_imu_offset=true`, `calibration_ready=false`.

This probe can narrow a delivery problem. It **still cannot by itself**
attribute every unmatched sample to DDS loss versus executor congestion.
Do not call the latency between a post-publication hook and subscriber
callback the physical IMU acquisition age or calibration offset.

## Safe no-device verification and image build

The Stage 4B branch is stacked on
`feature/stage4-isolated-imu-overlap` (PR #160), which is not yet
merged into main. Make a separate clean worktree, preserving all
uncommitted local changes:

```bash
cd /home/kevin/Code/leroy
git fetch origin feature/stage4b-offline-evidence-readiness
git worktree add --detach ../leroy-stage4b \
  origin/feature/stage4b-offline-evidence-readiness
cd ../leroy-stage4b/ros
python3 -m unittest -v live_stage4b_test live_stage4_contract_test
bash -n live_stage4_imu_sidecar.sh
podman build -f Dockerfile.live-stage4-imu \
  -t localhost/leroy-ros:stage4b-imu .
```

Building an image does **not** access `/dev/ttyACM0`.
The image defaults to `/bin/false`. The existing independent
motor-power cutoff and Stage 4A consent requirements are unchanged.

### Exact next operations after an offline build

The existing launcher normally chooses `localhost/leroy-ros:stage4-imu`
(the previous accepted image). This branch adds a **strictly allowlisted**
override `LEROY_STAGE4_IMU_IMAGE=localhost/leroy-ros:stage4b-imu`
to select the new image deliberately. It refuses arbitrary tags. Before
MCU use, the wrapper verifies Stage 4B instrumentation inside that image
in a separate `--network=none`, no-device, read-only container.
Check the locally built image without MCU:

```bash
podman image exists localhost/leroy-ros:stage4b-imu
podman run --rm --pull=never --network=none --read-only \
  --entrypoint /bin/sh localhost/leroy-ros:stage4b-imu -c '
  test -s /opt/stage4/live_stage4b_delivery_trace.py
  grep -F stage4b_delivery_trace /opt/stage4/live_stage4_imu_overlap.py
'
```

A **future separately supervised physical session** may use the same
existing Stage 3 diagnostic parents, but only if they're running and
the sensor-only startup/fresh live costmap diagnostic have passed,
motor-driver and manipulator rails are independently disconnected,
the chassis and E-stop are safe, the operator is present and the stable
STM32 `/dev/serial/by-id/` identity is checked again. Only then, from
the Stage 4B `ros` directory:

```bash
LEROY_STAGE4_IMU_IMAGE=localhost/leroy-ros:stage4b-imu \
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
LEROY_STAGE4_MCU_BY_ID=/dev/serial/by-id/<verified-stm32-id> \
bash ./live_stage4_imu_sidecar.sh
```

Replace the placeholder by the actually verified stable ID. The script
again checks the real costmap and explicitly asks `STAGE4_IMU_ONLY`
before the MCU port opens; it retains the prior 75s supervised START/
STOP-ACK sequence and guarantees no new motor writes. Look for
`STAGE4B_IMAGE_TRACE=VERIFIED_OFFLINE`,
`STAGE4A_SELECTED_IMAGE=localhost/leroy-ros:stage4b-imu`
and `stage4b_delivery_trace` in `STAGE4A_RESULT_JSON`.

A physical Stage 4B run can still be **incomplete** even if Stage 4A
transport passes; do not silently equate IMU match fraction with DDS
network loss. Preserve the full console transcript in a new local log
file without overwriting previous evidence. A missing STOP ACK still
means unknown MCU state and mandatory continued physical motor cutoff.

**Do not run the Stage 4B instrumented image against the MCU until the
code, image and physical-session gate have been separately reviewed.**
Do not change Stage 3's live camera or XRSLAM namespaces to make it work.

## Calibration evidence still missing

1. **Source-to-subscriber delivery:** repeat with exact-stamp matching
   after software-only review. Investigate missing/late callbacks and
   available callback service/CPU occupancy before making a throughput
   claim.
2. **Camera–IMU temporal phase:** collect a correlated *physical*
   visual/inertial motion event with traceable camera timestamp semantics,
   raw MCU acquisition ticks, independently justified sign convention,
   multiple distinct trials and uncertainty. Reuse the prior M21
   metadata-only capture/analysis contracts as appropriate. Their
   synthetic feature lag is not real exposure phase.
3. **Rigid extrinsics:** camera optical center, physical camera–IMU
   translation and rotation, and the declared robot-frame IMU mapping
   need independently measured data. Existing `sensor_mounts.yaml`
   is marked `provisional`; its zero camera translation and modelled
   camera pitch are not certified calibration.
4. **Noise and bias:** supervised stationary IMU sequences with source
   timestamps, device/temperature/environment context, analysis of
   bias, variance and longer-run drift; no borrowing EuRoC template
   noise densities as if calibrated for Leroy.
5. **XRSLAM:** only after the above gates should `monocular_imu`
   diagnostics be considered, still with
   `pose_convention_validated=false`, `enable_map_tf=false`,
   no `/odom`, no navigation goals and no motor power.

**No Stage 4B VIO or calibration PASS is claimed by this code.**
