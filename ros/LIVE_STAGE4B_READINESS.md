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

**Do not run the Stage 4B instrumented image against the MCU until the
code, image and physical-session gate are separately reviewed.** Do not
change Stage 3's live camera or XRSLAM namespaces to make it work.

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
