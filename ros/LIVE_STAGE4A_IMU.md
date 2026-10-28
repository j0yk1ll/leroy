# Stage 4A — real STM32 IMU transport alongside Stage 3, WITHOUT VIO or motion

Status: **operator-observed physical Stage 4A transport PASS (2026-10-28);
not VIO, calibration or navigation acceptance**. The implementation remains
in PR #160 until reviewed and merged.

Stage 3 already passed with real USB camera → rectification → CPU DepthART
→ raw optical PointCloud2 → diagnostic Nav2 costmap, while XRSLAM's
monocular native estimator received 6,000 real images. Both containers
received `tracking_status: initializing` and
`localization_valid: false`; no `/odom` message appeared in a bounded
six-second test.

**Stage 4A has a different acceptance boundary:**
`/imu/data_raw` from the REAL STM32 MCU is concurrently observable in
the *same isolated DDS graph* as the real camera/depth/costmap and
still-monocular XRSLAM. This proves only basic IMU packet and clock
continuity, transport and receipt-time overlap. It does **not**
measure the physical camera–IMU offset, IMU noise, sensor extrinsics,
tracking quality or metric scale.

## Non-negotiable safety

The MCU at `/dev/ttyACM0` is **motor-capable** even if the caller
intends to read sensors. Its CDC open can reset the firmware. Both motor
power rails and manipulator power must remain **physically disconnected**;
chassis secured, E-stop checked and operator supervising an immediate
independent cutoff. No unattended operation.

The only UART/CDC bytes the Stage 4A application writes are the
unchanged M18:
- `set_imu_stream_hz 50\r\n` (START)
- `set_imu_stream_hz 0\r\n` (STOP)

The existing M18 ROS publisher, packet parser and clock-rate mapper
remain unchanged. Unlike M20, Stage 4A permits an existing
`/depthart/depth` and XRSLAM node, because they have already been
accepted in Stage 3. It still rejects motor/controller graph nodes
and topics, duplicate/rogue IMU publication, any real `/odom` or
dynamic `/tf` sample, and a `true` localization-valid latch.

An ACKed START **and** an ACKed STOP are mandatory. A missing STOP
ACK is an *uncertain MCU state*, not a successful test. Keep motor power
cut off and investigate physically before another attempt.

## Prepare and verify WITHOUT the MCU

Start from a clean updated `main` worktree and check out this Stage 4A
feature branch separately. Your existing dirty development checkout
must not be reset or cleaned. With existing local Stage 3 costmap and
XRSLAM images built, build the NEW image **without running it**:

```bash
cd ros
bash -n live_stage4_imu_sidecar.sh
python3 -m unittest -v live_stage4_contract_test

podman build -f Dockerfile.live-stage4-imu \
  -t localhost/leroy-ros:stage4-imu .
```

The image inherits the existing Stage 3 costmap runtime but its default
command is `/bin/false` (inert). It installs the unchanged M18 source,
strict parser, and existing M16 observer's Python dependencies only.
It does **not** start a bridge, serial port, XRSLAM VIO, Nav2 controller,
firmware flash or motor driver.

## Explicit physical attempt: bounded Stage 4A IMU stream

First start the **previously accepted Stage 3 isolated containers**
`leroy-live-costmap` and `leroy-live-xrslam`. Do not stop or restart
them just to run Stage 4A; they must already be passing the Stage 3
camera/costmap and monocular-status gates.

On Arch Linux, inspect the MCU's actual stable ID:

```bash
ls -l /dev/serial/by-id/
```

Then with the motor rails physically disconnected, chassis secured,
E-stop checked and operator at the cutoff, set the exact stable ID
you verified (placeholder below must be replaced with the real path):

```bash
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
LEROY_STAGE4_MCU_BY_ID=/dev/serial/by-id/<verified-stm32-id> \
bash ./live_stage4_imu_sidecar.sh
```

The wrapper requires an **interactive** `STAGE4_IMU_ONLY` confirmation.
It refuses to run as root, refuses host network/IPC/PID namespaces,
refuses unexpected device identity, requires shareable IPC and a
privately isolated Stage 3 parent, refuses an MCU visible inside
either Stage 3 container, and refuses to overwrite any named existing
sidecar. It also executes a fresh read-only Stage 3 costmap acceptance probe
**before** prompting for MCU-session consent or mapping the serial device.
Only this new disposable child maps `/dev/ttyACM0`.
The child's filesystem is read-only, capabilities are dropped and its
run is bounded. It does not map `/dev/video2`.

### Required evidence for Stage 4A PASS

```text
STAGE4A_PRE_SERIAL_DDS_GRAPH=PASS
STAGE4A_IMU_START_ACK=PASS
STAGE4A_IMU_STOP_ACK=PASS
STAGE4A_RESULT_JSON={...}
STAGE4A_IMU_OVERLAP=PASS_TRANSPORT_ONLY
STAGE4A_PROCESS_RC=0
STAGE4A=PASS_TRANSPORT_ONLY_NOT_VIO_OR_CALIBRATION
```

The JSON must show independently received CameraInfo, real
`/camera/image_rect`, `/imu/data_raw` from the uniquely named
`m18_imu_source`, overlapping receipt intervals, monotonically
timestamped samples, an accepted MCU clock-rate estimate and
zero odometry/dynamic TF messages. No ROS publisher count alone
satisfies a PASS.

The numerical camera–IMU offset is **not** computed from receipt overlap:
USB CDC transport latency, initial MCU clock anchoring and camera
timestamp lag prevent that inference. We must measure temporal phase
using suitable sensor motion/correlation or calibrated test data in a
later dedicated step. **Never invent `camera_imu_offset_sec=0`** to
bypass XRSLAM's VIO guard.

## Observed physical Stage 4A acceptance — 2026-10-28

The operator ran the candidate code on Arch Linux using the real STM32F401
CDC device at its checked stable `/dev/serial/by-id/` path (resolving to
`/dev/ttyACM0`), and the already running Stage 3 rootless costmap and
monocular XRSLAM containers. The physical motor-rail isolation was
**operator-attested**, not measured by software. No hardware actuation was
authorized.

The pre-serial, read-only real DepthART/Nav2 costmap probe returned
`LIVE_DEPTH_COSTMAP=PASS` with 3 optical clouds, 3 grids,
`cloud_age_last_s=1.075`, `lethal_cells=true`,
`unknown_cells=true`, frame `base_link` (lethal surfaces remain
**unclassified**, not for physical navigation).

Exact acceptance markers:

```text
STAGE4A_PRE_SERIAL_DDS_GRAPH=PASS
STAGE4A_IMU_START_ACK=PASS
STAGE4A_IMU_STOP_ACK=PASS
STAGE4A_IMU_OVERLAP=PASS_TRANSPORT_ONLY
STAGE4A_PROCESS_RC=0
STAGE4A=PASS_TRANSPORT_ONLY_NOT_VIO_OR_CALIBRATION
```

The operator-supplied `STAGE4A_RESULT_JSON` reported:

| Signal / gate | Observed |
| --- | --- |
| Bounded session | 75.0 s |
| Real MCU packets received | 3,750 |
| ROS IMU messages published by M18 source | 3,750 |
| Source IMU rate | 50.0 Hz |
| Passive ROS IMU messages observed | 1,533 |
| Rectified camera images observed | 727 |
| CameraInfo messages observed | 1,490 |
| Concurrent receipt overlap | 74.896 s |
| MCU clock-rate estimate | -31.55 ppm, ready |
| Source acquisition age (last / max) | 0.0031 / 0.0091 s |
| Packet malformed, sequence, gaps, calibration withheld | 0 each |
| Timestamp / stale/future / ASCII / unexpected serial faults | 0 each |
| Odometry messages | 0 |
| Dynamic TF messages | 0 |
| START / STOP acknowledgements | both true |
| Terminal fault / error | null / null |
| Camera–IMU absolute offset validated | false |
| Camera–IMU extrinsics / gyro noise validated | false |
| XRSLAM VIO started / metric odometry authorized | false / false |

**Scope of PASS:** The 3,750 IMU publications versus 1,533 received by the
passive observer reflect this observer's callback servicing under a
concurrent CPU DepthART load; this single run does **not** establish the
cause of the difference or end-to-end loss-free 50 Hz subscriber delivery.
The source achieved 50.0 Hz and all source-integrity faults stayed zero.
Receipt-time overlap is **not** calibrated camera/IMU acquisition-time
alignment. The -31.55 ppm estimate describes MCU/host *clock rate* only,
not their absolute epoch/latency relationship.

This operator result completes **Stage 4A transport acceptance only**.
Do not enable `sensor.mode=monocular_imu`, invent a camera–IMU offset,
permit `/odom`, `map -> odom`, navigation goals or motor power based on
this evidence. Physical sensors and their timing still need independent
calibration.

## Next steps after live Stage 4A PASS

1. Characterize the IMU noise and camera–IMU temporal alignment with
   stationary and non-driving supervised sensor sequences.
2. Validate camera/IMU rigid-body extrinsics, ROS axis mapping and XRSLAM
   device-template fields.
3. Then permit a `sensor.mode=monocular_imu` *diagnostic* XRSLAM
   sidecar with an independently measured offset, while keeping
   `pose_convention_validated=false` and `enable_map_tf=false`.
4. Keep the Nav2 costmap diagnostic, no odom, no planner/controller
   and motor power disconnected until independent qualification.

The physical Stage 4A transport run is accepted as above. Remaining
uncertainties are camera–IMU temporal phase, intrinsic/extrinsic calibration,
noise characteristics, end-to-end subscriber delivery and actual VIO
tracking. The candidate has not been deployed as a production runtime.
