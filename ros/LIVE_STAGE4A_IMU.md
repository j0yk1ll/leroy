# Stage 4A — real STM32 IMU transport alongside Stage 3, WITHOUT VIO or motion

Status: **code candidate / NO live trial performed under this change**.

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
sidecar. Only this new disposable child maps `/dev/ttyACM0`.
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

The Stage 4A code remains unverified on the physical MCU until the
operator runs it and provides the above evidence.
