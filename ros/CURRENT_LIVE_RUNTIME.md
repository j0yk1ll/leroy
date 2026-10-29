# Leroy: current live runtime and path to autonomous startup

**Maintained integration index (2026-10-29).** Use this guide to distinguish operator-observed live behavior from historical milestone experiments. This is not a motor-release or navigation-authority document.

## Verified on physical Leroy

| Component | Verified current behavior | Relevant source |
| --- | --- | --- |
| Stage 3 front camera | Real `/camera/image_rect` and matching `/camera/camera_info`; observed 235 rectified images and 353 CameraInfo in a ~12 s independent boot observation | `live_stage3_start.sh`, `live_depth_pipeline.launch.py` |
| DepthART | Real CPU depth pipeline; raw diagnostic optical point cloud observed 8 times in ~12 s; individual cloud receipt gaps can be seconds | `ros/src/leroy_nav2_perception/leroy_nav2_perception/depth_obstacles.py` |
| XRSLAM | Live adapter in **camera-only monocular** mode, `initializing`; `/xrslam/localization_valid=false`; advertises `/odom` endpoint but does **not establish valid metric odometry** | `live_stage3_monocular_sidecar.sh` |
| Stage 4C | Real stationary camera + 50 Hz IMU four-topic ~65 s bag, SHA-256 verified. The independent IMU baseline has 3,250 samples; no calibrated camera–IMU phase/extrinsics | `LIVE_STAGE4C_CALIBRATION_EVIDENCE.md`; draft PR #165 |
| Boot readiness | The real Stage 3 read-only ROS sidecar observed all five expected publishers, exact image/info stamps and zero frame errors; `LIVE_BOOT_SENSORS=PASS_READ_ONLY`; `LIVE_BOOT_MOTION=BLOCKED_UNCOMMISSIONED` | `live_boot_readiness.sh`; draft PR #166 |

These are runtime observations, not operator-independent physical authentication or permission to drive.

## Actual software interfaces for the next live milestone

- `ros/boot_localization_core.py`: bounded fail-closed boot-state model; `ros/live_boot_readiness.py`: real Stage 3 sensor input observation without any ROS publishers. Both currently produce **no wheel/motor requests**.
- The historical **M22/PR #156** `leroy_bridge` MCU interface includes a separately opted-in **signed** `/leroy/motor/signed_motion` + `/leroy/motor/signed_rearm` path with fresh-session verification, ACK/STOP and watchdog logic. It is **not running in Stage 3** and is not ready for autonomous deployment. Do not connect the startup supervisor to it without a separately reviewed real command-authority and stop design.
- The older `leroy_motor_manual_commission` and keyboard-hold/pulse programs are operator-attended motor commissioning tools, **not** startup navigation or collision avoidance. They require their own verified mechanical/electrical safeguards and exclusive CDC ownership.
- The Nav2 MPPI M5–M16 integration and M14 gate use **synthetic/diagnostic** command topics. They do not own a physical motor path. `ros/src/leroy_nav2/config/hardware.disabled.yaml` remains `enabled: false`.

## Deprecated assumptions: do not resurrect them as mandatory setup tasks

The user has marked the old, unconfigured GPIO E-stop/power feedback inputs and incomplete wheel-encoder calibration assumptions as **deprecated for the intended next-generation startup architecture**.

Consequently, the checked-in `ros/src/leroy_bridge/config/leroy_bridge.yaml` no longer lists the three empty GPIO feedback parameters or the two inert right-encoder sign placeholders. Those values previously matched *disabled defaults*; removing them does **not** change current active behavior. The historical bridge code and its regression tests remain available.

**Important source-level truth:** if protected signing keys are provisioned to the **old** `leroy_bridge`, its `PhysicalMotorFeedback` implementation still expects real configured GPIO inputs to establish its software permission, and its legacy FL/RL encoder estimator is an opt-in degraded fallback. This is **legacy-implementation coupling**, not a physical statement that the robot has or must add those specific GPIO wires or encoder signs. Do not bypass that code or spoof `estop_released` / `motor_rails_verified`. Build and review a replacement physical-safe-state interface that matches the actual robot's cutoff and motor-driver hardware instead.

The independent ability to inhibit/stop motor power, verify the stop response, measure the physical swept footprint, detect nearby obstacles *on all sides*, obtain reliable closed-loop heading and assign exclusive authenticated motor-command ownership remain real requirements. They are not waived by declaring an old GPIO or encoder integration deprecated.

## One live operating workflow — today (read-only)

Prerequisites: the existing guarded, isolated `leroy-live-costmap` and `leroy-live-xrslam` Stage 3 containers are already running. On the Arch host:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
git fetch origin design/boot-localization-sweep
git switch --detach origin/design/boot-localization-sweep
bash ./live_boot_readiness.sh
```

This consumes existing real camera/DepthART/XRSLAM ROS topics for 12 seconds without starting the MCU or opening a motor-control channel. Do not repeat it simply to measure FPS; the operator has already confirmed it passes.

## Real E-stop topology and live yaw integration (2026-10-29)

The operator reports a **hardwired E-stop that cuts power to all
downstream robot components, shutting the robot down**. This is the
intended mechanical/electrical stopping topology. There is **no
requirement to invent obsolete GPIO status offsets** for that cutoff.
Software cannot read a GPIO state from an unpowered computer or MCU.
After the E-stop is released and power returns, every motor command
session must begin inhibited; reset must never cause unattended
motion. Actual shutdown and stopping performance remain matters for
the supervised physical commissioning procedure.

The immediate supported upgrade is **live relative yaw from the real
50 Hz MCU IMU stream**, independent of XRSLAM's monocular initialization:

- `ros/boot_yaw_core.py`: first establishes the session-specific gyro-Z
  bias from at least 2.5 s of stationary IMU samples, then integrates
  the original MCU-mapped stamp differences to produce an **unwrapped
  relative heading**, with latched loss/clock/freshness faults.
- `ros/live_boot_yaw.py`: subscribes to real `/imu/data_raw` from the
  single accepted `/m18_imu_source`, and publishes only
  `/leroy_boot/relative_yaw_status` (JSON String, diagnostic status)
  in ROS domain 170. Each message identifies the owner, heading,
  session bias, freshness-monitoring state and explicit
  `motor_motion_authorized=false`.
- `ros/live_boot_yaw.sh`: one disposable, read-only, **no-device**
  sidecar that joins Stage 3's isolated DDS and refuses an absent M18
  source. It DOES NOT map the MCU, modify motor power, publish
  `/cmd_vel`, activate Nav2 or request an XRSLAM pose.
- `ros/boot_yaw_core_test.py`: 11 no-device tests; no additional
  camera-FPS benchmark or rosbag needed.

**Live test requires simultaneous execution in TWO terminals.**
The operator's first 75-second Stage 4B rerun completed successfully
(3750/3750 IMU packets at 50 Hz, all original stamps matched, zero
command writes, MCU START/STOP acknowledged). But the yaw command was
entered **after** the Stage 4A source finished, yielding
`BLOCKED_REAL_STAGE4A_M18_IMU_SOURCE_NOT_RUNNING`. This is an
expected sequencing block, not evidence of a broken IMU or DDS graph.
The source container uses `--rm` and disappears when its 75 s window
ends. No extra calibration capture is needed.

In terminal 1, with the physically isolated motor rails and all
unchanged Stage 4A guards satisfied, run:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
LEROY_STAGE4_IMU_IMAGE=localhost/leroy-ros:stage4b-imu \
LEROY_STAGE4_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
bash ./live_stage4_imu_sidecar.sh
```

Keep terminal 1 running. As soon as its output says
`STAGE4A_IMU_START_ACK=PASS`, run this in terminal 2 **before**
`STAGE4A_IMU_STOP_ACK=PASS`:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
bash ./live_boot_yaw.sh
```

The yaw sidecar runs ~25 s and prints a live result before the 75 s
source window finishes. Look for:
`BOOT_LIVE_YAW_JSON={...}`,
`BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY`,
`BOOT_LIVE_MOTION=BLOCKED_NO_ACTUATION`.
This is **real IMU integration**, not a simulated input, but a
stationary observation alone cannot prove yaw accuracy under rotation
or authorize motors.

**Isolation caveat:** the existing M18 Stage 4A source runner still
requires motor rails physically disconnected while MCU logic and
sensors operate. The whole-robot E-stop **cannot satisfy that
condition while simultaneously powering the IMU**; don't claim it
does. If there is no separate means of isolating motor power, do not
misrepresent the condition or bypass the old runner. A new reviewed
commissioning path must instead reflect the actual hardwired cutoff
and independently bound any unexpected movement.

## Continuous live IMU + relative yaw (supervised sensor-only service)

The previous 75-second M18 capture and 25-second yaw diagnostic have now
been complemented by **one continuous sensor-only operation**. The
independent STM32 IMU publisher retains its single serial writer and
the already accepted M18 message parser/time mapper; its only MCU
writes are `set_imu_stream_hz 50` (START) and
`set_imu_stream_hz 0` (STOP). The yaw observer runs in a **separate
no-device sidecar** and can publish status continuously at 5 Hz.
Both join the existing isolated Stage 3 graph.

The one new operating command is:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
git fetch origin feature/live-boot-persistent-imu-yaw
git switch --detach origin/feature/live-boot-persistent-imu-yaw
LEROY_STAGE4_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
LEROY_STAGE4_MOTOR_ISOLATION_ATTESTED=YES \
bash ./live_boot_session.sh
```

**Only attest physical isolation if the motor/manipulator supply is
actually disconnected independently of the powered MCU.**
The operator's whole-robot E-stop cuts *all* downstream power and
cannot itself meet the simultaneously powered-MCU / disconnected-motor
condition; no software flag proves a hardware separation. If the
motor power is not separately disconnected, do **not** run this script.

The command refuses concurrent Stage4A/boot source ownership, root
containers, non-isolated Stage 3 namespaces, changed MCU by-id device
and unexpected/duplicate ROS motor graph participants. It prompts for
`BOOT_IMU_ONLY`, then starts the source and yaw together; it has no
75s timeout or 25s probe deadline. It prints
`BOOT_LIVE_SESSION=IMU_STREAM_READY` when the MCU START command is
acknowledged and
`BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY` when heading is first
established. Keep the command running to retain the live heading feed
for downstream read-only boot status consumers.

Stop with Ctrl-C. The launcher asks the MCU for STOP, checks its ACK
and removes the source container. If the source fails, yaw is lost,
and the session does not restart by itself. If the STOP ACK is absent,
treat the motor-capable MCU state as unverified and isolate its
power. The continuous stream is **not** an actuation or navigation
authorization; the startup sweep core's real physical-release and
obstacle-coverage evidence remain unset.

The earlier `live_stage4_imu_sidecar.sh` and
`live_boot_yaw.sh` 75s/25s acceptance recipes remain unchanged
unless explicitly run in persistent mode by the new launcher. No
additional fixed-length capture or synthetic motion study is needed
to exercise this path.

## Shortest path to a physical autonomous yaw sweep

1. **Identify the deployed drive/motor safety interface**, not old YAML guesses: current independent cutoff and E-stop mechanism, motor safe-state and test evidence, command owner, motion protocol, and what the wheel/heading feedback actually is. Record verified hardware facts in a separate current configuration, not as magic `True` booleans. No implicit enable.
2. **Closed-loop yaw before VIO:** provide a verified monotonic, robot-frame heading source from actual IMU/wheel sensors, accounting for the measured stationary gyro offset and drift. It does not need metric XRSLAM initialization to observe yaw, but heading accuracy, timestamp and stop behavior must be qualified.
3. **Obstacle coverage:** a forward monocular DepthART cloud (currently sparse and with stale intervals) does not prove that the robot's *entire swept radius* is free. Confirm measured footprint—including manipulator—and all-round sensing or an equivalent independently qualified protective means. Treat unknown/old data as occupied.
4. **Commission a supervised, bounded yaw-only turn:** validate independent stop, effective motor command path, yaw direction, overshoot, speed/stop response and sensor timeout behavior with physical safeguards; do not assume ±90° is inherently safe.
5. **Unattended boot:** only after the supervised motion result and full clearance policy are accepted. A yaw sweep alone may not initialize current monocular XRSLAM, which requires translational parallax. An autonomous translation needs its own verified safe corridor and navigation authority.

A new no-device simulation is not the next milestone: the pure supervisor (17 tests) and live camera/depth/XRSLAM ingestion have already passed.

## Repository maintenance policy

- **Current, live and directly runnable**: `live_stage3_start.sh`, `live_stage3_monocular_sidecar.sh`, `live_boot_readiness.sh`, `boot_localization_core.py`, the commissioned Stage 4C capture scripts, corresponding active source packages/configs. Explicitly check deployment branch/image and operator observations before treating any source as running.
- **Historical evidence to preserve, not execute as the next recipe**: `M*`, `m*`, old `utils/imu` ChArUco and MCU bridge-era scripts, stale robot-container / old `/dev/video6` instructions. They document earlier tests and decisions, and some contain commands that assume superseded device mapping or motor controls.
- **Archive candidates only after dependency and safety review**: old selftests/fixtures and dead deployment documentation. Do not bulk-delete or automatically alter safety-critical code, firmware or command protocols merely because names are old.

Next maintainer action is **hardware architecture reconciliation**, not re-calibrating deprecated encoder signs or inventing GPIO pin offsets.
