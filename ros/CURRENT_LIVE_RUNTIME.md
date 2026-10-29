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

### Clean `main` rebuild and physical Stage 3 acceptance (2026-10-29)

The operator used `/home/kevin/Code/leroy-clean-main/ros` after the
repository cleanup and reported both:

```text
LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED
LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS
```

The detailed live output confirmed the **host /dev/video8 → container
/dev/video2** mapping; both Nav2 costmap lifecycle transitions; two
DepthART/costmap probes that reached `READY` and `PASS` with real
optical clouds, costmap grids, lethal cells and unknown cells; and the
XRSLAM process with `tracking_status=initializing` and
`localization_valid=false`. No `/odom` message was observed in the
launcher’s six-second check. Transient `WAITING` states for the depth
publisher resolved to `READY`.

This demonstrates **successful fresh-checkout construction and a live,
nonactuating Stage 3 sensor runtime**. The source is the operator's
terminal output; it is not an independently captured hardware trace.

### Clean `main` real STM32 gyro/calibration acceptance (2026-10-29)

Using the same fresh-checkout Stage 3 graph, the operator started
`live_boot_session.sh` and reported:

```text
BOOT_LIVE_SESSION=STARTING_M18_SOURCE
BOOT_LIVE_SESSION=IMU_STREAM_READY
BOOT_LIVE_YAW_LAUNCH=STARTING_REAL_MCU_GYRO_DDS_ONLY
BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY
BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY
```

The emitted `leroy_auto_imu_calibration_v1` status used
`sensor_input_kind=physical_stm32_50hz` and
`source=live_ros_m18_imu`; it reported:

| Observed field | Value |
| --- | ---: |
| `samples` | 401 |
| `window_elapsed_s` | 8.0 |
| `window_restarts` | 0 |
| `gyro_z_noise_std_rad_s` | 0.002388587265975862 |
| `heading_bias_rad_s` | -0.0015555555555555557 |
| `independent_bias_rad_s` | -0.0013765586034912719 |
| `bias_disagreement_rad_s` | 0.0001789969520642838 |
| `split_bias_delta_rad_s` | 0.000023059701492537206 |
| `yaw_drift_deg` | 0.08161474076119325 |

The state was `VERIFIED_STATIONARY_CANDIDATE` with the explicit
reason `STATIONARY_GYRO_CONSISTENCY_PASSED_ONLY`. As designed,
`absolute_heading_validated`,
`physical_heading_accuracy_validated`,
`camera_imu_extrinsics_validated`,
`physical_motion_authorized`, and `motor_motion_authorized`
were all false.

### Clean `main` completed IMU-stream shutdown acceptance (2026-10-29)

The operator then interrupted the same running session with Ctrl+C.
The actual reported terminal output concluded:

```text
BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY
BOOT_LIVE_MOTION=BLOCKED_NO_ACTUATION
BOOT_IMU_SOURCE_START_ACK=PASS
BOOT_IMU_SOURCE_STOP_ACK=PASS
BOOT_IMU_SOURCE_RESULT=PASS_OPERATOR_STOPPED
BOOT_LIVE_SESSION=STOPPED_NO_MOTION_AUTHORITY
```

The last `leroy_boot_yaw_v1` status was `HEADING_ACTIVE`, with
`accepted_samples=55743`, `yaw_relative_deg=0.5137217008341133`,
`last_stamp_delta_s=0.019999159000000002`, and
`motor_motion_authorized=false`. These are real IMU stream and
relative-heading observations, **not** independently validated absolute
orientation accuracy.

**Scope:** The operator has now demonstrated fresh-checkout image
reconstruction, physical Stage 3 camera/depth/costmap/XRSLAM diagnostics,
physical STM32 IMU START, real eight-second stationary gyro
verification, and a successful IMU stream STOP acknowledgment on
supervised shutdown. The source is the operator-supplied terminal logs.
This is a **sensor-stream STOP**, not a verified wheel or power cutoff,
motor-driver braking response, collision protection, metric SLAM
localization, or Nav2 autonomous motion. The signed motor bridge
remains uncommissioned.


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

Prerequisites: the existing guarded, isolated `leroy-live-costmap` and `leroy-live-xrslam` Stage 3 containers are already running. Use the **clean current main checkout** on the Arch host:

```bash
cd /home/kevin/Code/leroy-clean-main/ros
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

### Live acceleration fault and bounded heading snapshots

The operator confirmed the persistent session started successfully on
real hardware: `BOOT_LIVE_SESSION=IMU_STREAM_READY` and
`BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY`. A later attempt to
sample manually rotated yaw hung in `ros2 topic echo --once`: the
heading estimator had faulted, ending its publisher. The session
correctly received `BOOT_IMU_SOURCE_STOP_ACK=PASS` from the MCU.
This is not yet a physical 90-degree yaw-accuracy acceptance.

The reported heading fault was `ACCEL_MAGNITUDE_IMPLAUSIBLE` after
11,504 IMU samples. Previously the estimator enforced a
**stationary-only** near-gravity acceleration magnitude during
the entire active yaw session. Actual manual rotation can generate
accelerations outside that window; the precise failing IMU sample
was not logged, so its exact cause remains unproven.

The estimator now requires near-gravity measurements while
calibrating *stationary gyro bias*, but permits plausible dynamic
acceleration once relative heading has been established. Grossly
invalid acceleration, nonfinite sensor data, gyro faults, active
timestamp gaps, stale IMU and source-ownership faults still latch.
The reported `last_accel_norm_m_s2` and `dynamic_accel_samples`
make motion transients visible without falsely presenting calibrated
heading accuracy.

A separate read-only one-shot query now has a built-in 6-second
timeout; it never hangs when the yaw publisher stops:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
bash ./live_boot_yaw_snapshot.sh
```

Run the snapshot before and after a known, independently measured
*unpowered* manual rotation; compare the two
`BOOT_YAW_SNAPSHOT_JSON.yaw_relative_deg` values. The script
returns `BOOT_YAW_SNAPSHOT=BLOCKED_...` if no live source or fresh
heading exists. It has no MCU access or ROS command publisher.

### Operator-observed clockwise yaw sign (2026-10-29)

The operator took two **fresh live read-only yaw snapshots**, with a
manual approximately 90-degree **clockwise** rotation between them,
using the sensor-only service with motor command output absent:

| Phase | Relative yaw | Sample count | Freshness |
| --- | ---: | ---: | ---: |
| Before rotation | −0.0386634149° | 481 | 200.4 ms |
| After clockwise rotation | −91.0251943239° | 1070 | 106.0 ms |

Observed difference: **−90.9865309091°**. This establishes the
expected clockwise-negative ROS yaw sign and a plausible angle scale
under manual handling. It is **not a subdegree sensor-calibration claim**:
the reference was a user-reported manual ~90° turn rather than an
independently measured precision angle. Gyro-Z bias in both snapshots
was −0.0015275591 rad/s, and `dynamic_accel_samples` reached 16
without invalidating the second heading.

Both reported `BOOT_YAW_SNAPSHOT=PASS_FRESH_READ_ONLY` and
`state=HEADING_ACTIVE`. A STOP ACK for this specific run has not
yet been supplied; don't claim shutdown acceptance from these two
snapshots. Heading is verified for sign and approximate scale only;
physical motor stopping, all-around swept-space clearance and the
motion command authority remain uncommissioned.

The earlier `live_stage4_imu_sidecar.sh` and
`live_boot_yaw.sh` 75s/25s acceptance recipes remain unchanged
unless explicitly run in persistent mode by the new launcher. No
additional fixed-length capture or synthetic motion study is needed
to exercise this path.

## Live feedback-based yaw tracking — software integration

The real STM32 IMU was independently observed delivering 50 Hz data,
the continuous gyro-Z estimator produced `HEADING_ACTIVE`, and a
manually performed ~90° **clockwise** chassis turn gave a signed
relative-yaw change of **−90.9865°**. This is physical sign and
approximate scale confirmation, **not precision odometry calibration**.

The next code path, `boot_yaw_tracking_core.py`, accepts live
monotonic relative-heading observations and tracks the intended
three boot yaw waypoints **−90°, +90°, 0°** relative to the
heading at session start. It computes a proportional, bounded
`diagnostic_wz_rad_s` *proposal only*. It requires fresh,
single-owner live feedback, a stationary dwell at each target,
keeps within an explicit yaw envelope and latches on source
loss, stale data, invalid timestamps, overshoot, or deadline.

The ROS adapter `live_manual_yaw_witness.py` has exactly one
subscriber (the live `/leroy_boot/relative_yaw_status` topic),
**no publishers**, and no MCU or motor device access. The entrypoint
`live_manual_yaw_witness.sh` runs a separate isolated sidecar
against the already running persistent boot IMU/yaw session. The
local operator moves the *unpowered* chassis by hand; the software
witness records waypoint/error/dwell progress using real feedback.

For an **optional, supervised full-manual-sweep witness** with
motor power independently disconnected and the continuous
`live_boot_session.sh` service already started in another
terminal:

```bash
cd /home/kevin/Code/leroy-boot-sweep/ros
git fetch origin feature/live-supervised-yaw-control
git switch --detach origin/feature/live-supervised-yaw-control
bash ./live_manual_yaw_witness.sh
```

Move to −90° clockwise and hold still, then +90° relative to the
original heading (a 180° counterclockwise change) and hold still,
then back to 0° and hold still. The witness checks a roughly
0.5-second stable dwell inside ±3° and prints
`MANUAL_YAW_SWEEP=PASS_REAL_GYRO_WITNESS_NO_ACTUATION` only
if all three are observed within the bounded session.
`MANUAL_YAW_PROGRESS_JSON` prints target/error and the
non-actuating nominal rate proposal. Do not mistake that value
for safe PWM, a motor command, or an approved operating speed.
User interruption, stale yaw, source loss or an exceeded
envelope returns `MANUAL_YAW_SWEEP=BLOCKED_...`, with
`MANUAL_YAW_MOTOR=BLOCKED_NO_COMMAND_PATH`. This witness
never stops or modifies the independent IMU-source session.

### First live manual yaw witness observation (2026-10-29)

The operator turned the unpowered chassis clockwise and counterclockwise
several times while the new `live_manual_yaw_witness.sh` listened to
the real IMU-relative heading. The observer successfully changed from
`AWAITING_HEADING` to `TRACKING` on the expected single live yaw
source, with the first target near `-90.2°` (origin-relative target
`-90°`). Live feedback showed both direction signs and tracking
errors, including:

- `error_deg=-175.09`: observed yaw was approximately `+84.89°`
  relative to the IMU's session origin, opposite the requested clockwise
  first waypoint.
- `error_deg=-0.19`: observed heading was almost exactly at the
  first target, but measured yaw rate was `+0.416 rad/s`; the state
  correctly remained `TARGET_MOVING_NOT_SETTLED`.
- `error_deg=+4.57`: observed yaw was about `4.57°` past the target,
  outside the witness's `±3°` hold band.

No `SETTLING`, waypoint advance or `DONE` occurred in the
supplied progress lines. This is **real feedback ingestion and target
error validation**, not proof that the full three-waypoint sweep has
been completed. The `diagnostic_wz_rad_s` values are neither motor
commands nor physically commissioned speed limits.

In response, the witness progress now includes
`current_relative_deg` (relative to the first observed heading),
`target_relative_deg` (`-90`, `+90`, then `0`) and
`operator_action` (explicit clockwise/counterclockwise/hold
instruction). Progress updates every two seconds instead of ten
while the witnessed session is running. Its motor-control boundary
and all latched fault rules are unchanged.

**Motor commissioning is a separate engineering stage.**
The historical M22 signed bridge has a session-bound operator
rearm, sole serial writer and active command inhibit. It is
still coupled to a deprecated GPIO feedback assumption and
has no measured all-around obstacle clearance. The hardwired
whole-robot E-stop is real but not a remotely readable
software heartbeat: the running ROS application cannot
assume the E-stop has been exercised, nor that power restoration
releases new command authority. The numeric diagnostic rate
must remain disconnected from `/cmd_vel`, signed motion
envelopes and wheel PWM until the actual motor safe-state
interface, stopping distance and entire swept obstacle
envelope have been physically verified and reviewed.
The present read-only integration deliberately uses **no
fabricated safety flags** to pretend these checks passed.

## Automatic real-IMU calibration at session startup

The persistent real M18/yaw service now automatically performs an
**additional stationary gyro quality check** after it establishes
`HEADING_ACTIVE`. There is no second operator command or new MCU/USB
owner. It uses the existing 50 Hz STM32 IMU messages and the
existing live relative-yaw estimator in the same device-free ROS yaw
container.

`ros/boot_auto_calibration_core.py` collects at least 350 real IMU
samples in a **continuous eight-second stationary holdout** after the
initial 2.5-second gyro bias. It checks:

- magnitude of gravity and low gyro rate during that holdout, resetting
  the incomplete window when the chassis is moved (at most four resets);
- consecutive timestamp and receipt continuity, with regression faults;
- gyro-Z sample standard deviation and consistency between first and
  second halves of the stationary holdout;
- independent *later-sample* gyro-Z mean versus the startup bias; and
- yaw drift across the stationary interval.

If the observed measurements fall within **provisional diagnostic
thresholds**, the existing persistent yaw process emits

```text
BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY
BOOT_AUTO_CALIBRATION_JSON={...}
```

and publishes structured JSON `std_msgs/msg/String` on
`/leroy_boot/auto_imu_calibration_status` in ROS domain 170.
The existing read-only `live_boot_readiness.py` now consumes this
topic from the single expected live yaw node, verifies recent status,
minimum sample/window evidence and non-authorizing fields, and reports
`auto_imu_stationary_candidate_observed` plus measured
`auto_imu_stationary_candidate` metrics. It still reports
`source_calibration_validated=false` and
`physical_motion_authorized=false`; an 8-second gyro holdout is not
camera–IMU calibration or physical motion release.
This separate status never sets `physical_heading_accuracy_validated`,
`absolute_heading_validated`, `camera_imu_extrinsics_validated` or
`motor_motion_authorized` to true. If Leroy moves too early, the
calibration is withheld/reset rather than filled with hypothetical
stationary samples. Invalid stamps or repeatedly fragmented windows
latch a calibration-specific fault.

**To use it**, fetch `feature/autonomous-imu-calibration` and start
the **same** already commissioned `live_boot_session.sh` with the
existing physically disconnected motor rails. Remain stationary
during the initial automatic holdout (~11 seconds from first valid
IMU acquisition). The stream keeps running afterwards. No new
manual ±90° witness or stationary capture campaign is required.

This is **autonomous stationary sensor calibration/verification**, not
an autonomous powered chassis calibration turn. The successful
operator-observed approximately 90° clockwise manual turn provides
a yaw sign/rough scale reference but cannot calibrate actual wheel
speed, PID gains, stopping distance, camera–IMU clock phase or stereo/
monocular metric scale. Those require their own measured physical
signals. The current deprecated GPIO interlocks must not be spoofed:
the user-reported whole-robot hardwired E-stop remains the physical
cutoff, and returning power never grants a motor session. Until
the independent power-stop, complete swept-space observation and
exclusive signed motor controller are integrated against the actual
robot, the auto-calibration result is **not a motion permit**.

## Motor-capable autonomous yaw calibration (integrated signed-controller code)

The **actuating controller code** now exists on
`feature/actuated-autonomous-yaw-calibration`. Unlike earlier manual
witnesses, `live_actuated_yaw_calibration.py` can produce
**nonzero signed motor commands** to the pre-existing M22
`/leroy/motor/signed_motion` ingress, after the separately configured
bridge acknowledges a valid signed session rearm. It tracks the
whole −90°, +90°, return-to-zero yaw sequence from the physical
`/leroy_bridge` IMU stream, performing its own automatic stationary
gyro calibration and eight-second holdout first.

**One MCU owner:** the motor-capable `leroy-signed-bridge` is the sole
serial writer. The earlier separate `leroy-boot-imu-source` and
`leroy-stage4-imu` M18 publisher must not be running. The new
actuating controller opens **no** STM32 serial device. It subscribes
to the bridge's `/imu/data_raw`, protected motor session ID and a
separately issued safety lease, then sends HMAC-signed
`/leroy/motor/signed_rearm` and `/leroy/motor/signed_motion`
messages. The M22 bridge retains independent MCU watchdog, wheel
packet sequencing, zero and STOP acknowledgments, key custody and
its own physical interlocks. The bridge now also publishes the actual
`/leroy/motor/authority_status` from its internal `MotorAuthority`
evaluation, including the current session, `ARMED` state,
`physical_release_verified`, MCU watchdog acknowledgment, CDC health,
and fresh monotonic stamp. The controller **requires** this fresh,
bridge-owned state after operator rearm; elapsed time alone never
enables a signed nonzero command. There is **no raw /cmd_vel fallback**.

**Continuous independent physical safety lease:** the new
`autonomous_motor_yaw_core.py` accepts `leroy_swept_safety_lease_v1`
only from `/leroy_physical_safety_monitor` over
`/leroy/safety/rotation_lease`, and validates an HMAC signed with a
third protected, *distinct* monitor key, current motor-bridge session
ID, strictly increasing sequence, at most 150ms freshness, whole
swept footprint measured/observed/clear, independently proven stop,
measured braking distance with positive clearance margin, unique
motor owner, and physical motor output interlock. The controller
contains **no lease signer or fallback self-assertion**; ROS graph
identity alone does not grant motion. Stop-completion requires an
independently reported stop state and settled IMU heading, with a
signed zero command and latched shutdown after completion. Velocity
limits and rate slew are capped in software but must also remain
within the safety monitor's physically measured operating envelope.

A 20Hz command tick which sees no *new* IMU heading sends
signed zero, rather than driving on a repeated timestamp. The
controller also latches if measured yaw rate exceeds the rate
authorized by the physical monitor. Stop release requires sustained
zero output and independently reported wheel/chassis stopping.

The rootless entrypoint `live_actuated_yaw_calibration.sh` requires:
a live, separately commissioned bridge container named
`leroy-signed-bridge`; a physically independent, authenticated
monitor named `leroy-physical-safety-monitor`; the isolated Stage 3
parent; three distinct rootless-owned protected role keys; and
explicit local one-session operator approval. It refuses simultaneous
M18 MCU source containers, and its own Podman container has no MCU
or camera devices. It cannot replace the hardware E-stop, which
the operator reports as a whole-robot power cutoff, nor should power
restoration automatically rearm the motors.

**Current physical deployment remains BLOCKED.** Leroy has not
demonstrated a commissioned all-around safety monitor, a measured
whole-chassis swept footprint, qualified stopping distance,
motor-bridge motion response, or a validated bridge-local interlock
replacement for its deprecated GPIO status assumptions. Camera-only
DepthART plus an unknown/partial costmap does not satisfy the
independent 360-degree lease contract, and the approved M18
`IMU_START/STOP` stream cannot share serial with the motor bridge.
The code is now connected through the existing **real signed motor
ingress**, not through fake motion or a diagnostic-only output, but
under today's hardware conditions it must **refuse arming**.

This software may be reviewed and regression-tested without
changing the active Stage 3 setup. Do **not** launch the powered
calibration with dummy safety-monitor key material, a spoofed ROS
publisher, invented GPIO pins, or motor power connected while using
the old isolated IMU diagnostic runner. A human-typed
`ARM_PHYSICAL_AUTONOMOUS_YAW` approves a single reviewed session;
it is not itself an independent collision-stop sensor.

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
