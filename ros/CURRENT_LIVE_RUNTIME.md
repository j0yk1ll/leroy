# Leroy — current integrated live runtime

**Maintained integration index (2026-10-29).** The earlier Stage 3
(camera/depth/XRSLAM) and Stage 4 (STM32 IMU/yaw) milestones are
**complete as sensor-integration work**. They are no longer separate
operating modes or separate operator workflows. There is **one
non-actuating Leroy runtime**, with a single command:

```bash
cd /home/kevin/Code/leroy-clean-main/ros
LEROY_MOTOR_RAILS_DISCONNECTED=YES \
LEROY_MCU_BY_ID=/dev/serial/by-id/usb-Robot_Controller_STM32F401_Robot_CDC_F401-ROBOT-if00 \
bash ./leroy.sh run
```

To validate or rebuild a fresh checkout without connecting devices:

```bash
bash ./leroy.sh check
bash ./leroy.sh build
```

The integrated launcher **does not enable autonomous navigation or
motor motion**. Component separation *inside* the runtime remains
necessary because the camera and STM32 serial port require different
physical-device owners and explicit fault-containment boundaries.
Legacy Stage-named wrapper scripts and OCI image tags are retained as
implementation details to preserve the real hardware-validated
startup path and offline regression tests. Do not run those wrappers
as parallel active sessions.

## One runtime and one command boundary

1. The host Arch process checks its rootless Podman environment,
   the verified front camera and stable STM32 identity, operator
   motor-rail isolation attestation, local images and session name
   collisions.
2. The isolated `leroy-live-costmap` container is the only camera
   owner. Host `/dev/video8` is mapped to container `/dev/video2`.
   It runs the rectifier, CPU DepthART optical point clouds, and
   diagnostic Nav2 rolling costmap with provisional static TF.
3. The `leroy-live-xrslam` container joins the same isolated DDS
   network, IPC and PID namespace without any hardware device map.
   It consumes the existing camera; XRSLAM remains monocular,
   `initializing` with metric localization invalid.
4. After the existing typed `BOOT_IMU_ONLY` consent, the
   `leroy-boot-imu-source` is the **only** MCU USB CDC owner.
   It supplies the actual 50 Hz `/imu/data_raw` source. The
   `leroy-live-boot-yaw-persistent` container subscribes without
   mapping the MCU, estimates the relative gyro Z bias and heading,
   then performs its independent eight-second stationary holdout.
5. Ctrl+C stops the foreground gyro observer and causes the IMU
   source to attempt `set_imu_stream_hz 0`, requiring
   `BOOT_IMU_SOURCE_STOP_ACK=PASS` for a clean session result.
   The unified wrapper then stops XRSLAM and the parent camera
   graph in reverse dependency order. Unexpected surviving MCU
   owners fail the wrapper and trigger an attempted cleanup.

The single operator workflow is **attended and sensor-only**. The
hardwired E-stop cuts downstream robot power, but neither an
environment flag nor the ROS process can electrically verify that
motor-driver rails have been independently isolated. They must be
physically disconnected while the motor-capable STM32 USB interface
remains powered. The integrated launcher refuses an already-running
motor bridge or duplicate IMU publisher.

## Clean checkout: real hardware evidence

The operator rebuilt the image family from an independent clean
`main` checkout on October 29, 2026:

```text
LEROY_CURRENT_IMAGES=BUILT_FROM_CHECKOUT_NO_HARDWARE_LAUNCHED
```

The original component-level integration runners then produced:

| Component | Operator-observed result |
| --- | --- |
| Camera | Host `/dev/video8` to container `/dev/video2`; rectified camera stream |
| DepthART | Real optical clouds received in both acceptance probes |
| Diagnostic costmap | Both lifecycle transitions passed, active, lethal and unknown cells observed |
| XRSLAM | Live node, `initializing`, `localization_valid=false`; no `/odom` in 6s |
| Integrated camera/depth status | `LIVE_STAGE3_START=PASS_CAMERA_ONLY_CONCURRENT_DDS` |
| STM32 gyro stream | `BOOT_LIVE_SESSION=IMU_STREAM_READY` |
| Heading | `BOOT_LIVE_YAW=PASS_RELATIVE_GYRO_HEADING_ONLY` |
| Stationary holdout | `BOOT_AUTO_CALIBRATION=PASS_STATIONARY_GYRO_CONSISTENCY_ONLY` |
| MCU stream start | `BOOT_IMU_SOURCE_START_ACK=PASS` |
| MCU stream stop | `BOOT_IMU_SOURCE_STOP_ACK=PASS` |
| Session termination | `BOOT_IMU_SOURCE_RESULT=PASS_OPERATOR_STOPPED`; `BOOT_LIVE_SESSION=STOPPED_NO_MOTION_AUTHORITY` |

The gyro holdout used 401 actual STM32 samples at 50 Hz over eight
seconds without restarts, with reported relative yaw drift
0.08161474076119325°, bias disagreement 0.0001789969520642838
rad/s and gyro-Z noise 0.002388587265975862 rad/s.
The completed yaw session observed 55,743 accepted IMU samples and
reported `yaw_relative_deg=0.5137217008341133` on exit.
This relative heading is **not** validated absolute orientation.

**Evidence boundary:** The results above come from the operator's
actual terminal output, not an independently archived hardware
trace. They establish the clean checkout's source/image, sensor
startup, stationary calibration and MCU **IMU-stream STOP** behavior
via the original verified launchers. The new unified `leroy.sh run`
orchestration still needs one supervised on-robot acceptance to
demonstrate the same sequence as a single foreground command. The
IMU STOP acknowledgment is not proof of physical wheel stop.

## Remaining engineering work, not another stage

The *sensor-integration milestones are complete*. That does not
mean localization or autonomous physical movement is complete.

- **Metric localization:** XRSLAM currently runs in camera-only
  monocular mode and remains `initializing`. Camera/IMU timing,
  extrinsics, scale and trustworthy metric odometry are not
  commissioned. Pure chassis yaw may not generate enough
  translational parallax for monocular initialization.
- **Obstacle protection:** DepthART's front-facing camera does not
  independently observe every side or the complete yaw-swept
  physical footprint. Provisional depth/costmap geometry is
  diagnostic and not collision-avoidance clearance evidence.
- **Motor bridge:** The historical M22 signed bridge and newer
  `live_actuated_yaw_calibration.py` implement possible signed
  motor commands but **are not physically commissioned**. The old
  bridge expects unconfigured GPIO feedback and the experimental
  motor calibration launcher requires a separate uncommissioned
  physical safety service. Do not synthesize healthy GPIO values or
  inject synthetic permits. A physical hardwired cutoff is present
  but motor stop response and automatic fault handling remain to
  be measured.
- **Real navigation:** `ros/src/leroy_nav2/config/hardware.disabled.yaml`
  remains disabled; synthetic MPPI/odom tests do not authorize
  real `/cmd_vel` or wheel PWM. Only an exclusive, commissioned
  motion owner may eventually connect navigation output.

## Deprecated assumptions and historical evidence

Old Stage 3 / Stage 4 documentation used distinct startup
procedures, individual test images and milestone-specific labels.
That separation is **historical**, not the present architecture.
The retired per-stage CI tests and scripts remain as
backward-compatible regression contracts where still required;
their execution is not a separate operator procedure.

The deprecated `motor_gpio_chip`,
`motor_estop_feedback_offset` and
`motor_power_feedback_offset` placeholders are not present in the
active bridge YAML. The legacy `PhysicalMotorFeedback` source still
requires actual hardware feedback when its signed mode is
provisioned. This gap is not resolved by faking a GPIO, and does
not affect the accepted sensor-only runtime.

Current read-only readiness is available via
`ros/live_boot_readiness.sh` against the already running camera
graph. The read-only yaw snapshot remains a diagnostic, not an additional startup phase.
The standalone manually rotated yaw witness was retired as an optional
experiment; the supported yaw sensor runtime is unchanged.

For original milestone investigations and scripts, retrieve the
unmodified `archive/full-runtime-history-2026-10-29` snapshot.
The former `LIVE_STAGE3_REPRODUCIBLE.md` is historical evidence
in that archive. Follow `README.md` and `leroy.sh` for the
current integrated workflow.
