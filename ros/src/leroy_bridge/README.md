# leroy_bridge

ROS 2 Jazzy serial bridge for Leroy's MCU. It owns **hardware telemetry and
motor commands**, not the robot's normal odometry or sensor TF.

Publishes `/imu/data_raw` (`sensor_msgs/msg/Imu`, best effort, depth 10,
`imu_link`, nominal 100 Hz) and `/wheel/encoder_counts`
(`leroy_msgs/msg/EncoderCounts`, best effort, depth 10). Subscribes to
`/cmd_vel` (`geometry_msgs/msg/Twist`, reliable, depth 10). The MCU
timestamp/sequence fields are preserved; right-encoder values are never
fabricated. Raw IMU measurements use +X forward, +Y left, +Z up, rad/s and
m/s², with unavailable quaternion orientation
(`orientation_covariance[0] = -1`).

## Odometry ownership

The default configuration sets `degraded_odometry_enabled: false` and
`publish_odom_tf: false`. The node defaults match. When degraded odometry is
disabled, the bridge **does not create a /odom publisher or a dynamic TF
broadcaster**. The separate `leroy_xrslam` adapter owns `/odom` and dynamic
`odom -> base_link`.

The historical FL/RL-encoder plus IMU-yaw estimator remains source-available
only as a deliberately enabled diagnostic fallback. Never run it alongside
XRSLAM; enabling it does **not** happen automatically after tracking loss.

## Supervised low-duty motor testing and motor interlock

The ROS node's default `pwm_full_scale_wheel_velocity_rad_s: 0.0`
**disables motor output** when no explicit configuration is supplied.
The robot YAML now opts into a **provisional**, supervised low-duty motor
test with `pwm_full_scale_wheel_velocity_rad_s: 20.0` and `max_pwm: 64`.
The value `20.0` is an *unmeasured conversion reference*: wheel rad/s
corresponding to PWM 255, **not a measured physical speed limit**. At
0.16 rad/s commanded chassis yaw, it produces approximately +/-46 PWM
at the wheels with Leroy's configured geometry. The 64/255 PWM cap is an
independent duty limit, not a guaranteed chassis-speed limit. A
**smaller** full-scale wheel velocity makes a given `/cmd_vel` produce
**more** duty; it is not a safer/lower speed setting.

The bridge scales all four wheel commands together if they would exceed
`max_pwm`, retaining mecanum velocity ratios. NaN/inf velocity commands
are rejected. Do not operate without a verified hardware E-stop, a cleared
supervised area, and first checking wheel direction and actual motion at
low duty. Low PWM can stall the motors, and loaded speed can differ from
the conversion reference. Observe the chassis, adjust only based on
measurements, and immediately stop on anomalous behavior.

The bridge retains `watchdog_ms: 500`, `keepalive_period_s: 0.10`,
`cmd_vel_timeout_s: 0.25`, periodic command timeout checks and a stop on
shutdown. The automatic temporal calibration script additionally refuses
to arm if `max_pwm > 64` or if its watchdog/timeout checks fail. Its
`--arm` mode still requires both explicit physical safety confirmations
and supervises visual tracking; a working camera does **not** replace
collision avoidance. Wheel radius (0.020 m), half-wheelbase (0.180 m),
and half-track (0.275 m) remain necessary for the motor-control kinematics
even with odometry disabled.

This is a **source configuration change**, not a live ROS parameter update:
rebuild and restart the bridge before the new mapping takes effect.
Merely changing the YAML or running `ros2 param set` will not update
the bridge's cached motor conversion fields.

Run the ROS-independent regression tests before hardware trials:

```bash
python3 -m pytest ros/src/leroy_bridge/test/test_motor_pwm.py \
  ros/src/leroy_bridge/test/test_xrslam_contract.py -q
```

## MCU-to-ROS timestamp mapping

The STM32F401 generates uint32 microsecond timestamps using SysTick, derived
from its nominal 84 MHz system clock. Its crystal and the host's clock can
differ in rate. A six-minute stationary-IMU test observed the receipt-minus-
header offset moving approximately **-2.5 ms/minute** (roughly 42 ppm of
relative clock-rate mismatch), with the original one-time offset mapper.

The bridge now uses `leroy_bridge.clock_sync.McuClockSync`, a testable
ROS-independent clock mapper:

- Unwrap the MCU's uint32 microseconds, including the normal ~71.6-minute
  rollover, while detecting reordered samples, MCU restarts and long gaps.
- Select low-transport-delay observations within 5-second bins, fit a robust
  affine *rate* using up to four minutes of receipt timestamps, and update
  the estimated host/MCU rate gradually. Rate estimates exceeding ±200 ppm
  repeatedly result in an explicit clock fault instead of an unbounded fit.
- Integrate the estimated rate over each new MCU interval rather than
  retroactively reprojecting the entire uptime when the estimate changes.
  Preserve monotonic timestamps and identical stamps for a shared MCU tick.
- Detect host ROS-clock steps relative to the system monotonic clock.
  Discontinuities stop sensor publication, command zero wheel speed,
  withhold keepalives so the MCU's 500 ms watchdog can stop motion if the
  stop packet is lost, and require a supervised bridge restart.
  No automatic VIO reinitialization.
- Log the estimated host-relative MCU clock-rate correction in ppm once the
  rate estimator has enough observations and roughly once per minute later.

### Limitations

**One-way USB serial receipt is not a clock synchronization handshake.**
Its unknown/variable delivery delay prevents inferring a verified *absolute*
MCU-to-camera offset. The first receipt is anchored with a conservative
5 ms transport guard; this does **not** prove that samples were acquired
exactly 5 ms before receipt. Rate integration does not subsequently correct
this initial phase offset. Crystal drift and host-clock discipline remain
distinct from physical sensor time offset.

Do **not** use the rate correction to bypass XRSLAM's camera–IMU
synchronization, measured extrinsics, pose direction, or loss-of-tracking
checks. Keep `use_xrslam:=false` and motors disabled until hardware
validation. A future MCU timestamp echo / two-way exchange could measure
absolute clock offset more accurately.

### Testing

ROS-independent regression tests:

```bash
cd /leroy_ws
source /opt/ros/jazzy/setup.bash
source /leroy_ws/install/setup.bash
python3 -m pytest src/leroy_bridge/test/test_clock_sync.py -q
```

The tests simulate 360 seconds of a 42 ppm mismatch, a 90-minute run with
an estimated rate change and uint32 timer rollover, serial jitter, out-of-order
timestamps, a firmware reboot, large gaps and a host wall-clock step. They do
**not** replace a fresh six-minute real STM32 clock
audit, especially with a concurrent camera capture.

After rebuilding the bridge, check `ros2 topic delay /imu/data_raw -w 30`
and the log line `MCU clock-rate estimate` over six minutes. We expect
the *trend* in delay to flatten after the estimator converges; a nonzero
constant receipt-minus-header delay is not necessarily a fault. If the
bridge reports a clock discontinuity, do not restart XRSLAM or command the
motors without inspecting the underlying cause.

See `ros/XRSLAM_INTEGRATION.md` for the staged live checks.
