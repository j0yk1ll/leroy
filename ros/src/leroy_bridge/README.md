## Observed A/D strafe direction on Leroy

During attended motor commissioning, the operator confirmed that the
original nominal **A** pattern moved Leroy right and **D** moved it left,
while forward/backward movement worked correctly. The shared manual
commissioning direction table has been calibrated by exchanging only
the A/D vectors. This affects **both manual pulse and evdev hold modes**.

Manual controls are now A=left, D=right; W/S and Q/E are unchanged.
This does NOT silently modify firmware platform presets or production
ROS `mecanum_twist_to_pwm` kinematics. Before production lateral driving,
verify the robot's physical (+Y) convention and correct ROS/MCU mapping
with measured sensor feedback separately.

## Experimental high-duty mecanum commissioning (STM32 reflash required)

The platform PWM uses an 8-bit duty value: **255** is maximum, and
**256 is invalid**. The production `set_wheels_tx` MCU protocol retains
its original strict **±64** ceiling. Supervised keyboard commissioning
can now request up to **±255**, using a *different* MCU command named
`set_wheels_calibration_tx` with a watchdog of **at most 200ms**.

For any `--pwm` above 64, add `--experimental-high-pwm`. The keyboard
hold is limited to **500ms per press** (release fully before the next
press), and the MCU ramp is raised modestly to 8 duty/10ms. This is a
calibration-only command, NOT a new production signed-authority path,
and is **NOT obstacle-aware**. A hardware cutoff is still required.
High duty can cause hazardous stall currents, driver overheating and
sudden unexpected movement, especially on the ground. First check
physical cutoff, direction, wheel support, current draw and supply
droop. Start at 80 or 96 rather than jumping to 255; do not try
higher values on a loaded robot without confirming motor and driver
ratings and stopping behavior.

Both the ROS image **and the STM32 firmware must be updated**. On the
Arch host, after fetching the branch:

```bash
cd /home/kevin/Code/leroy/firmware
make build
# With independent motor rail cut, MCU SWD/ST-Link connected,
# and no normal ROS bridge/commissioner process:
make flash
```

`make` without a target also flashes; use `make build` when you only
intend to compile. Do not flash through a live motor session.

Sample high-duty flags for the otherwise unchanged Podman hold command:

```bash
--keyboard-device /dev/input/event2 \
--sequence-state-file /leroy-state/motor_sequence \
--pwm 96 --experimental-high-pwm \
--wheel-polarities +1,+1,+1,+1
```

Keep the `leroy-motor-sequences` Podman volume mounted. The low-duty
(1..64) controller retains the production `set_wheels_tx` protocol,
and the discrete one-shot SPACE+pulse mode stays limited to 64.

## Persistent motor sequencing (required for commissioning)

The STM32 rejects stale nonzero wheel transactions across USB reconnects.
The commissioning controller therefore **requires** `--sequence-state-file`
pointing at a durable host-backed volume. Each sequence number is written
and synced to disk before being sent. Keep this volume across Podman runs;
do not reset/delete it during normal testing.

One-time host setup:

```bash
podman volume create leroy-motor-sequences
```

Add the following to the tested Podman run command:

```bash
--mount type=volume,src=leroy-motor-sequences,dst=/leroy-state
```

Add this argument to `ros2 run ... leroy_motor_manual_commission`:

```bash
--sequence-state-file /leroy-state/motor_sequence
```

On the first run after migrating from the old timestamp-based controller,
the STM32 may still retain a higher accepted sequence than the new file.
In that case the MCU **must remain inhibited**. With motor power physically
cut off, separately power-cycle the STM32/USB, then retry. Do not advance
a rejected sequence blindly or disable the MCU stale-sequence check.

Keyboard release uses an integer `EVIOCGRAB` argument of 0; passing a
pointer to packed zero inadvertently repeated the grab and caused
`EBUSY` on exit. Closing the descriptor releases the grab in the kernel
even if explicit ungrab fails.

## Supervised hold-mode PWM calibration

The `--keyboard-device /dev/input/eventN` hold mode retains the original
32/255 default, but now accepts an explicit `--pwm` setting up to 64/255,
matching the existing STM32 `set_wheels_tx` firmware ceiling. No firmware
change, watchdog extension, production authorization change or encoder/SLAM
dependency is needed. At 64/255, duty is only 25% of full-scale, and PWM
duty is **not** a verified speed, torque or motor current limit.

Do not diagnose loaded wheel stall by immediately commanding maximum duty.
First check that the wheels turn in their expected directions on supports,
then measure supply voltage and motor current under load. A stationary motor
can draw high stall current; check motor/driver temperatures and cut power
on binding or electrical faults. For brief, carefully controlled trials
with the cutoff operator present, start at 40, then 48, then 56 and at most
64 (one step at a time, only after release-to-STOP is measured at each step).
Do not run autonomous or obstacle approach tests, or assume the supported
PWM range guarantees successful floor travel. If 64 cannot overcome static
friction, inspect supply droop, current limiting, gearing, roller alignment
and the robot mass instead of bypassing the MCU limit.

Example change to the previous command: `--pwm 48`.
Use the same tested `/dev/input/event2` keyboard and
`--group-add keep-groups` Podman group mapping.

## Attended ROS manual pulse commissioning (wheel-lifted only)

The separate `leroy_motor_manual_commission` executable uses ROS 2 for
**requested-PWM/status telemetry**, but accepts motion input **only from local
/dev/tty**, never from ROS `/cmd_vel` or a remote subscriber. It opens the
STM32 CDC exclusively and **must be run instead of `leroy_bridge_node`**.
Do not run the bridge and this tool together. This is a discrete one-shot
pulse interface, not continuous teleoperation, collision avoidance or a
replacement for the production signed authority.

After building and sourcing the Jazzy workspace, on the laptop, with all
wheels securely suspended and the independent cutoff physically proven to
interrupt motor rails even while USB powers the MCU:

```bash
ros2 run leroy_bridge leroy_motor_manual_commission -- \
  --port /dev/ttyACM0 --pwm 24 --duration-ms 150 \
  --sequence-state-file /leroy-state/motor_sequence \
  --wheel-polarities +1,+1,+1,+1
```

The four signs are **only provisional placeholders** until wheel-rotation
directions have been observed and recorded individually; do not infer that
these values produce the correct physical forward direction. The program
requires exact interactive `MANUAL_PULSE_READY` confirmation. Press SPACE,
then within 500 ms a single direction key: `w/s` forward/back, `a/d`
strafe left/right, `q/e` rotate CCW/CW; `x` requests sequenced STOP and
Ctrl-C exits. Every accepted key triggers at most one <=150 ms pulse
(max ±32/255 PWM); held keys do not provide continuous motion. MCU watchdog
is pulse duration +100 ms, motor acceleration is bounded using the firmware
ramp, and no keepalive is emitted. MCU STOP ACK does not verify physical
stopping. The operator must stay at the physical cutoff.

This stage deliberately requires no encoders, XRSLAM or DepthART. It does
**not** make floor driving collision-safe. Before floor driving, confirm
actual wheel polarity and kinematics, residual coasting, cutoff performance,
floor clearance, a genuinely supervised controller, and additional obstacle
protection. Status topics `/leroy/manual/requested_pwm` and
`/leroy/manual/status` describe **requests**, not measured wheel duty or
physical stop. The normal production bridge/config is unchanged.

For host-only tests:

```bash
cd ros
PYTHONPATH="$PWD/src/leroy_bridge" PYTHONDONTWRITEBYTECODE=1 \
  python3 -m unittest discover -s src/leroy_bridge/test \
    -p test_motor_manual_commission.py -v
```

# leroy_bridge

## PR #156 — complete software motor-control path (NOT hardware-commissioned)

This branch now contains **implementation**, not another ROS diagnostics
suite:

- `MotorAuthority`: session-bound signed rearm plus a one-second healthy
  dwell, non-replayed signed command, bounded mecanum four-wheel duty
  (maximum ±64), strict monotonic expiry and latched stop/fault behavior.
- `motor_signer.py`: separate `leroy_motor_controller_signer` subscribes
  to actual Nav2 `/cmd_vel` (`geometry_msgs/Twist`), rejects invalid
  inputs and signs commands with the controller key for the fresh bridge
  session. `leroy_motor_operator_grant` requires the physically present
  operator to enter `ARM_MOTORS` on `/dev/tty`, then signs a **separate**
  one-shot rearm request. Neither component can independently enable motors.
- `motor_key_provision.py`: explicit `leroy_motor_provision_keys`
  creates separate 32-byte cryptographic role keys under a pre-existing
  private, owner-only directory, without printing key material.
  The bridge reads both through O_NOFOLLOW, verifies size and ownership,
  and creates no motor ROS subscriptions by default.
- `motor_cdc_protocol.py` plus `firmware/src/command.c`: the host's
  **single** `_send_command` CDC writer issues
  `set_wheels_tx seq FL FR RL RR` only after a signed one-use decision
  and rechecks physical feedback immediately before writing. MCU
  `ok set_wheels_tx seq=N` acknowledgement must match exactly the one
  outstanding transaction inside 150 ms. A separate
  `stop_wheels_tx seq` overrides a pending motion ACK; the MCU zeros
  wheel duties, disables wheel-driver enable lines and responds
  `ok stop_wheels_tx seq=N`. Missing/mismatched ACK terminates
  the command session. **MCU command acceptance is not physical
  wheel-stopped confirmation.** Old open-loop movement commands and
  legacy nonzero `set_wheels` are disabled in the updated firmware.
- `motor_gpio_feedback.py`: read-only libgpiod-v2 feedback from
  TWO separate active-high inputs belonging to the independently
  hardwired motor-power cutoff: E-stop loop healthy and power
  contactor feedback. Required chip path/offsets are deliberately
  absent by default. No ROS parameter or GPIO software output
  substitutes for an independent power-disconnect circuit.
- `motor_sensor_health.py`: real, typed, freshness-limited data from
  XRSLAM `/xrslam/localization_valid` and
  `/xrslam/tracking_status`, metric `/odom`, dynamic `/tf`
  (`odom → base_link`), DepthART `/depthart/depth`,
  and `/local_costmap/costmap`. Absent/stale/malformed messages
  block authority. This is an input plausibility gate, **not proof
  that sensor timing, extrinsics, footprint or physical obstacle
  detection is calibrated**.
- MCU watchdog: the firmware boots with it disabled. A signed session
  requires a 1–500 ms watchdog ACK and cannot extend it. Bridge
  keepalive is permitted only while the signed command/authority is
  continuously fresh; lost controller input or fault stops keepalives.
  This is separate from the required independent electrical cutoff.

### First attended wheel pulse without E-stop GPIO telemetry

A **hardwired cutoff upstream of the motor fuse block can be physically
independent without providing a readable GPIO status line**. Do not invent
or force GPIO true values into the automatic ROS motor authority. The
unprovisioned production ROS bridge remains fail-closed.

For one attended, wheels-lifted bench commissioning pulse, the ROS bridge
MUST first be stopped so there is only one USB CDC controller. Run
`python3 -m leroy_bridge.motor_bench_pulse --port /dev/ttyACM0
--wheel fl --pwm 18` from the repo with
`PYTHONPATH=$PWD/ros/src/leroy_bridge` on a host with Python pyserial.
The script refuses noninteractive execution, insists on an explicit operator
confirmation of mechanical support and a *verified working hardwired
motor-supply cutoff*, and sends one sequence-numbered nonzero packet to
one wheel only (maximum ±48/255 duty, operator-selected 20–150 ms). It first requests MCU
driver disable, configures the bounded firmware watchdog (pulse duration + 100 ms) with an exact
ACK, never sends keepalive, and requests an exact sequence-matched STOP
ACK in both normal and exception paths. Failure or absent ACK is a stop
condition; independent power cutoff is always the final safeguard.
USB may continue powering MCU logic after motor cutoff opens.

This is **not** signed ROS/Nav2 actuation, not a bypass for unverified
autonomous readiness, and not evidence that wheels physically stopped.
The MCU watchdog zeroes commanded outputs only while MCU main loop
runs; the separate hardware cutoff is essential. Check the motor-rail
voltage with the cutoff operated *while the laptop USB is connected*,
ensure the wheel cannot reach the floor or an operator, and only then
run the one-shot pulse. Begin with `--wheel fl --pwm 24 --duration-ms 150` only if
independent motor cutoff has been physically verified with USB connected,
wheels are securely supported, and the previous 60 ms pulse showed only a
small twitch. If that barely moves, stop before trying higher duty and
record the supply/driver wiring and observations. The script can be run with motor supply
disconnected first as a communication/protocol rehearsal. It never
configures the production authority's E-stop or rail inputs.

## Operator configuration, intentionally absent

**No default robot activation, image switch, GPIO pin guess or
firmware flash is part of this PR.** Normal `leroy_bridge.yaml` has
empty motor key paths and no GPIO chip/offsets. Provision keys and
read-only physical feedback ONLY in a supervised, safety-reviewed
commissioning session. `python3-gpiod` with the libgpiod **v2** Python
API and access to the exact `/dev/gpiochip*` device are needed.
The actual emergency-stop contactor circuit must be built/wired
independently of ROS/Linux/STM32 and shown to interrupt motor power.

On a provisioned host, ROS binaries are:
`leroy_motor_provision_keys --directory <private-dir>`,
`leroy_motor_controller_signer --ros-args -p
motion_key_file:=<private-motion.key>`, and
`leroy_motor_operator_grant --operator-key-file
<private-operator.key>`. Bridge parameters:
`motor_motion_key_file`, `motor_operator_key_file`,
`motor_gpio_chip`, `motor_estop_feedback_offset`,
`motor_power_feedback_offset`. These are **not executable
deployment instructions** until installed firmware, verified MCU
stop behavior, independent cutoff and real source calibration
are established.

A separate supervised powered-wheel acceptance is still required.
No real rover hardware, MCU, motor supply, firmware flash, depth
sensor or Nav2 goal has been accessed in this branch. The checked-in
motor settings remain provisional. The existing unit file
`test_motor_authority.py` is focused code coverage, **not a new
diagnostic framework**; no operator execution result has been
claimed.

---

## Historical bridge background (before PR #156)

The sections below preserve the previous M22D/M22G diagnostic and
motor-inhibit history. Where they say nonzero motion, MCU firmware or
signed ingress has not been implemented, they describe the **prior
baseline**, not the new opt-in PR #156 source described above.
The newly implemented hardware/software path has **not** been
physically deployed or accepted.

ROS 2 Jazzy serial bridge for Leroy's MCU. It owns hardware
telemetry and is the intended future actuator interface, but **M22D removes
the raw `/cmd_vel` subscription and inhibits ALL nonzero wheel commands**.
No software authority or motor-motion ingress is enabled by M22D.

Publishes `/imu/data_raw` (`sensor_msgs/msg/Imu`, best effort, depth 10,
`imu_link`, nominal 100 Hz) and `/wheel/encoder_counts`
(`leroy_msgs/msg/EncoderCounts`, best effort, depth 10). The MCU
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

## M22D actuator ingress locked (software-only)

The bridge intentionally creates **no raw `/cmd_vel` subscription**.
The new PR #156 signed-only proposal/rearm topics (documented above)
are absent without protected keys and cannot authorize physical motion
without separate interlock inputs. Its unique
`_send_command` serial-writer path calls the ROS-free
`leroy_bridge.command_ingress.command_to_bytes` validator BEFORE
any serial I/O. It admits only exact `set_wheels 0 0 0 0`,
`keepalive`, sensor stream rate setters with the firmware's
supported values, and a strictly positive `set_watchdog_ms`
setting (1–60000). Any nonzero `set_wheels`, malformed command,
unlisted opcode, newline injection or watchdog disable is rejected.

The YAML file retains prior *provisional* `wheel_velocity_to_pwm_gain:
12.75` and `max_pwm: 64` values for historical comparison, but
these parameters cannot enable wheel output in M22D. The legacy
`cmd_vel_timeout_s` read-only parameter also remains for compatibility;
there is no `/cmd_vel` callback or velocity-timeout timer in M22D.
The historical mecanum conversion function remains independently
testable, but the production bridge node no longer uses it to emit
motion. M22A–M22C are separate synthetic authority fixtures and are
**NOT** wired into this bridge.

This is a deliberately motion-inhibiting software transition, not the
final authorized software control architecture. A follow-on milestone
must introduce a single independently validated, session-bound,
replay-resistant bridge authority and a fake-serial fault suite before
any nonzero motion path is considered.

**Opening the physical MCU CDC serial port can itself energize wheels or
the manipulator, independent of ROS or `set_wheels` checks.**
M22D source-level inhibition does not prove physical stop or ACK.
Do not launch the real bridge or reconnect hardware under this test
milestone; operator-controlled physical test authorization and motor
rail isolation are separate prerequisites.

To test the source-level CDC allowlist without ROS/serial/devices:

```bash
cd ros
PYTHONPATH="$PWD/src/leroy_bridge:$PWD" PYTHONDONTWRITEBYTECODE=1 \
  python3 -u src/leroy_bridge/test/test_m22d_command_ingress.py
```

## M22G terminal serial transport fault behavior

The production bridge now **terminally latches** unexpected serial
read errors, write errors, flush errors and short writes. It refuses
all later CDC writes (including keepalives and zero-wheel shutdown
packets) until the bridge process is restarted. The serial reader is
quiesced on fault. A normal reader exit after shutdown does not create
a new fault. A rejected unsafe command does not independently count as
a serial transport failure. The earlier M22D zero-wheel-only command
allowlist remains unchanged, and the signed M22E candidate is still
not wired into production. No ROS `/cmd_vel` subscriber is present.

**A latched fault is NOT proof of an MCU stop.** A write may have been
partially delivered, or a real MCU could retain a previous command.
The software stops transmitting keepalives after detecting failure
but no physical stop timing/ACK has been established. The physical
serial port may energize wheels/manipulator on open regardless of
this mechanism. Do not use this code change as authorization for
physical bridge/robot launch.

Validation: `ros/src/leroy_bridge/test/test_m22g_transport_fault.py`
extracts and executes actual bridge write/read methods against
in-memory serial failures. `ros/m22f_fake_bridge_ros_selftest.py`
runs the entire actual ROS bridge against a fake serial device in
an isolated no-device container. See
`ros/M22G_TERMINAL_CDC_FAULT_LATCH.md`.
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
