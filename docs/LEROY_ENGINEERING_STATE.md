# Leroy Engineering State

Last updated: 2026-11-12

## Purpose

Persistent engineering handover for Leroy.

Read this document before proposing investigations, architectural changes,
hardware tests, or implementation work.

IMPORTANT:
- Preserve established decisions unless new evidence justifies revisiting them.
- Do not repeat completed investigations without a specific reason.
- Distinguish verified facts from unresolved questions.
- Inspect existing engineering records before declaring calibration missing.
- Prefer concrete implementation progress over exploratory test proliferation.
- Give the operator ONE bounded shell command per interaction.
- Always distinguish Arch Linux host commands from Podman container commands.

## Development environment

Verified:
- Arch Linux development host.
- ROS 2 Jazzy.
- Podman Ubuntu development container: `leroy-dev`.
- Engineering checkout:
  `~/leroy-engineering/step175-depth-acceptance`
- STM32F401 microcontroller.
- MCU USB CDC serial device used during commissioning: `/dev/ttyACM0`.
- Direct attended input device used during commissioning: `/dev/input/event2`.

Operational constraints:
- `leroy-calibration-sensors` was stopped to release MCU serial ownership.
- Avoid simultaneous ownership of `/dev/ttyACM0`.
- Motor-driver power remains disconnected during software-only work.
- An independent physical cutoff removes downstream electrical power.
- The cutoff is NOT a readable GPIO or software interlock.
- Firmware's default `make` target may flash hardware.
  Use explicit build/flash targets deliberately.

## Established architectural decisions

### Localization

DECISION:
- Use SLAM as the primary source of localization and odometry.
- Do not use wheel encoders for navigation odometry.
- Right-side wheel encoders have an established hardware fault.
- Left-side encoder availability does not justify reintroducing
  encoder-derived navigation odometry.
- Do not hardcode PWM-to-speed estimates into navigation.

DECISION:
- The measured PWM-to-speed relationship is commissioning data.
- If later required, expose it through robot-specific configuration,
  potentially managed by `leroy_description`.
- Such a configuration must be optional and must not masquerade as
  measured odometry.

Existing implementation:
- `leroy_localization` contains the XRSLAM integration.
- XRSLAM can publish `/odom` and `odom -> base_link` after its
  metric pose and coordinate convention have been validated.
- `pose_convention_validated=false` must remain in place until
  supported by actual validation evidence.
- `enable_map_tf=false` remains appropriate until map alignment
  is validated.
- The MCU bridge's degraded wheel/IMU odometry is disabled by default.
- The MCU bridge must not compete with XRSLAM for `/odom` or
  `odom -> base_link`.
- Existing XRSLAM visual-only and experimental metric preview outputs
  must not be misrepresented as validated navigation odometry.

IMPORTANT:
- Camera–IMU calibration has been investigated in previous work.
- Its complete historical status is NOT established by this handover.
- Search existing repository records, calibration artifacts, and
  prior engineering documentation before concluding anything is missing.
- Do not restart calibration merely because a previous conversation
  lost its context.

### Depth perception and autonomous navigation

ESTABLISHED DECISION:
- Do not require accurate dense metric depth as a prerequisite for autonomy.
- Monocular depth estimation primarily supplies relative scene geometry,
  obstacle relationships, and free-space information.
- The time-of-flight sensor provides independent local range evidence;
  it is not expected to make the entire monocular depth map metrically exact.
- Do not repeatedly attempt global metric-depth calibration as a prerequisite
  for progressing with perception and autonomous-navigation development.
- Use uncertainty-aware obstacle avoidance and conservative safety margins.
- Conventional Nav2 components still require consistent metric geometry for
  robot footprint, collision clearance, and trajectory planning.
- Validate those safety-critical geometric quantities where necessary,
  without demanding accurate metric distance for every scene pixel.
- Keep autonomous motor authority gated until the relevant localization,
  obstacle-detection, and stopping safeguards are demonstrated.
- Distinguish relative visual understanding from validated metric pose;
  neither should be misrepresented as the other.

### Motor authority

DECISION:
- Keep the existing attended direct-commissioning path separate from
  production signed navigation motor authority.
- Preserve one-use arming and Space-held deadman behavior.
- Preserve MCU watchdog, sequence-specific ACK validation, and
  explicit sequenced STOP.
- Treat a command ACK as evidence of MCU command acceptance,
  not as proof of physical stopping.
- The independent physical cutoff remains the final hardware safeguard.

## Verified motor commissioning

Hardware:
- Four driven wheels.
- All four wheels rotate in the correct forward direction.
- Right wheels required more starting PWM than left wheels.
- Right wheels were mechanically turnable by hand.

Observed unloaded behavior:
- PWM 4/255: motors responded; wheels did not rotate.
- PWM 8/255: slight wheel rotation.
- PWM 10/255: left wheels rotated, right wheels did not.
- PWM 12/255: right wheels could rotate after manual assistance.
- PWM 16/255: all four rotated; right wheels intermittently stalled.
- PWM 32/255: all four rotated reliably and stopped on Space release.
- PWM 64/255: all four rotated reliably and stopped on Space release.

Verified ground-level behavior:
- PWM 64/255 successfully drove Leroy forward.
- Leroy tracked straight without observed drift.
- All four wheels stopped immediately when Space was released.
- STM32 final sequenced STOP was acknowledged.

Measured forward-motion experiment:
- PWM: 64/255.
- Ground distance: 2.000 metres.
- Space-hold duration from Linux input event timestamps: 13.683 seconds.
- Average distance divided by Space-hold time: about 0.146 m/s.
- This is NOT direct motor-on-time measurement or encoder odometry.
- Do not hardcode this value into SLAM or navigation.

Current direct commissioning configuration:
- Requested PWM: (64, 64, 64, 64).
- Attended Python commissioning ceiling: 64/255.
- Firmware calibration command supports up to 255/255.
- Firmware calibration watchdog maximum: 200 ms.
- Direct session configured MCU watchdog to 150 ms.
- Persistent motor command sequence ledger is in the operator's
  local state directory and must NEVER be reset casually.

Verified commissioning commits:
- 99af40e — Commission attended four-wheel driving and
  STOP ACK preemption.
- 21564ec — Preserve MCU calibration and persistent motor sequencing.

Relevant source:
- `ros/src/leroy_mcu/direct_commissioning_input.py`
- `ros/src/leroy_mcu/direct_commissioning_runtime.py`
- `ros/src/leroy_mcu/direct_commissioning_writer.py`
- `ros/src/leroy_mcu/leroy_mcu/motor_cdc_protocol.py`
- `ros/src/leroy_mcu/leroy_mcu/motor_sequence_ledger.py`
- `ros/src/leroy_safety/leroy_safety/attended_commissioning.py`
- `firmware/src/command.c`

## STOP ACK issue and correction

Observed:
- During an earlier commissioning session, a final STOP was
  unconfirmed with `MISSING_MISMATCHED_OR_STALE_ACK`.

Cause addressed:
- STOP preempted an outstanding wheel transaction.
- A delayed ACK for the displaced wheel command could incorrectly
  fault the pending STOP transaction.

Correction:
- Recognize only the exact displaced wheel command's delayed ACK.
- Do not count it as STOP confirmation.
- Require STOP's own sequence-matched ACK.
- Continue faulting unrelated ACKs and missing STOP ACKs.

Verification:
- Targeted preemption checks passed.
- Subsequent live MCU and powered motor tests completed with
  acknowledged final STOP.

## Existing SLAM integration

Known paths:
- `ros/src/leroy_localization/src/leroy_xrslam_node.cpp`
- `ros/src/leroy_localization/config/leroy_monocular_imu_slam.yaml`
- `ros/src/leroy_localization/config/leroy_experimental_rgbd_imu_slam.yaml`
- `ros/src/leroy_localization/ROS_SENSOR_CONFIG.md`
- `ros/src/leroy_bringup/launch/robot.launch.py`
- `ros/src/leroy_navigation/launch/nav2.launch.py`
- `ros/src/leroy_description/config/sensor_mounts.yaml`

Verified from source inspection:
- `robot.launch.py` defaults `use_xrslam` to false.
- XRSLAM has gated navigation odometry and TF publication.
- Unvalidated visual-only localization remains separate.
- MCU degraded encoder-based odometry is disabled by default.
- MCU bridge already subscribes to localization validity and `/odom`.

Not established here:
- Whether all previous camera–IMU calibration tasks are complete.
- Whether current XRSLAM metric scale and coordinate conventions
  have passed the required real-sensor validation.
- Whether Nav2 is ready for authorized autonomous operation.

Do not treat any of these as automatically failed or completed.

## Remaining unrelated worktree changes

At the time of the motor commissioning commits, unrelated work
remained in perception scripts, safety gateway, calibration assets,
and step175 depth-acceptance artifacts.

Do not stage, overwrite, clean, or revert those changes as part
of motor commissioning or SLAM integration.

## Next engineering milestone

Advance the existing XRSLAM localization system toward verified
metric localization using the calibration and sensor evidence
already available in the repository.

START HERE:
1. Locate existing engineering handovers and calibration results.
2. Reconcile them against XRSLAM's actual remaining validation gates.
3. Select ONE specific outstanding gate and implement or validate it.
4. Preserve all existing motor and navigation safety boundaries.

Do not start by re-investigating wheel encoders.
Do not introduce PWM-derived navigation odometry.
Do not assume camera–IMU calibration is missing.
Do not create an alternative SLAM implementation.
