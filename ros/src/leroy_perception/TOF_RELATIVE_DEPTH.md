# Experimental VL53L1X metric range anchor (NOT navigation-ready)

This experimental branch adds the VL53L1X at seven-bit I²C address **0x29**
to the existing STM32F401 I2C1 bus (PB8 SCL, PB9 SDA), shared with the
ICM-20948. It is an **opt-in sensor-only** extension.

## Firmware

- Driver: `firmware/src/tof.c`, `firmware/include/tof.h`; the 16-bit-register
  I²C transactions reuse the IMU's bounded, single-threaded I²C1 helpers.
- ST Ultra Lite Driver reference configuration, **SHORT** mode, nominal
  **100 ms timing budget**, 200 ms intermeasurement interval (~5 Hz).
- The sensor is optional. Failure to probe does not disable the IMU.
- No ToF stream on boot; `set_tof_stream_hz 5` enables streaming,
  `set_tof_stream_hz 0` disables; `get_tof_stream` is read-only status.
- Output: `telemetry tof seq=N t_us=N distance_mm=N status=N valid=0|1`.
  The **raw ST ULD** status value **9** is range-valid; do not compare with
  the Pololu Arduino library's translated enum, where `RangeValid` is 0.
- A sample timestamp is the MCU readout time, not the actual laser
  acquisition instant; this uncertainty requires calibration.
- The ToF transmission is deliberately skipped if the CDC TX buffer is
  busy, preserving the existing IMU/motor-control traffic priority.
- No new motor command, wheel enable, or actuator protocol is introduced.

**Firmware has NOT been cross-compiled, flashed, or hardware-tested.**
The MCU I²C1 speed remains the existing 400 kHz; the Arduino prototype
used 100 kHz. The ToF I/O-pad config bit matches Pololu's default
\`init(io_2v8=true)\`, rather than the raw ULD 1V8 default.
**Measure the actual supply, I²C pull-up voltage, and shared-bus reliability**
before flashing; this is a code assumption, not physical verification. Use `make -C firmware build`, NEVER plain
`make -C firmware` (its default target flashes hardware).

## ROS

- MCU bridge parameter `tof_stream_hz` defaults to **0**. Set it to
  `5` only for attended hardware validation with motor power isolated.
- When enabled, publishes `sensor_msgs/msg/Range` on `/tof/range`
  with frame `vl53l1x_link`; invalid status/range => NaN. Also emits
  `/tof/status` (raw status + validity) for audit.
- No new TF is fabricated for the ToF frame. Survey its physical pose
  relative to the rectified camera; do not copy the provisional camera
  mounts to this new sensor.
- `leroy_perception/tof_scale_node.py` accepts a **relative depth**
  `32FC1` image and `CameraInfo`, plus the measured ToF range.
  It outputs `/experimental/tof_metric_depth` **only if**
  `extrinsics_measured=true`, `time_alignment_measured=true`,
  `linear_relative_depth_validated=true`, a valid measured
  ToF->camera origin/quaternion, a fresh sample, and a low-variation
  region covering the approximate ToF beam cone.
- The ToF beam is roughly 27° wide, not one exact pixel; its reported
  return may come from a non-central surface. Scaling is rejected on
  mixed surfaces, missing calibration or stale sample. Even a pass does
  NOT certify absolute metric accuracy across the full depth image.
- Projection of the *nominal beam axis* into the camera for a range r:
  `P_camera = t_camera_tof + R_camera_tof @ [0, 0, r]`,
  `u = fx*P.x/P.z + cx`, `v = fy*P.y/P.z + cy`.
  For a translated sensor, the projected pixel **depends on r**.
- The relative model must output **depth proportional to distance**,
  not inverse depth/disparity. The existing metric Video Depth Anything
  model is unchanged. An actual relative-model checkpoint/publisher
  must be separately selected and benchmarked; this branch supplies
  the adapter and guarded scaling math, not a model replacement.

## Validation gates

1. Offline static tests: command allowlist, register transaction
   correctness, motor-state invariants, ToF projection and scaling
   rejection logic, firmware cross-compile and ROS package tests.
2. Hardware bench with **motor power physically disconnected**:
   power level/logic level, 0x29 probe, shared 0x68/0x69 IMU traffic,
   all stop conditions, 5 Hz samples, raw range status, target-distance
   checks at ~0.40, 0.70, 1.00 m and wall/edge ambiguity.
3. Camera rig calibration: measure ToF->camera 6-DoF extrinsics and
   timing; align ROI and sensor cone in recorded images.
4. Determine whether relative depth is **linear in distance**, stable
   frame-to-frame, and superior to the existing metric checkpoint
   against independent measured distances. A single ToF ray cannot
   calibrate arbitrary nonlinear predictions.
5. Only then connect a **separately isolated** scaled-depth topic to
   experimental `rgbd_imu` XRSLAM, with pose-convention, map TF, Nav2,
   firmware/motor authority and live release all **disabled**.
6. Evaluate independent physical trajectory/pose and obstacle
   localization before any navigation-enabling decision.

This branch does **not** grant metric localization validity, reliable
obstacle depth, navigation TF, or motor authority.
