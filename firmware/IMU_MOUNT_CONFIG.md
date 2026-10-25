# Leroy IMU mounting configuration (source-only candidate)

**Status:** This branch is a review-only refactor, **not flashed or
deployed to the motor-capable STM32**, and it does not authorize opening
USB CDC, moving the chassis, recording streams, or enabling Nav2.

## Edit one dedicated mounting profile

The physical ICM-20948 package-to-robot axis convention now lives in
`firmware/include/imu_mount_config.h`. It selects a signed permutation
of the raw X, Y and Z axes. The six configuration fields are
`IMU_MOUNT_ROBOT_X_SOURCE`, `IMU_MOUNT_ROBOT_X_SIGN`, and their
Y/Z equivalents.

Current defaults **preserve the previously established mounting**:

| Published robot-aligned axis | Input chip axis | Sign |
| --- | --- | --- |
| +X forward | ICM X | −1 |
| +Y left | ICM Y | −1 |
| +Z up | ICM Z | +1 |

The firmware applies the same transform once to raw acceleration and
once to the **bias-corrected** raw gyroscope vector; the
`accel_forward_mm_s2`, `accel_left_mm_s2`,
`accel_up_mm_s2`, `gyro_roll_mrad_s`, `gyro_pitch_mrad_s`
and `gyro_yaw_mrad_s` output fields retain their original units,
names and interpretation. Raw uncalibrated magnetometer readings are
**not rotated**. The MCU serial command format, stream rates, pinout,
watchdog and motor-control logic are unchanged.

`firmware/include/imu_axis_transform.h` is a pure signed-permutation
helper with compile-time validation: source indices must be exactly
0/1/2 once each, signs must be +1/−1, and the determinant must be +1.
It rejects mirrored frames and accidental duplicate axes.

### Why a C header, not runtime YAML?

The STM32F401 firmware is bare metal with no filesystem or YAML
parser. This compile-time configuration keeps physical mounting
values out of `imu.c` but **requires a firmware rebuild and a
separately approved flash** for installation changes. It is not
hot-reloadable or host-controlled. Do not add a new serial command
to modify the transform during a running session: that would change
sensor frame semantics midstream.

A future architecture could instead publish *unrotated raw package
axes* under a distinct sensor frame and apply orientation in a
versioned, validated ROS frame/transform; that is a different wire
contract requiring a separate migration and cannot silently replace
the accepted M18/M20 telemetry.

### ROS transform ownership

Current `/imu/data_raw` already contains the firmware-rotated,
**robot-aligned** acceleration and gyro vectors, stamped as
`imu_link`. Therefore the canonical
`ros/src/leroy_description/urdf/leroy.urdf.xacro`
should **not add a second 180° rotation**; the nominal
`base_link -> imu_link` rotation remains identity in the current
representation. If the chip's physical orientation changes, update
this firmware profile and independently validate that the published
robot-frame convention still matches ROS.

The IMU *position* relative to `base_link`, the physical origin of
`base_link`, and the camera 6-DoF mounting geometry remain in the
canonical ROS Xacro/robot description. This source-only refactor
does not claim those currently provisional position/pitch values
are measured, nor change XRSLAM's provisional 0.0 s
camera–IMU time offset.

### No-device review test

From the repository root, on the new draft branch:

```bash
python3 -B firmware/test/test_imu_mount_config.py
```

This test uses only the host `cc` compiler and a temporary scratch
directory. It verifies that the current profile exactly reproduces
`(-ICM_X, -ICM_Y, +ICM_Z)` for **both** acceleration and bias-corrected
gyro vectors, that an alternative proper right-handed axis permutation
works, and that duplicate axes/reflections fail compilation. It
checks the actual `imu.c` source calls the helper twice. It does NOT
build or flash a complete STM32 firmware image, start ROS, or
access any device.

**Operator result: PASS (host-only; no hardware), after correcting the
relative invocation path from the `ros/` checkout directory.** The
operator reported the following exact markers:

```text
IMU_MOUNT_DEFAULT_PARITY=PASS_NEG_X_NEG_Y_POS_Z
IMU_MOUNT_ALTERNATE_RIGHT_HANDED_AXIS_PERMUTATION=PASS
IMU_MOUNT_REFLECTION=REJECTED_AT_COMPILE_TIME
IMU_MOUNT_DUPLICATE_AXIS=REJECTED_AT_COMPILE_TIME
IMU_MOUNT_CONFIG=PASS_SOURCE_ONLY_NO_HARDWARE
IMU_MOUNT_FIRMWARE_FLASH=NOT_PERFORMED
IMU_MOUNT_MCU_SERIAL_AND_MOTOR_ACCESS=NONE
```

This accepts **only** the host C fixture checks and source-call
verification. A complete cross-compiled STM32 image has not been
built or flashed, and behavior of the hardware/ROS running image is
unchanged. This source-only test should not be repeated without
a reason (changed transform/profile/toolchain source).

Original expected terminal markers:

```text
IMU_MOUNT_CONFIG=PASS_SOURCE_ONLY_NO_HARDWARE
IMU_MOUNT_FIRMWARE_FLASH=NOT_PERFORMED
IMU_MOUNT_MCU_SERIAL_AND_MOTOR_ACCESS=NONE
```

An optional **compile-only**, separate environment check is
`make -C firmware build`; it requires an installed ARM toolchain and
libopencm3. **Do not run bare `make` or `make -C firmware`**:
the existing Makefile's default target invokes `flash` after building.
A successful local build is not an authorization to flash or run
the motor-capable MCU.

### Deploy boundary

The deployed firmware is unchanged until an operator deliberately
reviews and authorizes an image build/flash with independent motor
rail disconnection, secured robot, verified E-stop and full version
identity. Any CDC-open operation can reset motor-capable MCU
firmware; the accepted ROS 2 Stage 3 sensor-only runs and PRs
#150–#154 must not be treated as flash/motion permissions.

**No MCU flash or hardware test has been performed by this PR.**
