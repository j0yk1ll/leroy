#pragma once

/*
 * Leroy's physical IMU mounting orientation — build-time hardware profile.
 *
 * ICM-20948 package axes -> published ROBOT-aligned axes:
 *   +robot X (forward) = -ICM X
 *   +robot Y (left)    = -ICM Y
 *   +robot Z (up)      = +ICM Z
 *
 * Change THESE six constants for a future IMU installation. Each output
 * axis selects one unique source axis and a sign (+1 or -1). Together they
 * MUST represent a proper right-handed rotation (determinant +1).
 *
 * Firmware is bare metal (no filesystem/YAML parser): this file is the
 * compile-time profile, not a runtime parameter. Any change requires a
 * separate reviewed firmware rebuild and an independently authorized
 * physical flash/validation. Merely editing this header does NOT update
 * the installed MCU firmware or authorize motor/CDC access.
 *
 * Only raw accelerometer + bias-corrected gyro vectors use this mapping.
 * Magnetometer values intentionally remain raw and uncalibrated.
 *
 * IMPORTANT: /imu/data_raw already publishes robot-aligned coordinates
 * in imu_link. ROS TF imu_link orientation must remain identity relative
 * to robot-aligned body axes; DO NOT apply this rotation a second time
 * in the robot URDF. The IMU's *translation* and all camera extrinsics
 * belong in canonical ros/src/leroy_description/urdf/leroy.urdf.xacro.
 */

#define IMU_MOUNT_RAW_AXIS_X 0
#define IMU_MOUNT_RAW_AXIS_Y 1
#define IMU_MOUNT_RAW_AXIS_Z 2

#define IMU_MOUNT_ROBOT_X_SOURCE IMU_MOUNT_RAW_AXIS_X
#define IMU_MOUNT_ROBOT_X_SIGN (-1)

#define IMU_MOUNT_ROBOT_Y_SOURCE IMU_MOUNT_RAW_AXIS_Y
#define IMU_MOUNT_ROBOT_Y_SIGN (-1)

#define IMU_MOUNT_ROBOT_Z_SOURCE IMU_MOUNT_RAW_AXIS_Z
#define IMU_MOUNT_ROBOT_Z_SIGN (+1)
