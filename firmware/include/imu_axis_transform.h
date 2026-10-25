#pragma once

/*
 * Pure, dependency-free IMU signed-permutation transform for the
 * config in imu_mount_config.h. Works in firmware and host-only tests.
 * Never touches ROS, serial, motors, or the ICM-20948 registers.
 */
#include <stdint.h>
#include "imu_mount_config.h"

#define IMU_MOUNT_VALID_AXIS(axis) ((axis) >= 0 && (axis) <= 2)
#define IMU_MOUNT_VALID_SIGN(sign) ((sign) == 1 || (sign) == -1)

/* Each source coordinate must be used exactly once. */
_Static_assert(IMU_MOUNT_VALID_AXIS(IMU_MOUNT_ROBOT_X_SOURCE), "invalid IMU robot X axis");
_Static_assert(IMU_MOUNT_VALID_AXIS(IMU_MOUNT_ROBOT_Y_SOURCE), "invalid IMU robot Y axis");
_Static_assert(IMU_MOUNT_VALID_AXIS(IMU_MOUNT_ROBOT_Z_SOURCE), "invalid IMU robot Z axis");
_Static_assert(IMU_MOUNT_ROBOT_X_SOURCE != IMU_MOUNT_ROBOT_Y_SOURCE, "duplicate IMU X/Y source");
_Static_assert(IMU_MOUNT_ROBOT_X_SOURCE != IMU_MOUNT_ROBOT_Z_SOURCE, "duplicate IMU X/Z source");
_Static_assert(IMU_MOUNT_ROBOT_Y_SOURCE != IMU_MOUNT_ROBOT_Z_SOURCE, "duplicate IMU Y/Z source");
_Static_assert(IMU_MOUNT_VALID_SIGN(IMU_MOUNT_ROBOT_X_SIGN), "invalid IMU X sign");
_Static_assert(IMU_MOUNT_VALID_SIGN(IMU_MOUNT_ROBOT_Y_SIGN), "invalid IMU Y sign");
_Static_assert(IMU_MOUNT_VALID_SIGN(IMU_MOUNT_ROBOT_Z_SIGN), "invalid IMU Z sign");

/* Even permutations of (0,1,2) are (0,1,2), (1,2,0), (2,0,1).
 * Requiring permutation parity * sign product = +1 prevents
 * an accidental reflected/left-handed coordinate frame. */
#define IMU_MOUNT_EVEN_PERMUTATION ( \
    (IMU_MOUNT_ROBOT_X_SOURCE == 0 && IMU_MOUNT_ROBOT_Y_SOURCE == 1 && IMU_MOUNT_ROBOT_Z_SOURCE == 2) || \
    (IMU_MOUNT_ROBOT_X_SOURCE == 1 && IMU_MOUNT_ROBOT_Y_SOURCE == 2 && IMU_MOUNT_ROBOT_Z_SOURCE == 0) || \
    (IMU_MOUNT_ROBOT_X_SOURCE == 2 && IMU_MOUNT_ROBOT_Y_SOURCE == 0 && IMU_MOUNT_ROBOT_Z_SOURCE == 1) )
#define IMU_MOUNT_PARITY (IMU_MOUNT_EVEN_PERMUTATION ? 1 : -1)
_Static_assert(
    IMU_MOUNT_PARITY * IMU_MOUNT_ROBOT_X_SIGN *
    IMU_MOUNT_ROBOT_Y_SIGN * IMU_MOUNT_ROBOT_Z_SIGN == 1,
    "IMU mounting must be a proper right-handed rotation");

/* For valid ICM-20948 ADC int16 inputs and int16-minus-bias gyro
 * differences, sign multiplication cannot overflow int32_t. */
static inline void imu_mount_transform_vector(const int32_t src[3], int32_t robot[3])
{
    const int32_t x = src[IMU_MOUNT_ROBOT_X_SOURCE] * IMU_MOUNT_ROBOT_X_SIGN;
    const int32_t y = src[IMU_MOUNT_ROBOT_Y_SOURCE] * IMU_MOUNT_ROBOT_Y_SIGN;
    const int32_t z = src[IMU_MOUNT_ROBOT_Z_SOURCE] * IMU_MOUNT_ROBOT_Z_SIGN;
    robot[0] = x;
    robot[1] = y;
    robot[2] = z;
}
