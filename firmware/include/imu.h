#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature_raw;

    bool magnetometer_valid;
    int16_t mag_x;
    int16_t mag_y;
    int16_t mag_z;
} imu_sample_t;

/* Robot-frame sample using integer milli-units internally.
 *
 * Axis convention established on the assembled robot:
 *   +forward = -ICM X
 *   +left    = -ICM Y
 *   +up      = +ICM Z
 *   +yaw     = counter-clockwise = +ICM gyro Z
 *
 * accel_*_mm_s2 are millimetres/second^2 (1/1000 m/s^2).
 * gyro_*_mrad_s are milliradians/second (1/1000 rad/s).
 */
typedef struct {
    int32_t accel_forward_mm_s2;
    int32_t accel_left_mm_s2;
    int32_t accel_up_mm_s2;

    int32_t gyro_roll_mrad_s;
    int32_t gyro_pitch_mrad_s;
    int32_t gyro_yaw_mrad_s;

    /* Magnetometer remains raw/un-calibrated for the higher-level estimator.
     * Do not interpret these values as an absolute heading in firmware. */
    bool magnetometer_valid;
    int16_t mag_x_raw;
    int16_t mag_y_raw;
    int16_t mag_z_raw;

    bool gyro_calibrated;
} imu_robot_sample_t;

void imu_init(void);
bool imu_available(void);
bool imu_magnetometer_available(void);
uint8_t imu_i2c_address(void);
uint8_t imu_who_am_i(void);
bool imu_read(imu_sample_t *sample);

/* Gyro zero-rate calibration. The robot should be stationary. Automatic
 * startup calibration uses the same routine but is non-fatal if it cannot
 * gather enough stationary samples. */
bool imu_calibrate_gyro(void);
bool imu_gyro_calibrated(void);
void imu_get_gyro_bias(int32_t *x, int32_t *y, int32_t *z);

/* Read calibrated accel/gyro data transformed into the robot coordinate
 * frame and converted to physical milli-units. */
bool imu_read_robot(imu_robot_sample_t *sample);
