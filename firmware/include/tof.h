#pragma once

#include <stdbool.h>
#include <stdint.h>

/* VL53L1X is diagnostic-only. Its measured range is not robot odometry,
 * independent obstacle clearance, or authorization to enable motors. */
typedef struct {
    uint32_t t_us;             /* MCU time on receipt, NOT laser emission time */
    uint16_t distance_mm;     /* raw sensor distance */
    uint8_t range_status;     /* VL53L1X raw RANGE_STATUS (9 means valid) */
    bool valid;
} tof_sample_t;

void tof_init(void);          /* call only after imu_init() owns I2C1 */
bool tof_available(void);
bool tof_set_enabled(bool enabled); /* only the MCU serial telemetry owns this */
void tof_poll(void);          /* nonblocking; called from main loop */
bool tof_take_sample(tof_sample_t *sample);
