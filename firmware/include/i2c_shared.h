#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Non-reentrant register access to I2C1, initialized and owned by imu_init().
 * Called only from the STM32 main loop; never from an ISR. VL53L1X uses a
 * 16-bit big-endian register address, unlike ICM-20948's 8-bit addresses. */
bool i2c_shared_write16_reg8(uint8_t address, uint16_t reg, uint8_t value);
bool i2c_shared_read16_reg8(uint8_t address, uint16_t reg, uint8_t *value);
