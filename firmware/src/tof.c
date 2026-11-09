/* Diagnostic VL53L1X ranging on the existing STM32F401 I2C1 bus.
 * Copyright notice: register addresses/default configuration derived from
 * ST VL53L1X Ultra Lite Driver; values cross-checked against the
 * SPDX-License-Identifier: GPL-2.0 OR BSD-2-Clause Linux vl53l1x-i2c driver.
 * This file is distributed under SPDX-License-Identifier: BSD-2-Clause.
 *
 * Important: ST "range status" 9 denotes a valid raw ULD result, unlike
 * Pololu's mapped VL53L1X::RangeValid enum (value 0). Never conflate them.
 *
 * This is bench-candidate firmware, NOT flashed or sensor-validated.
 * The sensor beam is a cone and its range does not identify a camera pixel.
 */
#include "tof.h"
#include "i2c_shared.h"
#include "timebase.h"

#include <stddef.h>

#define ADDR 0x29U
#define MODEL_H 0x010FU
#define BOOT_STATUS 0x00E5U
#define RANGE_STATUS 0x0089U
#define RANGE_MM 0x0096U
#define READY_GPIO 0x0031U
#define CLEAR_INTERRUPT 0x0086U
#define MODE_START 0x0087U
#define CONFIG_BEGIN 0x002DU
#define POLL_INTERVAL_MS 10U
#define VHV_START_TIMEOUT_MS 300U

/* 0x002D..0x0087 inclusive: ST Ultra Lite Driver baseline configuration.
 * Register 0x2E is set to 0x01 (2V8 I/O pad) matching the user's tested
 * Pololu init(io_2v8=true) default. Hardware board/logic voltage MUST
 * still be verified on the real 3V3-bus robot before flashing.
 * Short mode and 100ms budget are applied after first VHV calibration. */
static const uint8_t default_config[91] = {
    0x00,0x01,0x00,0x01,0x02,0x00,0x02,0x08,
    0x00,0x08,0x10,0x01,0x01,0x00,0x00,0x00,
    0x00,0xFF,0x00,0x0F,0x00,0x00,0x00,0x00,
    0x00,0x20,0x0B,0x00,0x00,0x02,0x0A,0x21,
    0x00,0x00,0x05,0x00,0x00,0x00,0x00,0xC8,
    0x00,0x00,0x38,0xFF,0x01,0x00,0x08,0x00,
    0x00,0x01,0xCC,0x0F,0x01,0xF1,0x0D,0x01,
    0x68,0x00,0x80,0x08,0xB8,0x00,0x00,0x00,
    0x00,0x0F,0x89,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x01,0x0F,0x0D,0x0E,0x0E,0x00,
    0x00,0x02,0xC7,0xFF,0x9B,0x00,0x00,0x00,
    0x01,0x00,0x00
};

enum state { MISSING, FIRST_VHV, STREAMING };
static enum state g_state;
static uint32_t g_stage_started_ms;
static uint32_t g_last_poll_ms;
static bool g_emit_enabled;
static bool g_pending;
static tof_sample_t g_sample;

static bool wr(uint16_t reg, uint8_t v)
{
    return i2c_shared_write16_reg8(ADDR, reg, v);
}

static bool rd(uint16_t reg, uint8_t *v)
{
    return i2c_shared_read16_reg8(ADDR, reg, v);
}

static bool rd16(uint16_t reg, uint16_t *value)
{
    uint8_t high, low;
    if (value == NULL || !rd(reg, &high) ||
        !rd((uint16_t)(reg + 1U), &low)) {
        return false;
    }
    *value = (uint16_t)(((uint16_t)high << 8U) | low);
    return true;
}

static bool wr16(uint16_t reg, uint16_t value)
{
    return wr(reg, (uint8_t)(value >> 8U)) &&
           wr((uint16_t)(reg + 1U), (uint8_t)value);
}

static bool wr32(uint16_t reg, uint32_t value)
{
    for (unsigned i = 0; i < 4U; ++i) {
        if (!wr((uint16_t)(reg + i),
                (uint8_t)(value >> (24U - i * 8U)))) {
            return false;
        }
    }
    return true;
}

static bool ready(bool *yes)
{
    uint8_t value = 0U;
    if (!rd(READY_GPIO, &value)) {
        return false;
    }
    /* Default GPIO polarity is active-low: 0 means data ready. */
    *yes = (value & 1U) == 0U;
    return true;
}

static bool activate_short_100ms(void)
{
    /* ULD SHORT preset: effective measurement range ~1.3m.
     * Do not use this as an obstacle detector beyond that limit. */
    static const uint16_t regs[] = {
        0x004BU,0x0060U,0x0063U,0x0069U,
        0x0078U,0x0079U,0x007AU,0x007BU
    };
    static const uint8_t values[] = {
        0x14U,0x07U,0x05U,0x38U,0x07U,0x05U,0x06U,0x06U
    };
    for (unsigned i = 0; i < sizeof(values); ++i) {
        if (!wr(regs[i], values[i])) {
            return false;
        }
    }
    /* Short mode 100ms timing budget from ST ULD. */
    if (!wr16(0x005EU, 0x02E1U) || !wr16(0x0061U, 0x0388U)) {
        return false;
    }
    uint16_t osc = 0U;
    if (!rd16(0x00DEU, &osc)) {
        return false;
    }
    const uint32_t pll = (uint32_t)(osc & 0x03FFU);
    if (pll == 0U) {
        return false;
    }
    /* 200ms intermeasurement period; wider than the 100ms budget. */
    const uint32_t ticks = pll * 200U * 1075U / 1000U;
    return wr32(0x006CU, ticks) &&
           wr(CLEAR_INTERRUPT, 0x01U) &&
           wr(MODE_START, 0x40U);
}

void tof_init(void)
{
    g_state = MISSING;
    g_emit_enabled = false;
    g_pending = false;
    g_last_poll_ms = millis();

    uint16_t model = 0U;
    uint8_t boot = 0U;
    /* ToF is entirely optional. Failed probe must not disable ICM-20948. */
    if (!rd16(MODEL_H, &model) || model != 0xEACCU ||
        !rd(BOOT_STATUS, &boot) || (boot & 1U) == 0U) {
        return;
    }

    for (unsigned i = 0; i < sizeof(default_config); ++i) {
        if (!wr((uint16_t)(CONFIG_BEGIN + i), default_config[i])) {
            return;
        }
    }
    if (!wr(CLEAR_INTERRUPT, 0x01U) || !wr(MODE_START, 0x40U)) {
        return;
    }
    g_state = FIRST_VHV;
    g_stage_started_ms = millis();
}

bool tof_available(void)
{
    return g_state != MISSING;
}

bool tof_set_enabled(bool enabled)
{
    if (enabled && g_state == MISSING) {
        return false;
    }
    g_emit_enabled = enabled;
    if (!enabled) {
        g_pending = false;
    }
    return true;
}

void tof_poll(void)
{
    if (g_state == MISSING) {
        return;
    }
    const uint32_t now = millis();
    if ((uint32_t)(now - g_last_poll_ms) < POLL_INTERVAL_MS) {
        return;
    }
    g_last_poll_ms = now;

    bool has_data = false;
    if (!ready(&has_data)) {
        g_state = MISSING;
        g_pending = false;
        return;
    }
    if (!has_data) {
        if (g_state == FIRST_VHV &&
            (uint32_t)(now - g_stage_started_ms) > VHV_START_TIMEOUT_MS) {
            g_state = MISSING;
        }
        return;
    }

    if (g_state == FIRST_VHV) {
        if (!wr(CLEAR_INTERRUPT, 0x01U) ||
            !wr(MODE_START, 0x00U) ||
            !wr(0x0008U, 0x09U) ||
            !wr(0x000BU, 0x00U) ||
            !activate_short_100ms()) {
            g_state = MISSING;
            return;
        }
        g_state = STREAMING;
        return;
    }

    uint8_t raw_status = 0U;
    uint16_t distance = 0U;
    if (!rd(RANGE_STATUS, &raw_status) ||
        !rd16(RANGE_MM, &distance) ||
        !wr(CLEAR_INTERRUPT, 0x01U)) {
        g_state = MISSING;
        g_pending = false;
        return;
    }

    g_sample.t_us = micros();
    g_sample.distance_mm = distance;
    g_sample.range_status = (uint8_t)(raw_status & 0x1FU);
    g_sample.valid = g_sample.range_status == 9U &&
                     distance >= 40U && distance <= 1300U;
    g_pending = g_emit_enabled;
}

bool tof_take_sample(tof_sample_t *sample)
{
    if (sample == NULL || !g_pending) {
        return false;
    }
    *sample = g_sample;
    g_pending = false;
    return true;
}
