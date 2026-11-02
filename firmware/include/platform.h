#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PLATFORM_STOPPED = 0,
    PLATFORM_FORWARD,
    PLATFORM_BACKWARD,
    PLATFORM_LEFT,
    PLATFORM_RIGHT,
    PLATFORM_CW,
    PLATFORM_CCW,
} platform_motion_t;

void platform_init(void);
void platform_fast_tick(void); /* Called at 20 kHz from TIM4 ISR. */
void platform_update(void);    /* Call frequently from main loop. */

void platform_set_motion(platform_motion_t motion);
void platform_set_wheels(int16_t fl, int16_t fr, int16_t rl, int16_t rr);
bool platform_wheels_active(void);
void platform_stop(void);
void platform_disable(void);

bool platform_set_speed(uint16_t speed);
bool platform_set_ramp(uint16_t step);

uint16_t platform_get_speed(void);
uint16_t platform_get_ramp(void);
platform_motion_t platform_get_motion(void);
const char *platform_motion_name(platform_motion_t motion);
