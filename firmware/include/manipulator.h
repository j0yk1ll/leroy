#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    MANIPULATOR_STOPPED = 0,
    MANIPULATOR_UP,
    MANIPULATOR_DOWN,
} manipulator_motion_t;

typedef enum {
    MANIP_EVENT_NONE = 0,
    MANIP_EVENT_TOP_LIMIT,
    MANIP_EVENT_BOTTOM_LIMIT,
} manipulator_event_t;

void manipulator_init(void);
void manipulator_fast_tick(void); /* Called at 20 kHz from TIM4 ISR. */
void manipulator_update(void);    /* Acceleration update from main loop. */

bool manipulator_move_up(void);
bool manipulator_move_down(void);
void manipulator_stop(void);

bool manipulator_set_speed(uint32_t steps_per_second);
bool manipulator_set_acceleration(uint32_t steps_per_second_squared);
bool manipulator_set_dir_inverted(bool inverted);

uint32_t manipulator_get_speed(void);
uint32_t manipulator_get_acceleration(void);
bool manipulator_get_dir_inverted(void);
int32_t manipulator_get_position(void);
manipulator_motion_t manipulator_get_motion(void);
const char *manipulator_motion_name(manipulator_motion_t motion);

manipulator_event_t manipulator_take_event(void);
