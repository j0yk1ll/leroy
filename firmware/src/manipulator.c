#include "manipulator.h"

#include "board.h"
#include "limits.h"
#include "timebase.h"

#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define FAST_TICK_HZ 20000U
#define SPEED_SCALE 1000U
#define PHASE_THRESHOLD (FAST_TICK_HZ * SPEED_SCALE)
#define MANIPULATOR_MAX_SPEED 5000U
#define MANIPULATOR_MAX_ACCELERATION 10000U

static volatile manipulator_motion_t g_motion = MANIPULATOR_STOPPED;
static volatile manipulator_event_t g_event = MANIP_EVENT_NONE;
static volatile uint32_t g_current_speed_milli;
static volatile uint32_t g_phase;
static volatile bool g_step_high;
static volatile int32_t g_position;

static uint32_t g_max_speed = 50U;
static uint32_t g_acceleration = 10U;
static bool g_dir_inverted;
static uint32_t g_last_accel_ms;

static void set_step(bool high)
{
    if (high) {
        gpio_set(MANIP_STEP_PORT, MANIP_STEP_PIN);
    } else {
        gpio_clear(MANIP_STEP_PORT, MANIP_STEP_PIN);
    }
    g_step_high = high;
}

static void set_direction(manipulator_motion_t motion)
{
    bool physical_high = (motion == MANIPULATOR_UP);
    if (g_dir_inverted) {
        physical_high = !physical_high;
    }

    if (physical_high) {
        gpio_set(MANIP_DIR_PORT, MANIP_DIR_PIN);
    } else {
        gpio_clear(MANIP_DIR_PORT, MANIP_DIR_PIN);
    }
}

static void stop_from_isr(manipulator_event_t event)
{
    g_motion = MANIPULATOR_STOPPED;
    g_current_speed_milli = 0;
    g_phase = 0;
    set_step(false);
    g_event = event;
}

void manipulator_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(MANIP_DIR_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    MANIP_DIR_PIN);
    gpio_set_output_options(MANIP_DIR_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            MANIP_DIR_PIN);

    gpio_mode_setup(MANIP_STEP_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    MANIP_STEP_PIN);
    gpio_set_output_options(MANIP_STEP_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            MANIP_STEP_PIN);

    gpio_clear(MANIP_DIR_PORT, MANIP_DIR_PIN);
    gpio_clear(MANIP_STEP_PORT, MANIP_STEP_PIN);

    g_motion = MANIPULATOR_STOPPED;
    g_event = MANIP_EVENT_NONE;
    g_current_speed_milli = 0;
    g_phase = 0;
    g_step_high = false;
    g_position = 0;
    g_last_accel_ms = millis();
}

void manipulator_fast_tick(void)
{
    /* Every STEP pulse is exactly one 20 kHz tick (50 us) high. */
    if (g_step_high) {
        set_step(false);
    }

    const manipulator_motion_t motion = g_motion;
    if (motion == MANIPULATOR_STOPPED) {
        return;
    }

    if (motion == MANIPULATOR_UP && top_limit_pressed()) {
        stop_from_isr(MANIP_EVENT_TOP_LIMIT);
        return;
    }

    if (motion == MANIPULATOR_DOWN && bottom_limit_pressed()) {
        g_position = 0;
        stop_from_isr(MANIP_EVENT_BOTTOM_LIMIT);
        return;
    }

    const uint32_t speed = g_current_speed_milli;
    if (speed == 0) {
        return;
    }

    uint32_t phase = g_phase + speed;
    if (phase >= PHASE_THRESHOLD) {
        phase -= PHASE_THRESHOLD;
        set_step(true);

        if (motion == MANIPULATOR_UP) {
            ++g_position;
        } else {
            --g_position;
        }
    }
    g_phase = phase;
}

void manipulator_update(void)
{
    const uint32_t now = millis();
    const uint32_t elapsed = (uint32_t)(now - g_last_accel_ms);
    if (elapsed == 0) {
        return;
    }
    g_last_accel_ms = now;

    if (g_motion == MANIPULATOR_STOPPED) {
        g_current_speed_milli = 0;
        return;
    }

    const uint32_t target = g_max_speed * SPEED_SCALE;
    uint32_t current = g_current_speed_milli;

    /*
     * g_acceleration is steps/s^2. At a 1 ms update interval that is exactly
     * g_acceleration milli-steps/s of speed change per millisecond.
     */
    uint64_t delta64 = (uint64_t)g_acceleration * elapsed;
    uint32_t delta = (delta64 > UINT32_MAX) ? UINT32_MAX : (uint32_t)delta64;

    if (current < target) {
        uint32_t next = current + delta;
        if (next < current || next > target) {
            next = target;
        }
        current = next;
    } else if (current > target) {
        current = (delta >= current - target) ? target : current - delta;
    }

    g_current_speed_milli = current;
}

bool manipulator_move_up(void)
{
    if (top_limit_pressed()) {
        return false;
    }

    g_motion = MANIPULATOR_STOPPED;
    set_step(false);
    g_phase = 0;
    g_current_speed_milli = 0;
    set_direction(MANIPULATOR_UP);
    g_motion = MANIPULATOR_UP;
    g_last_accel_ms = millis();
    return true;
}

bool manipulator_move_down(void)
{
    if (bottom_limit_pressed()) {
        g_position = 0;
        return false;
    }

    g_motion = MANIPULATOR_STOPPED;
    set_step(false);
    g_phase = 0;
    g_current_speed_milli = 0;
    set_direction(MANIPULATOR_DOWN);
    g_motion = MANIPULATOR_DOWN;
    g_last_accel_ms = millis();
    return true;
}

void manipulator_stop(void)
{
    g_motion = MANIPULATOR_STOPPED;
    g_current_speed_milli = 0;
    g_phase = 0;
    set_step(false);
}

bool manipulator_set_speed(uint32_t steps_per_second)
{
    if (steps_per_second < 1U || steps_per_second > MANIPULATOR_MAX_SPEED) {
        return false;
    }
    g_max_speed = steps_per_second;
    return true;
}

bool manipulator_set_acceleration(uint32_t steps_per_second_squared)
{
    if (steps_per_second_squared < 1U ||
        steps_per_second_squared > MANIPULATOR_MAX_ACCELERATION) {
        return false;
    }
    g_acceleration = steps_per_second_squared;
    return true;
}

bool manipulator_set_dir_inverted(bool inverted)
{
    if (g_motion != MANIPULATOR_STOPPED) {
        return false;
    }
    g_dir_inverted = inverted;
    return true;
}

uint32_t manipulator_get_speed(void)
{
    return g_max_speed;
}

uint32_t manipulator_get_acceleration(void)
{
    return g_acceleration;
}

bool manipulator_get_dir_inverted(void)
{
    return g_dir_inverted;
}

int32_t manipulator_get_position(void)
{
    return g_position;
}

manipulator_motion_t manipulator_get_motion(void)
{
    return g_motion;
}

const char *manipulator_motion_name(manipulator_motion_t motion)
{
    switch (motion) {
    case MANIPULATOR_UP:   return "up";
    case MANIPULATOR_DOWN: return "down";
    case MANIPULATOR_STOPPED:
    default:               return "stopped";
    }
}

manipulator_event_t manipulator_take_event(void)
{
    cm_disable_interrupts();
    const manipulator_event_t event = g_event;
    g_event = MANIP_EVENT_NONE;
    cm_enable_interrupts();
    return event;
}
