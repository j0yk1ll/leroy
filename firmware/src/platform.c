/* TODO(AUTONOMY) [actual wheel calibration]: Measure and document the
 * PWM polarities, minimum moving duty, per-wheel acceleration/braking,
 * mecanum yaw/strafe direction and current/thermal limits. Keep the
 * actuator power-off default, hardware cutoff and watchdog behavior intact.
 * Acceptance: supported-wheel and attended-floor tests record real wheel
 * speed, current, stop distance and wheel-slip response for all four axes.
 */

#include "platform.h"

#include "board.h"
#include "timebase.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define DRIVE_MODE_MECANUM 1
#define PWM_MAX 255
#define PLATFORM_RAMP_INTERVAL_MS 10U

/* PA0..PA7 correspond directly to the eight entries below. */
enum {
    PWM_FR_R = 0,
    PWM_FR_L = 1,
    PWM_RR_R = 2,
    PWM_RR_L = 3,
    PWM_RL_R = 4,
    PWM_RL_L = 5,
    PWM_FL_R = 6,
    PWM_FL_L = 7,
    PWM_CHANNEL_COUNT = 8,
};

typedef struct {
    uint8_t rpwm_index;
    uint8_t lpwm_index;
    uint32_t en_port;
    uint16_t en_pin;
    bool invert;
} motor_t;

static const motor_t MOTOR_FR = { PWM_FR_R, PWM_FR_L, FR_EN_PORT, FR_EN_PIN, false };
static const motor_t MOTOR_RR = { PWM_RR_R, PWM_RR_L, RR_EN_PORT, RR_EN_PIN, false };
static const motor_t MOTOR_RL = { PWM_RL_R, PWM_RL_L, RL_EN_PORT, RL_EN_PIN, true  };
static const motor_t MOTOR_FL = { PWM_FL_R, PWM_FL_L, FL_EN_PORT, FL_EN_PIN, true  };

static volatile uint8_t g_pwm_duty[PWM_CHANNEL_COUNT];
static uint16_t g_pwm_accumulator[PWM_CHANNEL_COUNT];

static int16_t g_target_fl;
static int16_t g_target_fr;
static int16_t g_target_rl;
static int16_t g_target_rr;

static int16_t g_current_fl;
static int16_t g_current_fr;
static int16_t g_current_rl;
static int16_t g_current_rr;

static uint8_t g_enable_mask;
static uint16_t g_platform_speed = 10;
static uint16_t g_platform_ramp = 255;
static uint32_t g_last_ramp_ms;
static platform_motion_t g_motion = PLATFORM_STOPPED;

static int16_t clamp_pwm(int32_t value)
{
    if (value > PWM_MAX) {
        return PWM_MAX;
    }
    if (value < -PWM_MAX) {
        return -PWM_MAX;
    }
    return (int16_t)value;
}

static int16_t approach(int16_t current, int16_t target, uint16_t step)
{
    if (current < target) {
        int32_t next = (int32_t)current + step;
        return (next > target) ? target : (int16_t)next;
    }

    if (current > target) {
        int32_t next = (int32_t)current - step;
        return (next < target) ? target : (int16_t)next;
    }

    return current;
}

static bool driver_enabled(uint8_t index)
{
    return (g_enable_mask & (1U << index)) != 0;
}

static void refresh_enable_pins(void)
{
    if (driver_enabled(0)) gpio_set(MOTOR_FR.en_port, MOTOR_FR.en_pin);
    else                   gpio_clear(MOTOR_FR.en_port, MOTOR_FR.en_pin);

    if (driver_enabled(1)) gpio_set(MOTOR_RR.en_port, MOTOR_RR.en_pin);
    else                   gpio_clear(MOTOR_RR.en_port, MOTOR_RR.en_pin);

    if (driver_enabled(2)) gpio_set(MOTOR_RL.en_port, MOTOR_RL.en_pin);
    else                   gpio_clear(MOTOR_RL.en_port, MOTOR_RL.en_pin);

    if (driver_enabled(3)) gpio_set(MOTOR_FL.en_port, MOTOR_FL.en_pin);
    else                   gpio_clear(MOTOR_FL.en_port, MOTOR_FL.en_pin);
}

static void set_enable_mask(uint8_t mask)
{
    g_enable_mask = mask & 0x0FU;
    refresh_enable_pins();
}

static void set_motor_duty(uint8_t driver_index, const motor_t *motor, int16_t speed)
{
    speed = clamp_pwm(speed);

    if (motor->invert) {
        speed = (int16_t)-speed;
    }

    if (!driver_enabled(driver_index) || speed == 0) {
        g_pwm_duty[motor->rpwm_index] = 0;
        g_pwm_duty[motor->lpwm_index] = 0;
        return;
    }

    if (speed > 0) {
        g_pwm_duty[motor->rpwm_index] = (uint8_t)speed;
        g_pwm_duty[motor->lpwm_index] = 0;
    } else {
        g_pwm_duty[motor->rpwm_index] = 0;
        g_pwm_duty[motor->lpwm_index] = (uint8_t)(-speed);
    }
}

static void apply_outputs(void)
{
    set_motor_duty(0, &MOTOR_FR, g_current_fr);
    set_motor_duty(1, &MOTOR_RR, g_current_rr);
    set_motor_duty(2, &MOTOR_RL, g_current_rl);
    set_motor_duty(3, &MOTOR_FL, g_current_fl);
}

static void set_targets(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    if (g_enable_mask == 0) {
        set_enable_mask(0x0F);
    }

    g_target_fl = clamp_pwm(fl);
    g_target_fr = clamp_pwm(fr);
    g_target_rl = clamp_pwm(rl);
    g_target_rr = clamp_pwm(rr);
}

static void apply_motion_targets(void)
{
    const int16_t speed = (int16_t)g_platform_speed;

    switch (g_motion) {
    case PLATFORM_FORWARD:
        set_targets(speed, speed, speed, speed);
        break;

    case PLATFORM_BACKWARD:
        set_targets(-speed, -speed, -speed, -speed);
        break;

    case PLATFORM_LEFT:
#if DRIVE_MODE_MECANUM
        set_targets(-speed, speed, speed, -speed);
#else
        set_targets(speed / 3, speed, speed / 3, speed);
#endif
        break;

    case PLATFORM_RIGHT:
#if DRIVE_MODE_MECANUM
        set_targets(speed, -speed, -speed, speed);
#else
        set_targets(speed, speed / 3, speed, speed / 3);
#endif
        break;

    case PLATFORM_CW:
        set_targets(speed, -speed, speed, -speed);
        break;

    case PLATFORM_CCW:
        set_targets(-speed, speed, -speed, speed);
        break;

    case PLATFORM_STOPPED:
    default:
        platform_stop();
        break;
    }
}

void platform_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Motor PWM outputs PA0..PA7. */
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    GPIO0 | GPIO1 | GPIO2 | GPIO3 |
                    GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,
                            GPIO0 | GPIO1 | GPIO2 | GPIO3 |
                            GPIO4 | GPIO5 | GPIO6 | GPIO7);
    gpio_clear(GPIOA, GPIO0 | GPIO1 | GPIO2 | GPIO3 |
                       GPIO4 | GPIO5 | GPIO6 | GPIO7);

    /* Driver enables. */
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    FR_EN_PIN | RR_EN_PIN | RL_EN_PIN | FL_EN_PIN);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ,
                            FR_EN_PIN | RR_EN_PIN | RL_EN_PIN | FL_EN_PIN);
    gpio_clear(GPIOB, FR_EN_PIN | RR_EN_PIN | RL_EN_PIN | FL_EN_PIN);

    for (unsigned i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        g_pwm_duty[i] = 0;
        g_pwm_accumulator[i] = 0;
    }

    g_target_fl = g_target_fr = g_target_rl = g_target_rr = 0;
    g_current_fl = g_current_fr = g_current_rl = g_current_rr = 0;
    g_enable_mask = 0;
    g_motion = PLATFORM_STOPPED;
    g_last_ramp_ms = millis();
}

void platform_fast_tick(void)
{
    /*
     * 20 kHz 8-bit pulse-density modulation. This avoids requiring eight
     * hardware PWM channels on PA0..PA7 while keeping switching above the
     * audible range for most duty values.
     */
    uint16_t high_mask = 0;

    for (unsigned i = 0; i < PWM_CHANNEL_COUNT; ++i) {
        const uint8_t duty = g_pwm_duty[i];

        if (duty == 0) {
            continue;
        }

        if (duty == 255) {
            high_mask |= (uint16_t)(1U << i);
            continue;
        }

        uint16_t acc = (uint16_t)(g_pwm_accumulator[i] + duty);
        if (acc >= 255U) {
            acc -= 255U;
            high_mask |= (uint16_t)(1U << i);
        }
        g_pwm_accumulator[i] = acc;
    }

    const uint16_t low_mask = (uint16_t)(0x00FFU & ~high_mask);
    GPIO_BSRR(GPIOA) = (uint32_t)high_mask | ((uint32_t)low_mask << 16);
}

void platform_update(void)
{
    const uint32_t now = millis();
    if ((uint32_t)(now - g_last_ramp_ms) < PLATFORM_RAMP_INTERVAL_MS) {
        return;
    }
    g_last_ramp_ms = now;

    g_current_fl = approach(g_current_fl, g_target_fl, g_platform_ramp);
    g_current_fr = approach(g_current_fr, g_target_fr, g_platform_ramp);
    g_current_rl = approach(g_current_rl, g_target_rl, g_platform_ramp);
    g_current_rr = approach(g_current_rr, g_target_rr, g_platform_ramp);

    apply_outputs();
}

void platform_set_motion(platform_motion_t motion)
{
    g_motion = motion;
    apply_motion_targets();
}

void platform_set_wheels(int16_t fl, int16_t fr, int16_t rl, int16_t rr)
{
    /* Direct per-wheel mode for the ROS hardware bridge. Keep the existing
     * ramp and motor-output path, but bypass predefined motion patterns. */
    g_motion = PLATFORM_STOPPED;
    set_targets(fl, fr, rl, rr);
}

bool platform_wheels_active(void)
{
    return g_target_fl != 0 || g_target_fr != 0 ||
           g_target_rl != 0 || g_target_rr != 0 ||
           g_current_fl != 0 || g_current_fr != 0 ||
           g_current_rl != 0 || g_current_rr != 0;
}

void platform_stop(void)
{
    g_motion = PLATFORM_STOPPED;
    g_target_fl = g_target_fr = g_target_rl = g_target_rr = 0;
    g_current_fl = g_current_fr = g_current_rl = g_current_rr = 0;
    apply_outputs();
}

void platform_disable(void)
{
    platform_stop();
    set_enable_mask(0);
}

bool platform_set_speed(uint16_t speed)
{
    if (speed > PWM_MAX) {
        return false;
    }

    g_platform_speed = speed;
    if (g_motion != PLATFORM_STOPPED) {
        apply_motion_targets();
    }
    return true;
}

bool platform_set_ramp(uint16_t step)
{
    if (step < 1 || step > PWM_MAX) {
        return false;
    }

    g_platform_ramp = step;
    return true;
}

uint16_t platform_get_speed(void)
{
    return g_platform_speed;
}

uint16_t platform_get_ramp(void)
{
    return g_platform_ramp;
}

platform_motion_t platform_get_motion(void)
{
    return g_motion;
}

const char *platform_motion_name(platform_motion_t motion)
{
    switch (motion) {
    case PLATFORM_FORWARD:  return "forward";
    case PLATFORM_BACKWARD: return "backward";
    case PLATFORM_LEFT:     return "left";
    case PLATFORM_RIGHT:    return "right";
    case PLATFORM_CW:       return "cw";
    case PLATFORM_CCW:      return "ccw";
    case PLATFORM_STOPPED:
    default:                 return "stopped";
    }
}
