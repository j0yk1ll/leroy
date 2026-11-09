/* TEMPORARY SENSOR-WIRING DIAGNOSTIC, NOT ROBOT FIRMWARE.
 *
 * NO platform_init(), NO manipulator_init(), NO motor commands, NO I2C
 * peripheral, NO encoder polling, NO sensor transactions, NO TIM4 ISR.
 * Only PB8/PB9 and USB CDC configured; other MCU GPIOs stay at reset state.
 *
 * On boot PB8/PB9 alternate every 5 seconds, one line held open-drain LOW
 * and the other released (external pull-up gives HIGH). Both release
 * after a maximum of 120 seconds regardless of USB connectivity.
 * Commands: diag_id, status, cycle, low_scl, low_sda, release.
 *
 * Requires independent physical isolation of all motor-driver power.
 * Never deploy as production firmware.
 */
#include "board.h"
#include "usb_serial.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

#define LINE_SCL GPIO8
#define LINE_SDA GPIO9
#define PHASE_MS 5000U
#define CYCLE_TIMEOUT_MS 120000U
#define MANUAL_TIMEOUT_MS 15000U

typedef enum {
    RELEASED = 0,
    LOW_SCL,
    LOW_SDA,
    CYCLING
} line_mode_t;

static volatile uint32_t g_millis;
static line_mode_t g_mode;
static uint32_t g_started_at_ms;
static uint32_t g_phase_started_ms;
static bool g_phase_scl_low;

void sys_tick_handler(void)
{
    ++g_millis;
}

static uint32_t now_ms(void)
{
    return g_millis;
}

static void release_lines(void)
{
    gpio_set(GPIOB, LINE_SCL | LINE_SDA);
    g_mode = RELEASED;
}

static void apply_scl_low(bool scl_low)
{
    /* Never push-pull HIGH; release BOTH before sinking a single line. */
    gpio_set(GPIOB, LINE_SCL | LINE_SDA);
    if (scl_low) {
        gpio_clear(GPIOB, LINE_SCL);
    } else {
        gpio_clear(GPIOB, LINE_SDA);
    }
    g_phase_scl_low = scl_low;
}

static void start_cycle(void)
{
    release_lines();
    const uint32_t now = now_ms();
    g_started_at_ms = now;
    g_phase_started_ms = now;
    apply_scl_low(true);
    g_mode = CYCLING;
}

static void activate_manual(line_mode_t mode)
{
    release_lines();
    g_started_at_ms = now_ms();
    apply_scl_low(mode == LOW_SCL);
    g_mode = mode;
}

static void update_lines(void)
{
    if (g_mode == RELEASED) {
        return;
    }
    const uint32_t now = now_ms();
    const uint32_t elapsed = (uint32_t)(now - g_started_at_ms);
    const uint32_t timeout = g_mode == CYCLING
        ? CYCLE_TIMEOUT_MS : MANUAL_TIMEOUT_MS;
    if (elapsed >= timeout) {
        release_lines();
        usb_serial_write_line("diag_i2c_gpio expired_released");
        return;
    }
    if (g_mode == CYCLING &&
        (uint32_t)(now - g_phase_started_ms) >= PHASE_MS) {
        /* Rebase after overruns; no high-frequency catch-up toggling. */
        g_phase_started_ms = now;
        apply_scl_low(!g_phase_scl_low);
    }
}

static void print_status(void)
{
    usb_serial_write("diag_i2c_gpio mode=");
    if (g_mode == CYCLING) {
        usb_serial_write(g_phase_scl_low ? "cycle_low_scl" : "cycle_low_sda");
    } else if (g_mode == LOW_SCL) {
        usb_serial_write("low_scl");
    } else if (g_mode == LOW_SDA) {
        usb_serial_write("low_sda");
    } else {
        usb_serial_write("released");
    }
    usb_serial_write(" phase_ms=");
    usb_serial_write_u32(PHASE_MS);
    usb_serial_write(" elapsed_ms=");
    usb_serial_write_u32(g_mode == RELEASED ? 0U :
                         (uint32_t)(now_ms() - g_started_at_ms));
    usb_serial_write(" remaining_ms=");
    if (g_mode == RELEASED) {
        usb_serial_write_u32(0U);
    } else {
        const uint32_t timeout = g_mode == CYCLING
            ? CYCLE_TIMEOUT_MS : MANUAL_TIMEOUT_MS;
        const uint32_t elapsed = (uint32_t)(now_ms() - g_started_at_ms);
        usb_serial_write_u32(elapsed < timeout ? timeout - elapsed : 0U);
    }
    usb_serial_write("\r\n");
}

int main(void)
{
    /* Match the existing board's 25MHz HSE/84MHz CPU/48MHz USB PLL. */
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    g_millis = 0U;
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(84000U - 1U);
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();

    rcc_periph_clock_enable(RCC_GPIOB);
    /* Preload HIGH (released), then switch to open-drain GPIO outputs. */
    gpio_set(GPIOB, LINE_SCL | LINE_SDA);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ,
                            LINE_SCL | LINE_SDA);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    LINE_SCL | LINE_SDA);
    release_lines();

    usb_serial_init();
    start_cycle(); /* automatic, bounded 5-second alternating test on boot */

    while (1) {
        /* Deadline applies even if the serial host disconnects. */
        update_lines();
        usb_serial_poll();

        char command[32];
        if (!usb_serial_read_line(command, sizeof(command))) {
            continue;
        }
        if (strcmp(command, "diag_id") == 0) {
            usb_serial_write_line("LER0Y_I2C_GPIO_DIAG_V2 NO_MOTOR NO_I2C");
        } else if (strcmp(command, "status") == 0) {
            print_status();
        } else if (strcmp(command, "cycle") == 0) {
            start_cycle();
            usb_serial_write_line("diag_i2c_gpio cycle phase_ms=5000 timeout_ms=120000");
        } else if (strcmp(command, "release") == 0) {
            release_lines();
            usb_serial_write_line("diag_i2c_gpio released");
        } else if (strcmp(command, "low_scl") == 0) {
            activate_manual(LOW_SCL);
            usb_serial_write_line("diag_i2c_gpio low_scl timeout_ms=15000");
        } else if (strcmp(command, "low_sda") == 0) {
            activate_manual(LOW_SDA);
            usb_serial_write_line("diag_i2c_gpio low_sda timeout_ms=15000");
        } else {
            usb_serial_write_line("error diag_command_not_allowed");
        }
    }
}
