/* TEMPORARY SENSOR-WIRING DIAGNOSTIC, NOT ROBOT FIRMWARE.
 *
 * NO platform_init(), NO manipulator_init(), NO motor commands, NO I2C
 * peripheral, NO encoder polling, NO sensor transactions, NO TIM4 ISR.
 * Only PB8/PB9 and USB CDC configured; other MCU GPIOs stay at reset state.
 *
 * PB8 and PB9 are open-drain outputs. GPIO high means Hi-Z (released), not
 * push-pull HIGH. Only one line can be pulled LOW at a time. Each pull expires
 * after 15 seconds (including during USB disconnect), then both release.
 *
 * Commands (exact ASCII lines): diag_id, status, low_scl, low_sda, release
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
#define LOW_TIMEOUT_MS 15000U

typedef enum {
    RELEASED = 0,
    LOW_SCL,
    LOW_SDA
} line_mode_t;

static volatile uint32_t g_millis;
static line_mode_t g_mode;
static uint32_t g_started_at_ms;

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

static void activate(line_mode_t mode)
{
    /* Release BOTH before pulling down the selected line. */
    release_lines();
    g_started_at_ms = now_ms();
    if (mode == LOW_SCL) {
        gpio_clear(GPIOB, LINE_SCL);
    } else if (mode == LOW_SDA) {
        gpio_clear(GPIOB, LINE_SDA);
    }
    g_mode = mode;
}

static void print_status(void)
{
    usb_serial_write("diag_i2c_gpio mode=");
    if (g_mode == LOW_SCL) {
        usb_serial_write("low_scl");
    } else if (g_mode == LOW_SDA) {
        usb_serial_write("low_sda");
    } else {
        usb_serial_write("released");
    }
    usb_serial_write(" limit_ms=");
    usb_serial_write_u32(LOW_TIMEOUT_MS);
    usb_serial_write(" elapsed_ms=");
    usb_serial_write_u32(g_mode == RELEASED ? 0U :
                         (uint32_t)(now_ms() - g_started_at_ms));
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

    /* preload ODR HIGH before selecting GPIO output mode */
    gpio_set(GPIOB, LINE_SCL | LINE_SDA);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ,
                            LINE_SCL | LINE_SDA);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
                    LINE_SCL | LINE_SDA);
    release_lines();

    usb_serial_init();

    while (1) {
        /* Deadline is independent of whether the USB host remains attached. */
        if (g_mode != RELEASED &&
            (uint32_t)(now_ms() - g_started_at_ms) >= LOW_TIMEOUT_MS) {
            release_lines();
            usb_serial_write_line("diag_i2c_gpio expired_released");
        }

        usb_serial_poll();

        char command[32];
        if (!usb_serial_read_line(command, sizeof(command))) {
            continue;
        }
        if (strcmp(command, "diag_id") == 0) {
            usb_serial_write_line("LER0Y_I2C_GPIO_DIAG_V1 NO_MOTOR NO_I2C");
        } else if (strcmp(command, "status") == 0) {
            print_status();
        } else if (strcmp(command, "release") == 0) {
            release_lines();
            usb_serial_write_line("diag_i2c_gpio released");
        } else if (strcmp(command, "low_scl") == 0) {
            activate(LOW_SCL);
            usb_serial_write_line("diag_i2c_gpio low_scl timeout_ms=15000");
        } else if (strcmp(command, "low_sda") == 0) {
            activate(LOW_SDA);
            usb_serial_write_line("diag_i2c_gpio low_sda timeout_ms=15000");
        } else {
            usb_serial_write_line("error diag_command_not_allowed");
        }
    }
}
