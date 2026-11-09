/* TEMPORARY DIAGNOSTIC: fixed-address STM32 I2C1 100 kHz ACK probe.
 * NO actuator/encoder/IMU/ToF application drivers, NO timer4 ISR,
 * NO ranging configuration writes. This probes fixed addresses and reads only identity registers.
 * It MUST NOT run with motor-driver power connected.
 *
 * USB commands: diag_id, status, probe, read_ids, release
 * probe addresses ONLY 0x29, 0x68, 0x69. No full bus scan.
 */
#include "board.h"
#include "usb_serial.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#define LINES (GPIO8 | GPIO9)
#define I2C_TIMEOUT_MS 20U
#define ERROR_BITS (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO | I2C_SR1_OVR | I2C_SR1_TIMEOUT)

static volatile uint32_t tick_ms;

void sys_tick_handler(void) { ++tick_ms; }

static void init_clock(void)
{
    rcc_clock_setup_pll(&rcc_hse_25mhz_3v3[RCC_CLOCK_3V3_84MHZ]);
    tick_ms = 0;
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_set_reload(84000U - 1U);
    systick_clear();
    systick_interrupt_enable();
    systick_counter_enable();
}

static void release_gpio(void)
{
    gpio_set(GPIOB, LINES);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_2MHZ, LINES);
    gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, LINES);
}

static void init_i2c(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    /* Both lines remain released before the peripheral takes ownership. */
    gpio_set(GPIOB, LINES);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, LINES);
    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP, LINES);
    gpio_set_af(GPIOB, GPIO_AF4, LINES);
    rcc_periph_reset_pulse(RST_I2C1);
    i2c_peripheral_disable(I2C1);
    /* APB1 peripheral clock is 42 MHz on the existing robot board. */
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 42U);
    i2c_enable_ack(I2C1);
    i2c_peripheral_enable(I2C1);
}

static void recover_i2c(void)
{
    i2c_peripheral_disable(I2C1);
    rcc_periph_reset_pulse(RST_I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, 42U);
    i2c_enable_ack(I2C1);
    i2c_peripheral_enable(I2C1);
}

static void print_result(uint8_t addr, const char *result, uint32_t sr1, uint32_t sr2)
{
    static const char hex[] = "0123456789abcdef";
    char a[] = "0x00";
    a[2] = hex[(addr >> 4U) & 15U];
    a[3] = hex[addr & 15U];
    usb_serial_write("i2c100 address=");
    usb_serial_write(a);
    usb_serial_write(" result=");
    usb_serial_write(result);
    usb_serial_write(" sr1=");
    usb_serial_write_u32(sr1);
    usb_serial_write(" sr2=");
    usb_serial_write_u32(sr2);
    usb_serial_write("\r\n");
}

static void probe_one(uint8_t address)
{
    uint32_t start = tick_ms;
    /* Precondition: no unexpected master owns I2C1. */
    while (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
        if ((uint32_t)(tick_ms - start) >= I2C_TIMEOUT_MS) {
            const uint32_t sr1 = I2C_SR1(I2C1), sr2 = I2C_SR2(I2C1);
            print_result(address, "busy_timeout", sr1, sr2);
            recover_i2c();
            return;
        }
    }

    i2c_send_start(I2C1);
    start = tick_ms;
    while (!(I2C_SR1(I2C1) & I2C_SR1_SB)) {
        const uint32_t sr1 = I2C_SR1(I2C1);
        if ((sr1 & ERROR_BITS) || (uint32_t)(tick_ms - start) >= I2C_TIMEOUT_MS) {
            const uint32_t sr2 = I2C_SR2(I2C1);
            print_result(address, (sr1 & ERROR_BITS) ? "start_error" : "start_timeout", sr1, sr2);
            recover_i2c();
            return;
        }
    }

    i2c_send_7bit_address(I2C1, address, I2C_WRITE);
    start = tick_ms;
    for (;;) {
        const uint32_t sr1 = I2C_SR1(I2C1);
        if (sr1 & I2C_SR1_ADDR) {
            /* Standard SR1 then SR2 sequence clears ADDR. */
            const uint32_t sr2 = I2C_SR2(I2C1);
            i2c_send_stop(I2C1);
            print_result(address, "ack", sr1, sr2);
            return;
        }
        if (sr1 & I2C_SR1_AF) {
            i2c_send_stop(I2C1);
            I2C_SR1(I2C1) &= ~I2C_SR1_AF;
            print_result(address, "nack", sr1, I2C_SR2(I2C1));
            return;
        }
        if ((sr1 & (ERROR_BITS & ~I2C_SR1_AF)) ||
            (uint32_t)(tick_ms - start) >= I2C_TIMEOUT_MS) {
            const uint32_t sr2 = I2C_SR2(I2C1);
            print_result(address, (sr1 & (ERROR_BITS & ~I2C_SR1_AF))
                ? "address_error" : "address_timeout", sr1, sr2);
            recover_i2c();
            return;
        }
    }
}


/* Read one immutable identification register. The only transmitted data
 * bytes are register POINTERS; never write device configuration values.
 * Uses STOP between pointer-set and single-byte receive, as the existing
 * Leroy driver does. Each wait is bounded and recovers on error. */
static bool wait_flags(uint32_t mask, const char **why)
{
    const uint32_t start = tick_ms;
    for (;;) {
        const uint32_t sr1 = I2C_SR1(I2C1);
        if (sr1 & (ERROR_BITS & ~I2C_SR1_AF)) {
            *why = "bus_error";
            return false;
        }
        if (sr1 & I2C_SR1_AF) {
            *why = "nack";
            return false;
        }
        if ((sr1 & mask) != 0U) {
            return true;
        }
        if ((uint32_t)(tick_ms - start) >= I2C_TIMEOUT_MS) {
            *why = "timeout";
            return false;
        }
    }
}

static bool wait_idle(const char **why)
{
    const uint32_t start = tick_ms;
    while (I2C_SR2(I2C1) & I2C_SR2_BUSY) {
        if ((uint32_t)(tick_ms - start) >= I2C_TIMEOUT_MS) {
            *why = "busy_timeout";
            return false;
        }
    }
    return true;
}

static bool start_address(uint8_t address, uint8_t direction, const char **why)
{
    i2c_send_start(I2C1);
    if (!wait_flags(I2C_SR1_SB, why)) {
        return false;
    }
    i2c_send_7bit_address(I2C1, address, direction);
    if (!wait_flags(I2C_SR1_ADDR, why)) {
        return false;
    }
    return true;
}

static bool read_identity_byte(uint8_t address, uint16_t register_number,
                               bool reg16, uint8_t *result, const char **why)
{
    if (!wait_idle(why) ||
        !start_address(address, I2C_WRITE, why)) {
        goto fail;
    }
    (void)I2C_SR1(I2C1);
    (void)I2C_SR2(I2C1);

    if (reg16) {
        i2c_send_data(I2C1, (uint8_t)(register_number >> 8U));
        if (!wait_flags(I2C_SR1_BTF, why)) {
            goto fail;
        }
    }

    i2c_send_data(I2C1, (uint8_t)register_number);
    if (!wait_flags(I2C_SR1_BTF, why)) {
        goto fail;
    }
    i2c_send_stop(I2C1);
    if (!wait_idle(why) ||
        !start_address(address, I2C_READ, why)) {
        goto fail;
    }

    /* STM32F4 single-byte RX: NACK before clearing ADDR, STOP before RXNE. */
    i2c_disable_ack(I2C1);
    (void)I2C_SR1(I2C1);
    (void)I2C_SR2(I2C1);
    i2c_send_stop(I2C1);
    if (!wait_flags(I2C_SR1_RxNE, why)) {
        goto fail;
    }
    *result = i2c_get_data(I2C1);
    i2c_enable_ack(I2C1);
    return true;

fail:
    recover_i2c();
    return false;
}

static void read_and_report(const char *name, uint8_t address,
                            uint16_t reg, bool reg16)
{
    uint8_t value = 0;
    const char *why = "unknown";
    usb_serial_write("i2c100 id=");
    usb_serial_write(name);
    usb_serial_write(" result=");
    if (read_identity_byte(address, reg, reg16, &value, &why)) {
        usb_serial_write("ok value_dec=");
        usb_serial_write_u32(value);
    } else {
        usb_serial_write(why);
    }
    usb_serial_write("\r\n");
}

int main(void)
{
    init_clock();
    rcc_periph_clock_enable(RCC_GPIOB);
    release_gpio();
    init_i2c();
    usb_serial_init();
    while (1) {
        usb_serial_poll();
        char command[32];
        if (!usb_serial_read_line(command, sizeof(command))) {
            continue;
        }
        if (strcmp(command, "diag_id") == 0) {
            usb_serial_write_line("LER0Y_I2C_ID_DIAG_V1 100KHZ NO_MOTOR");
        } else if (strcmp(command, "status") == 0) {
            usb_serial_write("i2c100 idle_sr1=");
            usb_serial_write_u32(I2C_SR1(I2C1));
            usb_serial_write(" idle_sr2=");
            usb_serial_write_u32(I2C_SR2(I2C1));
            usb_serial_write("\r\n");
        } else if (strcmp(command, "probe") == 0) {
            probe_one(0x29U);
            probe_one(0x68U);
            probe_one(0x69U);
            usb_serial_write_line("i2c100 probe_done");
        } else if (strcmp(command, "read_ids") == 0) {
            read_and_report("imu_whoami", 0x68U, 0x0000U, false);
            read_and_report("tof_model_h", 0x29U, 0x010FU, true);
            read_and_report("tof_model_l", 0x29U, 0x0110U, true);
            usb_serial_write_line("i2c100 read_ids_done");
        } else if (strcmp(command, "release") == 0) {
            /* I2C peripheral disabled; pins become open-drain released. */
            i2c_peripheral_disable(I2C1);
            release_gpio();
            usb_serial_write_line("i2c100 released");
        } else {
            usb_serial_write_line("error diag_command_not_allowed");
        }
    }
}
