/* TODO(AUTONOMY) [IMU acquisition fidelity]: Measure onboard sensor axis
 * alignment, timing phase vs camera exposure, temperature-dependent gyro
 * drift, accel noise and magnetometer distortion. Provide versioned
 * calibration and timestamp provenance to the ROS estimator; do not use
 * nominal IMU scale alone as metric pose or yaw accuracy evidence.
 * Acceptance: bench motion and repeated stationary/rotating sessions meet
 * measured timestamp, axis and uncertainty bounds with faults reported.
 */

#include "imu.h"

#include "board.h"
#include "timebase.h"

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/rcc.h>

#define ICM20948_ADDR_LOW   0x68U
#define ICM20948_ADDR_HIGH  0x69U
#define ICM20948_WHOAMI     0xEAU
#define AK09916_ADDR        0x0CU
#define AK09916_WHOAMI      0x09U

/* ICM-20948, user bank 0. */
#define ICM_REG_WHO_AM_I       0x00U
#define ICM_REG_USER_CTRL      0x03U
#define ICM_REG_PWR_MGMT_1     0x06U
#define ICM_REG_PWR_MGMT_2     0x07U
#define ICM_REG_INT_PIN_CFG    0x0FU
#define ICM_REG_ACCEL_XOUT_H   0x2DU
#define ICM_REG_GYRO_XOUT_H    0x33U
#define ICM_REG_TEMP_OUT_H     0x39U
#define ICM_REG_BANK_SEL       0x7FU

/* AK09916 compass, exposed through ICM-20948 bypass mode. */
#define AK_REG_WIA2            0x01U
#define AK_REG_ST1             0x10U
#define AK_REG_HXL             0x11U
#define AK_REG_ST2             0x18U
#define AK_REG_CNTL2           0x31U
#define AK_REG_CNTL3           0x32U

#define I2C_TIMEOUT_MS         10U

/* The ICM-20948 may require up to 100 ms after power-up before register
 * accesses are valid.  The MCU and sensor are normally powered at the same
 * time, so a cold boot can otherwise probe the device too early. */
#define ICM_STARTUP_DELAY_MS       120U
#define ICM_PROBE_RETRY_DELAY_MS    25U
#define ICM_PROBE_ATTEMPTS           8U

/* Startup gyro bias calibration. At ±250 dps the nominal gyro scale is
 * 131 LSB/(degree/s). Reject samples that imply appreciable motion so a
 * moving robot does not poison the zero-rate estimate. */
#define GYRO_CALIBRATION_SAMPLES       128U
#define GYRO_CALIBRATION_MAX_ATTEMPTS  256U
#define GYRO_CALIBRATION_INTERVAL_MS     4U
#define GYRO_STATIONARY_LIMIT_RAW      1000

static bool g_available;
static bool g_mag_available;
static uint8_t g_address;
static uint8_t g_whoami;
static bool g_gyro_calibrated;
static int32_t g_gyro_bias_x;
static int32_t g_gyro_bias_y;
static int32_t g_gyro_bias_z;

static void delay_ms(uint32_t delay)
{
    const uint32_t start = millis();
    while ((uint32_t)(millis() - start) < delay) {
    }
}

static void i2c_configure_peripheral(void)
{
    i2c_peripheral_disable(I2C1);
    i2c_set_speed(I2C1, i2c_speed_fm_400k, 42U);
    i2c_enable_ack(I2C1);
    i2c_peripheral_enable(I2C1);
}

static void i2c_bus_init(void)
{
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);

    gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_PULLUP,
                    IMU_SCL_PIN | IMU_SDA_PIN);
    gpio_set_output_options(GPIOB, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ,
                            IMU_SCL_PIN | IMU_SDA_PIN);
    gpio_set_af(GPIOB, GPIO_AF4, IMU_SCL_PIN | IMU_SDA_PIN);

    rcc_periph_reset_pulse(RST_I2C1);
    i2c_configure_peripheral();
}

static void i2c_recover(void)
{
    i2c_send_stop(I2C1);

    /* Clear the error conditions used by the polling helpers. */
    I2C_SR1(I2C1) &= ~(I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO |
                       I2C_SR1_OVR | I2C_SR1_TIMEOUT);

    rcc_periph_reset_pulse(RST_I2C1);
    i2c_configure_peripheral();
}

static bool i2c_error_present(void)
{
    return (I2C_SR1(I2C1) & (I2C_SR1_AF | I2C_SR1_BERR | I2C_SR1_ARLO |
                             I2C_SR1_OVR | I2C_SR1_TIMEOUT)) != 0U;
}

static bool wait_sr1(uint32_t mask)
{
    const uint32_t start = millis();

    while ((I2C_SR1(I2C1) & mask) == 0U) {
        if (i2c_error_present() ||
            (uint32_t)(millis() - start) >= I2C_TIMEOUT_MS) {
            i2c_recover();
            return false;
        }
    }

    return true;
}

static bool wait_bus_idle(void)
{
    const uint32_t start = millis();

    while ((I2C_SR2(I2C1) & I2C_SR2_BUSY) != 0U) {
        if ((uint32_t)(millis() - start) >= I2C_TIMEOUT_MS) {
            i2c_recover();
            return false;
        }
    }

    return true;
}

static bool write_register(uint8_t address, uint8_t reg, uint8_t value)
{
    if (!wait_bus_idle()) {
        return false;
    }

    i2c_send_start(I2C1);
    if (!wait_sr1(I2C_SR1_SB)) {
        return false;
    }

    i2c_send_7bit_address(I2C1, address, I2C_WRITE);
    if (!wait_sr1(I2C_SR1_ADDR)) {
        return false;
    }
    (void)I2C_SR1(I2C1);
    (void)I2C_SR2(I2C1);

    i2c_send_data(I2C1, reg);
    if (!wait_sr1(I2C_SR1_BTF)) {
        return false;
    }

    i2c_send_data(I2C1, value);
    if (!wait_sr1(I2C_SR1_BTF)) {
        return false;
    }

    i2c_send_stop(I2C1);
    return true;
}

static bool set_register_pointer(uint8_t address, uint8_t reg)
{
    if (!wait_bus_idle()) {
        return false;
    }

    i2c_send_start(I2C1);
    if (!wait_sr1(I2C_SR1_SB)) {
        return false;
    }

    i2c_send_7bit_address(I2C1, address, I2C_WRITE);
    if (!wait_sr1(I2C_SR1_ADDR)) {
        return false;
    }
    (void)I2C_SR1(I2C1);
    (void)I2C_SR2(I2C1);

    i2c_send_data(I2C1, reg);
    if (!wait_sr1(I2C_SR1_BTF)) {
        return false;
    }

    i2c_send_stop(I2C1);
    return true;
}

static bool read_register(uint8_t address, uint8_t reg, uint8_t *value)
{
    if (value == 0 || !set_register_pointer(address, reg) || !wait_bus_idle()) {
        return false;
    }

    i2c_send_start(I2C1);
    if (!wait_sr1(I2C_SR1_SB)) {
        return false;
    }

    i2c_send_7bit_address(I2C1, address, I2C_READ);
    if (!wait_sr1(I2C_SR1_ADDR)) {
        return false;
    }

    /* STM32F4 single-byte master receive sequence: NACK the byte before
     * clearing ADDR, then request STOP before waiting for RXNE. */
    i2c_disable_ack(I2C1);
    (void)I2C_SR1(I2C1);
    (void)I2C_SR2(I2C1);
    i2c_send_stop(I2C1);

    if (!wait_sr1(I2C_SR1_RxNE)) {
        i2c_enable_ack(I2C1);
        return false;
    }

    *value = i2c_get_data(I2C1);
    i2c_enable_ack(I2C1);
    return true;
}

static bool read_registers(uint8_t address, uint8_t first_reg,
                           uint8_t *values, unsigned count)
{
    if (values == 0) {
        return false;
    }

    /* Individual one-byte transactions deliberately avoid the fragile
     * multi-byte ACK/BTF corner cases of STM32's legacy I2C peripheral. Sensor
     * reads are command-driven for now, so this overhead is acceptable. */
    for (unsigned i = 0; i < count; ++i) {
        if (!read_register(address, (uint8_t)(first_reg + i), &values[i])) {
            return false;
        }
    }

    return true;
}

static int16_t be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8U) | p[1]);
}

static int16_t le16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[1] << 8U) | p[0]);
}

static bool select_bank(uint8_t bank);

static int32_t abs_i32(int32_t value)
{
    return value < 0 ? -value : value;
}

static int32_t scale_ratio_rounded(int32_t value, int64_t numerator,
                                   int64_t denominator)
{
    int64_t scaled = (int64_t)value * numerator;
    if (scaled >= 0) {
        scaled += denominator / 2;
    } else {
        scaled -= denominator / 2;
    }
    return (int32_t)(scaled / denominator);
}

/* Default reset range is ±2 g: 16384 LSB/g. Return mm/s^2. */
static int32_t accel_raw_to_mm_s2(int32_t raw)
{
    /* 1 g = 9.80665 m/s^2 = 9806.65 mm/s^2. */
    return scale_ratio_rounded(raw, 980665LL, 1638400LL);
}

/* Default reset range is ±250 dps: 131 LSB/(degree/s). Return mrad/s. */
static int32_t gyro_raw_to_mrad_s(int32_t raw)
{
    /* raw / 131 deg/s * pi / 180 * 1000 mrad/rad. */
    return scale_ratio_rounded(raw, 3141593LL, 23580000LL);
}

static bool read_gyro_raw(int16_t *x, int16_t *y, int16_t *z)
{
    if (x == 0 || y == 0 || z == 0 || !select_bank(0)) {
        return false;
    }

    uint8_t raw[6];
    if (!read_registers(g_address, ICM_REG_GYRO_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    *x = be16(&raw[0]);
    *y = be16(&raw[2]);
    *z = be16(&raw[4]);
    return true;
}

static bool select_bank(uint8_t bank)
{
    return write_register(g_address, ICM_REG_BANK_SEL,
                          (uint8_t)((bank & 0x03U) << 4U));
}

static bool probe_icm(uint8_t address, uint8_t *whoami)
{
    /* REG_BANK_SEL exists at 0x7f in every user bank. */
    if (!write_register(address, ICM_REG_BANK_SEL, 0x00U)) {
        return false;
    }
    return read_register(address, ICM_REG_WHO_AM_I, whoami);
}

static void init_magnetometer(void)
{
    uint8_t who = 0;
    g_mag_available = false;

    /* Release the auxiliary I2C bus and expose AK09916 at address 0x0c. */
    if (!select_bank(0) ||
        !write_register(g_address, ICM_REG_USER_CTRL, 0x00U) ||
        !write_register(g_address, ICM_REG_INT_PIN_CFG, 0x02U)) {
        return;
    }

    delay_ms(2U);

    if (!read_register(AK09916_ADDR, AK_REG_WIA2, &who) ||
        who != AK09916_WHOAMI) {
        return;
    }

    if (!write_register(AK09916_ADDR, AK_REG_CNTL3, 0x01U)) {
        return;
    }
    delay_ms(10U);

    /* Continuous measurement mode 4: 100 Hz, 16-bit output. */
    if (!write_register(AK09916_ADDR, AK_REG_CNTL2, 0x08U)) {
        return;
    }

    g_mag_available = true;
}

void imu_init(void)
{
    g_available = false;
    g_mag_available = false;
    g_address = 0U;
    g_whoami = 0U;
    g_gyro_calibrated = false;
    g_gyro_bias_x = 0;
    g_gyro_bias_y = 0;
    g_gyro_bias_z = 0;

    i2c_bus_init();

    /* On a cold power-up the MCU can reach this point before the IMU is
     * ready to accept register transactions.  Wait beyond the datasheet's
     * maximum register-access startup time, then retry both legal addresses
     * so a transient NACK does not make the IMU unavailable until reset. */
    delay_ms(ICM_STARTUP_DELAY_MS);

    uint8_t who = 0;
    for (unsigned attempt = 0; attempt < ICM_PROBE_ATTEMPTS; ++attempt) {
        if (probe_icm(ICM20948_ADDR_LOW, &who) &&
            who == ICM20948_WHOAMI) {
            g_address = ICM20948_ADDR_LOW;
            break;
        }

        if (probe_icm(ICM20948_ADDR_HIGH, &who) &&
            who == ICM20948_WHOAMI) {
            g_address = ICM20948_ADDR_HIGH;
            break;
        }

        delay_ms(ICM_PROBE_RETRY_DELAY_MS);
    }

    if (g_address == 0U) {
        return;
    }

    g_whoami = who;

    /* Reset, select the gyro PLL clock when available, and enable all accel
     * and gyro axes. The reset returns the device to user bank 0. */
    if (!select_bank(0) ||
        !write_register(g_address, ICM_REG_PWR_MGMT_1, 0x80U)) {
        return;
    }
    delay_ms(100U);

    if (!write_register(g_address, ICM_REG_BANK_SEL, 0x00U) ||
        !write_register(g_address, ICM_REG_PWR_MGMT_1, 0x01U) ||
        !write_register(g_address, ICM_REG_PWR_MGMT_2, 0x00U)) {
        return;
    }
    delay_ms(40U);

    g_available = true;
    init_magnetometer();

    /* Startup calibration is intentionally non-fatal. If the robot is being
     * moved during boot, raw IMU access remains available and calibration can
     * be repeated later with the calibrate_imu_gyro command. */
    (void)imu_calibrate_gyro();
}

bool imu_available(void)
{
    return g_available;
}

bool imu_magnetometer_available(void)
{
    return g_mag_available;
}

uint8_t imu_i2c_address(void)
{
    return g_address;
}

uint8_t imu_who_am_i(void)
{
    return g_whoami;
}

bool imu_read(imu_sample_t *sample)
{
    if (!g_available || sample == 0) {
        return false;
    }

    uint8_t raw[14];
    if (!select_bank(0) ||
        !read_registers(g_address, ICM_REG_ACCEL_XOUT_H, raw, sizeof(raw))) {
        return false;
    }

    sample->accel_x = be16(&raw[0]);
    sample->accel_y = be16(&raw[2]);
    sample->accel_z = be16(&raw[4]);
    sample->gyro_x = be16(&raw[6]);
    sample->gyro_y = be16(&raw[8]);
    sample->gyro_z = be16(&raw[10]);
    sample->temperature_raw = be16(&raw[12]);

    sample->magnetometer_valid = false;
    sample->mag_x = 0;
    sample->mag_y = 0;
    sample->mag_z = 0;

    if (g_mag_available) {
        uint8_t status = 0;
        if (read_register(AK09916_ADDR, AK_REG_ST1, &status) &&
            (status & 0x01U) != 0U) {
            uint8_t mag[6];
            uint8_t st2 = 0;
            if (read_registers(AK09916_ADDR, AK_REG_HXL, mag, sizeof(mag)) &&
                read_register(AK09916_ADDR, AK_REG_ST2, &st2) &&
                (st2 & 0x08U) == 0U) {
                sample->mag_x = le16(&mag[0]);
                sample->mag_y = le16(&mag[2]);
                sample->mag_z = le16(&mag[4]);
                sample->magnetometer_valid = true;
            }
        }
    }

    return true;
}

bool imu_calibrate_gyro(void)
{
    if (!g_available) {
        return false;
    }

    g_gyro_calibrated = false;

    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    unsigned accepted = 0;

    for (unsigned attempt = 0;
         attempt < GYRO_CALIBRATION_MAX_ATTEMPTS &&
         accepted < GYRO_CALIBRATION_SAMPLES;
         ++attempt) {
        int16_t x = 0;
        int16_t y = 0;
        int16_t z = 0;

        if (!read_gyro_raw(&x, &y, &z)) {
            return false;
        }

        /* Ignore samples taken while the chassis is noticeably rotating. */
        if (abs_i32(x) <= GYRO_STATIONARY_LIMIT_RAW &&
            abs_i32(y) <= GYRO_STATIONARY_LIMIT_RAW &&
            abs_i32(z) <= GYRO_STATIONARY_LIMIT_RAW) {
            sum_x += x;
            sum_y += y;
            sum_z += z;
            ++accepted;
        }

        delay_ms(GYRO_CALIBRATION_INTERVAL_MS);
    }

    if (accepted != GYRO_CALIBRATION_SAMPLES) {
        return false;
    }

    /* Round toward the nearest integer raw count. */
    const int32_t half = (int32_t)(GYRO_CALIBRATION_SAMPLES / 2U);
    g_gyro_bias_x = (sum_x >= 0 ? sum_x + half : sum_x - half) /
                    (int32_t)GYRO_CALIBRATION_SAMPLES;
    g_gyro_bias_y = (sum_y >= 0 ? sum_y + half : sum_y - half) /
                    (int32_t)GYRO_CALIBRATION_SAMPLES;
    g_gyro_bias_z = (sum_z >= 0 ? sum_z + half : sum_z - half) /
                    (int32_t)GYRO_CALIBRATION_SAMPLES;
    g_gyro_calibrated = true;
    return true;
}

bool imu_gyro_calibrated(void)
{
    return g_gyro_calibrated;
}

void imu_get_gyro_bias(int32_t *x, int32_t *y, int32_t *z)
{
    if (x != 0) {
        *x = g_gyro_bias_x;
    }
    if (y != 0) {
        *y = g_gyro_bias_y;
    }
    if (z != 0) {
        *z = g_gyro_bias_z;
    }
}

bool imu_read_robot(imu_robot_sample_t *sample)
{
    if (sample == 0) {
        return false;
    }

    imu_sample_t raw;
    if (!imu_read(&raw)) {
        return false;
    }

    /* The assembled robot's IMU is effectively rotated 180 degrees about Z
     * relative to the desired robot frame. This mapping also preserves a
     * right-handed coordinate frame. */
    sample->accel_forward_mm_s2 = accel_raw_to_mm_s2(-(int32_t)raw.accel_x);
    sample->accel_left_mm_s2 = accel_raw_to_mm_s2(-(int32_t)raw.accel_y);
    sample->accel_up_mm_s2 = accel_raw_to_mm_s2((int32_t)raw.accel_z);

    const int32_t corrected_x = (int32_t)raw.gyro_x - g_gyro_bias_x;
    const int32_t corrected_y = (int32_t)raw.gyro_y - g_gyro_bias_y;
    const int32_t corrected_z = (int32_t)raw.gyro_z - g_gyro_bias_z;

    sample->gyro_roll_mrad_s = gyro_raw_to_mrad_s(-corrected_x);
    sample->gyro_pitch_mrad_s = gyro_raw_to_mrad_s(-corrected_y);
    sample->gyro_yaw_mrad_s = gyro_raw_to_mrad_s(corrected_z);

    /* Keep compass data raw for host-side hard/soft-iron calibration and
     * orientation fusion.  The AK09916 axes have not been calibrated into
     * the robot frame yet. */
    sample->magnetometer_valid = raw.magnetometer_valid;
    sample->mag_x_raw = raw.mag_x;
    sample->mag_y_raw = raw.mag_y;
    sample->mag_z_raw = raw.mag_z;

    sample->gyro_calibrated = g_gyro_calibrated;
    return true;
}

