#include "command.h"

#include "encoders.h"
#include "imu.h"
#include "limits.h"
#include "manipulator.h"
#include "platform.h"
#include "timebase.h"
#include "usb_serial.h"

#include <stdbool.h>
#include <stdint.h>

#define COMMAND_BUFFER_SIZE 128U
#define WATCHDOG_MAX_MS 60000U
/* Signed bridge motion protocol: bounded duty and MCU watchdog.
 * These are SOFTWARE constraints, not a physical motor power cut. */
#define SIGNED_WHEEL_PWM_MAX 64
#define SIGNED_WHEEL_WATCHDOG_MAX_MS 500U
#define IMU_STREAM_MIN_TX_FREE 320U
#define IMU_STREAM_BASE_US 1000000U
#define ENCODER_STREAM_MIN_TX_FREE 192U
#define ENCODER_STREAM_BASE_US 1000000U

static uint32_t g_watchdog_ms;
static uint32_t g_last_activity_ms;
static bool g_watchdog_reported;
static bool g_motor_tx_seen;
static uint32_t g_motor_last_tx;

static uint32_t g_imu_stream_hz;
static uint32_t g_imu_stream_period_us;
static uint32_t g_imu_stream_next_us;
static uint32_t g_imu_stream_sequence;
static uint32_t g_imu_stream_dropped;
static uint32_t g_imu_stream_errors;

static uint32_t g_encoder_stream_hz;
static uint32_t g_encoder_stream_period_us;
static uint32_t g_encoder_stream_next_us;
static uint32_t g_encoder_stream_sequence;
static uint32_t g_encoder_stream_dropped;

static bool anything_moving(void)
{
    return platform_wheels_active() ||
           manipulator_get_motion() != MANIPULATOR_STOPPED;
}

static void refresh_activity(void)
{
    g_last_activity_ms = millis();
    g_watchdog_reported = false;
}

static void stop_all(void)
{
    platform_stop();
    manipulator_stop();
}

static bool is_separator(char c)
{
    return c == ' ' || c == '\t' || c == ',';
}

static bool strings_equal(const char *a, const char *b)
{
    if (a == NULL || b == NULL) {
        return false;
    }

    while (*a != '\0' && *b != '\0') {
        if (*a != *b) {
            return false;
        }
        ++a;
        ++b;
    }

    return *a == *b;
}

static bool parse_u32(const char *text, uint32_t *value)
{
    if (text == NULL || *text == '\0' || value == NULL) {
        return false;
    }

    uint32_t parsed = 0U;
    const char *p = text;

    while (*p != '\0') {
        if (*p < '0' || *p > '9') {
            return false;
        }

        const uint32_t digit = (uint32_t)(*p - '0');
        if (parsed > (UINT32_MAX - digit) / 10U) {
            return false;
        }

        parsed = parsed * 10U + digit;
        ++p;
    }

    *value = parsed;
    return true;
}

static bool parse_i32(const char *text, int32_t *value)
{
    if (text == NULL || *text == '\0' || value == NULL) {
        return false;
    }

    bool negative = false;
    if (*text == '-') {
        negative = true;
        ++text;
    } else if (*text == '+') {
        ++text;
    }
    if (*text == '\0') {
        return false;
    }

    uint32_t magnitude = 0U;
    while (*text != '\0') {
        if (*text < '0' || *text > '9') {
            return false;
        }
        const uint32_t digit = (uint32_t)(*text - '0');
        if (magnitude > (2147483648U - digit) / 10U) {
            return false;
        }
        magnitude = magnitude * 10U + digit;
        ++text;
    }

    if (negative) {
        if (magnitude > 2147483648U) {
            return false;
        }
        *value = (magnitude == 2147483648U) ? INT32_MIN : -(int32_t)magnitude;
    } else {
        if (magnitude > INT32_MAX) {
            return false;
        }
        *value = (int32_t)magnitude;
    }
    return true;
}

static bool valid_stream_hz(uint32_t hz)
{
    return hz == 0U || hz == 10U || hz == 20U || hz == 50U || hz == 100U;
}

static bool valid_imu_stream_hz(uint32_t hz)
{
    return valid_stream_hz(hz);
}

static void set_imu_stream_hz(uint32_t hz)
{
    g_imu_stream_hz = hz;
    g_imu_stream_period_us = (hz == 0U) ? 0U : (IMU_STREAM_BASE_US / hz);
    g_imu_stream_next_us = (hz == 0U)
        ? 0U
        : micros() + g_imu_stream_period_us;
}

static void set_encoder_stream_hz(uint32_t hz)
{
    g_encoder_stream_hz = hz;
    g_encoder_stream_period_us = (hz == 0U)
        ? 0U
        : (ENCODER_STREAM_BASE_US / hz);
    g_encoder_stream_next_us = (hz == 0U)
        ? 0U
        : micros() + g_encoder_stream_period_us;
}

static char *split_command(char *line, char **argument)
{
    char *p = line;
    while (*p != '\0' && is_separator(*p)) {
        ++p;
    }

    char *command = p;
    while (*p != '\0' && !is_separator(*p)) {
        ++p;
    }

    if (*p == '\0') {
        *argument = NULL;
        return command;
    }

    *p++ = '\0';
    while (*p != '\0' && is_separator(*p)) {
        ++p;
    }

    if (*p == '\0') {
        *argument = NULL;
        return command;
    }

    /* Keep the full argument tail. Single-value parsers still reject extra
     * tokens, while set_wheels consumes four values. */
    *argument = p;
    return command;
}

static char *next_argument(char **cursor)
{
    if (cursor == NULL || *cursor == NULL) {
        return NULL;
    }

    char *p = *cursor;
    while (*p != '\0' && is_separator(*p)) {
        ++p;
    }
    if (*p == '\0') {
        *cursor = p;
        return NULL;
    }

    char *token = p;
    while (*p != '\0' && !is_separator(*p)) {
        ++p;
    }
    if (*p != '\0') {
        *p++ = '\0';
    }
    *cursor = p;
    return token;
}

static bool parse_wheel_arguments(char *argument, int16_t *fl, int16_t *fr,
                                  int16_t *rl, int16_t *rr)
{
    if (argument == NULL || fl == NULL || fr == NULL || rl == NULL || rr == NULL) {
        return false;
    }

    int32_t values[4];
    char *cursor = argument;
    for (unsigned i = 0; i < 4U; ++i) {
        char *token = next_argument(&cursor);
        if (token == NULL || !parse_i32(token, &values[i]) ||
            values[i] < -255 || values[i] > 255) {
            return false;
        }
    }
    if (next_argument(&cursor) != NULL) {
        return false;
    }

    *fl = (int16_t)values[0];
    *fr = (int16_t)values[1];
    *rl = (int16_t)values[2];
    *rr = (int16_t)values[3];
    return true;
}

static void print_limits(void)
{
    usb_serial_write("bottom=");
    usb_serial_write(bottom_limit_pressed() ? "1" : "0");
    usb_serial_write(" top=");
    usb_serial_write_line(top_limit_pressed() ? "1" : "0");
}

static void print_config(void)
{
    usb_serial_write("platform_speed=");
    usb_serial_write_u32(platform_get_speed());
    usb_serial_write(" platform_ramp=");
    usb_serial_write_u32(platform_get_ramp());
    usb_serial_write(" manipulator_speed=");
    usb_serial_write_u32(manipulator_get_speed());
    usb_serial_write(" manipulator_acceleration=");
    usb_serial_write_u32(manipulator_get_acceleration());
    usb_serial_write(" dir_invert=");
    usb_serial_write(manipulator_get_dir_inverted() ? "1" : "0");
    usb_serial_write(" watchdog_ms=");
    usb_serial_write_u32(g_watchdog_ms);
    usb_serial_write(" imu_stream_hz=");
    usb_serial_write_u32(g_imu_stream_hz);
    usb_serial_write(" encoder_stream_hz=");
    usb_serial_write_u32(g_encoder_stream_hz);
    usb_serial_write("\r\n");
}

static void print_encoders(void)
{
    encoder_counts_t counts;
    encoders_get_counts(&counts);

    usb_serial_write("fr=");
    usb_serial_write_i32(counts.front_right);
    usb_serial_write(" rr=");
    usb_serial_write_i32(counts.rear_right);
    usb_serial_write(" rl=");
    usb_serial_write_i32(counts.rear_left);
    usb_serial_write(" fl=");
    usb_serial_write_i32(counts.front_left);
    usb_serial_write("\r\n");
}

static void write_fixed_milli(int32_t value)
{
    const bool negative = value < 0;
    const uint32_t magnitude = negative
        ? (uint32_t)(-(int64_t)value)
        : (uint32_t)value;
    const uint32_t whole = magnitude / 1000U;
    const uint32_t fraction = magnitude % 1000U;

    if (negative) {
        usb_serial_write("-");
    }
    usb_serial_write_u32(whole);
    usb_serial_write(".");
    if (fraction < 100U) {
        usb_serial_write("0");
    }
    if (fraction < 10U) {
        usb_serial_write("0");
    }
    usb_serial_write_u32(fraction);
}

static void print_imu_robot(void)
{
    if (!imu_available()) {
        usb_serial_write_line("imu_robot=unavailable");
        return;
    }

    imu_robot_sample_t sample;
    if (!imu_read_robot(&sample)) {
        usb_serial_write_line("imu_robot=read_error");
        return;
    }

    usb_serial_write("imu_robot=ok calibrated=");
    usb_serial_write(sample.gyro_calibrated ? "1" : "0");
    usb_serial_write(" accel_forward_mps2=");
    write_fixed_milli(sample.accel_forward_mm_s2);
    usb_serial_write(" accel_left_mps2=");
    write_fixed_milli(sample.accel_left_mm_s2);
    usb_serial_write(" accel_up_mps2=");
    write_fixed_milli(sample.accel_up_mm_s2);
    usb_serial_write(" gyro_roll_radps=");
    write_fixed_milli(sample.gyro_roll_mrad_s);
    usb_serial_write(" gyro_pitch_radps=");
    write_fixed_milli(sample.gyro_pitch_mrad_s);
    usb_serial_write(" gyro_yaw_radps=");
    write_fixed_milli(sample.gyro_yaw_mrad_s);
    usb_serial_write("\r\n");
}

static void print_imu_calibration(void)
{
    int32_t x = 0;
    int32_t y = 0;
    int32_t z = 0;
    imu_get_gyro_bias(&x, &y, &z);

    usb_serial_write("gyro_calibrated=");
    usb_serial_write(imu_gyro_calibrated() ? "1" : "0");
    usb_serial_write(" bias_x_raw=");
    usb_serial_write_i32(x);
    usb_serial_write(" bias_y_raw=");
    usb_serial_write_i32(y);
    usb_serial_write(" bias_z_raw=");
    usb_serial_write_i32(z);
    usb_serial_write("\r\n");
}

static void print_imu_stream(void)
{
    usb_serial_write("imu_stream_hz=");
    usb_serial_write_u32(g_imu_stream_hz);
    usb_serial_write(" seq=");
    usb_serial_write_u32(g_imu_stream_sequence);
    usb_serial_write(" dropped=");
    usb_serial_write_u32(g_imu_stream_dropped);
    usb_serial_write(" errors=");
    usb_serial_write_u32(g_imu_stream_errors);
    usb_serial_write("\r\n");
}

static void print_encoder_stream(void)
{
    usb_serial_write("encoder_stream_hz=");
    usb_serial_write_u32(g_encoder_stream_hz);
    usb_serial_write(" seq=");
    usb_serial_write_u32(g_encoder_stream_sequence);
    usb_serial_write(" dropped=");
    usb_serial_write_u32(g_encoder_stream_dropped);
    usb_serial_write("\r\n");
}

static void print_imu(void)
{
    if (!imu_available()) {
        usb_serial_write_line("imu=unavailable");
        return;
    }

    imu_sample_t sample;
    if (!imu_read(&sample)) {
        usb_serial_write_line("imu=read_error");
        return;
    }

    usb_serial_write("imu=ok address=");
    usb_serial_write_u32(imu_i2c_address());
    usb_serial_write(" whoami=");
    usb_serial_write_u32(imu_who_am_i());
    usb_serial_write(" ax=");
    usb_serial_write_i32(sample.accel_x);
    usb_serial_write(" ay=");
    usb_serial_write_i32(sample.accel_y);
    usb_serial_write(" az=");
    usb_serial_write_i32(sample.accel_z);
    usb_serial_write(" gx=");
    usb_serial_write_i32(sample.gyro_x);
    usb_serial_write(" gy=");
    usb_serial_write_i32(sample.gyro_y);
    usb_serial_write(" gz=");
    usb_serial_write_i32(sample.gyro_z);
    usb_serial_write(" temp_raw=");
    usb_serial_write_i32(sample.temperature_raw);

    if (!imu_magnetometer_available()) {
        usb_serial_write(" mag=unavailable");
    } else if (!sample.magnetometer_valid) {
        usb_serial_write(" mag=not_ready");
    } else {
        usb_serial_write(" mag=ok mx=");
        usb_serial_write_i32(sample.mag_x);
        usb_serial_write(" my=");
        usb_serial_write_i32(sample.mag_y);
        usb_serial_write(" mz=");
        usb_serial_write_i32(sample.mag_z);
    }

    usb_serial_write("\r\n");
}

static void print_status(void)
{
    usb_serial_write("platform=");
    if (platform_get_motion() == PLATFORM_STOPPED && platform_wheels_active()) {
        usb_serial_write("wheels");
    } else {
        usb_serial_write(platform_motion_name(platform_get_motion()));
    }
    usb_serial_write(" manipulator=");
    usb_serial_write(manipulator_motion_name(manipulator_get_motion()));
    usb_serial_write(" manipulator_position=");
    usb_serial_write_i32(manipulator_get_position());
    usb_serial_write(" bottom=");
    usb_serial_write(bottom_limit_pressed() ? "1" : "0");
    usb_serial_write(" top=");
    usb_serial_write(top_limit_pressed() ? "1" : "0");
    usb_serial_write(" watchdog_ms=");
    usb_serial_write_u32(g_watchdog_ms);
    usb_serial_write("\r\n");
}

static bool parse_motor_tx(char *tail, uint32_t *seq,
                           int16_t *fl, int16_t *fr,
                           int16_t *rl, int16_t *rr)
{
    if (tail == NULL) {
        return false;
    }
    char *cursor = tail;
    const char *raw_seq = next_argument(&cursor);
    if (!parse_u32(raw_seq, seq)) {
        return false;
    }
    int32_t values[4];
    for (unsigned i = 0U; i < 4U; ++i) {
        char *token = next_argument(&cursor);
        if (!parse_i32(token, &values[i]) ||
            values[i] < -SIGNED_WHEEL_PWM_MAX ||
            values[i] > SIGNED_WHEEL_PWM_MAX) {
            return false;
        }
    }
    if (next_argument(&cursor) != NULL) {
        return false;
    }
    *fl = (int16_t)values[0];
    *fr = (int16_t)values[1];
    *rl = (int16_t)values[2];
    *rr = (int16_t)values[3];
    return true;
}

static void print_help(void)
{
    usb_serial_write_line("commands:");
    usb_serial_write_line("  move_platform_forward");
    usb_serial_write_line("  move_platform_backward");
    usb_serial_write_line("  move_platform_left");
    usb_serial_write_line("  move_platform_right");
    usb_serial_write_line("  rotate_platform_cw");
    usb_serial_write_line("  rotate_platform_ccw");
    usb_serial_write_line("  move_manipulators_up");
    usb_serial_write_line("  move_manipulators_down");
    usb_serial_write_line("  stop_platform");
    usb_serial_write_line("  stop_manipulators");
    usb_serial_write_line("  stop_all");
    usb_serial_write_line("  set_platform_speed <0..255>");
    usb_serial_write_line("  set_platform_ramp <1..255>");
    usb_serial_write_line("  set_wheels 0 0 0 0 (legacy emergency zero only)");
    usb_serial_write_line("  set_wheels_tx <seq:u32> <fl:-64..64> <fr:-64..64> <rl:-64..64> <rr:-64..64>");
    usb_serial_write_line("  stop_wheels_tx <seq:u32>");
    usb_serial_write_line("  set_manipulator_speed <1..5000>");
    usb_serial_write_line("  set_manipulator_acceleration <1..10000>");
    usb_serial_write_line("  set_manipulator_dir_invert <0|1>");
    usb_serial_write_line("  set_watchdog_ms <0..60000>");
    usb_serial_write_line("  get_status");
    usb_serial_write_line("  get_limits");
    usb_serial_write_line("  get_config");
    usb_serial_write_line("  get_encoders");
    usb_serial_write_line("  reset_encoders");
    usb_serial_write_line("  get_imu");
    usb_serial_write_line("  get_imu_robot");
    usb_serial_write_line("  get_imu_calibration");
    usb_serial_write_line("  calibrate_imu_gyro");
    usb_serial_write_line("  set_imu_stream_hz <0|10|20|50|100>");
    usb_serial_write_line("  get_imu_stream");
    usb_serial_write_line("  set_encoder_stream_hz <0|10|20|50|100>");
    usb_serial_write_line("  get_encoder_stream");
    usb_serial_write_line("  keepalive");
    usb_serial_write_line("  help");
}

static void process_command(char *line)
{
    char *argument = NULL;
    char *command = split_command(line, &argument);

    if (command == NULL || *command == '\0') {
        return;
    }

    /* Legacy open-loop patterns cannot bypass signed bridge ownership. */
    if (strings_equal(command, "move_platform_forward") ||
        strings_equal(command, "move_platform_backward") ||
        strings_equal(command, "move_platform_left") ||
        strings_equal(command, "move_platform_right") ||
        strings_equal(command, "rotate_platform_cw") ||
        strings_equal(command, "rotate_platform_ccw")) {
        usb_serial_write_line("error legacy_motor_motion_disabled");
        return;
    }

    if (strings_equal(command, "move_platform_forward")) {
        platform_set_motion(PLATFORM_FORWARD);
        refresh_activity();
        usb_serial_write_line("ok move_platform_forward");
        return;
    }
    if (strings_equal(command, "move_platform_backward")) {
        platform_set_motion(PLATFORM_BACKWARD);
        refresh_activity();
        usb_serial_write_line("ok move_platform_backward");
        return;
    }
    if (strings_equal(command, "move_platform_left")) {
        platform_set_motion(PLATFORM_LEFT);
        refresh_activity();
        usb_serial_write_line("ok move_platform_left");
        return;
    }
    if (strings_equal(command, "move_platform_right")) {
        platform_set_motion(PLATFORM_RIGHT);
        refresh_activity();
        usb_serial_write_line("ok move_platform_right");
        return;
    }
    if (strings_equal(command, "rotate_platform_cw")) {
        platform_set_motion(PLATFORM_CW);
        refresh_activity();
        usb_serial_write_line("ok rotate_platform_cw");
        return;
    }
    if (strings_equal(command, "rotate_platform_ccw")) {
        platform_set_motion(PLATFORM_CCW);
        refresh_activity();
        usb_serial_write_line("ok rotate_platform_ccw");
        return;
    }

    if (strings_equal(command, "move_manipulators_up")) {
        if (!manipulator_move_up()) {
            usb_serial_write_line("blocked top_limit");
        } else {
            refresh_activity();
            usb_serial_write_line("ok move_manipulators_up");
        }
        return;
    }
    if (strings_equal(command, "move_manipulators_down")) {
        if (!manipulator_move_down()) {
            usb_serial_write_line("blocked bottom_limit");
        } else {
            refresh_activity();
            usb_serial_write_line("ok move_manipulators_down");
        }
        return;
    }

    if (strings_equal(command, "stop_platform")) {
        platform_stop();
        refresh_activity();
        usb_serial_write_line("ok stop_platform");
        return;
    }
    if (strings_equal(command, "stop_manipulators")) {
        manipulator_stop();
        refresh_activity();
        usb_serial_write_line("ok stop_manipulators");
        return;
    }
    if (strings_equal(command, "stop_all")) {
        stop_all();
        refresh_activity();
        usb_serial_write_line("ok stop_all");
        return;
    }

    uint32_t value = 0;

    if (strings_equal(command, "set_platform_speed")) {
        if (!parse_u32(argument, &value) || value > 255U ||
            !platform_set_speed((uint16_t)value)) {
            usb_serial_write_line("error set_platform_speed expects 0..255");
        } else {
            usb_serial_write_line("ok set_platform_speed");
        }
        return;
    }

    if (strings_equal(command, "set_platform_ramp")) {
        if (!parse_u32(argument, &value) || value < 1U || value > 255U ||
            !platform_set_ramp((uint16_t)value)) {
            usb_serial_write_line("error set_platform_ramp expects 1..255");
        } else {
            usb_serial_write_line("ok set_platform_ramp");
        }
        return;
    }

    if (strings_equal(command, "stop_wheels_tx")) {
        uint32_t seq = 0U;
        if (!parse_u32(argument, &seq)) {
            usb_serial_write_line("error stop_wheels_tx invalid_sequence");
            return;
        }
        /* Stopping is unconditional, even with a stale sequence.
         * Explicitly disable wheel driver enables, not just zero duty. */
        platform_disable();
        if (!g_motor_tx_seen || seq > g_motor_last_tx) {
            g_motor_last_tx = seq;
            g_motor_tx_seen = true;
        }
        usb_serial_write("ok stop_wheels_tx seq=");
        usb_serial_write_u32(seq);
        usb_serial_write("\r\n");
        return;
    }
    if (strings_equal(command, "set_wheels_tx")) {
        uint32_t seq = 0U;
        int16_t fl = 0, fr = 0, rl = 0, rr = 0;
        if (!parse_motor_tx(argument, &seq, &fl, &fr, &rl, &rr)) {
            usb_serial_write_line("error set_wheels_tx invalid_arguments");
        } else if (g_watchdog_ms == 0U ||
                   g_watchdog_ms > SIGNED_WHEEL_WATCHDOG_MAX_MS) {
            usb_serial_write_line("error set_wheels_tx watchdog_not_bounded");
        } else if (g_motor_tx_seen && seq <= g_motor_last_tx) {
            usb_serial_write_line("error set_wheels_tx stale_sequence");
        } else {
            g_motor_tx_seen = true;
            g_motor_last_tx = seq;
            platform_set_wheels(fl, fr, rl, rr);
            refresh_activity();
            usb_serial_write("ok set_wheels_tx seq=");
            usb_serial_write_u32(seq);
            usb_serial_write("\r\n");
        }
        return;
    }
    if (strings_equal(command, "set_wheels")) {
        int16_t fl = 0;
        int16_t fr = 0;
        int16_t rl = 0;
        int16_t rr = 0;
        if (!parse_wheel_arguments(argument, &fl, &fr, &rl, &rr)) {
            usb_serial_write_line("error set_wheels expects fl fr rl rr in -255..255");
        } else if (fl != 0 || fr != 0 || rl != 0 || rr != 0) {
            usb_serial_write_line("error legacy_nonzero_wheels_disabled");
        } else {
            platform_disable();
            refresh_activity();
            usb_serial_write_line("ok set_wheels");
        }
        return;
    }

    if (strings_equal(command, "set_manipulator_speed")) {
        if (!parse_u32(argument, &value) || !manipulator_set_speed(value)) {
            usb_serial_write_line("error set_manipulator_speed expects 1..5000");
        } else {
            usb_serial_write_line("ok set_manipulator_speed");
        }
        return;
    }

    if (strings_equal(command, "set_manipulator_acceleration")) {
        if (!parse_u32(argument, &value) ||
            !manipulator_set_acceleration(value)) {
            usb_serial_write_line("error set_manipulator_acceleration expects 1..10000");
        } else {
            usb_serial_write_line("ok set_manipulator_acceleration");
        }
        return;
    }

    if (strings_equal(command, "set_manipulator_dir_invert")) {
        if (!parse_u32(argument, &value) || value > 1U) {
            usb_serial_write_line("error set_manipulator_dir_invert expects 0 or 1");
        } else if (!manipulator_set_dir_inverted(value != 0U)) {
            usb_serial_write_line("error stop_manipulators before changing direction inversion");
        } else {
            usb_serial_write_line("ok set_manipulator_dir_invert");
        }
        return;
    }

    if (strings_equal(command, "set_imu_stream_hz")) {
        if (!parse_u32(argument, &value) || !valid_imu_stream_hz(value)) {
            usb_serial_write_line("error set_imu_stream_hz expects 0,10,20,50,100");
        } else if (value != 0U && !imu_available()) {
            usb_serial_write_line("error imu_unavailable");
        } else {
            set_imu_stream_hz(value);
            usb_serial_write_line("ok set_imu_stream_hz");
        }
        return;
    }

    if (strings_equal(command, "set_encoder_stream_hz")) {
        if (!parse_u32(argument, &value) || !valid_stream_hz(value)) {
            usb_serial_write_line("error set_encoder_stream_hz expects 0,10,20,50,100");
        } else {
            set_encoder_stream_hz(value);
            usb_serial_write_line("ok set_encoder_stream_hz");
        }
        return;
    }

    if (strings_equal(command, "set_watchdog_ms")) {
        if (!parse_u32(argument, &value) || value > WATCHDOG_MAX_MS) {
            usb_serial_write_line("error set_watchdog_ms expects 0..60000");
        } else {
            g_watchdog_ms = value;
            refresh_activity();
            usb_serial_write_line("ok set_watchdog_ms");
        }
        return;
    }

    if (strings_equal(command, "get_status")) {
        print_status();
        return;
    }
    if (strings_equal(command, "get_encoders")) {
        print_encoders();
        return;
    }
    if (strings_equal(command, "reset_encoders")) {
        encoders_reset();
        usb_serial_write_line("ok reset_encoders");
        return;
    }
    if (strings_equal(command, "get_imu")) {
        print_imu();
        return;
    }
    if (strings_equal(command, "get_imu_robot")) {
        print_imu_robot();
        return;
    }
    if (strings_equal(command, "get_imu_calibration")) {
        print_imu_calibration();
        return;
    }
    if (strings_equal(command, "get_imu_stream")) {
        print_imu_stream();
        return;
    }
    if (strings_equal(command, "get_encoder_stream")) {
        print_encoder_stream();
        return;
    }
    if (strings_equal(command, "calibrate_imu_gyro")) {
        if (anything_moving()) {
            usb_serial_write_line("error calibrate_imu_gyro requires robot stopped");
        } else if (!imu_available()) {
            usb_serial_write_line("error imu_unavailable");
        } else if (!imu_calibrate_gyro()) {
            usb_serial_write_line("error calibrate_imu_gyro failed keep robot stationary");
        } else {
            usb_serial_write_line("ok calibrate_imu_gyro");
        }
        return;
    }
    if (strings_equal(command, "get_limits")) {
        print_limits();
        return;
    }
    if (strings_equal(command, "get_config")) {
        print_config();
        return;
    }
    if (strings_equal(command, "keepalive")) {
        refresh_activity();
        usb_serial_write_line("ok keepalive");
        return;
    }
    if (strings_equal(command, "help") || strings_equal(command, "?")) {
        print_help();
        return;
    }

    usb_serial_write("error unknown_command ");
    usb_serial_write_line(command);
}

static void update_watchdog(void)
{
    if (g_watchdog_ms == 0U || !anything_moving()) {
        return;
    }

    if ((uint32_t)(millis() - g_last_activity_ms) > g_watchdog_ms) {
        stop_all();
        if (!g_watchdog_reported) {
            usb_serial_write_line("event watchdog_stop");
            g_watchdog_reported = true;
        }
    }
}

void command_init(void)
{
    g_watchdog_ms = 0;
    g_last_activity_ms = millis();
    g_watchdog_reported = false;
    g_motor_tx_seen = false;
    g_motor_last_tx = 0U;

    g_imu_stream_hz = 0U;
    g_imu_stream_period_us = 0U;
    g_imu_stream_next_us = 0U;
    g_imu_stream_sequence = 0U;
    g_imu_stream_dropped = 0U;
    g_imu_stream_errors = 0U;

    g_encoder_stream_hz = 0U;
    g_encoder_stream_period_us = 0U;
    g_encoder_stream_next_us = 0U;
    g_encoder_stream_sequence = 0U;
    g_encoder_stream_dropped = 0U;

    /*
     * Intentionally do not transmit a startup banner here.
     *
     * On Linux, a newly opened ttyACM device can briefly have the terminal
     * ECHO flag enabled before the client switches it to raw/no-echo mode.
     * Unsolicited bytes sent during that window can therefore be echoed by
     * the host back into this CDC OUT endpoint and be mistaken for a command.
     * The control protocol is request/response at startup instead.
     */
}

void command_poll(void)
{
    char line[COMMAND_BUFFER_SIZE];

    if (usb_serial_read_line(line, sizeof(line))) {
        process_command(line);
    }

    update_watchdog();
}

void command_emit_async_events(void)
{
    const manipulator_event_t event = manipulator_take_event();
    if (event == MANIP_EVENT_TOP_LIMIT) {
        usb_serial_write_line("event top_limit_stop");
    } else if (event == MANIP_EVENT_BOTTOM_LIMIT) {
        usb_serial_write_line("event bottom_limit_stop position_zeroed");
    }
}

static void emit_imu_telemetry(void)
{
    if (g_imu_stream_hz == 0U || !imu_available()) {
        return;
    }

    const uint32_t now = micros();
    if ((int32_t)(now - g_imu_stream_next_us) < 0) {
        return;
    }

    g_imu_stream_next_us += g_imu_stream_period_us;
    if ((int32_t)(now - g_imu_stream_next_us) >= 0) {
        g_imu_stream_next_us = now + g_imu_stream_period_us;
    }

    if (usb_serial_tx_free() < IMU_STREAM_MIN_TX_FREE) {
        ++g_imu_stream_dropped;
        return;
    }

    imu_robot_sample_t sample;
    if (!imu_read_robot(&sample)) {
        ++g_imu_stream_errors;
        return;
    }

    const uint32_t timestamp_us = micros();
    const uint32_t sequence = ++g_imu_stream_sequence;

    usb_serial_write("telemetry imu seq=");
    usb_serial_write_u32(sequence);
    usb_serial_write(" t_us=");
    usb_serial_write_u32(timestamp_us);
    usb_serial_write(" cal=");
    usb_serial_write(sample.gyro_calibrated ? "1" : "0");
    usb_serial_write(" af_mm_s2=");
    usb_serial_write_i32(sample.accel_forward_mm_s2);
    usb_serial_write(" al_mm_s2=");
    usb_serial_write_i32(sample.accel_left_mm_s2);
    usb_serial_write(" au_mm_s2=");
    usb_serial_write_i32(sample.accel_up_mm_s2);
    usb_serial_write(" gr_mrad_s=");
    usb_serial_write_i32(sample.gyro_roll_mrad_s);
    usb_serial_write(" gp_mrad_s=");
    usb_serial_write_i32(sample.gyro_pitch_mrad_s);
    usb_serial_write(" gy_mrad_s=");
    usb_serial_write_i32(sample.gyro_yaw_mrad_s);
    usb_serial_write(" mag_valid=");
    usb_serial_write(sample.magnetometer_valid ? "1" : "0");
    usb_serial_write(" mx_raw=");
    usb_serial_write_i32(sample.mag_x_raw);
    usb_serial_write(" my_raw=");
    usb_serial_write_i32(sample.mag_y_raw);
    usb_serial_write(" mz_raw=");
    usb_serial_write_i32(sample.mag_z_raw);
    usb_serial_write("\r\n");
}

static void emit_encoder_telemetry(void)
{
    if (g_encoder_stream_hz == 0U) {
        return;
    }

    const uint32_t now = micros();
    if ((int32_t)(now - g_encoder_stream_next_us) < 0) {
        return;
    }

    g_encoder_stream_next_us += g_encoder_stream_period_us;
    if ((int32_t)(now - g_encoder_stream_next_us) >= 0) {
        g_encoder_stream_next_us = now + g_encoder_stream_period_us;
    }

    if (usb_serial_tx_free() < ENCODER_STREAM_MIN_TX_FREE) {
        ++g_encoder_stream_dropped;
        return;
    }

    const uint32_t timestamp_us = micros();
    encoder_counts_t counts;
    encoders_get_counts(&counts);
    const uint32_t sequence = ++g_encoder_stream_sequence;

    usb_serial_write("telemetry enc seq=");
    usb_serial_write_u32(sequence);
    usb_serial_write(" t_us=");
    usb_serial_write_u32(timestamp_us);
    usb_serial_write(" fr=");
    usb_serial_write_i32(counts.front_right);
    usb_serial_write(" rr=");
    usb_serial_write_i32(counts.rear_right);
    usb_serial_write(" rl=");
    usb_serial_write_i32(counts.rear_left);
    usb_serial_write(" fl=");
    usb_serial_write_i32(counts.front_left);
    usb_serial_write("\r\n");
}

void command_emit_telemetry(void)
{
    emit_imu_telemetry();
    emit_encoder_telemetry();
}
