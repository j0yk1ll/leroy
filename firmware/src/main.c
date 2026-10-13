#include "command.h"
#include "encoders.h"
#include "imu.h"
#include "limits.h"
#include "manipulator.h"
#include "platform.h"
#include "timebase.h"
#include "usb_serial.h"

int main(void)
{
    clock_init();
    timebase_init();

    platform_init();
    encoders_init();
    limits_init();
    manipulator_init();
    imu_init();
    usb_serial_init();

    fast_tick_init();
    command_init();

    while (1) {
        usb_serial_poll();
        command_poll();
        platform_update();
        manipulator_update();
        command_emit_async_events();
        command_emit_telemetry();
    }
}
