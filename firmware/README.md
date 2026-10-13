# STM32F401 Robot Controller — bare-metal C

Standalone C firmware for the STM32F401 Black Pill robot controller.

The project uses:

- `arm-none-eabi-gcc` for compilation
- `libopencm3` for STM32 register/peripheral definitions and USB device support
- `stlink` / `st-flash` for programming through SWD
- native USB CDC ACM on PA11/PA12 for the remote-control command interface

The firmware preserves the platform motor mapping from the working Arduino firmware and uses the manipulator wiring verified during testing.

## Hardware target

- MCU: **STM32F401CEU6**
- Typical board: STM32F401 Black Pill / WeAct-style Black Pill
- Flash: 256 KiB
- SRAM: 64 KiB
- System clock: 84 MHz
- Clock assumption: **25 MHz HSE crystal**

If your particular F401 board has a different external crystal, change `clock_init()` in `src/timebase.c` before flashing.

## Pin map

All board-specific definitions are centralized in `include/board.h`.

### Platform — BTS7960 drivers

| Wheel / driver | RPWM | LPWM | EN |
|---|---:|---:|---:|
| Front-right / DRV1 | PA0 | PA1 | PB0 |
| Rear-right / DRV2 | PA2 | PA3 | PB1 |
| Rear-left / DRV3 | PA4 | PA5 | PB10 |
| Front-left / DRV4 | PA6 | PA7 | PB12 |

Rear-left and front-left are inverted in software to match the tested mechanical orientation.

### Wheel encoders

| Encoder | C1 | C2 |
|---|---:|---:|
| Front-right | PA8 | PA9 |
| Rear-right | PA15 | PB3 |
| Rear-left | PB4 | PB5 |
| Front-left | PB6 | PB7 |

All four encoders are decoded in x4 quadrature mode using EXTI interrupts on both A/B edges. Counts are signed raw transition counts and are exposed with `get_encoders`. Direction sign depends on the physical A/B wiring; verify it on the robot before using the counts for odometry.

### Manipulator stepper / DRV8825

| Signal | Pin |
|---|---:|
| DIR | PA10 |
| STEP | PB14 |
| Bottom limit | PC14 |
| Top limit | PC15 |

The limit switches are NC switches wired to ground with the MCU internal pull-ups enabled:

- `0` = clear / normal closed circuit
- `1` = limit pressed **or wire open**

This gives fail-safe behavior for a broken limit-switch wire.

### IMU — GY-ICM20948V2

| Function | Pins |
|---|---|
| I2C1 SCL | PB8 |
| I2C1 SDA | PB9 |

The firmware probes ICM-20948 addresses `0x68` and `0x69`, verifies `WHO_AM_I = 0xEA`, wakes the accelerometer/gyroscope, and enables bypass access to the internal AK09916 magnetometer at `0x0C`. I2C runs at 400 kHz. The driver returns raw sensor values first; calibration and frame transforms should be added only after the physical axis orientation is verified.

### Reserved/debug

| Function | Pins |
|---|---|
| Native USB FS | PA11 / PA12 |
| SWD | PA13 / PA14 |

Do not repurpose PA13/PA14 if you want to retain ST-Link debugging/programming.

---

# 1. Install dependencies on Arch Linux

Update the system first:

```bash
sudo pacman -Syu
```

Install the compiler, C library, STM32 support library, build tool, ST-Link utilities, and an optional serial terminal:

```bash
sudo pacman -S --needed \
    make \
    arm-none-eabi-binutils \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    arm-none-eabi-newlib \
    libopencm3 \
    stlink \
    picocom
```

The project intentionally uses the Arch-packaged `libopencm3`; you do not need to clone or build libopencm3 separately.

Verify the important tools:

```bash
arm-none-eabi-gcc --version
arm-none-eabi-objcopy --version
st-info --version
st-flash --version
```

The Arch `libopencm3` package installs its headers and STM32F4 library under `/usr/arm-none-eabi/`, which is what the included `Makefile` expects.

---

# 2. Build the firmware

From this project directory:

```bash
make build
```

Successful output is placed in `build/`:

```text
build/robot_firmware.elf   ELF with symbols/debug information
build/robot_firmware.bin   raw image used by st-flash
build/robot_firmware.hex   Intel HEX image
build/robot_firmware.map   linker map
```

The build also runs `arm-none-eabi-size` so you can see flash/RAM usage.

To rebuild from scratch:

```bash
make clean
make build
```

---

# 3. Connect the ST-Link

Power the robot electronics safely before connecting the programmer. The minimum SWD connections are:

| ST-Link | STM32F401 |
|---|---|
| SWDIO | PA13 / SWDIO |
| SWCLK | PA14 / SWCLK |
| GND | GND |
| VTref / 3.3 V sense | 3.3 V target reference, if your ST-Link requires it |
| NRST | NRST, optional but recommended |

## Power warning

The ST-Link and robot power system must share ground.

Do **not** connect an ST-Link 5 V output to a 3.3 V rail. Also avoid powering the target simultaneously from multiple sources unless you know how your specific ST-Link board and robot power rails are wired.

For many ST-Link adapters, the `3.3V`/`VREF` pin is used to sense target voltage; clones vary, so check the labels/documentation for your programmer.

---

# 4. Verify the ST-Link connection

With the target powered:

```bash
st-info --probe
```

You should see an STM32 device detected with flash/SRAM information.

If no target is detected, check:

1. target power
2. common ground
3. SWDIO and SWCLK are not swapped
4. PA13/PA14 are not being driven by other hardware
5. NRST connection if the MCU is difficult to attach to

The Arch `stlink` package installs udev rules for supported ST-Link programmers. If you installed the package while the programmer was already plugged in, unplug/replug it and try again.

---

# 5. Flash through ST-Link

Clean, build, and flash with the default target:

```bash
make
```

Build and flash in one command:

```bash
make flash
```

This runs the equivalent of:

```bash
st-flash --reset write build/robot_firmware.bin 0x08000000
```

The STM32F401 internal flash starts at `0x08000000`.

To mass-erase the MCU:

```bash
make erase
```

To reset it through ST-Link:

```bash
make reset
```

---

# 6. Remote-control connection

The firmware exposes a **USB CDC ACM** serial device through the Black Pill native USB pins PA11/PA12.

After flashing and resetting, connect the board's USB data connector to Linux and inspect:

```bash
dmesg --follow
```

or:

```bash
ls /dev/ttyACM*
```

A typical device is:

```text
/dev/ttyACM0
```

Open it with:

```bash
picocom -b 115200 /dev/ttyACM0
```

The `115200` setting is kept for convenience and compatibility with the old serial workflow; USB CDC itself is not clocked like a physical 115200-baud UART.

Commands are ASCII, one command per line.

---

# 7. Remote command interface

## Platform movement

```text
move_platform_forward
move_platform_backward
move_platform_left
move_platform_right
rotate_platform_cw
rotate_platform_ccw
stop_platform
```

Platform movement is persistent: a movement command continues until another platform command, `stop_platform`, `stop_all`, or the optional communications watchdog stops it.

## Manipulator movement

```text
move_manipulators_up
move_manipulators_down
stop_manipulators
```

Manipulator movement is persistent. Safety behavior is directional:

- upward movement stops at the top limit
- downward movement stops at the bottom limit
- a pressed top limit does not prevent moving down
- a pressed bottom limit does not prevent moving up
- hitting the bottom limit resets the internal manipulator step position to `0`

The position is open-loop. It counts commanded STEP pulses; there is no stepper encoder, so it cannot detect a missed mechanical step.

## Stop everything

```text
stop_all
```

## Configuration

Platform speed, 0–255:

```text
set_platform_speed 100
```

Platform ramp increment, 1–255 every 10 ms:

```text
set_platform_ramp 32
```

Manipulator maximum speed in steps/s:

```text
set_manipulator_speed 50
```

Manipulator acceleration in steps/s²:

```text
set_manipulator_acceleration 10
```

Invert manipulator direction mapping (only while stopped):

```text
set_manipulator_dir_invert 0
set_manipulator_dir_invert 1
```

Enable a communications watchdog, for example 1000 ms:

```text
set_watchdog_ms 1000
```

Disable the watchdog:

```text
set_watchdog_ms 0
```

When the watchdog is enabled, send periodically while motion is active:

```text
keepalive
```

Movement commands also refresh the watchdog timer.

## Status

```text
get_status
get_limits
get_config
get_encoders
reset_encoders
get_imu
get_imu_robot
get_imu_calibration
calibrate_imu_gyro
set_imu_stream_hz <0|10|20|50|100>
get_imu_stream
help
```

Examples:

```text
get_limits
bottom=0 top=0

get_encoders
fr=1248 rr=1251 rl=-1245 fl=-1249

get_imu
imu=ok address=105 whoami=234 ax=... ay=... az=... gx=... gy=... gz=... temp_raw=... mag=ok mx=... my=... mz=...
```

Encoder counts are x4 quadrature transitions, not revolutions or meters. Determine counts per wheel revolution experimentally before computing wheel speed or odometry.

`get_imu` reports raw signed 16-bit diagnostics. After reset the ICM-20948 defaults correspond to ±2 g accelerometer and ±250 dps gyro ranges unless those configuration registers are changed later. At those defaults the nominal scale factors are 16384 LSB/g and 131 LSB/(degree/s). Magnetometer data is reported only when the AK09916 data-ready flag is set.

At startup the firmware also attempts a stationary gyro zero-rate calibration. It gathers 128 low-motion samples and averages the raw gyro offsets. Calibration failure is non-fatal: raw IMU access remains available and `calibrate_imu_gyro` can be run later while the robot is stopped. `get_imu_calibration` reports the current raw bias values and calibration state.

`get_imu_robot` returns calibrated robot-frame values in physical units as fixed decimal text. The measured mounting transform is `forward=-ICM_X`, `left=-ICM_Y`, `up=+ICM_Z`; positive yaw is counter-clockwise and maps to `+ICM_GZ`. Output fields use m/s^2 for acceleration and rad/s for angular velocity. Example:

```text
imu_robot=ok calibrated=1 accel_forward_mps2=-0.120 accel_left_mps2=0.024 accel_up_mps2=9.821 gyro_roll_radps=0.001 gyro_pitch_radps=-0.002 gyro_yaw_radps=0.000
```

For state estimation on the Jetson, use the firmware-driven IMU stream instead of polling. Streaming is disabled by default. Enable one of the supported rates with `set_imu_stream_hz 10`, `20`, `50`, or `100`; use `set_imu_stream_hz 0` to disable it. `get_imu_stream` reports the configured rate, emitted sequence number, samples dropped because of USB backpressure, and IMU read errors.

Stream records are machine-oriented integer telemetry:

```text
telemetry imu seq=42 t_us=1864203 cal=1 af_mm_s2=-86 al_mm_s2=153 au_mm_s2=9888 gr_mrad_s=-4 gp_mrad_s=2 gy_mrad_s=-2 mag_valid=1 mx_raw=26 my_raw=542 mz_raw=-399
```

`t_us` is a monotonic 32-bit MCU timestamp in microseconds and wraps naturally after about 71.6 minutes; calculate timestamp deltas with unsigned 32-bit subtraction. Acceleration is reported in mm/s^2 and angular velocity in mrad/s so the host can parse exact integers and convert to SI (`/1000.0`) without floating-point formatting on the MCU. Magnetometer values remain raw and explicitly uncalibrated for host-side hard/soft-iron calibration and sensor fusion. Sequence numbers increase only for emitted samples. If USB is backpressured, the firmware drops telemetry rather than delaying motion commands, command responses, or safety events.

---

# 8. Implementation notes

## No Arduino or AccelStepper

The firmware is normal C. `main()` initializes the STM32 peripherals and then continuously services USB and the command parser.

Stepper timing is generated from a 20 kHz TIM4 interrupt instead of AccelStepper. The main loop updates the acceleration profile, while the interrupt produces deterministic STEP pulses.

Defaults are intentionally conservative because the vertical mechanism previously slipped at higher speeds:

```text
manipulator speed        = 50 steps/s
manipulator acceleration = 10 steps/s²
```

## Encoder and IMU acquisition

Encoder decoding lives in `src/encoders.c`. Each of the eight encoder signals occupies a unique EXTI line, so both edges can be captured without EXTI source conflicts. Invalid two-bit quadrature jumps are discarded rather than counted.

The ICM-20948 driver lives in `src/imu.c`. It uses I2C1 on PB8/PB9 with bounded polling timeouts so a missing or disconnected IMU does not prevent the robot firmware from booting. The initial interface is intentionally query-driven instead of continuously streaming sensor data. This keeps USB traffic predictable while hardware polarity, scaling, and calibration are being verified.

## Platform PWM

The existing wheel pin layout uses PA0 through PA7 for the eight BTS7960 RPWM/LPWM inputs. Those eight exact pins cannot all be mapped to eight independent hardware timer PWM channels on the STM32F401.

To preserve the tested wiring, `platform.c` uses **20 kHz 8-bit pulse-density modulation** from TIM4. This produces the same 0–255 average duty command without changing the physical pin map.

If you later redesign the PCB/pinout, moving the wheel PWM signals onto dedicated timer channels would reduce interrupt load and produce conventional hardware PWM.

## USB clock

Native USB requires an accurate 48 MHz USB clock. The project uses the common Black Pill 25 MHz HSE crystal and configures the STM32 to 84 MHz with a valid 48 MHz USB PLL output.

If the board runs but Linux never enumerates a USB device, verify the board's HSE frequency first.

## USB VBUS sensing

PA9 is already reserved by the front-right encoder. The firmware therefore configures the OTG FS peripheral without external VBUS sensing instead of consuming PA9 as USB VBUS.

## USB VID/PID

`src/usb_serial.c` currently uses:

```text
VID 0x1209
PID 0xF401
```

Treat these as development identifiers only. If the device will be distributed as a product, obtain/use identifiers appropriate for your product rather than relying on the development values in this example.

---

# 9. Project layout

```text
.
├── Makefile
├── README.md
├── stm32f401cc.ld
├── include
│   ├── board.h
│   ├── command.h
│   ├── limits.h
│   ├── manipulator.h
│   ├── platform.h
│   ├── timebase.h
│   └── usb_serial.h
└── src
    ├── command.c
    ├── limits.c
    ├── main.c
    ├── manipulator.c
    ├── platform.c
    ├── timebase.c
    └── usb_serial.c
```

The first file to edit for hardware changes is:

```text
include/board.h
```

---

# 10. Troubleshooting

## `arm-none-eabi-gcc: command not found`

Install/reinstall:

```bash
sudo pacman -S arm-none-eabi-gcc arm-none-eabi-newlib
```

## `cannot find -lopencm3_stm32f4`

Install:

```bash
sudo pacman -S libopencm3
```

Then verify:

```bash
ls /usr/arm-none-eabi/lib/libopencm3_stm32f4.a
```

## `st-info --probe` sees no target

Check target power, GND, SWDIO, SWCLK, and optionally connect NRST. Try unplugging/replugging the ST-Link after installing the `stlink` package.

## USB does not appear as `/dev/ttyACM*`

Check:

- a USB **data** cable is being used
- PA11/PA12 are connected to the USB connector correctly
- the board has the expected 25 MHz HSE crystal
- `dmesg --follow` while plugging the board in
- no other circuit is loading PA11/PA12

## Manipulator reports movement but physically slips

Reduce speed and acceleration first:

```text
set_manipulator_speed 30
set_manipulator_acceleration 5
```

Also verify DRV8825 current limit, motor supply, coil pairing, and mechanical load. The firmware is open-loop and cannot detect missed motor steps.

## Limit switch shows pressed unexpectedly

Because the switches are NC and fail-safe, a broken/disconnected limit wire is intentionally interpreted as `1` / pressed.

---

# 11. Optional ST-Link debugging with GDB

Start the ST-Link GDB server in one terminal:

```bash
st-util
```

Then in another terminal:

```bash
arm-none-eabi-gdb build/robot_firmware.elf
```

Inside GDB:

```gdb
target extended-remote :4242
monitor reset halt
load
break main
continue
```

This lets you inspect registers, set breakpoints, single-step C code, and debug faults without Arduino tooling.
