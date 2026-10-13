# Modular tscircuit robot-controller project

This archive splits the design into reusable tscircuit subcomponents instead of keeping the whole board in one `.tsx` file.

## Files

- `components/DRV8876MotorDriver.tsx` — one TI DRV8876 H-bridge plus charge-pump, VM decoupling and IPROPI resistor.
- `components/GY_ICM20948V2.tsx` — 10-pin socket/header representation of the **GY-ICM20948V2 module** in I2C mode, with motherboard pull-ups.
- `components/STM32F411CEU6.tsx` — bare STM32F411CEU6, 25 MHz HSE, VCAP, decoupling, VDDA filter, NRST and BOOT0 support.
- `components/MotorEncoderPort.tsx` — motor + encoder 6-pin connector.
- `components/PowerInput.tsx` — 12 V input, fuse, TVS, bulk capacitors, regulator interface and shared DRV8876 current-reference divider.
- `components/UsbFsPort.tsx` — USB FS breakout/interface placeholder.
- `components/SwdHeader.tsx` — 5-pin SWD header.
- `components/ManipulatorIO.tsx` — DIR/STEP manipulator connector.
- `components/LimitInputs.tsx` — top/bottom limit-switch connector with pull-ups and RC filtering.
- `components/index.ts` — exports all subcomponents.
- `robot_controller.circuit.tsx` — top-level board that instantiates the subcomponents and applies your supplied `board.h` pin map.

## Important implementation notes

### GY-ICM20948V2

The module commonly exposes the header labels:

`VCC, GND, SCL/SCLK, SDA/SDI, EDA, ECL, ADO/SDO, INT, NCS, FSYNC`.

The component models those 10 signals and configures it for I2C (`NCS` high, `AD0` low by default). **Check the exact physical left-to-right order on your module before sending the PCB to fabrication**; clone-board header orientation can differ even when the labels are the same.

### STM32F411CEU6

The UFQFPN48 mapping in `STM32F411CEU6.tsx` follows the STM32F411CE datasheet: PH0/PH1 are HSE, pin 22 is VCAP1, PB12 is pin 25, and PB8/PB9 are pins 45/46.

The board intentionally does not use an LSE crystal because your firmware uses PC14/PC15 for the limit switches.

### Motor-driver interface

The DRV8876 component currently preserves your existing two-input `RPWM/LPWM` naming so the design remains compatible with your current firmware structure. A later cleanup can change the driver to PH/EN mode and remap one hardware PWM + one direction GPIO per motor.

### Not fabrication-ready yet

The archive is structured for iterative tscircuit work, but these still need an exact production choice/reference layout before fabrication:

1. 12 V -> 3.3 V regulator.
2. USB-C receptacle + CC resistors + USB ESD protection.
3. Exact motor/encoder connector family.
4. Verification of the exact tscircuit UFQFPN48 footprint/center-pad implementation.
5. Crystal load capacitors based on the selected 25 MHz crystal.
6. Manual high-current copper pours and thermal-via design around each DRV8876; do not rely on generic autorouting for motor current paths.

## Current-limit intent

For the 12 V motors (~2.3 A specified stall current), the draft targets roughly 2.5 A current limiting rather than sizing around the old 43 A BTS7960 modules. Each DRV8876 uses a 1 kOhm IPROPI resistor and shares `MOTOR_VREF`.
