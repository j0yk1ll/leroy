import React from "react"
import {
  DRV8876MotorDriver,
  GYICM20948V2,
  STM32F411CEU6,
  MotorEncoderPort,
  PowerInput,
  MotorCurrentReference,
  UsbFsPort,
  SwdHeader,
  ManipulatorIO,
  LimitInputs,
} from "./components"

export default () => (
  <board width="86mm" height="68mm" layers={4} autorouter="auto-local" minTraceWidth="0.2mm">
    <hole name="H1" diameter="3.2mm" pcbX={-39} pcbY={30} />
    <hole name="H2" diameter="3.2mm" pcbX={39} pcbY={30} />
    <hole name="H3" diameter="3.2mm" pcbX={-39} pcbY={-30} />
    <hole name="H4" diameter="3.2mm" pcbX={39} pcbY={-30} />

    <PowerInput />
    <MotorCurrentReference />
    <STM32F411CEU6 name="U1" x={0} y={3} />

    <DRV8876MotorDriver name="U2" x={-31} y={18} in1Net="FR_RPWM" in2Net="FR_LPWM" sleepNet="FR_EN" out1Net="FR_MA" out2Net="FR_MB" ipropiNet="FR_IPROPI" />
    <DRV8876MotorDriver name="U3" x={31} y={18} in1Net="RR_RPWM" in2Net="RR_LPWM" sleepNet="RR_EN" out1Net="RR_MA" out2Net="RR_MB" ipropiNet="RR_IPROPI" />
    <DRV8876MotorDriver name="U4" x={31} y={-10} in1Net="RL_RPWM" in2Net="RL_LPWM" sleepNet="RL_EN" out1Net="RL_MA" out2Net="RL_MB" ipropiNet="RL_IPROPI" />
    <DRV8876MotorDriver name="U5" x={-31} y={-10} in1Net="FL_RPWM" in2Net="FL_LPWM" sleepNet="FL_EN" out1Net="FL_MA" out2Net="FL_MB" ipropiNet="FL_IPROPI" />

    <MotorEncoderPort name="J_FR" x={-40} y={18} motorA="FR_MA" motorB="FR_MB" encA="ENC1_C1" encB="ENC1_C2" />
    <MotorEncoderPort name="J_RR" x={40} y={18} motorA="RR_MA" motorB="RR_MB" encA="ENC2_C1" encB="ENC2_C2" />
    <MotorEncoderPort name="J_RL" x={40} y={-10} motorA="RL_MA" motorB="RL_MB" encA="ENC3_C1" encB="ENC3_C2" />
    <MotorEncoderPort name="J_FL" x={-40} y={-10} motorA="FL_MA" motorB="FL_MB" encA="ENC4_C1" encB="ENC4_C2" />

    <GYICM20948V2 name="J_IMU" x={0} y={20} sclNet="IMU_SCL" sdaNet="IMU_SDA" />
    <UsbFsPort />
    <SwdHeader />
    <ManipulatorIO />
    <LimitInputs />

    {/* Existing board.h pin map */}
    <trace from="U1.PA0" to="net.FR_RPWM" />
    <trace from="U1.PA1" to="net.FR_LPWM" />
    <trace from="U1.PA2" to="net.RR_RPWM" />
    <trace from="U1.PA3" to="net.RR_LPWM" />
    <trace from="U1.PA4" to="net.RL_RPWM" />
    <trace from="U1.PA5" to="net.RL_LPWM" />
    <trace from="U1.PA6" to="net.FL_RPWM" />
    <trace from="U1.PA7" to="net.FL_LPWM" />

    <trace from="U1.PB0" to="net.FR_EN" />
    <trace from="U1.PB1" to="net.RR_EN" />
    <trace from="U1.PB10" to="net.RL_EN" />
    <trace from="U1.PB13" to="net.FL_EN" />

    <trace from="U1.PA8" to="net.ENC1_C1" />
    <trace from="U1.PA9" to="net.ENC1_C2" />
    <trace from="U1.PA15" to="net.ENC2_C1" />
    <trace from="U1.PB3" to="net.ENC2_C2" />
    <trace from="U1.PB4" to="net.ENC3_C1" />
    <trace from="U1.PB5" to="net.ENC3_C2" />
    <trace from="U1.PB6" to="net.ENC4_C1" />
    <trace from="U1.PB7" to="net.ENC4_C2" />

    <trace from="U1.PB8" to="net.IMU_SCL" />
    <trace from="U1.PB9" to="net.IMU_SDA" />

    <trace from="U1.PA10" to="net.MANIP_DIR" />
    <trace from="U1.PB14" to="net.MANIP_STEP" />
    <trace from="U1.PC14" to="net.LIMIT_BOTTOM" />
    <trace from="U1.PC15" to="net.LIMIT_TOP" />

    <trace from="U1.PA11" to="net.USB_DM" />
    <trace from="U1.PA12" to="net.USB_DP" />
    <trace from="U1.PA13" to="net.SWDIO" />
    <trace from="U1.PA14" to="net.SWCLK" />
    <trace from="U1.NRST" to="net.NRST" />

    {/* PB12 is free in the supplied map and is used for shared active-low driver fault. */}
    <trace from="U1.PB12" to="net.MOTOR_NFAULT" />
    <resistor name="R_MOTOR_FAULT" resistance="10k" footprint="0402" pcbX={8} pcbY={5}
      connections={{ pin1: "net.MOTOR_NFAULT", pin2: "net.V3V3" }} />
  </board>
)
