import React from "react"
import { DRV8876 } from "@tsci/tscircuit.ti"

export type DRV8876MotorDriverProps = {
  name: string
  x: number
  y: number
  in1Net: string
  in2Net: string
  sleepNet: string
  out1Net: string
  out2Net: string
  ipropiNet: string
  faultNet?: string
  vmNet?: string
  gndNet?: string
  vrefNet?: string
  pwmMode?: boolean
}

/**
 * One complete DRV8876 brushed-DC channel including the local parts that should
 * live physically next to the IC.
 *
 * Default configuration preserves the existing BTS7960-style two-input firmware:
 *   EN/IN1 <- RPWM
 *   PH/IN2 <- LPWM
 *   nSLEEP <- per-motor enable
 *   PMODE  <- 3V3 (PWM two-input mode)
 *
 * For a future firmware revision, set pwmMode={false} and use IN1 as EN/PWM and
 * IN2 as PH/direction.
 */
export const DRV8876MotorDriver = ({
  name, x, y, in1Net, in2Net, sleepNet, out1Net, out2Net, ipropiNet,
  faultNet = "MOTOR_NFAULT", vmNet = "VMOTOR", gndNet = "GND",
  vrefNet = "MOTOR_VREF", pwmMode = true,
}: DRV8876MotorDriverProps) => (
  <group name={`${name}_GROUP`}>
    <DRV8876
      name={name}
      pcbX={x}
      pcbY={y}
      connections={{
        VM: `net.${vmNet}`,
        GND: `net.${gndNet}`,
        PGND: `net.${gndNet}`,
        "EN/IN1": `net.${in1Net}`,
        "PH/IN2": `net.${in2Net}`,
        nSLEEP: `net.${sleepNet}`,
        PMODE: pwmMode ? "net.V3V3" : `net.${gndNet}`,
        IMODE: `net.${gndNet}`,
        VREF: `net.${vrefNet}`,
        IPROPI: `net.${ipropiNet}`,
        nFAULT: `net.${faultNet}`,
        OUT1: `net.${out1Net}`,
        OUT2: `net.${out2Net}`,
      }}
    />

    {/* Place these directly at the VM/GND pins in the final layout. */}
    <capacitor name={`${name}_CVM_HF`} capacitance="100nF" footprint="0603" pcbX={x + 3.0} pcbY={y + 2.0}
      connections={{ pin1: `net.${vmNet}`, pin2: `net.${gndNet}` }} />
    <capacitor name={`${name}_CVM_BULK`} capacitance="47uF" footprint="1210" pcbX={x + 3.0} pcbY={y}
      connections={{ pin1: `net.${vmNet}`, pin2: `net.${gndNet}` }} />

    {/* DRV8876 charge-pump capacitors. */}
    <capacitor name={`${name}_CCP`} capacitance="22nF" footprint="0402" pcbX={x - 3.0} pcbY={y + 1.5}
      connections={{ pin1: `${name}.CPH`, pin2: `${name}.CPL` }} />
    <capacitor name={`${name}_CVCP`} capacitance="100nF" footprint="0402" pcbX={x - 3.0} pcbY={y - 1.5}
      connections={{ pin1: `${name}.VCP`, pin2: `net.${vmNet}` }} />

    {/* 1 kOhm IPROPI sense resistor; combine with shared VREF for ~2.5 A limit target. */}
    <resistor name={`${name}_RIPROPI`} resistance="1k" footprint="0603" pcbX={x + 2.5} pcbY={y - 2.5}
      connections={{ pin1: `net.${ipropiNet}`, pin2: `net.${gndNet}` }} />
  </group>
)
