import React from "react"

export type PowerInputProps = { x?: number; y?: number }

/** 12 V motor-rail entry, protection, and bulk storage. */
export const PowerInput = ({ x = 0, y = -30 }: PowerInputProps) => (
  <group name="POWER_INPUT_GROUP">
    <pinheader name="J_PWR" pinCount={2} pitch="5.08mm" footprint="pinrow2_p5.08mm" pcbX={x} pcbY={y}
      pinLabels={{ pin1: "12V", pin2: "GND" }} connections={{ "12V": "net.VIN12", GND: "net.GND" }} />
    <fuse name="F1" currentRating="10A" voltageRating="32V" footprint="1206" pcbX={x} pcbY={y + 4} />
    <trace from="J_PWR.12V" to="F1.pin1" thickness="2mm" />
    <trace from="F1.pin2" to="net.VMOTOR" thickness="2mm" />

    <diode name="D_TVS" variant="tvs" footprint="smb" pcbX={x + 7} pcbY={y + 4}
      connections={{ anode: "net.GND", cathode: "net.VMOTOR" }} />
    <capacitor name="C_BULK" capacitance="1000uF" footprint="radial" pcbX={x - 8} pcbY={y + 4}
      connections={{ pin1: "net.VMOTOR", pin2: "net.GND" }} />
    <capacitor name="C_IN_MID" capacitance="10uF" footprint="1210" pcbX={x - 4} pcbY={y + 7}
      connections={{ pin1: "net.VMOTOR", pin2: "net.GND" }} />
    <capacitor name="C_IN_HF" capacitance="100nF" footprint="0603" pcbX={x - 1} pcbY={y + 7}
      connections={{ pin1: "net.VMOTOR", pin2: "net.GND" }} />

    {/* Deliberate regulator interface until an exact buck + reference layout is chosen. */}
    <pinheader name="J_REG" pinCount={3} pitch="2.54mm" pcbX={x + 16} pcbY={y + 4}
      pinLabels={{ pin1: "VIN12", pin2: "GND", pin3: "V3V3" }}
      connections={{ VIN12: "net.VMOTOR", GND: "net.GND", V3V3: "net.V3V3" }} />
  </group>
)

export const MotorCurrentReference = ({ x = -11, y = -20 }: { x?: number; y?: number }) => (
  <group name="MOTOR_CURRENT_REFERENCE">
    <resistor name="R_VREF_TOP" resistance="3.3k" footprint="0603" pcbX={x - 2} pcbY={y}
      connections={{ pin1: "net.V3V3", pin2: "net.MOTOR_VREF" }} />
    <resistor name="R_VREF_BOT" resistance="10k" footprint="0603" pcbX={x + 2} pcbY={y}
      connections={{ pin1: "net.MOTOR_VREF", pin2: "net.GND" }} />
    <capacitor name="C_VREF" capacitance="100nF" footprint="0402" pcbX={x + 5} pcbY={y}
      connections={{ pin1: "net.MOTOR_VREF", pin2: "net.GND" }} />
  </group>
)
