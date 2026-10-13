import React from "react"

export type MotorEncoderPortProps = {
  name: string
  x: number
  y: number
  motorA: string
  motorB: string
  encA: string
  encB: string
}

/** Combined 2-wire motor + 4-wire encoder connector. */
export const MotorEncoderPort = ({ name, x, y, motorA, motorB, encA, encB }: MotorEncoderPortProps) => (
  <pinheader
    name={name}
    pinCount={6}
    pitch="2.54mm"
    footprint="pinrow6_p2.54mm"
    pcbX={x}
    pcbY={y}
    pcbRotation={90}
    pinLabels={{ pin1: "M1", pin2: "M2", pin3: "ENC_3V3", pin4: "ENC_GND", pin5: "ENC_A", pin6: "ENC_B" }}
    connections={{
      M1: `net.${motorA}`,
      M2: `net.${motorB}`,
      ENC_3V3: "net.V3V3",
      ENC_GND: "net.GND",
      ENC_A: `net.${encA}`,
      ENC_B: `net.${encB}`,
    }}
  />
)
