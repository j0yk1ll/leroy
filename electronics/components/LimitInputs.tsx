import React from "react"

export const LimitInputs = ({ x = 30, y = -18 }: { x?: number; y?: number }) => (
  <group name="LIMIT_INPUTS_GROUP">
    <pinheader name="J_LIMIT" pinCount={4} pitch="2.54mm" footprint="pinrow4_p2.54mm" pcbX={x} pcbY={y}
      pinLabels={{ pin1: "BOTTOM", pin2: "TOP", pin3: "3V3", pin4: "GND" }}
      connections={{ BOTTOM: "net.LIMIT_BOTTOM", TOP: "net.LIMIT_TOP", "3V3": "net.V3V3", GND: "net.GND" }} />
    <resistor name="R_LIMIT_BOTTOM" resistance="10k" footprint="0402" pcbX={x - 2} pcbY={y + 3}
      connections={{ pin1: "net.LIMIT_BOTTOM", pin2: "net.V3V3" }} />
    <resistor name="R_LIMIT_TOP" resistance="10k" footprint="0402" pcbX={x + 2} pcbY={y + 3}
      connections={{ pin1: "net.LIMIT_TOP", pin2: "net.V3V3" }} />
    <capacitor name="C_LIMIT_BOTTOM" capacitance="10nF" footprint="0402" pcbX={x - 2} pcbY={y + 4.5}
      connections={{ pin1: "net.LIMIT_BOTTOM", pin2: "net.GND" }} />
    <capacitor name="C_LIMIT_TOP" capacitance="10nF" footprint="0402" pcbX={x + 2} pcbY={y + 4.5}
      connections={{ pin1: "net.LIMIT_TOP", pin2: "net.GND" }} />
  </group>
)
