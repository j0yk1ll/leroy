import React from "react"

export const ManipulatorIO = ({ x = 30, y = -26 }: { x?: number; y?: number }) => (
  <pinheader name="J_MANIP" pinCount={4} pitch="2.54mm" footprint="pinrow4_p2.54mm" pcbX={x} pcbY={y}
    pinLabels={{ pin1: "DIR", pin2: "STEP", pin3: "3V3", pin4: "GND" }}
    connections={{ DIR: "net.MANIP_DIR", STEP: "net.MANIP_STEP", "3V3": "net.V3V3", GND: "net.GND" }} />
)
