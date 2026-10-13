import React from "react"

export const SwdHeader = ({ x = 16, y = 28 }: { x?: number; y?: number }) => (
  <pinheader name="J_SWD" pinCount={5} pitch="2.54mm" footprint="pinrow5_p2.54mm" pcbX={x} pcbY={y}
    pinLabels={{ pin1: "3V3", pin2: "SWDIO", pin3: "SWCLK", pin4: "NRST", pin5: "GND" }}
    connections={{ "3V3": "net.V3V3", SWDIO: "net.SWDIO", SWCLK: "net.SWCLK", NRST: "net.NRST", GND: "net.GND" }} />
)
