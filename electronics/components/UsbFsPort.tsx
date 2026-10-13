import React from "react"

/**
 * USB FS breakout interface. The exact USB-C connector/ESD part is intentionally
 * left as a small replaceable subcomponent rather than pretending a generic
 * footprint is fabrication-ready.
 */
export const UsbFsPort = ({ x = 0, y = 28 }: { x?: number; y?: number }) => (
  <pinheader name="J_USB_FS" pinCount={4} pitch="2.54mm" footprint="pinrow4_p2.54mm" pcbX={x} pcbY={y}
    pinLabels={{ pin1: "VBUS", pin2: "DM", pin3: "DP", pin4: "GND" }}
    connections={{ VBUS: "net.USB_VBUS", DM: "net.USB_DM", DP: "net.USB_DP", GND: "net.GND" }} />
)
