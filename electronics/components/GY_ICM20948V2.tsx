import React from "react"

export type GYICM20948V2Props = {
  name?: string
  x: number
  y: number
  sclNet?: string
  sdaNet?: string
  intNet?: string
  addressHigh?: boolean
}

/**
 * Socket/header representation of the GY-ICM20948V2 module, not the bare
 * 3 x 3 mm ICM-20948 IC.
 *
 * Common module header labels:
 * VCC, GND, SCL/SCLK, SDA/SDI, EDA, ECL, ADO/SDO, INT, NCS, FSYNC.
 *
 * This project uses I2C. NCS is held high, AD0 is selectable, and the auxiliary
 * I2C/FSYNC pins are brought to labeled nets so they are not silently lost.
 * Verify the physical left-to-right order against the exact module you own
 * before fabrication; clone modules occasionally rotate/reorder headers.
 */
export const GYICM20948V2 = ({
  name = "J_IMU",
  x,
  y,
  sclNet = "IMU_SCL",
  sdaNet = "IMU_SDA",
  intNet = "IMU_INT",
  addressHigh = false,
}: GYICM20948V2Props) => (
  <group name={`${name}_GROUP`}>
    <pinheader
      name={name}
      pinCount={10}
      pitch="2.54mm"
      footprint="pinrow10_p2.54mm"
      pcbX={x}
      pcbY={y}
      pinLabels={{
        pin1: "VCC",
        pin2: "GND",
        pin3: "SCL_SCLK",
        pin4: "SDA_SDI",
        pin5: "EDA",
        pin6: "ECL",
        pin7: "AD0_SDO",
        pin8: "INT",
        pin9: "NCS",
        pin10: "FSYNC",
      }}
      connections={{
        VCC: "net.V3V3",
        GND: "net.GND",
        SCL_SCLK: `net.${sclNet}`,
        SDA_SDI: `net.${sdaNet}`,
        EDA: "net.IMU_AUX_SDA",
        ECL: "net.IMU_AUX_SCL",
        AD0_SDO: addressHigh ? "net.V3V3" : "net.GND",
        INT: `net.${intNet}`,
        NCS: "net.V3V3",
        FSYNC: "net.IMU_FSYNC",
      }}
    />

    {/* External pull-ups make the motherboard independent of module clone details. */}
    <resistor name={`${name}_R_SCL`} resistance="4.7k" footprint="0402" pcbX={x - 2} pcbY={y + 3}
      connections={{ pin1: `net.${sclNet}`, pin2: "net.V3V3" }} />
    <resistor name={`${name}_R_SDA`} resistance="4.7k" footprint="0402" pcbX={x + 2} pcbY={y + 3}
      connections={{ pin1: `net.${sdaNet}`, pin2: "net.V3V3" }} />
    <capacitor name={`${name}_C_LOCAL`} capacitance="100nF" footprint="0402" pcbX={x} pcbY={y + 4.5}
      connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
  </group>
)
