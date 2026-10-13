import React from "react"

export const stm32f411ceu6Pins = {
  pin1: "VBAT", pin2: "PC13", pin3: "PC14", pin4: "PC15",
  pin5: "PH0", pin6: "PH1", pin7: "NRST", pin8: "VSSA",
  pin9: "VDDA", pin10: "PA0", pin11: "PA1", pin12: "PA2",
  pin13: "PA3", pin14: "PA4", pin15: "PA5", pin16: "PA6",
  pin17: "PA7", pin18: "PB0", pin19: "PB1", pin20: "PB2",
  pin21: "PB10", pin22: "VCAP1", pin23: "VSS1", pin24: "VDD1",
  pin25: "PB12", pin26: "PB13", pin27: "PB14", pin28: "PB15",
  pin29: "PA8", pin30: "PA9", pin31: "PA10", pin32: "PA11",
  pin33: "PA12", pin34: "PA13", pin35: "VSS2", pin36: "VDD2",
  pin37: "PA14", pin38: "PA15", pin39: "PB3", pin40: "PB4",
  pin41: "PB5", pin42: "PB6", pin43: "PB7", pin44: "BOOT0",
  pin45: "PB8", pin46: "PB9", pin47: "VSS3", pin48: "VDD3",
}

export type STM32F411CEU6Props = { name?: string; x?: number; y?: number }

/** Bare STM32F411CEU6 support circuit approximating the WeAct Blackpill clocking. */
export const STM32F411CEU6 = ({ name = "U1", x = 0, y = 0 }: STM32F411CEU6Props) => (
  <group name={`${name}_GROUP`}>
    <chip
      name={name}
      manufacturerPartNumber="STM32F411CEU6"
      footprint="qfn48_w7mm_h7mm_p0.5mm"
      pinLabels={stm32f411ceu6Pins}
      pcbX={x}
      pcbY={y}
    />

    <trace from={`${name}.VDD1`} to="net.V3V3" />
    <trace from={`${name}.VDD2`} to="net.V3V3" />
    <trace from={`${name}.VDD3`} to="net.V3V3" />
    <trace from={`${name}.VDDA`} to="net.V3V3_A" />
    <trace from={`${name}.VBAT`} to="net.V3V3" />
    <trace from={`${name}.VSS1`} to="net.GND" />
    <trace from={`${name}.VSS2`} to="net.GND" />
    <trace from={`${name}.VSS3`} to="net.GND" />
    <trace from={`${name}.VSSA`} to="net.GND" />

    <capacitor name={`${name}_C_VDD1`} capacitance="100nF" footprint="0402" pcbX={x - 5} pcbY={y + 7}
      connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <capacitor name={`${name}_C_VDD2`} capacitance="100nF" footprint="0402" pcbX={x - 3.5} pcbY={y + 7}
      connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <capacitor name={`${name}_C_VDD3`} capacitance="100nF" footprint="0402" pcbX={x - 2} pcbY={y + 7}
      connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />
    <capacitor name={`${name}_C_BULK`} capacitance="4.7uF" footprint="0603" pcbX={x} pcbY={y + 7}
      connections={{ pin1: "net.V3V3", pin2: "net.GND" }} />

    {/* Internal regulator stabilization: mandatory VCAP capacitor. */}
    <capacitor name={`${name}_C_VCAP`} capacitance="4.7uF" footprint="0603" pcbX={x + 4} pcbY={y + 5}
      connections={{ pin1: `${name}.VCAP1`, pin2: "net.GND" }} />

    {/* VDDA filtering. */}
    <resistor name={`${name}_R_AVDD`} resistance="10" footprint="0603" pcbX={x + 3} pcbY={y + 7}
      connections={{ pin1: "net.V3V3", pin2: "net.V3V3_A" }} />
    <capacitor name={`${name}_C_AVDD1`} capacitance="100nF" footprint="0402" pcbX={x + 5} pcbY={y + 7}
      connections={{ pin1: "net.V3V3_A", pin2: "net.GND" }} />
    <capacitor name={`${name}_C_AVDD2`} capacitance="1uF" footprint="0603" pcbX={x + 6.5} pcbY={y + 7}
      connections={{ pin1: "net.V3V3_A", pin2: "net.GND" }} />

    {/* 25 MHz HSE to stay close to the WeAct Blackpill environment. */}
    <crystal name={`${name}_Y_HSE`} frequency="25MHz" footprint="crystal_3225" pcbX={x - 7} pcbY={y + 1} />
    <trace from={`${name}.PH0`} to={`${name}_Y_HSE.pin1`} />
    <trace from={`${name}.PH1`} to={`${name}_Y_HSE.pin2`} />
    <capacitor name={`${name}_C_HSE1`} capacitance="12pF" footprint="0402" pcbX={x - 8} pcbY={y - 1}
      connections={{ pin1: `${name}_Y_HSE.pin1`, pin2: "net.GND" }} />
    <capacitor name={`${name}_C_HSE2`} capacitance="12pF" footprint="0402" pcbX={x - 6} pcbY={y - 1}
      connections={{ pin1: `${name}_Y_HSE.pin2`, pin2: "net.GND" }} />

    <resistor name={`${name}_R_NRST`} resistance="10k" footprint="0402" pcbX={x + 5} pcbY={y + 3}
      connections={{ pin1: `${name}.NRST`, pin2: "net.V3V3" }} />
    <capacitor name={`${name}_C_NRST`} capacitance="100nF" footprint="0402" pcbX={x + 6.5} pcbY={y + 3}
      connections={{ pin1: `${name}.NRST`, pin2: "net.GND" }} />
    <resistor name={`${name}_R_BOOT0`} resistance="100k" footprint="0402" pcbX={x + 5} pcbY={y + 1}
      connections={{ pin1: `${name}.BOOT0`, pin2: "net.GND" }} />
  </group>
)
