#pragma once

#include <libopencm3/stm32/gpio.h>

/*
 * STM32F401 Black Pill robot wiring.
 *
 * Wheel controller mapping is preserved from the latest integrated Arduino
 * firmware. The manipulator pins reflect the wiring verified during testing.
 */

/* BTS7960 PWM inputs: all eight signals are on GPIOA PA0..PA7. */
#define FR_RPWM_PORT GPIOA
#define FR_RPWM_PIN  GPIO0
#define FR_LPWM_PORT GPIOA
#define FR_LPWM_PIN  GPIO1

#define RR_RPWM_PORT GPIOA
#define RR_RPWM_PIN  GPIO2
#define RR_LPWM_PORT GPIOA
#define RR_LPWM_PIN  GPIO3

#define RL_RPWM_PORT GPIOA
#define RL_RPWM_PIN  GPIO4
#define RL_LPWM_PORT GPIOA
#define RL_LPWM_PIN  GPIO5

#define FL_RPWM_PORT GPIOA
#define FL_RPWM_PIN  GPIO6
#define FL_LPWM_PORT GPIOA
#define FL_LPWM_PIN  GPIO7

/* BTS7960 enable inputs. */
#define FR_EN_PORT GPIOB
#define FR_EN_PIN  GPIO0
#define RR_EN_PORT GPIOB
#define RR_EN_PIN  GPIO1
#define RL_EN_PORT GPIOB
#define RL_EN_PIN  GPIO10
#define FL_EN_PORT GPIOB
#define FL_EN_PIN  GPIO13

/* Wheel quadrature encoders. */
#define ENC1_C1_PORT GPIOA
#define ENC1_C1_PIN  GPIO8
#define ENC1_C2_PORT GPIOA
#define ENC1_C2_PIN  GPIO9
#define ENC2_C1_PORT GPIOA
#define ENC2_C1_PIN  GPIO15
#define ENC2_C2_PORT GPIOB
#define ENC2_C2_PIN  GPIO3
#define ENC3_C1_PORT GPIOB
#define ENC3_C1_PIN  GPIO4
#define ENC3_C2_PORT GPIOB
#define ENC3_C2_PIN  GPIO5
#define ENC4_C1_PORT GPIOB
#define ENC4_C1_PIN  GPIO6
#define ENC4_C2_PORT GPIOB
#define ENC4_C2_PIN  GPIO7

/* GY-ICM20948V2 / ICM-20948 on I2C1, AF4. */
#define IMU_SCL_PORT GPIOB
#define IMU_SCL_PIN  GPIO8
#define IMU_SDA_PORT GPIOB
#define IMU_SDA_PIN  GPIO9

/* DRV8825 manipulator driver -- verified wiring. */
#define MANIP_DIR_PORT  GPIOA
#define MANIP_DIR_PIN   GPIO10
#define MANIP_STEP_PORT GPIOB
#define MANIP_STEP_PIN  GPIO14

/* NC limit switches wired to GND with internal pull-ups. */
#define LIMIT_BOTTOM_PORT GPIOC
#define LIMIT_BOTTOM_PIN  GPIO14
#define LIMIT_TOP_PORT    GPIOC
#define LIMIT_TOP_PIN     GPIO15

/* Native USB FS used for the remote-control CDC ACM serial interface. */
#define USB_DM_PORT GPIOA
#define USB_DM_PIN  GPIO11
#define USB_DP_PORT GPIOA
#define USB_DP_PIN  GPIO12

/* SWD: PA13 SWDIO, PA14 SWCLK. Do not repurpose these pins. */
