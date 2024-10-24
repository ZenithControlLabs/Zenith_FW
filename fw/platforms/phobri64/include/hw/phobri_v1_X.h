#ifndef PHOBRI_V1_X_H
#define PHOBRI_V1_X_H

#include "zenith/types.h"
#include <hardware/i2c.h>

#define JOYBUS_PIN 0
#define BTN_A_PIN 7
#define BTN_B_PIN 18
#define BTN_START_PIN 29
#define BTN_ZR_PIN 5
#define BTN_ZL_PIN 28
#define BTN_R_PIN 4
#define BTN_L_PIN 23

#define BTN_CU_PIN 2
#define BTN_CD_PIN 6
#define BTN_CL_PIN 1
#define BTN_CR_PIN 3

#define BTN_DU_PIN 20
#define BTN_DD_PIN 21
#define BTN_DL_PIN 22
#define BTN_DR_PIN 19

// TODO: change
#define DEBUG_TX_PIN 24
#define DEBUG_UART uart1

// TODO: change
#define STICK_I2C_INTF i2c0
#define STICK_I2C_SCL 17
#define STICK_I2C_SDA 16

#define STICK_ADC_DRDY_N 11

// v1.0
#define STICK_HX_DRDY 8
#define STICK_HY_DRDY 9
#define I2C_HX_ADDR 0x1E
#define I2C_HY_ADDR 0x1C

// v1.1
#define STICK_HALL_DRDY 8
#define I2C_HALL_ADDR 0x1E

#define I2C_ADC_ADDR 0x18

#endif /* PHOBRI_V1_X_H */
