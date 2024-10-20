#ifndef PHOBRI_PROTO_H
#define PHOBRI_PROTO_H

#include "zenith/types.h"

#define JOYBUS_PIN 11
#define BTN_A_PIN 20
#define BTN_B_PIN 21
#define BTN_START_PIN 8
#define BTN_ZR_PIN 18
#define BTN_ZL_PIN 10
#define BTN_R_PIN 19
#define BTN_L_PIN 9

#define BTN_CU_PIN 15
#define BTN_CD_PIN 16
#define BTN_CL_PIN 17
#define BTN_CR_PIN 14

#define BTN_DU_PIN 25
#define BTN_DD_PIN 22
#define BTN_DL_PIN 24
#define BTN_DR_PIN 23

#define STICK_SPI_CLK 26
#define STICK_SPI_TX 27
#define STICK_SPI_RX 28
#define STICK_SPI_CS 29
#define STICK_SPI_INTF spi1
#define STICK_FLIP_ADC_CHANNELS true

#define DEBUG_TX_PIN 12
#define DEBUG_UART uart0

void phobri_proto_core1_init(void);

void phobri_proto_read_analog(analog_data_t *analog);

void phobri_proto_core1_inject();

#endif /* PHOBRI_PROTO_H */