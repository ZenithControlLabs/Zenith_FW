/*
Public-facing API for Zenith.
*/

#ifndef ZENITH_H
#define ZENITH_H

#include "zenith/types.h"

// hardware callbacks
void cb_zenith_init_hardware(void);
int cb_zenith_read_controller_switch(void);
void cb_zenith_switch_input(int index);
void cb_zenith_read_buttons(btn_data_t *buttons);
void cb_zenith_read_analog(analog_data_t *analog_data);
// if ZTH_SEPARATE_CAL_READ, calibration will use this instead to read analog
void cb_zenith_read_analog_cal(analog_data_t *analog_data);

// app callbacks
void cb_zenith_core1_init(void);
void cb_zenith_core0_inject(void);
void cb_zenith_core1_inject(void);
bool cb_zenith_user_webusb_cmd(uint8_t *in, uint8_t *out);
void cb_zenith_user_settings_reset(uint8_t *data);

uint8_t *zenith_get_user_settings_ptr(void);
void zenith_start(void);

#endif // ZENITH_H