#ifndef ZENITH_COMMS_H
#define ZENITH_COMMS_H

#define GAMEPAD_PIO pio1
#define GAMEPAD_SM 0

#include "zenith/types.h"

typedef enum { COMMS_MODE_N64, COMMS_MODE_GAMECUBE } comms_mode_t;

void comms_init();

void comms_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog);

#endif // ZENITH_COMMS_H