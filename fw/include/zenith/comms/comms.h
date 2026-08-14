#ifndef ZENITH_COMMS_H
#define ZENITH_COMMS_H

#define GAMEPAD_PIO pio1
#define GAMEPAD_SM 0

#include "zenith/types.h"

typedef enum {
    COMMS_MODE_N64 = 0,
    COMMS_MODE_GAMECUBE = 1,
    COMMS_MODE_XINPUT = 2,
    COMMS_MODE_COUNT
} comms_mode_t;

/* Button-map targets that do not necessarily select a Joybus transport. */
typedef enum {
    REMAP_MODE_N64 = COMMS_MODE_N64,
    REMAP_MODE_GAMECUBE = COMMS_MODE_GAMECUBE,
    REMAP_MODE_XINPUT = COMMS_MODE_XINPUT,
    REMAP_MODE_SWITCH = 3,
    REMAP_MODE_COUNT
} remap_mode_t;

void comms_init();

void comms_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog);

#endif // ZENITH_COMMS_H
