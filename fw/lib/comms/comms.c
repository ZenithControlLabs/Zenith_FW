#include "zenith/includes.h"

void comms_init() {
    switch (_settings[_profile].comms_mode) {
    case COMMS_MODE_N64: {
        n64_init();
    } break;
    case COMMS_MODE_GAMECUBE: {
        // gamecube_init();
    } break;
    }
}

void comms_task(uint32_t timestamp, btn_data_t *buttons,
                analog_data_t *analog) {
    switch (_settings[_profile].comms_mode) {
    case COMMS_MODE_N64: {
        n64_comms_task(timestamp, buttons, analog);
    } break;
    case COMMS_MODE_GAMECUBE: {
        // gamecube_comms_task(timestamp, buttons, analog);
    } break;
    }
}