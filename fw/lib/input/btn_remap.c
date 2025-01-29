#include "zenith/includes.h"

void apply_btn_remap(btn_remap_profile_t *remap_profile, btn_data_t *in,
                     btn_data_t *out) {
    uint32_t r = 0;
    for (int i = 0; i < 32; i++) {
        uint8_t remap_code = remap_profile->p[i];

        // 0xFF => disable this button.
        if (remap_code == 0xFF)
            continue;
        // TODO: assumption that compiler will store bitfield as b1 in LSB, for
        // performance
        uint8_t btn = (in->r >> i) & 0x01;
        // remap_code == 0 means don't change, so just put it back
        r |= btn << (remap_code ? (remap_code - 1) : i);
        // debug_print("Source button: %d=%d; remap code: %d\n", i, btn,
        //             remap_code);
    }
    out->r = r;
}

void btn_remap_task(btn_data_t *in, btn_data_t *out) {
    switch (_settings[_profile].comms_mode) {
    case COMMS_MODE_N64: {
        apply_btn_remap(&_settings[_profile].btn_remap_profile_n64, in, out);
        break;
    }
    case COMMS_MODE_GAMECUBE: {
        apply_btn_remap(&_settings[_profile].btn_remap_profile_gamecube, in, out);
        break;
    }
    }
}