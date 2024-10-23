#ifndef ZENITH_BTN_REMAP_H
#define ZENITH_BTN_REMAP_H

// TODO: maybe convert this to a union so it is a little more ergonomic
// to set from the userspace library configuration
typedef struct {
    uint8_t p[32];
} btn_remap_profile_t;

void btn_remap_task(btn_data_t *in, btn_data_t *out);

#endif // ZENITH_BTN_REMAP_H