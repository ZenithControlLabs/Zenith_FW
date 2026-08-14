#ifndef ZENITH_XINPUT_H
#define ZENITH_XINPUT_H

#include <stdbool.h>
#include <stdint.h>

bool xinput_ready(void);
bool xinput_send(btn_data_t *buttons, analog_data_t *analog);

#endif
