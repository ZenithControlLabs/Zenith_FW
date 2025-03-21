#ifndef JOYBUS_CONS_H
#define JOYBUS_CONS_H

#include <hardware/pio.h>
#include <pico/types.h>
#include <stdint.h>

#include "zenith/comms/n64.h"
#include "zenith/settings.h"

// Time to wait for a response, in microseconds. This value seems to work for most controllers.
// This prevents things from freezing up if a controller is not connected or fails to respond once.
#define JOYBUS_TIMEOUT 250

uint joybus_cons_port_init(joybus_port_t *port, uint pin, PIO pio, int sm,
                           int offset);
uint joybus_cons_port_change_pin(joybus_port_t *port, uint pin);

void init_joybus();
void change_joybus_index(int index);

uint32_t read_joybus_ctlr();

#endif // JOYBUS_CONS_H