#ifndef ZENITH_N64_H
#define ZENITH_N64_H

#include "zenith/types.h"
#include "zenith/utilities/interval.h"
#include <stdint.h>

typedef struct {
    union {
        struct {
            uint8_t dpad_right : 1;
            uint8_t dpad_left : 1;
            uint8_t dpad_down : 1;
            uint8_t dpad_up : 1;
            uint8_t button_start : 1;
            uint8_t button_z : 1;
            uint8_t button_b : 1;
            uint8_t button_a : 1;
        };
        uint8_t buttons_1;
    };

    union {
        struct {
            uint8_t cpad_right : 1;
            uint8_t cpad_left : 1;
            uint8_t cpad_down : 1;
            uint8_t cpad_up : 1;
            uint8_t button_r : 1;
            uint8_t button_l : 1;
            uint8_t reserved : 1;
            uint8_t reset : 1;
        };
        uint8_t buttons_2;
    };

    int8_t stick_x;
    int8_t stick_y;
} __attribute__((packed)) n64_input_t;

enum N64Command {
    PROBE = 0x00,
    RESET = 0xFF,
    POLL = 0x01,
    READ_EXPANSION_BUS = 0x02,
    WRITE_EXPANSION_BUS = 0x03,
};

typedef struct __attribute__((packed)) {
    uint16_t device;
    uint8_t status;
} n64_status_t;

static const n64_status_t default_n64_status = {
    .device = 0x0005,
    .status = 0x02,
};

static const n64_status_t other_status = {
    .device = 0xFFFF,
    .status = 0xFF,
};

/**
 * @brief A structure representing a Joybus instance on a given GPIO pin
 */
typedef struct {
    uint pin;
    PIO pio;
    uint sm;
    uint offset;
    pio_sm_config config;
} joybus_port_t;

void n64_init();

void n64_comms_task(uint32_t timestamp, btn_data_t *buttons,
                    analog_data_t *analog);

#endif // ZENITH_N64_H