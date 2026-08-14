#include "zenith/includes.h"

#include <string.h>

#define SW_OUT_SUBCOMMAND 0x01
#define SW_OUT_RUMBLE 0x10
#define SW_OUT_INFO 0x80

static uint8_t _pending[64];
static uint16_t _pending_len;
static bool _pending_valid;
static bool _init_sent;
static uint8_t _timer;

/* Stable local identity. Bluetooth is not implemented, but hosts request it. */
static const uint8_t _mac[6] = {0x98, 0xB6, 0xE9, 0x21, 0x64, 0x00};

static void pack_xy12(uint16_t x, uint16_t y, uint8_t out[3]) {
    x &= 0x0FFF;
    y &= 0x0FFF;
    out[0] = (uint8_t)x;
    out[1] = (uint8_t)((x >> 8) | ((y & 0x0F) << 4));
    out[2] = (uint8_t)(y >> 4);
}

static uint16_t axis_to_switch(ax_t axis) {
    float scaled = (axis + 1.0f) * 2047.5f;
    if (scaled < 0.0f)
        scaled = 0.0f;
    if (scaled > 4095.0f)
        scaled = 4095.0f;
    return (uint16_t)scaled;
}

static void set_input(uint8_t *out, btn_data_t *buttons,
                      analog_data_t *analog) {
    btn_data_t mapped = {0};
    btn_remap_for_mode(REMAP_MODE_SWITCH, buttons, &mapped);

    out[3] = (uint8_t)mapped.r;
    out[4] = (uint8_t)(mapped.r >> 8);
    out[5] = (uint8_t)(mapped.r >> 16);
    pack_xy12(axis_to_switch(analog->ax1), axis_to_switch(analog->ax2),
              &out[6]);
    pack_xy12(2048, 2048, &out[9]);
}

static void set_common(uint8_t *out, btn_data_t *buttons,
                       analog_data_t *analog) {
    out[1] = _timer++;
    out[2] = 0x91; /* full battery, charging, USB powered */
    set_input(out, buttons, analog);
}

static uint8_t spi_byte(uint8_t page, uint8_t address) {
    static uint8_t stick_cal[18];
    static bool stick_cal_ready;
    if (!stick_cal_ready) {
        pack_xy12(0x7FF, 0x7FF, &stick_cal[0]);
        pack_xy12(0x800, 0x800, &stick_cal[3]);
        pack_xy12(0x7FF, 0x7FF, &stick_cal[6]);
        pack_xy12(0x800, 0x800, &stick_cal[9]);
        pack_xy12(0x7FF, 0x7FF, &stick_cal[12]);
        pack_xy12(0x7FF, 0x7FF, &stick_cal[15]);
        stick_cal_ready = true;
    }

    if (page == 0x60) {
        if (address <= 0x0F)
            return 0xFF;
        if (address == 0x12) return 0x03;
        if (address == 0x13) return 0x02;
        if (address == 0x1B) return 0x01;
        if (address >= 0x3D && address <= 0x4E)
            return stick_cal[address - 0x3D];
        if (address == 0x4F) return 0xFF;
        /* Neutral dark-gray body, black buttons and grips. */
        if (address >= 0x50 && address <= 0x52) return 0x32;
        if (address >= 0x53 && address <= 0x5B) return 0x0A;
        if (address == 0x80) return 80;
        if (address == 0x81) return 253;
        if (address == 0x84) return 198;
        if (address == 0x85) return 15;
        static const uint8_t stick_params[18] = {
            15,48,97,174,144,217,212,20,84,65,21,84,199,121,156,51,54,99};
        if (address >= 0x86 && address <= 0x97)
            return stick_params[address - 0x86];
        if (address >= 0x98 && address <= 0xA9)
            return stick_params[address - 0x98];
        return 0;
    }
    if (page == 0x80)
        return 0xFF;
    if (page >= 0x20 && page <= 0x40) {
        if (address == 0x00 || address == 0x26) return 0x95;
        if (address == 0x01 || address == 0x27) return 0x22;
        if (address == 0x24 || address == 0x4A) return 0x68;
        return 0;
    }
    return 0;
}

static void spi_read(const uint8_t *in, uint8_t *out) {
    uint8_t address = in[11];
    uint8_t page = in[12];
    uint8_t length = in[15];
    if (length > 29)
        length = 29;
    memcpy(&out[15], &in[11], 5);
    out[19] = length;
    for (uint8_t i = 0; i < length; ++i)
        out[20 + i] = spi_byte(page, (uint8_t)(address + i));
}

static void info_reply(const uint8_t *in, uint8_t *out) {
    out[0] = 0x81;
    switch (in[1]) {
    case 0x01:
        out[1] = 0x01;
        out[3] = 0x03;
        for (int i = 0; i < 6; ++i)
            out[4 + i] = _mac[5 - i];
        break;
    case 0x02:
    case 0x03:
        out[1] = in[1];
        break;
    default:
        break;
    }
}

static void command_reply(const uint8_t *in, uint8_t *out,
                          btn_data_t *buttons, analog_data_t *analog) {
    uint8_t command = in[10];
    out[0] = 0x21;
    set_common(out, buttons, analog);
    out[13] = 0x80;
    out[14] = command;

    switch (command) {
    case 0x01: { /* pairing */
        static const uint8_t pro_controller[25] = {
            0x00,0x25,0x08,'P','r','o',' ','C','o','n','t','r','o','l','l','e','r',
            0,0,0,0,0,0,0,0x68};
        out[13] = 0x81;
        if (in[11] == 1 || in[11] == 4) {
            out[15] = 1;
            for (int i = 0; i < 6; ++i)
                out[16 + i] = _mac[5 - i];
            memcpy(&out[22], pro_controller, sizeof(pro_controller));
        } else if (in[11] == 2) {
            out[15] = 2;
            for (int i = 0; i < 16; ++i)
                out[16 + i] = (uint8_t)(0xA5u + i) ^ 0xAAu;
        } else {
            out[15] = 3;
        }
        break;
    }
    case 0x02: /* device info */
        out[13] = 0x82;
        out[15] = 0x04;
        out[16] = 0x33;
        out[17] = 0x03;
        out[18] = 0x02;
        memcpy(&out[19], _mac, sizeof(_mac));
        out[25] = 0x01;
        out[26] = 0x02;
        break;
    case 0x03: /* set input report mode */
    case 0x08: /* shipment mode */
    case 0x22: /* set NFC state */
    case 0x30: /* player LEDs */
    case 0x38: /* home LED */
    case 0x40: /* IMU */
    case 0x48: /* vibration */
        break;
    case 0x04: /* trigger elapsed time */
        out[13] = 0x83;
        for (int i = 0; i < 14; i += 2) {
            out[15 + i] = 100;
            out[16 + i] = 0;
        }
        break;
    case 0x10: /* SPI read */
        out[13] = 0x90;
        spi_read(in, out);
        break;
    default:
        break;
    }
}

void switch_protocol_init(void) {
    memset(_pending, 0, sizeof(_pending));
    _pending_len = 0;
    _pending_valid = false;
    _init_sent = false;
    _timer = 0;
}

void switch_protocol_queue_output(uint8_t report_id, const uint8_t *data,
                                  uint16_t len) {
    if (report_id != 0) {
        _pending[0] = report_id;
        if (len > 63) len = 63;
        memcpy(&_pending[1], data, len);
        _pending_len = len + 1;
    } else {
        if (len > sizeof(_pending)) len = sizeof(_pending);
        memcpy(_pending, data, len);
        _pending_len = len;
    }
    _pending_valid = _pending_len > 0;
}

bool switch_protocol_make_report(uint8_t report[64], btn_data_t *buttons,
                                 analog_data_t *analog) {
    memset(report, 0, 64);
    if (!_init_sent) {
        report[0] = 0x81;
        report[1] = 0x01;
        report[3] = 0x03;
        for (int i = 0; i < 6; ++i)
            report[4 + i] = _mac[5 - i];
        _init_sent = true;
        return true;
    }

    if (_pending_valid) {
        _pending_valid = false;
        if (_pending[0] == SW_OUT_INFO && _pending_len >= 2) {
            if (_pending[1] != 0x04)
                info_reply(_pending, report);
            else
                goto standard;
            return true;
        }
        if (_pending[0] == SW_OUT_SUBCOMMAND && _pending_len >= 11) {
            command_reply(_pending, report, buttons, analog);
            return true;
        }
        if (_pending[0] != SW_OUT_RUMBLE)
            goto standard;
    }

standard:
    report[0] = 0x30;
    set_common(report, buttons, analog);
    return true;
}
