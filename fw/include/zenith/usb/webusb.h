#ifndef ZENITH_WEBUSB_H
#define ZENITH_WEBUSB_H

#include <math.h>
#include <stdint.h>

extern bool _webusb_output_enabled;

typedef enum {
    // Get firmware version
    WEBUSB_CMD_FW_GET = 0xA1,

    WEBUSB_CMD_CALIBRATION_START = 0x01,
    WEBUSB_CMD_CALIBRATION_ADVANCE = 0x02,
    WEBUSB_CMD_CALIBRATION_UNDO = 0x03,
    WEBUSB_CMD_CALIBRATION_STATUS_GET = 0xA2,

    WEBUSB_CMD_NOTCH_SET = 0x04,
    WEBUSB_CMD_NOTCHES_GET = 0xA4,

    WEBUSB_CMD_REMAP_SET = 0x05,
    WEBUSB_CMD_REMAP_GET = 0xA5,

    WEBUSB_CMD_MAG_THRESH_SET = 0x06,
    WEBUSB_CMD_MAG_THRESH_GET = 0xA6,

    WEBUSB_CMD_GATE_LIMITER_SET = 0x07,
    WEBUSB_CMD_GATE_LIMITER_GET = 0xA7,

    WEBUSB_CMD_UPDATE_FW = 0xF1,
    WEBUSB_CMD_COMMIT_SETTINGS = 0xF2,
    WEBUSB_CMD_RESET_SETTINGS = 0xF3,
} webusb_cmd_t;

#define WEBUSB_CMD_USER_MASK 0b11110000
#define WEBUSB_CMD_USER_VAL 0b01100000

/*
// For ease of communication, the angles are sent to webusb in signed 16-bit
// format. Because they're just ranges though, they're not supposed to be
// signed.

// The *2 (1/M_PI_2 = 1/(M_PI/2) => 2 * 1/M_PI) is because the notch snapping
// checks the angle on both sides of a region. So it would be doubled by
// default. To compensate, we internally store half, so we dont have to half it
// every notch remap call, but whenever we send it, we need to double it.
#define DEADZONE_ANG_TO_WEBUSB(x) (int16_t)((x / M_PI_2) * (1 << (16 - 1)))
#define DEADZONE_ANG_FROM_WEBUSB(x) ((((float)x) / (1 << (16 - 1))) * M_PI_2)
*/

// defining for access to stick data through WebUSB
extern analog_data_t _analog_data;
extern analog_data_t _analog_data_processed;

// The WebUSB interface will send output reports separately over the USB.
// But it doesn't need to be sending that when the controller is just being used
// for HID, so it will disable itself after this many reports sent without a
// confirm on the JS side.
#define WEBUSB_OE_CONFIRM_THRESHOLD 1

void webusb_save_confirm();
void webusb_command_processor(uint8_t *data, const uint32_t data_size);
// It's called input report, but this is from the perspective of the USB host.
// This is an output of the controller.
void webusb_input_report();

#endif // ZENITH_WEBUSB_H