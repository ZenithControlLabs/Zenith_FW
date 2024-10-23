/*
 * Copyright (c) [2023] [Mitch Cairns/Handheldlegend, LLC]
 * All rights reserved.
 *
 * This source code is licensed under the provisions of the license found in the
 * LICENSE file in the root directory of this source tree.
 */

#include "zenith/includes.h"

bool _usb_clear = false;

// Default 8ms (8000us)
uint32_t _usb_rate = 0;

void usb_set_interval(usb_rate_t rate) { _usb_rate = rate; }

int usb_init(void) {
    usb_set_interval(USBRATE_8);
    return tusb_init();
}

int hid_report(btn_data_t *buttons, analog_data_t *analog,
               analog_data_t *analog_raw) {
    hid_gamepad_report_ext_t report = {.x = (int8_t)(analog->ax1 * 128.0),
                                       .y = (int8_t)(analog->ax2 * -128.0),
                                       .z = (int8_t)(analog->ax3 * 128.0),
                                       .rx = (int8_t)(analog->ax4 * 128.0),
                                       .ry = (int8_t)(analog->ax5 * 128.0),
                                       .rz = (int8_t)(analog->ax6 * 128.0),
                                       .hat = GAMEPAD_HAT_CENTERED,
                                       .buttons = buttons->r,
                                       .ax1_raw = analog_raw->ax1,
                                       .ax2_raw = analog_raw->ax2};

    return tud_hid_report(0x4, &report, sizeof(hid_gamepad_report_ext_t));
}

void usb_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog,
              analog_data_t *analog_raw) {
    tud_task();

    if (interval_resettable_run(timestamp, _usb_rate, _usb_clear)) {
        if (tud_hid_ready()) {
            hid_report(buttons, analog, analog_raw);
        }
    } else {
        _usb_clear = false;
    }
}

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET DEVICE DESCRIPTOR
// Application return pointer to descriptor
uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&DEVICE_DESCRIPTOR;
}

// Invoked when received GET CONFIGURATION DESCRIPTOR
// Application return pointer to descriptor
// Descriptor contents must exist long enough for transfer to complete
uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index; // for multiple configurations
    return (uint8_t const *)&CONFIGURATION_DESCRIPTOR;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)reqlen;

    return 0;
}

// Invoked when report complete
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report,
                                uint16_t len) {
    _usb_clear = true;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type, uint8_t const *buffer,
                           uint16_t bufsize) {}

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return HID_REPORT_DESCRIPTOR;
}

static uint16_t _desc_str[64];

// Invoked when received GET STRING DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long
// enough for transfer to complete
uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    uint8_t chr_count;

    if (index == 0) {
        memcpy(&_desc_str[1], STRING_DESCRIPTOR[0], 2);
        chr_count = 1;
    } else {
        // Note: the 0xEE index string is a Microsoft OS 1.0 Descriptors.
        // https://docs.microsoft.com/en-us/windows-hardware/drivers/usbcon/microsoft-defined-usb-descriptors

        const char *str = STRING_DESCRIPTOR[index];

        // Cap at max char... WHY?
        chr_count = strlen(str);
        if (chr_count > 31)
            chr_count = 31;

        // Convert ASCII string into UTF-16
        for (uint8_t i = 0; i < chr_count; i++) {
            _desc_str[1 + i] = str[i];
        }
    }

    // first byte is length (including header), second byte is string type
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

// Vendor Device Class CB for receiving data
void tud_vendor_rx_cb(uint8_t itf) {
    debug_print("WebUSB Data Received.\n");
    uint8_t buffer[64] = {0};
    uint32_t size = 0;
    tud_vendor_n_read(itf, buffer, 64);
    webusb_command_processor(buffer);
}

// Invoked when a control transfer occurred on an interface of this class
// Driver response accordingly to the request and the transfer stage
// (setup/data/ack) return false to stall control endpoint (e.g unsupported
// request)
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
    // nothing to with DATA & ACK stage
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    switch (request->bmRequestType_bit.type) {
    case TUSB_REQ_TYPE_VENDOR:
        switch (request->bRequest) {
        case VENDOR_REQUEST_WEBUSB:
            // match vendor request in BOS descriptor
            // Get landing page url
            return tud_control_xfer(rhport, request,
                                    (void *)(uintptr_t)&URL_DESCRIPTOR,
                                    URL_DESCRIPTOR.bLength);

        case VENDOR_REQUEST_MICROSOFT:
            if (request->wIndex == 7) {
                // Get Microsoft OS 2.0 compatible descriptor
                uint16_t total_len;
                memcpy(&total_len, desc_ms_os_20 + 8, 2);

                return tud_control_xfer(rhport, request,
                                        (void *)(uintptr_t)desc_ms_os_20,
                                        total_len);
            } else {
                return false;
            }

        default:
            break;
        }
        break;

    case TUSB_REQ_TYPE_CLASS:
        debug_print("Vendor Request: %x", request->bRequest);

        // response with status OK
        return tud_control_status(rhport, request);
        break;

    default:
        break;
    }

    // stall unknown request
    return false;
}