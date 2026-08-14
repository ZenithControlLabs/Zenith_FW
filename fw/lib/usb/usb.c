#include "zenith/includes.h"

#include <string.h>

static bool _usb_clear;
static uint32_t _usb_rate;

void usb_set_interval(usb_rate_t rate) { _usb_rate = rate; }

static bool xinput_mode(void) {
    return _settings[_profile].comms_mode == COMMS_MODE_XINPUT;
}

int usb_init(void) {
    usb_set_interval(xinput_mode() ? USBRATE_1 : USBRATE_8);
    switch_protocol_init();
    return tusb_init();
}

static bool switch_send(btn_data_t *buttons, analog_data_t *analog) {
    uint8_t report[64];
    if (!switch_protocol_make_report(report, buttons, analog))
        return false;
    return tud_hid_report(report[0], &report[1], 63);
}

void usb_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog,
              analog_data_t *analog_raw) {
    (void)analog_raw;
    tud_task();

    if (!interval_resettable_run(timestamp, _usb_rate, _usb_clear)) {
        _usb_clear = false;
        return;
    }

    if (xinput_mode()) {
        xinput_send(buttons, analog);
    } else if (tud_hid_ready()) {
        switch_send(buttons, analog);
    }
}

uint8_t const *tud_descriptor_device_cb(void) {
    return (const uint8_t *)(xinput_mode() ? &XINPUT_DEVICE_DESCRIPTOR
                                           : &SWITCH_DEVICE_DESCRIPTOR);
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return xinput_mode() ? XINPUT_CONFIGURATION_DESCRIPTOR
                         : SWITCH_CONFIGURATION_DESCRIPTOR;
}

uint8_t const *tud_descriptor_bos_cb(void) {
    return xinput_mode() ? NULL : desc_bos;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                               hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen) {
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;
    return 0;
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report,
                                uint16_t len) {
    (void)instance;
    (void)report;
    (void)len;
    _usb_clear = true;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                           hid_report_type_t report_type,
                           uint8_t const *buffer, uint16_t bufsize) {
    (void)instance;
    (void)report_type;
    if (!xinput_mode())
        switch_protocol_queue_output(report_id, buffer, bufsize);
}

uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance) {
    (void)instance;
    return xinput_mode() ? NULL : HID_REPORT_DESCRIPTOR;
}

static uint16_t _desc_str[64];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;
    const char **strings = xinput_mode() ? XINPUT_STRING_DESCRIPTOR
                                         : SWITCH_STRING_DESCRIPTOR;
    uint8_t chr_count;
    if (index == 0) {
        memcpy(&_desc_str[1], strings[0], 2);
        chr_count = 1;
    } else {
        if (index > 3)
            return NULL;
        const char *str = strings[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31)
            chr_count = 31;
        for (uint8_t i = 0; i < chr_count; ++i)
            _desc_str[1 + i] = str[i];
    }
    _desc_str[0] = (TUSB_DESC_STRING << 8) | (2 * chr_count + 2);
    return _desc_str;
}

void tud_vendor_rx_cb(uint8_t itf, uint8_t const *buffer, uint16_t bufsize) {
    (void)buffer;
    (void)bufsize;
    uint8_t local_buffer[64] = {0};
    uint32_t read = tud_vendor_n_read(itf, local_buffer, sizeof(local_buffer));
    if (read)
        webusb_command_processor(local_buffer, read);
}

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *request) {
    if (xinput_mode())
        return false;
    if (stage != CONTROL_STAGE_SETUP)
        return true;

    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR) {
        if (request->bRequest == VENDOR_REQUEST_WEBUSB)
            return tud_control_xfer(rhport, request,
                                    (void *)(uintptr_t)&URL_DESCRIPTOR,
                                    URL_DESCRIPTOR.bLength);
        if (request->bRequest == VENDOR_REQUEST_MICROSOFT &&
            request->wIndex == 7) {
            uint16_t total_len;
            memcpy(&total_len, desc_ms_os_20 + 8, 2);
            return tud_control_xfer(rhport, request,
                                    (void *)(uintptr_t)desc_ms_os_20,
                                    total_len);
        }
    }
    if (request->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS)
        return tud_control_status(rhport, request);
    return false;
}
