#include "zenith/includes.h"

#include "device/usbd_pvt.h"

#include <limits.h>
#include <string.h>

typedef struct {
    uint8_t interface_number;
    uint8_t ep_in;
    uint8_t ep_out;
    CFG_TUSB_MEM_ALIGN uint8_t in[32];
    CFG_TUSB_MEM_ALIGN uint8_t out[32];
} xinput_interface_t;

static xinput_interface_t _xinput;

static void driver_reset(uint8_t rhport) {
    (void)rhport;
    tu_memclr(&_xinput, sizeof(_xinput));
}

static void driver_init(void) { driver_reset(0); }

static uint16_t driver_open(uint8_t rhport,
                            tusb_desc_interface_t const *interface,
                            uint16_t max_len) {
    if (interface->bInterfaceClass != 0xFF ||
        interface->bInterfaceSubClass != 0x5D || max_len < 39)
        return 0;

    const uint8_t *descriptor = tu_desc_next(interface);
    uint8_t endpoints = 0;
    while (endpoints < interface->bNumEndpoints) {
        if (tu_desc_type(descriptor) == TUSB_DESC_ENDPOINT) {
            const tusb_desc_endpoint_t *ep =
                (const tusb_desc_endpoint_t *)descriptor;
            if (!usbd_edpt_open(rhport, ep))
                return 0;
            if (tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN)
                _xinput.ep_in = ep->bEndpointAddress;
            else
                _xinput.ep_out = ep->bEndpointAddress;
            ++endpoints;
        }
        descriptor = tu_desc_next(descriptor);
    }
    _xinput.interface_number = interface->bInterfaceNumber;
    if (_xinput.ep_out)
        usbd_edpt_xfer(rhport, _xinput.ep_out, _xinput.out,
                       sizeof(_xinput.out));
    return 39;
}

static bool driver_control(uint8_t rhport, uint8_t stage,
                           tusb_control_request_t const *request) {
    (void)rhport;
    (void)stage;
    (void)request;
    return true;
}

static bool driver_xfer(uint8_t rhport, uint8_t ep_addr,
                        xfer_result_t result, uint32_t transferred) {
    (void)transferred;
    if (result != XFER_RESULT_SUCCESS)
        return false;
    if (ep_addr == _xinput.ep_out)
        return usbd_edpt_xfer(rhport, _xinput.ep_out, _xinput.out,
                              sizeof(_xinput.out));
    return true;
}

static const usbd_class_driver_t XINPUT_DRIVER = {
#if CFG_TUSB_DEBUG >= 2
    .name = "XINPUT",
#endif
    .init = driver_init,
    .reset = driver_reset,
    .open = driver_open,
    .control_xfer_cb = driver_control,
    .xfer_cb = driver_xfer,
    .sof = NULL,
};

usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count) {
    if (_settings[_profile].comms_mode == COMMS_MODE_XINPUT) {
        *driver_count = 1;
        return &XINPUT_DRIVER;
    }
    *driver_count = 0;
    return NULL;
}

typedef struct __attribute__((packed)) {
    uint8_t report_id;
    uint8_t report_size;
    uint8_t buttons_1;
    uint8_t buttons_2;
    uint8_t trigger_l;
    uint8_t trigger_r;
    int16_t left_x;
    int16_t left_y;
    int16_t right_x;
    int16_t right_y;
    uint8_t reserved[6];
} xinput_report_t;

static int16_t axis_to_xinput(ax_t axis) {
    float scale = _settings[_profile].usb_stick_scale;
    if (!(scale >= 0.0f && scale <= 2.0f))
        scale = 1.0f;
    axis *= scale;
    float scaled = axis * 32767.0f;
    if (scaled > INT16_MAX) scaled = INT16_MAX;
    if (scaled < INT16_MIN) scaled = INT16_MIN;
    return (int16_t)scaled;
}

bool xinput_ready(void) {
    return tud_ready() && _xinput.ep_in &&
           !usbd_edpt_busy(0, _xinput.ep_in);
}

bool xinput_send(btn_data_t *buttons, analog_data_t *analog) {
    if (!xinput_ready())
        return false;

    btn_data_t mapped = {0};
    btn_remap_for_mode(REMAP_MODE_XINPUT, buttons, &mapped);
    xinput_report_t report = {
        .report_id = 0,
        .report_size = sizeof(xinput_report_t),
        .buttons_1 = (uint8_t)mapped.r,
        .buttons_2 = (uint8_t)(mapped.r >> 8),
        .trigger_l = (mapped.r & (1u << 16)) ? 0xFF : 0,
        .trigger_r = (mapped.r & (1u << 17)) ? 0xFF : 0,
        .left_x = axis_to_xinput(analog->ax1),
        .left_y = axis_to_xinput(analog->ax2),
        .right_x = 0,
        .right_y = 0,
        .reserved = {0},
    };
    memcpy(_xinput.in, &report, sizeof(report));
    if (!usbd_edpt_claim(0, _xinput.ep_in))
        return false;
    bool sent = usbd_edpt_xfer(0, _xinput.ep_in, _xinput.in, sizeof(report));
    usbd_edpt_release(0, _xinput.ep_in);
    return sent;
}
