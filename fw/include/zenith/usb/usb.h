#ifndef ZENITH_USB_H
#define ZENITH_USB_H

#define HID_REPORT_GAMEPAD 4

extern bool _usb_busy;

typedef struct __attribute__((packed)) {
    int8_t x;    ///< Delta x  movement of left analog-stick
    int8_t y;    ///< Delta y  movement of left analog-stick
    int8_t z;    ///< Delta z  movement of right analog-joystick
    int8_t rz;   ///< Delta Rz movement of right analog-joystick
    int8_t rx;   ///< Delta Rx movement of analog left trigger
    int8_t ry;   ///< Delta Ry movement of analog right trigger
    uint8_t hat; ///< Buttons mask for currently pressed buttons in the DPad/hat
    uint32_t buttons; ///< Buttons mask for currently pressed buttons
    float ax1_raw;
    float ax2_raw;
} hid_gamepad_report_ext_t;

int usb_init(void);

void usb_task(uint32_t timestamp, btn_data_t *buttons, analog_data_t *analog,
              analog_data_t *analog_raw);

#endif // ZENITH_USB_H