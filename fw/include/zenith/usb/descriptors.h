#ifndef ZENITH_DESCRIPTORS_H
#define ZENITH_DESCRIPTORS_H

#include <tusb.h>

extern const tusb_desc_device_t SWITCH_DEVICE_DESCRIPTOR;
extern const tusb_desc_device_t XINPUT_DEVICE_DESCRIPTOR;

extern const uint8_t HID_REPORT_DESCRIPTOR[];

extern const uint8_t SWITCH_CONFIGURATION_DESCRIPTOR[];
extern const uint8_t XINPUT_CONFIGURATION_DESCRIPTOR[];

extern const tusb_desc_webusb_url_t URL_DESCRIPTOR;

extern const char *SWITCH_STRING_DESCRIPTOR[];
extern const char *XINPUT_STRING_DESCRIPTOR[];

#endif // ZENITH_DESCRIPTORS_H
