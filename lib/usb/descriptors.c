#include "zenith/includes.h"

// clang-format off

const tusb_desc_device_t DEVICE_DESCRIPTOR = {
    .bLength = 18,
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0210, // Changed from 0x0200 to 2.1 for BOS & WebUSB
    .bDeviceClass = 0x00,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,

    .bMaxPacketSize0 = 64,
    .idVendor = ZTH_VID,
    .idProduct = ZTH_PID,

    .bcdDevice = 0x0100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

#define REPORT_ID_GAMEPAD 4

// Generic Gamepad HID descriptor
const uint8_t HID_REPORT_DESCRIPTOR [] = {
   HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
    HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
    HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
        /* Report ID if any */\
        HID_REPORT_ID(REPORT_ID_GAMEPAD)\
        /* 8 bit X, Y, Z, Rz, Rx, Ry (min -127, max 127 ) */ \
        HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_RZ                    ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
        HID_LOGICAL_MIN    ( 0x81                                   ) ,\
        HID_LOGICAL_MAX    ( 0x7f                                   ) ,\
        HID_REPORT_COUNT   ( 6                                      ) ,\
        HID_REPORT_SIZE    ( 8                                      ) ,\
        HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* 8 bit DPad/Hat Button Map  */ \
        HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
        HID_USAGE          ( HID_USAGE_DESKTOP_HAT_SWITCH           ) ,\
        HID_LOGICAL_MIN    ( 1                                      ) ,\
        HID_LOGICAL_MAX    ( 8                                      ) ,\
        HID_PHYSICAL_MIN   ( 0                                      ) ,\
        HID_PHYSICAL_MAX_N ( 315, 2                                 ) ,\
        HID_REPORT_COUNT   ( 1                                      ) ,\
        HID_REPORT_SIZE    ( 8                                      ) ,\
        HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* 32 bit Button Map */ \
        HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
        HID_USAGE_MIN      ( 1                                      ) ,\
        HID_USAGE_MAX      ( 32                                     ) ,\
        HID_LOGICAL_MIN    ( 0                                      ) ,\
        HID_LOGICAL_MAX    ( 1                                      ) ,\
        HID_REPORT_COUNT   ( 32                                     ) ,\
        HID_REPORT_SIZE    ( 1                                      ) ,\
        HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
        /* Raw reporting */
        HID_USAGE_PAGE_N     ( HID_USAGE_PAGE_VENDOR, 2                  ) ,\
        HID_USAGE (0x01),\
        HID_LOGICAL_MIN    ( 0x80                                   ) ,\
        HID_LOGICAL_MAX    ( 0x7F                                   ) ,\
        HID_REPORT_COUNT   ( 2                                    ) ,\
        HID_REPORT_SIZE    ( 32                                      ) ,\
        HID_INPUT        ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE | HID_WRAP_NO | HID_LINEAR | HID_PREFERRED_STATE | HID_NO_NULL_POSITION | HID_NON_VOLATILE ) ,\
    HID_COLLECTION_END             ,\

};


const uint8_t CONFIGURATION_DESCRIPTOR[] = {
     // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 2, 0, 64, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 500),

    // Interface
    9, TUSB_DESC_INTERFACE, 0x00, 0x00, 0x02, TUSB_CLASS_HID, 0x00, 0x00, 0x00,
    // HID Descriptor
    9, HID_DESC_TYPE_HID, U16_TO_U8S_LE(0x0111), 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(sizeof(HID_REPORT_DESCRIPTOR)),
    // Endpoint Descriptor
    7, TUSB_DESC_ENDPOINT, 0x81, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(64), 8,
    // Endpoint Descriptor
    7, TUSB_DESC_ENDPOINT, 0x01, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(64), 8,

    // Alternate Interface for WebUSB
    // Interface
    9, TUSB_DESC_INTERFACE, 0x01, 0x00, 0x02, TUSB_CLASS_VENDOR_SPECIFIC, 0x00, 0x00, 0x00,
    // Endpoint Descriptor
    7, TUSB_DESC_ENDPOINT, 0x82, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
    // Endpoint Descriptor
    7, TUSB_DESC_ENDPOINT, 0x02, TUSB_XFER_BULK, U16_TO_U8S_LE(64), 0,
};

const tusb_desc_webusb_url_t URL_DESCRIPTOR =
    {
        .bLength = 3 + sizeof(ZTH_WEBUSB_URL) - 1,
        .bDescriptorType = 3, // WEBUSB URL type
        .bScheme = 1,         // 0: http, 1: https
        .url = ZTH_WEBUSB_URL};

const char* STRING_DESCRIPTOR[] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},                // 0: is supported language is English (0x0409)
    ZTH_MANUFACTURER,              // 1: Manufacturer
    ZTH_PRODUCT,        // 2: Product
    "000000",           // 3: Serials, should use chip ID
};

// clang-format on