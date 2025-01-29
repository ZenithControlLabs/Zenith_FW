#ifndef ZENITH_CFG_H
#define ZENITH_CFG_H

#define ZTH_WEBUSB_URL "zenithcontrollabs.github.io/Zenith_FW/web/platforms/remapper"
#define ZTH_MANUFACTURER "Zenith Control Labs"
#define ZTH_PRODUCT "N64 Remapper"

#define ZTH_PID 0x0002

#define ZENITH_SERIAL_PIN 11

#define ZTH_SEPARATE_CAL_READ 1

#define ZTH_N64_REMAP_DEFAULT                                                  \
    {                                                                          \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF,      \
            0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0               \
    }

// Uncomment to build the firmware for the switcher.
// #define ZENITH_SWITCHER

// GPIO pin to read the switch value from
#define SWITCH_PIN 10

// GPIO pin for the first controller
#define JOYBUS_CTLR0 2

// GPIO pin for the second controller
#define JOYBUS_CTLR1 3

#endif // ZENITH_CFG_H