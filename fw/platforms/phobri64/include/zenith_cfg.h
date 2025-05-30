#ifndef ZENITH_CFG_H
#define ZENITH_CFG_H

#define ZTH_WEBUSB_URL "zenithcontrollabs.github.io/Zenith_FW/web/platforms/phobri64"
#define ZTH_MANUFACTURER "Zenith Control Labs"
#define ZTH_PRODUCT "Phobri64"

#define ZTH_PID 0x0001

#include "hw_intf.h"
#define ZENITH_SERIAL_PIN JOYBUS_PIN

#define ZTH_N64_REMAP_DEFAULT                                                  \
    {                                                                          \
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0,  \
            0, 0, 0, 0, 0, 0, 0, 0, 0                                          \
    }

#ifdef HW_PHOBRI_PROTO
#define ZTH_SEPARATE_CAL_READ 1
#endif

#define ZTH_STICK_INTERVAL 2000


typedef struct {
    int16_t h_x;
    int16_t h_y;
    int16_t h_z;
    int16_t h_temp;
    uint16_t ax;
    uint16_t ay;
} v1_1_datacollect_t;

extern v1_1_datacollect_t dc;

#endif // ZENITH_CFG_H