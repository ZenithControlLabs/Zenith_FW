#ifndef ZENITH_CFG_DEF_H
#define ZENITH_CFG_DEF_H

#ifndef ZTH_WEBUSB_URL
#error "Must define ZTH_WEBUSB_URL in zenith_cfg!"
#endif

#ifndef ZTH_MANUFACTURER
#error                                                                         \
    "Must define ZTH_MANUFACTURER in zenith_cfg.h! Will be the USB vendor name"
#endif

#ifndef ZTH_PRODUCT
#error "Must define ZTH_PRODUCT in zenith_cfg! Will be the USB product name"
#endif

#ifndef ZTH_VID
#define ZTH_VID 0x7318
#endif

#ifndef ZTH_PID
#define ZTH_PID 0x0000
#endif

#ifndef ZTH_LINEARIZATION_EN
#define ZTH_LINEARIZATION_EN 0
#endif

// if you want to use a different set of analog data for calibration,
// enable this and implement cb_zenith_read_analog_cal
// useful for implementing multisampling or other means of pre-processing data
// going into calibration
#ifndef ZTH_SEPARATE_CAL_READ
#define ZTH_SEPARATE_CAL_READ 0
#endif

#ifndef ZTH_USB_INTERVAL
#define ZTH_USB_INTERVAL USBRATE_8
#endif

#ifndef ZTH_STICK_INTERVAL
#define ZTH_STICK_INTERVAL 500
#endif

#ifndef ZTH_WEBUSB_INTERVAL
#define ZTH_WEBUSB_INTERVAL 16000
#endif

#ifndef ZTH_N64_REMAP_DEFAULT
#define ZTH_N64_REMAP_DEFAULT                                                  \
    { 0 }
#endif

#ifndef ZTH_GAMECUBE_REMAP_DEFAULT
#define ZTH_GAMECUBE_REMAP_DEFAULT                                             \
    { 0 }
#endif

#endif // ZENITH_CFG_DEF_H