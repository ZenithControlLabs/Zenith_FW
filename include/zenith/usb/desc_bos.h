#ifndef DESC_BOS_H
#define DESC_BOS_H

#include <stdint.h>

enum { VENDOR_REQUEST_WEBUSB = 1, VENDOR_REQUEST_MICROSOFT = 2 };
#define ITF_NUM_VENDOR 1

extern const uint8_t desc_bos[];
extern const uint8_t desc_ms_os_20[];

#endif // DESC_BOS_H