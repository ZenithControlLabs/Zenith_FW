#ifndef ZENITH_TYPES_H
#define ZENITH_TYPES_H

#include <stdint.h>

typedef union {
    struct {
        uint8_t b1 : 1;
        uint8_t b2 : 1;
        uint8_t b3 : 1;
        uint8_t b4 : 1;
        uint8_t b5 : 1;
        uint8_t b6 : 1;
        uint8_t b7 : 1;
        uint8_t b8 : 1;
        uint8_t b9 : 1;
        uint8_t b10 : 1;
        uint8_t b11 : 1;
        uint8_t b12 : 1;
        uint8_t b13 : 1;
        uint8_t b14 : 1;
        uint8_t b15 : 1;
        uint8_t b16 : 1;
        uint8_t b17 : 1;
        uint8_t b18 : 1;
        uint8_t b19 : 1;
        uint8_t b20 : 1;
        uint8_t b21 : 1;
        uint8_t b22 : 1;
        uint8_t b23 : 1;
        uint8_t b24 : 1;
        uint8_t b25 : 1;
        uint8_t b26 : 1;
        uint8_t b27 : 1;
        uint8_t b28 : 1;
        uint8_t b29 : 1;
        uint8_t b30 : 1;
        uint8_t b31 : 1;
    } __attribute__((packed)) s;
    uint32_t r;
} btn_data_t;

// Represents the range of an axis from -1.0 to 1.0.
// This is an arbitrary range, but it's meant to be easiest to
// convert to different representations used by different devices.
typedef float ax_t;

#define AX_TO_UINT8(x)                                                         \
    ((uint8_t)((int8_t)((x) * (1 << (8 - 1))) + (1 << (8 - 1))))
#define AX_TO_INT8(x) ((int8_t)((x) * (1 << (8 - 1))))
#define AX_TO_INT32(x) ((int32_t)((x) * (1 << (32 - 1))))
#define INT_N_TO_AX(x, N) (((ax_t)(x)) / (1 << (N - 1)))
#define UINT_N_TO_AX(x, N)                                                     \
    ((ax_t)((int)((x) - (1 << (N - 1)))) / (1 << (N - 1)))

typedef struct {
    ax_t ax1;
    ax_t ax2;
    ax_t ax3;
    ax_t ax4;
    ax_t ax5;
    ax_t ax6;
} analog_data_t;

typedef enum {
    USBRATE_8 = 7600,
    USBRATE_1 = 600,
} usb_rate_t;

#endif // ZENITH_TYPES_H