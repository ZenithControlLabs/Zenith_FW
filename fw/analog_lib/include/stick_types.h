#ifndef STICK_TYPES_H
#define STICK_TYPES_H

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

#endif // STICK_TYPES_H