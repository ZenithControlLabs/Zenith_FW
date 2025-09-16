#ifndef __SETTINGS_H
#define __SETTINGS_H 

#include "stick.h"
#include "intf.h"

// Last page of flash
#define SETTINGS_FLASH_ADDR 0x080FF000
// Magic data
#define MAGIC 0xCAFEBEEFDEADBABE

typedef struct {
    float x_m;
    float x_b;
    float y_m;
    float y_b;
} dac_calib_t;

typedef struct {
    stick_config_t stick_config;
    calib_results_t calib_results;
    dac_calib_t dac_calib;
    bool polarity[CHAN_END];
} settings_t __attribute__((aligned(16)));
#define SETTINGS_SIZE_DOUBLEWORDS (sizeof(settings_t) + ((1<<3)-1)) >> 3

extern settings_t g_settings;
extern const uint64_t g_magic;

bool settings_load();
void settings_flush();

#endif // __SETTINGS_H
