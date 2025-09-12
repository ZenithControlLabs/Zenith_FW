#ifndef __SETTINGS_H
#define __SETTINGS_H 

#include "stick.h"

typedef struct {
    stick_config_t stick_config;
    calib_results_t calib_results;
} settings_t;

extern settings_t g_settings;

bool settings_load();
void settings_flush();

#endif // __SETTINGS_H
