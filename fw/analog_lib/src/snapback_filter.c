#include "snapback_filter.h"
#include "stick.h"
#include <math.h>

float g_alpha = 0.f;

inline __attribute__((always_inline)) float lpf(float prev, float meas) {
    return prev + g_alpha * (meas - prev);
}

void snapback_filter(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out, const stick_config_t *stick_config) {
    static ax_t x_prev = 0.f;
    static ax_t y_prev = 0.f;
    static ax_t vx_prev = 0.f;
    static ax_t vy_prev = 0.f;
    static float cutoff_hz_prev = 0.f;
    
    if (cutoff_hz_prev != stick_config->cutoff_hz) {
        cutoff_hz_prev = stick_config->cutoff_hz;
        float rc = 1/(M_PI_2 * cutoff_hz_prev);   
        g_alpha = al_g_dt / (rc + al_g_dt);   
    }

    *x_out = lpf(x_prev, x_in);
    *y_out = lpf(y_prev, y_in);
    vx_prev = ((*x_out) - x_prev) / al_g_dt;
    vy_prev = ((*y_out) - y_prev) / al_g_dt;
    x_prev = *x_out;
    y_prev = *y_out;
}

// attempts at doing something better
/*
#define CUTOFF_MIN 200
#define CUTOFF_MAX 2000
#define CUTOFF_INC(delay) (CUTOFF_MAX - CUTOFF_MIN)/(delay/al_g_dt)

void snapback_filter(const ax_t x_in, const ax_t y_in, ax_t *x_out, ax_t *y_out, const stick_config_t *stick_config) {
    // These are the predictions for the state that the input measurement represents,
    static ax_t x_p = 0.f;
    static ax_t y_p = 0.f;
    static ax_t raw_x_p = 0.f;
    static ax_t raw_y_p = 0.f;
    static ax_t vx_p = 0.f;
    static ax_t vy_p = 0.f;
    static int snapback_timer = 0;
    float cutoff = CUTOFF_MAX - snapback_timer * CUTOFF_INC(2.2/30.f);
    snapback_timer = snapback_timer ? snapback_timer - 1 : 0;
    float rc = 1/(M_PI_2 * cutoff);   
    float a = al_g_dt / (rc + al_g_dt);   
    float b = 0.0f; // (1.0 + (1.0 - fabs(x_p)) / 5.f)

    float vx = x_in - raw_x_p / al_g_dt;
    float vy = y_in - raw_y_p / al_g_dt;
    float vmag = (vx * vx) + (vy * vy);
    float mag = (x_in * x_in) + (y_in * y_in);

    if (((fabs(vx) < fabs(vx_p)) || (fabs(vy) < fabs(vy_p))) && (mag < 0.02f) && (vmag > stick_config->cutoff_hz)) {
        snapback_timer = (int)((2.2/30.f)/al_g_dt);
    }

    vx_p = vx;
    vy_p = vy;

    // 1. Update Current State
    float rx = x_in - x_p;
    float ry = y_in - y_p;
    
    x_p += a * rx;
    vx_p += (b * rx) / al_g_dt; 

    y_p += a * ry;
    vy_p += (b * ry) / al_g_dt; 

    // 2. Set outputs
    *x_out = x_p;
    *y_out = y_p;

    // 3. Predict next state
    x_p += (vx_p * al_g_dt);
    // assume constant velocity

    y_p += (vy_p * al_g_dt);
    // assume constant velocity
}*/