#include "stick.h"
#include "notch_remap.h"
#include "linearize.h"
#include "snapback_filter.h"

calib_results_t* al_g_calib_results;
stick_config_t* al_g_stick_config;
float al_g_dt = 0;

volatile int _cal_step = 0;
ax_t raw_cal_points_x[CALIBRATION_NUM_STEPS];
ax_t raw_cal_points_y[CALIBRATION_NUM_STEPS];

/*
Cleaned calibration point array layout:

[
    CENTER,
    RIGHT,
    UPRIGHT,
    UP,
    UPLEFT,
    LEFT,
    DOWNLEFT,
    DOWN,
    DOWNRIGHT
]
*/

void fold_center_points(const ax_t raw_cal_points_x[],
                        const ax_t raw_cal_points_y[], ax_t cleaned_points_x[],
                        ax_t cleaned_points_y[]) {
    cleaned_points_x[0] = 0;
    cleaned_points_y[0] = 0;

    for (int i = 0; i < NUM_NOTCHES; i++) {
        // each origin reading is summed together
        cleaned_points_x[0] += raw_cal_points_x[i * 2];
        cleaned_points_y[0] += raw_cal_points_y[i * 2];

        // for the notches, copy point into cleaned list
        cleaned_points_x[i + 1] = raw_cal_points_x[i * 2 + 1];
        cleaned_points_y[i + 1] = raw_cal_points_y[i * 2 + 1];
    }

    // remove the largest and smallest origin values to remove outliers
    // first, find their indices

    int smallestX = 0;
    int largestX = 0;
    int smallestY = 0;
    int largestY = 0;
    for (int i = 0; i < NUM_NOTCHES; i++) {
        if (raw_cal_points_x[i * 2] < raw_cal_points_x[smallestX]) {
            // record the new smallest index
            smallestX = i * 2;
        } else if (raw_cal_points_x[i * 2] > raw_cal_points_x[largestX]) {
            // record the new largest index
            largestX = i * 2;
        }

        if (raw_cal_points_y[i * 2] < raw_cal_points_y[smallestY]) {
            // record the new smallest index
            smallestY = i * 2;
        } else if (raw_cal_points_y[i * 2] > raw_cal_points_y[largestY]) {
            // record the new largest index
            largestY = i * 2;
        }
    }
    // subtract the smallest and largest values
    cleaned_points_x[0] -= raw_cal_points_x[smallestX];
    cleaned_points_x[0] -= raw_cal_points_x[largestX];
    cleaned_points_y[0] -= raw_cal_points_y[smallestY];
    cleaned_points_y[0] -= raw_cal_points_y[largestY];

    // divide by the total number of calibration steps/2 to get the average
    // origin value except it's minus 4 steps since we removed outliers
    cleaned_points_x[0] = cleaned_points_x[0] / ((float)NUM_NOTCHES - 2);
    cleaned_points_y[0] = cleaned_points_y[0] / ((float)NUM_NOTCHES - 2);
}

void calibration_finish(void) {
    // We're done calibrating. Do the math to save our calibration parameters
    ax_t cleaned_points_x[NUM_NOTCHES + 1];
    ax_t cleaned_points_y[NUM_NOTCHES + 1];
    for (int i = 0; i < CALIBRATION_NUM_STEPS; i++) {
        al_debug_print("Raw Cal point:  %d; (x,y) = (%f, %f)\n\r", i,
                    raw_cal_points_x[i], raw_cal_points_y[i]);
    }
    fold_center_points(raw_cal_points_x, raw_cal_points_y, cleaned_points_x,
                       cleaned_points_y);
    for (int i = 0; i <= NUM_NOTCHES; i++) {
        al_debug_print("Clean Cal point:  %d; (x,y) = (%f, %f)\n\r", i,
                    cleaned_points_x[i], cleaned_points_y[i]);
    }

#if ZTH_LINEARIZATION_EN
    // One less because center is becoming 0 implcitly
    ax_t linearized_points_x[NUM_NOTCHES];
    ax_t linearized_points_y[NUM_NOTCHES];

    linearize_cal(cleaned_points_x, cleaned_points_y, linearized_points_x,
                  linearized_points_y, al_g_calib_results);

    // Copy the linearized points we have just found to phobri's internal data
    // sturcture.
    for (int i = 0; i < NUM_NOTCHES; i++) {
        al_g_calib_results->notch_points_x_in[i] = linearized_points_x[i];
        al_g_calib_results->notch_points_y_in[i] = linearized_points_y[i];
        al_debug_print("Linearized point:  %d; (x,y) = (%f, %f)\n", i,
                    linearized_points_x[i], linearized_points_y[i]);
    }
#else
    float x_flip = cleaned_points_x[1] > cleaned_points_x[5] ? 1 : -1; 
    float y_flip = cleaned_points_y[3] > cleaned_points_y[7] ? 1 : -1;
    // Linearization normally moves the center point to an implicit 0,0.
    // To carry forth the assumption for notch calibration, we will do the same.
    for (int i = 0; i < NUM_NOTCHES; i++) {
        // TODO: notch calibration assumes that the notches have an increasing
        // angle; doing this will mess it up if the sensor is negative to go up
        // in Y; need to figure out the appropriate place in the code to
        // compensate for this
        al_g_calib_results->notch_points_x_in[i] =
            (cleaned_points_x[i + 1] - cleaned_points_x[0]) * x_flip;
        al_g_calib_results->notch_points_y_in[i] =
            (cleaned_points_y[i + 1] - cleaned_points_y[0]) * y_flip;
        al_debug_print("Notch Point in point:  %d; (x,y) = (%f, %f)\n", i,
                    al_g_calib_results->notch_points_x_in[i],
                    al_g_calib_results->notch_points_y_in[i]);
    }
    // copy over center offset
    al_g_calib_results->fit_coeffs_x[0] = cleaned_points_x[0];
    al_g_calib_results->fit_coeffs_y[0] = cleaned_points_y[0];
    // set direction for each axis
    al_g_calib_results->fit_coeffs_x[1] = x_flip;
    al_g_calib_results->fit_coeffs_y[1] = y_flip;

#endif // ZTH_LINEARIZATON_EN


    notch_calibrate(al_g_calib_results->notch_points_x_in,
                    al_g_calib_results->notch_points_y_in,
                    al_g_stick_config->notch_points_x,
                    al_g_stick_config->notch_points_y,
                    al_g_calib_results);
    al_debug_print("Calibrated!\n");
    al_g_calib_results->calibrated = true;
    /*al_debug_print("X coeffs: %f %f %f %f, Y coeffs: %f %f %f %f\n",
           _settings.calib_results.fit_coeffs_x[0],
           _settings.calib_results.fit_coeffs_x[1],
           _settings.calib_results.fit_coeffs_x[2],
           _settings.calib_results.fit_coeffs_x[3],
           _settings.calib_results.fit_coeffs_y[0],
           _settings.calib_results.fit_coeffs_y[1],
           _settings.calib_results.fit_coeffs_y[2],
           _settings.calib_results.fit_coeffs_y[3]);*/
    _cal_step = 0;
}

void analoglib_cal_advance(analog_data_t *in) {
    // failsafe - this function should not be called if incrementing the step
    // would lead to an invalid state
    if (_cal_step < 1 || _cal_step > CALIBRATION_NUM_STEPS)
        return;

    raw_cal_points_x[_cal_step - 1] = in->ax1;
    raw_cal_points_y[_cal_step - 1] = in->ax2;
    al_debug_print("Raw X value collected: %f\n\rRaw Y value collected: %f\n\r",
                in->ax1, in->ax2);
    _cal_step++;

    if (_cal_step > CALIBRATION_NUM_STEPS) {
        calibration_finish();
    } else {
        al_debug_print("Calibration Step [%d/%d]\n\r", _cal_step,
                    CALIBRATION_NUM_STEPS);
    }
}

void analoglib_cal_undo(void) {
    // Go back one calibration step, only if we are actually calibrating and
    // not at the beginning.
    if (_cal_step > 1) {
        _cal_step--;
    }
    al_debug_print("Calibration Step [%d/%d]\n", _cal_step, CALIBRATION_NUM_STEPS);
}


void analoglib_init(calib_results_t *settings_calib_results, stick_config_t *settings_stick_config, const float dt) {
    al_g_calib_results = settings_calib_results;
    al_g_stick_config = settings_stick_config;
    al_g_dt = dt;
}

void analoglib_process(analog_data_t *in, analog_data_t *out,
                   const bool gate_limiter_enable) {

#if ZTH_LINEARIZATION_EN
    ax_t notch_remap_in_x = linearize(in->ax1, calib_results->fit_coeffs_x);
    ax_t notch_remap_in_y = linearize(in->ax2, calib_results->fit_coeffs_y);
#else
    ax_t notch_remap_in_x = al_g_calib_results->fit_coeffs_x[1] * (in->ax1 - al_g_calib_results->fit_coeffs_x[0]);
    ax_t notch_remap_in_y = al_g_calib_results->fit_coeffs_y[1] * (in->ax2 - al_g_calib_results->fit_coeffs_y[0]);
#endif

    ax_t remapped_x, remapped_y;
    notch_remap(notch_remap_in_x, notch_remap_in_y, &remapped_x, &remapped_y,
                gate_limiter_enable, al_g_calib_results, al_g_stick_config);

    ax_t filtered_x, filtered_y;
    snapback_filter(remapped_x, remapped_y, &filtered_x, &filtered_y, al_g_stick_config);

    out->ax1 = fmin(1.0, fmax(-1.0, filtered_x));
    out->ax2 = fmin(1.0, fmax(-1.0, filtered_y));
}


