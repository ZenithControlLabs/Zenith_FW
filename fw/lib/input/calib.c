#include "zenith/includes.h"

volatile int _cal_step = 0;
volatile _Atomic cal_msg_t _cal_msg;
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

void calibration_advance(analog_data_t *in) {
    // failsafe - this function should not be called if incrementing the step
    // would lead to an invalid state
    if (_cal_step < 1 || _cal_step > CALIBRATION_NUM_STEPS)
        return;

    raw_cal_points_x[_cal_step - 1] = in->ax1;
    raw_cal_points_y[_cal_step - 1] = in->ax2;
    debug_print("Raw X value collected: %f\nRaw Y value collected: %f\n",
                in->ax1, in->ax2);
    _cal_step++;

    if (_cal_step > CALIBRATION_NUM_STEPS) {
        calibration_finish();
    } else {
        debug_print("Calibration Step [%d/%d]\n", _cal_step,
                    CALIBRATION_NUM_STEPS);
    }
}

void calibration_undo(void) {
    // Go back one calibration step, only if we are actually calibrating and
    // not at the beginning.
    if (_cal_step > 1) {
        _cal_step--;
    }
    debug_print("Calibration Step [%d/%d]\n", _cal_step, CALIBRATION_NUM_STEPS);
}

void calibration_finish(void) {
    // We're done calibrating. Do the math to save our calibration parameters
    ax_t cleaned_points_x[NUM_NOTCHES + 1];
    ax_t cleaned_points_y[NUM_NOTCHES + 1];
    for (int i = 0; i < CALIBRATION_NUM_STEPS; i++) {
        debug_print("Raw Cal point:  %d; (x,y) = (%f, %f)\n", i,
                    raw_cal_points_x[i], raw_cal_points_y[i]);
    }
    fold_center_points(raw_cal_points_x, raw_cal_points_y, cleaned_points_x,
                       cleaned_points_y);
    for (int i = 0; i <= NUM_NOTCHES; i++) {
        debug_print("Clean Cal point:  %d; (x,y) = (%f, %f)\n", i,
                    cleaned_points_x[i], cleaned_points_y[i]);
    }

#if ZTH_LINEARIZATION_EN
    // One less because center is becoming 0 implcitly
    ax_t linearized_points_x[NUM_NOTCHES];
    ax_t linearized_points_y[NUM_NOTCHES];

    linearize_cal(cleaned_points_x, cleaned_points_y, linearized_points_x,
                  linearized_points_y, &(_settings[_profile].calib_results));

    // Copy the linearized points we have just found to phobri's internal data
    // sturcture.
    for (int i = 0; i < NUM_NOTCHES; i++) {
        _settings[_profile].calib_results.notch_points_x_in[i] = linearized_points_x[i];
        _settings[_profile].calib_results.notch_points_y_in[i] = linearized_points_y[i];
        debug_print("Linearized point:  %d; (x,y) = (%f, %f)\n", i,
                    linearized_points_x[i], linearized_points_y[i]);
    }
#else
    // Linearization normally moves the center point to an implicit 0,0.
    // To carry forth the assumption for notch calibration, we will do the same.
    for (int i = 0; i < NUM_NOTCHES; i++) {
        // TODO: notch calibration assumes that the notches have an increasing
        // angle; doing this will mess it up if the sensor is negative to go up
        // in Y; need to figure out the appropriate place in the code to
        // compensate for this
        _settings[_profile].calib_results.notch_points_x_in[i] =
            (cleaned_points_x[i + 1] - cleaned_points_x[0]);
        _settings[_profile].calib_results.notch_points_y_in[i] =
            (cleaned_points_y[i + 1] - cleaned_points_y[0]);
        debug_print("Notch Point in point:  %d; (x,y) = (%f, %f)\n", i,
                    _settings[_profile].calib_results.notch_points_x_in[i],
                    _settings[_profile].calib_results.notch_points_y_in[i]);
    }
#endif // ZTH_LINEARIZATON_EN

    notch_calibrate(_settings[_profile].calib_results.notch_points_x_in,
                    _settings[_profile].calib_results.notch_points_y_in,
                    _settings[_profile].stick_config.notch_points_x,
                    _settings[_profile].stick_config.notch_points_y,
                    &(_settings[_profile].calib_results));
    debug_print("Calibrated!\n");
    _settings[_profile].calib_results.calibrated = true;
    /*debug_print("X coeffs: %f %f %f %f, Y coeffs: %f %f %f %f\n",
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