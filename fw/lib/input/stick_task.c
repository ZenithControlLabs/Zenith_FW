#include "zenith/input/stick_task.h"

volatile _Atomic cal_msg_t _cal_msg;

static inline void calib_task(analog_data_t *in, analog_data_t *out) {
    // We will always read the controller. Forget ZTH_STICK_INTERVAL,
    // we are not running any processing that depends on the rate of update,
    // we just need it always updated for raw stick reporting.
    cb_zenith_read_analog(in);
    // The WebUSB task sends us a message through this variable
    // indicating when we will advance or undo a calibration step
    // We read and then clear the message flag to indicate that it is read.
    switch (atomic_load(&_cal_msg)) {
    case CALIB_ADVANCE: {
        atomic_store(&_cal_msg, CALIB_NONE);
        // The userspace can configure to either use the value just read for
        // calibration, or to do something special to get the calibration value.
        // Most useful for oversampling.
#if ZTH_SEPARATE_CAL_READ
        cb_zenith_read_analog_cal(in);
#endif
        analoglib_cal_advance(in);
        break;
    }
    case CALIB_UNDO: {
        atomic_store(&_cal_msg, CALIB_NONE);
        analoglib_cal_undo();
        break;
    }
    default:
        break;
    }
}

void stick_task(uint32_t timestamp, analog_data_t *in, analog_data_t *out) {
    // If we are calibrating.. follow special procedure
    if (_cal_step > 0) {
        calib_task(in, out);
        return;
    }

    // We only run the main loop of interest every ZTH_STICK_INTERVAL
    if (!interval_run(timestamp, ZTH_STICK_INTERVAL))
        return;

    cb_zenith_read_analog(in);
    process_stick(in, out, _settings[_profile].gate_limiter_enable, &(_settings[_profile].calib_results),
                  &(_settings[_profile].stick_config));
}