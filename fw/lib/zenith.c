#include "zenith/includes.h"

btn_data_t _buttons = {0};
btn_data_t _buttons_processed = {0};

analog_data_t _analog_data = {0};
analog_data_t _analog_data_processed = {0};

volatile _Atomic uint32_t _timestamp = 0;

void zenith_loop_core0(void) {
    for (;;) {
        atomic_store(&_timestamp, time_us_32());

#ifdef ZENITH_SWITCHER
        atomic_store(&_profile, cb_zenith_read_controller_switch());
#endif

        cb_zenith_read_buttons(&_buttons);
        btn_remap_task(&_buttons, &_buttons_processed);

        usb_task(atomic_load(&_timestamp), &_buttons,
                 &_analog_data_processed, &_analog_data);

        comms_task(atomic_load(&_timestamp), &_buttons_processed,
                   &_analog_data_processed);

        cb_zenith_core0_inject();
    }
}

void zenith_loop_core1(void) {
    cb_zenith_core1_init();
#ifdef ZENITH_SWITCHER
    int last_profile = _profile;
#endif

    for (;;) {
        settings_core1_handle_commit();

#ifdef ZENITH_SWITCHER
        if (_profile != last_profile) {
            last_profile = _profile;
            multicore_lockout_start_blocking();
            analoglib_init(&_settings[_profile].calib_results, &_settings[_profile].stick_config);
            cb_zenith_switch_input(_profile);
            multicore_lockout_end_blocking();
        }
#endif

        stick_task(atomic_load(&_timestamp), &_analog_data,
                   &_analog_data_processed);

        cb_zenith_core1_inject();
    }
}

void zenith_start() {
    settings_load();

    cb_zenith_init_hardware();

    /* Boot shortcuts choose and persist a USB personality before USB starts.
       A takes priority if both buttons are held, preserving the recovery path
       to the Switch Pro + WebUSB personality. */
    btn_data_t boot_buttons = {0};
    sleep_ms(5);
    cb_zenith_read_buttons(&boot_buttons);
    comms_mode_t boot_mode = _settings[_profile].comms_mode;
    if (boot_buttons.s.b1) {
        boot_mode = COMMS_MODE_N64;
    } else if (boot_buttons.s.b2) {
        boot_mode = COMMS_MODE_XINPUT;
    }
    if (_settings[_profile].comms_mode != boot_mode) {
        _settings[_profile].comms_mode = boot_mode;
        settings_inform_commit();
    }

    usb_init();

    comms_init();

    analoglib_init(&_settings[_profile].calib_results, &_settings[_profile].stick_config, 1/((float)ZTH_STICK_INTERVAL));

    multicore_lockout_victim_init();

    multicore_launch_core1(zenith_loop_core1);

    zenith_loop_core0();
}
