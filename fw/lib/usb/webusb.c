#include "zenith/includes.h"
#include "notch_remap.h"

const char* git_version = GIT_VERSION;

uint8_t _webusb_out_buffer[64] = {0x00};
int _webusb_output_cnt = 0;

void webusb_save_confirm() {
    debug_print("Sending Save receipt...\n");
    memset(_webusb_out_buffer, 0, 64);
    _webusb_out_buffer[0] = 0xF1;
    tud_vendor_n_write(0, _webusb_out_buffer, 64);
    tud_vendor_n_flush(0);
}

bool webusb_ready_blocking(int timeout) {
    if (timeout > 0) {
        int internal = timeout;
        while (!tud_vendor_n_write_available(0) && (internal > 0)) {
            sleep_us(100);
            tud_task();
            internal--;
        }

        if (!internal) {
            debug_print("Disabling webusb output..\n");
            return false;
        }

        return true;
    } else {
        return false;
    }

    return true;
}

void webusb_command_processor(uint8_t *data, const uint32_t data_size) {
    if (data == NULL || data_size == 0)
        return;
    memset(_webusb_out_buffer, 0, sizeof(_webusb_out_buffer));

    if ((data[0] & WEBUSB_CMD_USER_MASK) == WEBUSB_CMD_USER_VAL) {
        bool perform_write =
            cb_zenith_user_webusb_cmd(data, _webusb_out_buffer);
        if (perform_write && webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
        return;
    }

    bool webusb_output_en = false;
    switch (data[0]) {
    case WEBUSB_CMD_FW_GET: {
        _webusb_out_buffer[0] = WEBUSB_CMD_FW_GET;
        _webusb_out_buffer[2] = ZTH_FW_MAJOR;
        _webusb_out_buffer[1] = (ZTH_FW_MINOR << 4) | (ZTH_FW_PATCH & 0xF);
        size_t verslen = strlen(git_version);
        if (verslen < (64 - 3)) {
            memcpy(_webusb_out_buffer+3, git_version, verslen+1);
        }
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_CALIBRATION_STATUS_GET: {
        debug_print("WebUSB: Got calibration STATUS GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_CALIBRATION_STATUS_GET;
        _webusb_out_buffer[1] = _settings[_profile].calib_results.calibrated;
        _webusb_out_buffer[2] = _cal_step;
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_CALIBRATION_START: {
        debug_print("WebUSB: Got calibration START command.\n");
        if (_cal_step < 1) {
            _cal_step = 1;
            debug_print("Starting calibration!\nCalibration Step [%d/%d]\n",
                        _cal_step, CALIBRATION_NUM_STEPS);
        }
        _webusb_out_buffer[0] = WEBUSB_CMD_CALIBRATION_START;
    } break;

    case WEBUSB_CMD_CALIBRATION_ADVANCE: {
        atomic_store(&_cal_msg, CALIB_ADVANCE);
        debug_print("WebUSB: Got calibration ADVANCE command. (msg=%d)\n",
                    _cal_msg);
        _webusb_out_buffer[0] = WEBUSB_CMD_CALIBRATION_ADVANCE;
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_CALIBRATION_UNDO: {
        debug_print("WebUSB: Got calibration UNDO command.\n");
        atomic_store(&_cal_msg, CALIB_UNDO);
        _webusb_out_buffer[0] = WEBUSB_CMD_CALIBRATION_UNDO;
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_NOTCH_SET: {
        debug_print("WebUSB: Got notch point SET command.\n");
        uint8_t notch = data[1];
        if (notch > NUM_NOTCHES) {
            debug_print("Notch out of range?\n");
            break;
        }
        _settings[_profile].stick_config.notch_points_x[notch] =
            INT_N_TO_AX((int8_t)data[2], 8);
        _settings[_profile].stick_config.notch_points_y[notch] =
            INT_N_TO_AX((int8_t)data[3], 8);
        memcpy(_settings[_profile].stick_config.angle_deadzones + notch, data + 4,
               sizeof(float));
        // recompute notch calibration
        notch_calibrate(_settings[_profile].calib_results.notch_points_x_in,
                        _settings[_profile].calib_results.notch_points_y_in,
                        _settings[_profile].stick_config.notch_points_x,
                        _settings[_profile].stick_config.notch_points_y,
                        &(_settings[_profile].calib_results));
    } break;
    case WEBUSB_CMD_NOTCHES_GET: {
        debug_print("WebUSB: Got notch points GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_NOTCHES_GET;
        for (int i = 0; i < NUM_NOTCHES; i++) {
            _webusb_out_buffer[i * 6 + 1] =
                AX_TO_INT8(_settings[_profile].stick_config.notch_points_x[i]);
            _webusb_out_buffer[i * 6 + 2] =
                AX_TO_INT8(_settings[_profile].stick_config.notch_points_y[i]);

            memcpy(_webusb_out_buffer + (i * 6 + 3),
                   _settings[_profile].stick_config.angle_deadzones + i, sizeof(float));
        }
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_REMAP_SET: {
        debug_print("WebUSB: Got Remap SET command.\n");
        remap_mode_t c = (remap_mode_t)data[1];
        uint8_t btn = data[2];
        uint8_t bind = data[3];
        if (c >= REMAP_MODE_COUNT || btn >= 32 ||
            (bind != 0xFF && bind > 32))
            break;
        switch (c) {
        case REMAP_MODE_N64: {
            _settings[_profile].btn_remap_profile_n64.p[btn] = bind;
            break;
        }
        case REMAP_MODE_GAMECUBE: {
            _settings[_profile].btn_remap_profile_gamecube.p[btn] = bind;
            break;
        }
        case REMAP_MODE_XINPUT:
            _settings[_profile].btn_remap_profile_xinput.p[btn] = bind;
            break;
        case REMAP_MODE_SWITCH:
            _settings[_profile].btn_remap_profile_switch.p[btn] = bind;
            break;
        default:
            break;
        }
        // remap_listen_enable(data[1], data[2]);
    } break;

    case WEBUSB_CMD_REMAP_GET: {
        debug_print("WebUSB: Got Remap GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_REMAP_GET;
        _webusb_out_buffer[1] = data[1];
        remap_mode_t c = (remap_mode_t)data[1];
        switch (c) {
        case REMAP_MODE_N64: {
            memcpy(_webusb_out_buffer + 2, _settings[_profile].btn_remap_profile_n64.p,
                   32);
            break;
        }
        case REMAP_MODE_GAMECUBE: {
            memcpy(_webusb_out_buffer + 2,
                   _settings[_profile].btn_remap_profile_gamecube.p, 32);
            break;
        }
        case REMAP_MODE_XINPUT:
            memcpy(_webusb_out_buffer + 2,
                   _settings[_profile].btn_remap_profile_xinput.p, 32);
            break;
        case REMAP_MODE_SWITCH:
            memcpy(_webusb_out_buffer + 2,
                   _settings[_profile].btn_remap_profile_switch.p, 32);
            break;
        default:
            memset(_webusb_out_buffer + 2, 0xFF, 32);
            break;
        }
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_COMMS_MODE_SET:
        if (data[1] < COMMS_MODE_COUNT) {
            _settings[_profile].comms_mode = (comms_mode_t)data[1];
            _webusb_out_buffer[0] = WEBUSB_CMD_COMMS_MODE_SET;
            _webusb_out_buffer[1] = data[1];
            webusb_output_en = true;
        }
        break;

    case WEBUSB_CMD_COMMS_MODE_GET:
        _webusb_out_buffer[0] = WEBUSB_CMD_COMMS_MODE_GET;
        _webusb_out_buffer[1] = _settings[_profile].comms_mode;
        webusb_output_en = true;
        break;

    case WEBUSB_CMD_RAW_N64_GET: {
        btn_data_t mapped = {0};
        btn_remap_for_mode(REMAP_MODE_N64, &_buttons, &mapped);
        n64_input_t n64 = {0};
        n64.button_a = mapped.s.b1;
        n64.button_b = mapped.s.b2;
        n64.cpad_up = mapped.s.b3;
        n64.cpad_down = mapped.s.b4;
        n64.cpad_left = mapped.s.b5;
        n64.cpad_right = mapped.s.b6;
        n64.button_start = mapped.s.b7;
        n64.button_l = mapped.s.b8;
        n64.button_r = mapped.s.b9;
        n64.button_z = mapped.s.b10;
        n64.dpad_down = mapped.s.b11;
        n64.dpad_left = mapped.s.b12;
        n64.dpad_right = mapped.s.b13;
        n64.dpad_up = mapped.s.b14;
        n64.stick_x = (int8_t)(_analog_data_processed.ax1 * 127.0f);
        n64.stick_y = (int8_t)(_analog_data_processed.ax2 * 127.0f);
        _webusb_out_buffer[0] = WEBUSB_CMD_RAW_N64_GET;
        memcpy(&_webusb_out_buffer[1], &n64, sizeof(n64));
        _webusb_out_buffer[5] = _settings[_profile].calib_results.calibrated;
        memcpy(&_webusb_out_buffer[8], &_analog_data.ax1, sizeof(float));
        memcpy(&_webusb_out_buffer[12], &_analog_data.ax2, sizeof(float));
        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_MAG_THRESH_SET: {
        debug_print("WebUSB: Got Magnitude Threshold SET command.\n");
        memcpy(&_settings[_profile].stick_config.mag_threshold, data + 4, sizeof(float));
    } break;

    case WEBUSB_CMD_MAG_THRESH_GET: {
        debug_print("WebUSB: Got Magnitude Threshold GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_MAG_THRESH_GET;
        memcpy(_webusb_out_buffer + 4, &_settings[_profile].stick_config.mag_threshold,
               sizeof(float));

        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_GATE_LIMITER_SET: {
        debug_print("WebUSB: Got Gate Limiter SET command.\n");
        memcpy(&_settings[_profile].gate_limiter_enable, data + 1, sizeof(bool));
    } break;

    case WEBUSB_CMD_GATE_LIMITER_GET: {
        debug_print("WebUSB: Got Gate Limiter GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_GATE_LIMITER_GET;
        memcpy(_webusb_out_buffer + 1, &_settings[_profile].gate_limiter_enable,
               sizeof(bool));

        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_LPF_CUTOFF_SET: {
        debug_print("WebUSB: Got LPF Cutoff SET command.\n");
        memcpy(&_settings[_profile].stick_config.cutoff_hz, data+4, sizeof(float));
    } break;

    case WEBUSB_CMD_LPF_CUTOFF_GET: {
        debug_print("WebUSB: Got LPF Cutoff GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_LPF_CUTOFF_GET;
        memcpy(_webusb_out_buffer+4, &_settings[_profile].stick_config.cutoff_hz, sizeof(float));

        webusb_output_en = true;
    } break;

    case WEBUSB_CMD_UPDATE_FW: {
        reset_usb_boot(0, 0);
    } break;
    case WEBUSB_CMD_COMMIT_SETTINGS: {
        debug_print("WebUSB: Got commit settings command.\n");
        settings_inform_commit();
    } break;
    case WEBUSB_CMD_RESET_SETTINGS: {
        debug_print("WebUSB: Got reset settings to factory command.\n");
        settings_reset_to_factory();
    } break;
    default: {
        break;
    }
    }
    if (webusb_output_en && webusb_ready_blocking(5000)) {
        tud_vendor_n_write(0, _webusb_out_buffer, 64);
        tud_vendor_n_flush(0);
    }
}
