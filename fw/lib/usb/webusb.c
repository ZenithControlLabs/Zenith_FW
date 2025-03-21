#include "zenith/includes.h"

const char* git_version = GIT_VERSION;

uint8_t _webusb_out_buffer[64] = {0x00};
bool _webusb_output_enabled = false;
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
            _webusb_output_enabled = false;
            return false;
        }

        return true;
    } else {
        return false;
    }

    return true;
}

void webusb_command_processor(uint8_t *data) {
    _webusb_output_enabled = true;

    if ((data[0] & WEBUSB_CMD_USER_MASK) == WEBUSB_CMD_USER_VAL) {
        bool perform_write =
            cb_zenith_user_webusb_cmd(data, _webusb_out_buffer);
        if (perform_write && webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
        return;
    }

    switch (data[0]) {
    case WEBUSB_CMD_FW_GET: {
        _webusb_out_buffer[0] = WEBUSB_CMD_FW_GET;
        _webusb_out_buffer[2] = ZTH_FW_MAJOR;
        _webusb_out_buffer[1] = (ZTH_FW_MINOR << 4) | (ZTH_FW_PATCH & 0xF);
        size_t verslen = strlen(git_version);
        if (verslen < (64 - 3)) {
            memcpy(_webusb_out_buffer+3, git_version, verslen+1);
        }
        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
        _webusb_output_enabled = true;
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
        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
    } break;

    case WEBUSB_CMD_CALIBRATION_UNDO: {
        debug_print("WebUSB: Got calibration UNDO command.\n");
        atomic_store(&_cal_msg, CALIB_UNDO);
        _webusb_out_buffer[0] = WEBUSB_CMD_CALIBRATION_UNDO;
        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
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
        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
    } break;

    case WEBUSB_CMD_REMAP_SET: {
        debug_print("WebUSB: Got Remap SET command.\n");
        comms_mode_t c = (comms_mode_t)data[1];
        uint8_t btn = data[2];
        uint8_t bind = data[3];
        switch (c) {
        case COMMS_MODE_N64: {
            _settings[_profile].btn_remap_profile_n64.p[btn] = bind;
        }
        case COMMS_MODE_GAMECUBE: {
            _settings[_profile].btn_remap_profile_gamecube.p[btn] = bind;
            break;
        }
        }
        // remap_listen_enable(data[1], data[2]);
    } break;

    case WEBUSB_CMD_REMAP_GET: {
        debug_print("WebUSB: Got Remap GET command.\n");
        _webusb_out_buffer[0] = WEBUSB_CMD_REMAP_GET;
        _webusb_out_buffer[1] = data[1];
        comms_mode_t c = (comms_mode_t)data[1];
        switch (c) {
        case COMMS_MODE_N64: {
            memcpy(_webusb_out_buffer + 2, _settings[_profile].btn_remap_profile_n64.p,
                   32);
            break;
        }
        case COMMS_MODE_GAMECUBE: {
            memcpy(_webusb_out_buffer + 2,
                   _settings[_profile].btn_remap_profile_gamecube.p, 32);
            break;
        }
        }
        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
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

        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
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

        if (webusb_ready_blocking(5000)) {
            tud_vendor_n_write(0, _webusb_out_buffer, 64);
            tud_vendor_n_flush(0);
        }
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
}