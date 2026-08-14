#include "zenith/includes.h"

zenith_settings_t _settings[PROFILE_COUNT] = {0};
volatile _Atomic int _profile = 0;
volatile _Atomic bool _please_commit = false;

/* Settings layout used before usb_stick_scale was added. */
typedef struct {
    uint32_t settings_ver;
    comms_mode_t comms_mode;
    calib_results_t calib_results;
    stick_config_t stick_config;
    btn_remap_profile_t btn_remap_profile_n64;
    btn_remap_profile_t btn_remap_profile_gamecube;
    btn_remap_profile_t btn_remap_profile_xinput;
    btn_remap_profile_t btn_remap_profile_switch;
    bool gate_limiter_enable;
    uint8_t user_settings[USER_SETTINGS_SIZE];
} zenith_settings_v5003_t;

void settings_reset_to_factory() {
    // clang-format off
    const zenith_settings_t set = {
        .settings_ver = SETTINGS_VER,
        .comms_mode = COMMS_MODE_N64,
        .calib_results = {
            .calibrated = false,
            .affine_coeffs = {0},
            .boundary_angles = {0},
            .fit_coeffs_x = {0},
            .fit_coeffs_y = {0},
            .notch_points_x_in = {0},
            .notch_points_y_in = {0}
        },
        .stick_config = {
            .notch_points_x = {
                INT_N_TO_AX(85, 8), INT_N_TO_AX(70, 8), INT_N_TO_AX(0, 8),
                INT_N_TO_AX(-70, 8), INT_N_TO_AX(-85, 8), INT_N_TO_AX(-70, 8),
                INT_N_TO_AX(0, 8), INT_N_TO_AX(70, 8)
            },
            .notch_points_y = {
                INT_N_TO_AX(0, 8), INT_N_TO_AX(70, 8), INT_N_TO_AX(85, 8),
                INT_N_TO_AX(70, 8), INT_N_TO_AX(0, 8), INT_N_TO_AX(-70, 8),
                INT_N_TO_AX(-85, 8), INT_N_TO_AX(-70, 8)
            },
            .angle_deadzones = {0},
            .mag_threshold = .8, // 80% into the notch by default
            .cutoff_hz = 450
        },
        .btn_remap_profile_n64 = {
            .p = ZTH_N64_REMAP_DEFAULT
        },
        .btn_remap_profile_gamecube = {
            .p = ZTH_GAMECUBE_REMAP_DEFAULT
        },
        .btn_remap_profile_xinput = {
            .p = ZTH_XINPUT_REMAP_DEFAULT
        },
        .btn_remap_profile_switch = {
            .p = ZTH_SWITCH_REMAP_DEFAULT
        },
        .gate_limiter_enable = false,
        .usb_stick_scale = 1.0f,
        .user_settings = {0}
    };
    // clang-format on
    for (int i = 0; i < PROFILE_COUNT; ++i) {
        cb_zenith_user_settings_reset(_settings[i].user_settings);
        memcpy(&_settings[i], &set, sizeof(*_settings));
    }
}

void __not_in_flash_func(settings_core1_handle_commit)() {

    if (!atomic_load(&_please_commit))
        return;

    // Any other command, we will pause the other core and disable interrupts.

    multicore_lockout_start_blocking();

    // Store interrupts status and disable
    uint32_t ints = save_and_disable_interrupts();

    // Check that we are less than our flash sector size
    static_assert(sizeof(_settings) <= FLASH_SECTOR_SIZE);

    // Calculate storage bank address via index
    uint32_t memoryAddress = FLASH_OFFSET + (FLASH_SECTOR_SIZE);

    // Create blank page data
    uint8_t page[FLASH_SECTOR_SIZE] = {0x00};
    // Copy settings into our page buffer
    memcpy(page, _settings, sizeof(*_settings) * PROFILE_COUNT);

    // Erase the settings flash sector
    flash_range_erase(memoryAddress, FLASH_SECTOR_SIZE);

    // Program the flash sector with our page
    flash_range_program(memoryAddress, page, FLASH_SECTOR_SIZE);

    // Restore interrups
    restore_interrupts(ints);
    multicore_lockout_end_blocking();

    atomic_store(&_please_commit, false);
}

void settings_load() {
    static_assert(sizeof(zenith_settings_t) <= FLASH_SECTOR_SIZE);
    const uint8_t *target_read =
        (const uint8_t *)(XIP_BASE + FLASH_OFFSET + (FLASH_SECTOR_SIZE));
    memcpy(_settings, target_read, sizeof(*_settings) * PROFILE_COUNT);

    if (_settings[0].settings_ver == 0x5003) {
        zenith_settings_v5003_t previous[PROFILE_COUNT];
        memcpy(previous, target_read, sizeof(previous));
        settings_reset_to_factory();
        for (int i = 0; i < PROFILE_COUNT; ++i) {
            _settings[i].comms_mode = previous[i].comms_mode;
            _settings[i].calib_results = previous[i].calib_results;
            _settings[i].stick_config = previous[i].stick_config;
            _settings[i].btn_remap_profile_n64 =
                previous[i].btn_remap_profile_n64;
            _settings[i].btn_remap_profile_gamecube =
                previous[i].btn_remap_profile_gamecube;
            _settings[i].btn_remap_profile_xinput =
                previous[i].btn_remap_profile_xinput;
            _settings[i].btn_remap_profile_switch =
                previous[i].btn_remap_profile_switch;
            _settings[i].gate_limiter_enable =
                previous[i].gate_limiter_enable;
            memcpy(_settings[i].user_settings, previous[i].user_settings,
                   USER_SETTINGS_SIZE);
        }
        debug_print("Migrated settings to add USB stick scaling.\n");
        settings_inform_commit();
        return;
    }

    // Check for the integrity of our magic number.
    // If it doesn't match, settings structure has changed
    // and we should clear to factory defaults.
    if (_settings[0].settings_ver != SETTINGS_VER) {
        debug_print("Settings version does not match. Resetting... \n");
        settings_reset_to_factory();
        settings_inform_commit();
    }
}

void settings_inform_commit() { atomic_store(&_please_commit, true); }

inline uint8_t *zenith_get_user_settings_ptr(void) {
    return _settings[_profile].user_settings;
}
