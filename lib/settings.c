#include "zenith/includes.h"

zenith_settings_t _settings = {0};
volatile _Atomic bool _please_commit = false;

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
            .mag_threshold = .8 // 80% into the notch by default
        },
        .btn_remap_profile_n64 = {
            .p = ZTH_N64_REMAP_DEFAULT
        },
        .btn_remap_profile_gamecube = {
            .p = ZTH_GAMECUBE_REMAP_DEFAULT
        },
        .user_settings = {0}
    };
    // clang-format on
    cb_zenith_user_settings_reset(_settings.user_settings);
    memcpy(&_settings, &set, sizeof(_settings));
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
    memcpy(page, &_settings, sizeof(_settings));

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
    memcpy(&_settings, target_read, sizeof(_settings));

    // Check for the integrity of our magic number.
    // If it doesn't match, settings structure has changed
    // and we should clear to factory defaults.
    if (_settings.settings_ver != SETTINGS_VER) {
        debug_print("Settings version does not match. Resetting... \n");
        settings_reset_to_factory();
        settings_inform_commit();
    }
}

void settings_inform_commit() { atomic_store(&_please_commit, true); }

inline uint8_t *zenith_get_user_settings_ptr(void) {
    return _settings.user_settings;
}