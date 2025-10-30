#include "settings.h"
#include "main.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_flash_ex.h"

#include <string.h>

settings_t g_settings;
// We enforce the (possibly overly conservative) policy that the controller always boots into calibration mode whenever you have flashed it,
// otherwise you may not be able to use the menu options (we don't know what kind of signal the uncalibrated controller has).
// So if something bad happens, we want to leave re-flashing as a reliable escape hatch. Initializing g_magic to != MAGIC upon flashing accomplishes this.
volatile const uint64_t g_magic __attribute__((section(".text"))) = 0xFFFFFFFFFFFFFFFF;

void settings_reset_to_factory() {
    const settings_t set = {
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
            .cutoff_hz = 450.f,
        },
    };
    g_settings = set;
}

/**
  * @brief  Load stored settings from flash.
  * @retval True if we are loading for the first time i.e. we need factory initialization.
  */
bool settings_load() { 
  memcpy((void*)&g_settings, (void*)SETTINGS_FLASH_ADDR, sizeof(settings_t));
  return g_magic != MAGIC;
}

/**
  * @brief  Save current settings struct to flash.
  * @retval None
  */
void settings_flush() {
  const uint64_t* flash_settings = (const uint64_t*)(SETTINGS_FLASH_ADDR);
  const uint64_t* g_settings_u64 = (const uint64_t*)&g_settings;
  HAL_FLASH_Unlock();
  FLASH_EraseInitTypeDef EraseInit = {
    .TypeErase = FLASH_TYPEERASE_PAGES,
    .Banks = FLASH_BANK_1,
    .NbPages = 1,
    .Page = 64
  };
  uint32_t PageError = 0;
  if (HAL_FLASHEx_Erase(&EraseInit, &PageError) != HAL_OK || PageError != 0xFFFFFFFF) {
    Error_Handler();
  }
  for (int i = 0; i < SETTINGS_SIZE_DOUBLEWORDS; i++) {
    int dw_extra_bytes = ((i+1) * 8 - sizeof(settings_t));
    uint64_t dw_mask = dw_extra_bytes > 0 ? (((int64_t)(0xFF))<<(64-8)) >> 8*(8-dw_extra_bytes) : -1;
    if ((g_settings_u64[i] ^ flash_settings[i]) & dw_mask) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&flash_settings[i], g_settings_u64[i]);
    }
  }
  HAL_FLASH_Lock();
}