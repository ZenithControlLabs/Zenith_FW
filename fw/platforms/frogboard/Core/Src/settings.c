#include "settings.h"
#include "stm32l4xx_hal_flash.h"

#include <string.h>

settings_t g_settings;
const uint64_t g_magic __attribute__((section(".text"))) = 0;

/**
  * @brief  Load stored settings from flash.
  * @retval True if we are loading for the first time i.e. we need factory initialization.
  */
bool settings_load() { 
  memcpy((void*)&g_settings, (const void*)SETTINGS_FLASH_ADDR, sizeof(settings_t));
  return g_magic != MAGIC;
}

/**
  * @brief  Save current settings struct to flash.
  * @retval None
  */
void settings_flush() {
  const uint64_t* flash_settings = (const uint64_t*)SETTINGS_FLASH_ADDR;
  const uint64_t* g_settings_u64 = (const uint64_t*)&g_settings;
  HAL_FLASH_Unlock();
  for (int i = 0; i < SETTINGS_SIZE_DOUBLEWORDS; i++) {
    int dw_extra_bytes = ((i+1) * 8 - sizeof(settings_t));
    uint64_t dw_mask = dw_extra_bytes > 0 ? (((int64_t)(0xFF))<<(64-8)) >> 8*(8-dw_extra_bytes) : -1;
    if ((g_settings_u64[i] ^ flash_settings[i]) & dw_mask) {
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&flash_settings[i], g_settings_u64[i]);
    }
  }
  HAL_FLASH_Lock();
}