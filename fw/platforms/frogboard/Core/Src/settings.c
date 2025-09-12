#include "settings.h"

settings_t g_settings;

/**
  * @brief  Load stored settings from flash.
  * @retval True if we are loading for the first time i.e. we need factory initialization.
  */
bool settings_load() { return false; }

/**
  * @brief  Save current settings struct to flash.
  * @retval None
  */
void settings_flush() {}