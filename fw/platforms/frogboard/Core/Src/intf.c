#include "intf.h"

DAC_HandleTypeDef *g_intf_hdac = NULL;

static inline void write_dac_xy(uint16_t x, uint16_t y) {
  
}

/**
  * @brief  Initialize the output interface subsystem. Perform the I2C probe, set mode accordingly.
  * @param  hdac: The DAC handle to program in analog mode.
  * @retval None
  */
void intf_init(DAC_HandleTypeDef *hdac) {
    g_intf_hdac = hdac;
}

/**
  * @brief  Sets the value to be used as the output, depending on the mode.
  *         In I2C mode, the values are used to provide the mainboard with the raw ADC count.
  *         In analog mode, the DAC is programmed with the output. Only the top 12 bits are used.
  * @param  x: signed 16-bit x coord (center 0)
  * @param  y: signed 16-bit y coord (center 0)
  * @retval None
  */
void intf_out(int16_t x, int16_t y) {
  DAC1->DHR12RD = (((x >> 4) + 2048) << 16) | ((y >> 4) + 2048);
}

/**
  * @brief  Are we in analog (DAC) mode, or in I2C?
  * @retval True if in analog mode.
  */
bool intf_is_mode_analog() { return true; }