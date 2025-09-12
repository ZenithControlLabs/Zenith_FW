#include "intf.h"

DAC_HandleTypeDef *g_intf_hdac = NULL;

static inline void write_dac_xy(uint16_t x, uint16_t y) {
  HAL_DAC_SetValue(g_intf_hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (x << 16) | y);
}

/**
  * @brief  Initialize the output interface subsystem. Perform the I2C probe, set mode accordingly.
  * @param  hdac: The DAC handle to program in analog mode.
  * @retval None
  */
void intf_init(DAC_HandleTypeDef *hdac) {
    g_intf_hdac = hdac;
    HAL_DAC_Start(g_intf_hdac, DAC_CHANNEL_1);
    HAL_DAC_Start(g_intf_hdac, DAC_CHANNEL_2);
    DAC1->DHR12RD = 0;
}

/**
  * @brief  Sets the value to be used as the output, depending on the mode.
  *         In I2C mode, the values are used to provide the mainboard with the raw ADC count.
  *         In analog mode, the DAC is programmed with the output. Only the top 12 bits are used.
  * @param  x: signed 16-bit x coord (center 0)
  * @param  y: signed 16-bit y coord (center 0)
  * @retval None
  */
void intf_out(int16_t x, int16_t y) {}

/**
  * @brief  Are we in analog (DAC) mode, or in I2C?
  * @retval True if in analog mode.
  */
bool intf_is_mode_analog() { return true; }