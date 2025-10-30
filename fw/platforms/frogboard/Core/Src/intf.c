#include "intf.h"
#include "settings.h"
#include "stm32l4xx_hal_adc.h"

DAC_HandleTypeDef *g_intf_hdac = NULL;
ADC_HandleTypeDef *g_intf_hadc = NULL;

extern void Error_Handler(void);

void intf_adc_in(uint16_t *adc_res) {
  HAL_StatusTypeDef status = HAL_ADC_Start(g_intf_hadc);
  if (status != HAL_OK) {
    Error_Handler();
  }
  volatile int cnt = 0;
  int done = 0;
  while (!done) {
    if (cnt > CHAN_END) {
      Error_Handler();
    }
    while ((g_intf_hadc->Instance->ISR & ADC_ISR_EOC) == 0) { 
      asm volatile ("");
    }
    done = g_intf_hadc->Instance->ISR & ADC_ISR_EOS;
    adc_res[cnt++] = g_intf_hadc->Instance->DR;
  }
}

/**
  * @brief  Initialize the input subsystem with the appopriate ADC handle, as well as the
            output interface subsystem. Perform the I2C probe, set mode accordingly.
  * @param  hadc: The ADC handle to read from..
  * @param  hdac: The DAC handle to program in analog mode.
  * @retval None
  */
void intf_init(ADC_HandleTypeDef *hadc, DAC_HandleTypeDef *hdac) {
    g_intf_hadc = hadc;
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
  // y flipped cuz it is  ¯\_(ツ)_/¯ 
  // todo use g_intf_hdac instead (changed it to this directly while debugging)
  DAC1->DHR12RD = (((x >> 4) + 2048) << 16) | (((-y) >> 4) + 2048);
}

/**
  * @brief  Sets the value to be used as the output, depending on the mode.
  *         In I2C mode, the values are used to provide the mainboard with the raw ADC count.
  *         In analog mode, the DAC is programmed with the output. Only the top 12 bits are used.
  *         This function takes in float values and applies the stored output calibration first.
  * @param  x: x coordinate, -1.0 to 1.0
  * @param  y: y coordinate, -1.0 to 1.0
  * @retval None
  */
void intf_out_cal(ax_t x, ax_t y) {
  // STUB 
  intf_out(0,0);
}

/**
  * @brief  Are we in analog (DAC) mode, or in I2C?
  * @retval True if in analog mode.
  */
bool intf_is_mode_analog() { return true; }