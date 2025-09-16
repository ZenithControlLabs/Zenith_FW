#include "intf.h"
#include "settings.h"

DAC_HandleTypeDef *g_intf_hdac = NULL;

extern void Error_Handler(void);

void adc_read(ADC_HandleTypeDef *hadc, uint16_t *adc_res, bool apply_pol) {
  HAL_StatusTypeDef status = HAL_ADC_Start(hadc);
  if (status != HAL_OK) {
    Error_Handler();
  }
  volatile int cnt = 0;
  int done = 0;
  while (!done) {
    if (cnt > CHAN_END) {
      Error_Handler();
    }
    while ((hadc->Instance->ISR & ADC_ISR_EOC) == 0) { 
      asm volatile ("");
    }
    done = hadc->Instance->ISR & ADC_ISR_EOS;
    adc_res[cnt++] = hadc->Instance->DR;
  }
  if (apply_pol) {
    for (int chan = 0; chan < CHAN_END; chan++) {
      adc_res[chan] = g_settings.polarity[chan] ? adc_res[chan] : (((1<<16)-1) - adc_res[chan]);
    }
  }
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
  * @brief  Sets the value to be used as the output, depending on the mode.
  *         In I2C mode, the values are used to provide the mainboard with the raw ADC count.
  *         In analog mode, the DAC is programmed with the output. Only the top 12 bits are used.
  *         This function takes in float values and applies the stored output calibration first.
  * @param  x: x coordinate, -1.0 to 1.0
  * @param  y: y coordinate, -1.0 to 1.0
  * @retval None
  */
void intf_out_cal(ax_t x, ax_t y) {
  dac_calib_t *dc = &g_settings.dac_calib;
  ax_t xo = dc->x_m * x + dc->x_b;
  ax_t yo = dc->y_m * y + dc->y_b;
  intf_out(AX_TO_INT16(xo), AX_TO_INT16(yo));
}

/**
  * @brief  Are we in analog (DAC) mode, or in I2C?
  * @retval True if in analog mode.
  */
bool intf_is_mode_analog() { return true; }