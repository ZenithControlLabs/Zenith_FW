#include "menu.h"
#include "main.h"
#include "intf.h"
#include "settings.h"
#include "stick.h"
#include "stick_types.h"

typedef enum {
  FACTORY_CENTER,
  FACTORY_POL_X,
  FACTORY_POL_Y,
  FACTORY_R_X_HIGH,
  FACTORY_R_X_LOW,
  FACTORY_R_Y_HIGH,
  FACTORY_R_Y_LOW,
  FACTORY_END
} factory_cal_step_t;

// TODO: we probably do not need to be storing this crap twice
typedef struct {
    int16_t x;
    int16_t y;
} vec2_t;
vec2_t calibration_points_in[] = {
    { 0, 0 },
    { 85<<8, 0 },
    { 0, 0 },
    { 70<<8, 70<<8 },
    { 0, 0 },
    { 0, 85<<8 },
    { 0, 0 },
    { -(70<<8), 70<<8 },
    { 0, 0 },
    { -(85<<8), 0 },
    { 0, 0 }, 
    { -(70<<8), -(70<<8) },
    { 0, 0 },
    { 0, -(85<<8) },
    { 0, 0 },
    { 70<<8, -(70<<8) }
};

void stick_calibration(ADC_HandleTypeDef *hadc) {
  uint8_t btn_press_debounce = 0xFF;
  int step = 0;
  uint16_t adc_res[2];
  analog_data_t in;
  _cal_step = 1;
  while (step < NUM_NOTCHES * 2) {
    adc_read(hadc, adc_res, false);

    btn_press_debounce <<= 1;
    if (HAL_GPIO_ReadPin(STICK_BTN_GPIO_Port, STICK_BTN_Pin)) {
      btn_press_debounce |= 1;
    }

    intf_out(calibration_points_in[step].x, calibration_points_in[step].y);

    if ((btn_press_debounce & 0b11) == 0b01) {
      in.ax1 = UINT_N_TO_AX(adc_res[CHAN_X], 12);
      in.ax2 = UINT_N_TO_AX(adc_res[CHAN_Y], 12);
      analoglib_cal_advance(&in);
      step++;
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }
}

void factory_init(ADC_HandleTypeDef *hadc) {
  settings_reset_to_factory();
// TODO: unsure what we are doing with these modes 
// For now, factory calibration is just stick calibration
  stick_calibration(hadc);
/*  int step = 0;
  int cnt = 0;
  uint16_t adc_res[2];
  uint16_t center[2];
  int16_t out[2];
  uint8_t btn_press_debounce = 0;
  float high = 0;
  const float dy = (INT_N_TO_AX(100, 8) - INT_N_TO_AX(-100, 8));
  while (step < FACTORY_END) {
    adc_read(hadc, adc_res, false);
    btn_press_debounce <<= 1;
    if (HAL_GPIO_ReadPin(STICK_BTN_GPIO_Port, STICK_BTN_Pin)) {
      btn_press_debounce |= 1;
    }
    out[CHAN_X] = 0;
    out[CHAN_Y] = 0;
    switch (step) {
      case FACTORY_CENTER: {
        cnt++;
        center[CHAN_X] = (uint16_t)((float)(center[CHAN_X]) * (cnt-1.0)/cnt + 1.0/cnt * adc_res[CHAN_X]);
        center[CHAN_Y] = (uint16_t)((float)(center[CHAN_Y]) * (cnt-1.0)/cnt + 1.0/cnt * adc_res[CHAN_Y]);
        break;
      }
      // Expect signal to be positive
      case FACTORY_POL_X:
      case FACTORY_POL_Y: {
        int axis = step - FACTORY_POL_X;
        out[axis] = (85) << 8;
        g_settings.polarity[axis] = adc_res[axis] > center[axis];
        break;
      }
      case FACTORY_R_X_HIGH:
      case FACTORY_R_Y_HIGH: {
        int axis = (step - FACTORY_R_X_HIGH) >> 1;
        out[axis] = (100) << 8;
        high = UINT_N_TO_AX(adc_res[axis], 12);
        break;
      }
      case FACTORY_R_X_LOW: { 
        out[CHAN_X] = -(100 << 8);
        g_settings.dac_calib.x_m = dy / (high - g_adc_res[CHAN_X]);
        g_settings.dac_calib.x_b = INT_N_TO_AX(-100, 8) - g_settings.dac_calib.x_m * g_adc_res[CHAN_X];
        break;
      }
      case FACTORY_R_Y_LOW: {
        out[CHAN_Y] = -(100 << 8);
        g_settings.dac_calib.y_m = dy / (high - g_adc_res[CHAN_Y]);
        g_settings.dac_calib.y_b = INT_N_TO_AX(-100, 8) - g_settings.dac_calib.y_m * g_adc_res[CHAN_Y];
        break;
      }
    }
    intf_out(out[CHAN_X], out[CHAN_Y]);
    if ((btn_press_debounce & 0b111) == 0b011) {
      step++;
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      HAL_Delay(100);
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
  }*/
  settings_flush();
  if (g_magic != MAGIC)  {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&g_magic, MAGIC);
    HAL_FLASH_Lock();
  }
  return;
}

/**
  * @brief  Run menu routine.
  * @param  in: input data read from ADC, in ax_t format
  * @param  out: output stick value
  * @param  btn_press: Is the stick clicked?
  * @retval None
  */
void menu_process(analog_data_t *in, analog_data_t *out, bool btn_press) {}