#include "menu.h"
#include "main.h"
#include "intf.h"
#include "settings.h"
#include "stick.h"
#include "stick_types.h"
#include <math.h>

#define TIMER_BASED_CALIB

struct {
  int cal_step;
  enum {
    MENU_MAIN,
    MENU_CALIB,
    MENU_OUTPUTCAL,
    MENU_NOTCH_SET
  } menu;
  int set_calib_ind;
} g_menu_state = {
  .cal_step = 0,
  .menu = MENU_MAIN,
  .set_calib_ind = -1
};

ax_t reference_calibration_seq[] = {
  INT_N_TO_AX(0,8),
  INT_N_TO_AX(70,8),
  INT_N_TO_AX(85,8),
  INT_N_TO_AX(70,8),
  INT_N_TO_AX(0,8),
  INT_N_TO_AX(-70,8),
  INT_N_TO_AX(-85,8),
  INT_N_TO_AX(-70,8),
};

stick_menu_direction_t stick_val_to_menu_dir(analog_data_t *an) {
  const uint8_t neutral_thresh = 8;
  const uint8_t active_thresh = 60;
  int8_t mag_x = AX_TO_INT8(fabsf(an->ax1));
  int8_t mag_y = AX_TO_INT8(fabsf(an->ax2));

  if (mag_x < neutral_thresh && mag_y > active_thresh) {
    return signbit(an->ax2) ? DIR_DOWN : DIR_UP;
  } else if (mag_x > active_thresh && mag_y < neutral_thresh) {
    return signbit(an->ax1) ? DIR_LEFT : DIR_RIGHT;
  } else if (mag_x < neutral_thresh && mag_y < neutral_thresh) {
    return DIR_NEUTRAL;
  } else { 
    return DIR_UNKNOWN;
  }
}

bool get_menu_btn_press() {
    static uint8_t btn_press_debounce = 0;
    btn_press_debounce <<= 1;
    if (!HAL_GPIO_ReadPin(STICK_BTN_GPIO_Port, STICK_BTN_Pin)) {
        btn_press_debounce |= 1;
    }

    return btn_press_debounce == 0b00000001;
}

#define STICK_REPEAT_TIMER_THRESH 2000
#define STICK_REPEAT_TIMER_REPEAT 1000
stick_menu_direction_t get_menu_dir_input(analog_data_t *an) {
  static stick_menu_direction_t last_input = DIR_NEUTRAL;
  static bool was_neutral = true;
  static int repeat_timer = STICK_REPEAT_TIMER_THRESH;

  stick_menu_direction_t menu_input = stick_val_to_menu_dir(an);
  
  // Have stayed holding a certain direction
  if ((menu_input & 0b100) && last_input == menu_input) {
    repeat_timer--;
  } else {
    repeat_timer = STICK_REPEAT_TIMER_THRESH;
  }
  was_neutral = was_neutral || menu_input == DIR_NEUTRAL;
  // Went back to neutral AND we transitioned from neutral/unknown to a direction
  if ((was_neutral && (((last_input ^ menu_input) & 0b100)))
    || repeat_timer == 0) {
    was_neutral = false;
    last_input = menu_input;
    if (!repeat_timer) {
      repeat_timer = STICK_REPEAT_TIMER_REPEAT;
    }
    return menu_input;
  }

  last_input = menu_input;
  return DIR_NEUTRAL;
}

bool calib_process(analog_data_t *raw, analog_data_t *cal) {
  static uint32_t last_step_time = 0;

  if (last_step_time == 0) {
    last_step_time = HAL_GetTick();
  }
  
  int step = g_menu_state.cal_step;
  if ((step & 1) == 0) {
    cal->ax1 = reference_calibration_seq[0];
    cal->ax2 = reference_calibration_seq[0];
  } else {
    cal->ax1 = reference_calibration_seq[((step>>1)+2)&7]; // x starts at + 2 mod 8 in seq
    cal->ax2 = reference_calibration_seq[(step>>1)]; // y starts at 0
  }

#ifdef TIMER_BASED_CALIB
  if (HAL_GetTick() - last_step_time >= 2500) { // TODO: how many ticks ???
#else
  if (get_menu_btn_press()) {
#endif 
    analoglib_cal_advance(raw);
    step++;
    //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    //HAL_Delay(100);
    //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }

#ifdef TIMER_BASED_CALIB
  else if (step >= 1 && get_menu_btn_press()) {
    analoglib_cal_undo();
    step--;
    //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    //HAL_Delay(100);
    //HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
  }
#endif

  if (step >= NUM_NOTCHES * 2) {
    g_menu_state.cal_step = 0;
    last_step_time = 0;
    return true;
  } else {
    if (g_menu_state.cal_step != step) {
      g_menu_state.cal_step = step;
      last_step_time = HAL_GetTick();
    }
    return false;
  }
}

void output_calib_process() {
  int16_t x_out = 0;
  int16_t y_out = 0; 
  analog_data_t in;
  analog_data_t out;
  uint16_t adc_res[2];
    
  for (int axis = 0; axis < 2; axis++) {
    for (int x = -128; x < 127; x++) {
      while (1)  {
        intf_adc_in(adc_res);
  
        in.ax1 = UINT_N_TO_AX(adc_res[CHAN_X], 12);
        in.ax2 = UINT_N_TO_AX(adc_res[CHAN_Y], 12);
  
        analoglib_process(&in, &out, false);
  
        stick_menu_direction_t menu_input = get_menu_dir_input(&out);
        if (menu_input != DIR_NEUTRAL) {
          debug_print("curr = %d,%d\n\r", x_out, y_out);
        }
        x_out += (menu_input == DIR_RIGHT) ? -16 : ((menu_input == DIR_LEFT) ? 16 : 0);
        y_out += (menu_input == DIR_UP) ? 16 : ((menu_input == DIR_DOWN) ? -16 : 0);
        intf_out(x_out, y_out); 

        if (get_menu_btn_press()) {
          debug_print("n64 = %d | dac = %d\n\r", x, axis ? x_out : y_out);
          break;
        }
      }
    }
  }
}

void factory_init(ADC_HandleTypeDef *hadc) {
  debug_print("FrogBoard Factory Init\n\r");
  settings_reset_to_factory();

  uint16_t x = 0;
  while (!get_menu_btn_press()) {
    x += 16;
    intf_out(x,x);
  }

  // For now, factory calibration is just stick calibration
  bool done = false;
  uint16_t adc_res[2];
  analog_data_t raw;
  analog_data_t cal;
  _cal_step = 1; // TODO make this a better interface
  while (!done) {
    intf_adc_in(adc_res);
    raw.ax1 = UINT_N_TO_AX(adc_res[CHAN_X], 12);
    raw.ax2 = UINT_N_TO_AX(adc_res[CHAN_Y], 12);
    done = calib_process(&raw, &cal);
    intf_out(AX_TO_INT16(cal.ax1), AX_TO_INT16(cal.ax2)); 
  }

  settings_flush();
  if (g_magic != MAGIC)  {
    HAL_FLASH_Unlock();
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, (uint32_t)&g_magic, MAGIC);
    HAL_FLASH_Lock();
  }
  return;
}

void notch_calibrate(const ax_t in_points_x[], const ax_t in_points_y[],
                     const ax_t notch_points_x[], const ax_t notch_points_y[],
                     calib_results_t *calib_results);

/**
  * @brief  Run menu routine.
  * @param  raw: input data read from ADC, in ax_t format
  * @param  cal: calibrated data, which is used both to control the menu and as the output
  * @retval true if we are staying in settings mode
  */
bool menu_process(analog_data_t *raw, analog_data_t *cal) {
  static int16_t x_out, y_out = 0;
  switch (g_menu_state.menu) {
    case MENU_CALIB: {
      if (calib_process(raw, cal)) {
        g_menu_state.menu = MENU_MAIN;
      }
      break;
    }
    case MENU_OUTPUTCAL: {
      output_calib_process();
      g_menu_state.menu = MENU_MAIN;
      break;
    }
    case MENU_NOTCH_SET: {
      bool press = get_menu_btn_press();
      int ind = 0;
      if (g_menu_state.set_calib_ind < 0) {
        float ang = atan2f(cal->ax2, cal->ax1);
        ind = (int)roundf(ang/M_PI_4);
        ind = (ang < 0) ? ind + 8 : ind;
        float mag = (cal->ax2)*(cal->ax2) + (cal->ax1)*(cal->ax1);
        if (mag <= 0.3f) {
          cal->ax1 = 0;
          cal->ax2 = 0;
          if (press) { 
            g_menu_state.menu = MENU_MAIN;
          }
        } else {
          if (press) {
            // TODO: make it not work if the magnitude is in some deadzone?
            g_menu_state.set_calib_ind = ind; 
            x_out = AX_TO_INT16(g_settings.stick_config.notch_points_x[g_menu_state.set_calib_ind]);
            y_out = AX_TO_INT16(g_settings.stick_config.notch_points_y[g_menu_state.set_calib_ind]);
          }
          ang = (float)ind * M_PI_4;
          cal->ax1 = 0.7f * cosf(ang);
          cal->ax2 = 0.7f * sinf(ang);
        }
      } else {
        ind = g_menu_state.set_calib_ind;
        stick_menu_direction_t menu_input = stick_val_to_menu_dir(cal);
        x_out += (menu_input == DIR_RIGHT) ? 1 : ((menu_input == DIR_LEFT) ? -1 : 0);
        y_out += (menu_input == DIR_UP) ? 1 : ((menu_input == DIR_DOWN) ? -1 : 0);
        intf_out(x_out, y_out); 
        if (press) {
          g_settings.stick_config.notch_points_x[g_menu_state.set_calib_ind] = INT_N_TO_AX(x_out, 16);
          g_settings.stick_config.notch_points_y[g_menu_state.set_calib_ind] = INT_N_TO_AX(y_out, 16);
          notch_calibrate(g_settings.calib_results.notch_points_x_in,
                        g_settings.calib_results.notch_points_y_in,
                        g_settings.stick_config.notch_points_x,
                        g_settings.stick_config.notch_points_y,
                        &(g_settings.calib_results));
          g_menu_state.set_calib_ind = -1;
        }
        cal->ax1 = INT_N_TO_AX(x_out, 16);
        cal->ax2 = INT_N_TO_AX(y_out, 16);
      }
      
    }
    default:
    case MENU_MAIN: {
      if (!get_menu_btn_press()) break;

      switch (stick_val_to_menu_dir(cal)) {
        case DIR_LEFT: {
          debug_print("Enter calibration menu...\n\r");
          g_menu_state.menu = MENU_CALIB;
          _cal_step = 1; // TODO make this a better interface
          break;
        }
        case DIR_RIGHT: {
          debug_print("Enter output calibration menu...\n\r");
          g_menu_state.menu = MENU_OUTPUTCAL;
          break;
        }
        case DIR_UP: {
          debug_print("Leave settings mode...\n\r");
          return false;
        }
        case DIR_DOWN: {
          g_menu_state.menu = MENU_NOTCH_SET;
          break;
        }
        default: break;
      }
      break;
    }
  }
  return true;
}