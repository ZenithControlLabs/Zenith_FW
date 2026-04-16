/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
extern int g_timer_wrap;
extern volatile int g_poll_rdy;
extern volatile int g_poll_rdy_chk_ignore;
extern volatile int g_sleeping;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#ifdef DEBUG
#define debug_print(fmt, args...) printf(fmt, ##args)
#else
#define debug_print(fmt, args...)
#endif
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOA
#define AX_OUT_Pin GPIO_PIN_4
#define AX_OUT_GPIO_Port GPIOA
#define AY_OUT_Pin GPIO_PIN_5
#define AY_OUT_GPIO_Port GPIOA
#define AX_IN_Pin GPIO_PIN_6
#define AX_IN_GPIO_Port GPIOA
#define AY_IN_Pin GPIO_PIN_7
#define AY_IN_GPIO_Port GPIOA
#define A_SEL_Pin GPIO_PIN_8
#define A_SEL_GPIO_Port GPIOA
#define STICK_BTN_Pin GPIO_PIN_9
#define STICK_BTN_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
#ifdef FROGBOARD_NUCLEO_DEVBOARD
#undef MCO_Pin
#undef MCO_GPIO_Port
#undef VCP_TX_Pin
#undef VCP_TX_GPIO_Port
#undef AX_OUT_Pin
#undef AX_OUT_GPIO_Port
#undef AY_OUT_Pin
#undef AY_OUT_GPIO_Port
#undef AX_IN_Pin
#undef AX_IN_GPIO_Port
#undef AY_IN_Pin
#undef AY_IN_GPIO_Port
#undef STICK_BTN_Pin
#undef STICK_BTN_GPIO_Port
#undef SWDIO_Pin
#undef SWDIO_GPIO_Port
#undef SWCLK_Pin
#undef SWCLK_GPIO_Port
#undef VCP_RX_Pin
#undef VCP_RX_GPIO_Port
#undef LD3_Pin

#define LD3_GPIO_Port GPIOB
#define MCO_Pin GPIO_PIN_0
#define MCO_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define AX_OUT_Pin GPIO_PIN_4
#define AX_OUT_GPIO_Port GPIOA
#define AY_OUT_Pin GPIO_PIN_5
#define AY_OUT_GPIO_Port GPIOA
#define AX_IN_Pin GPIO_PIN_6
#define AX_IN_GPIO_Port GPIOA
#define AY_IN_Pin GPIO_PIN_7
#define AY_IN_GPIO_Port GPIOA
#define STICK_BTN_Pin GPIO_PIN_0
#define STICK_BTN_GPIO_Port GPIOB
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define LD3_Pin GPIO_PIN_3
#define LD3_GPIO_Port GPIOB
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
