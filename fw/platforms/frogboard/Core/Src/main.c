/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "stm32l4xx_hal_adc.h"
#include "stm32l4xx_hal_conf.h"
#include "stm32l4xx_hal_dac.h"
#include "stm32l4xx_hal_flash.h"
#include "stm32l4xx_hal_gpio.h"
#include "stm32l4xx_hal_lptim.h"

#include "stick.h"
#include "stick_types.h"

#include "intf.h"
#include "settings.h"
#include "menu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    int8_t x;
    int8_t y;
} vec2_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TIMER_LEN ((1 << 24) - 1)
#define POLL_RATE_HZ 2048
//#define BENCHMARK_SPEED
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

DAC_HandleTypeDef hdac1;

LPTIM_HandleTypeDef hlptim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
volatile int g_poll_rdy = 1;
volatile int g_sleeping = 0;
uint16_t g_adc_res[2];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_LPTIM1_Init(void);
/* USER CODE BEGIN PFP */
#define PRINTF2UART2 int __io_putchar(int data)
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef BENCHMARK_SPEED
void speed_benchmark() {
  debug_print("This is FrogBoard! (speed benchmark)\n\r");
  vec2_t calibration_points_in[] = {
    { 3, 0 },
    { 96, 0 },
    { 0, 1 },
    { 70, 74 },
    { 0, 1 },
    { -1, 85 },
    { 0, 0 },
    { -60, 80 },
    { 0, 2 },
    { -100, 0 },
    { 0, 1 }, 
    { -70, -70 },
    { 1, 1 },
    { 0, -82 },
    { 0, 0 },
    { 80, -80 }
};
  calib_results_t test_calib_results = {0};
  stick_config_t test_stick_config = {
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
    .mag_threshold = .8 // 80% into the notch by default
};
  analoglib_init(&test_calib_results, &test_stick_config);
  analog_data_t in;
  // Skip "start" command
  _cal_step = 1;
  for (int i = 0; i < 16; i++) {
    in.ax1 = INT_N_TO_AX(calibration_points_in[i].x, 8);
    in.ax2 = INT_N_TO_AX(calibration_points_in[i].y, 8);
    analoglib_cal_advance(&in);
  }
  SysTick->CTRL = 0x07;     // CLKSRC | ENABLE, interrupt enable
  int thing = 0;
  while (1) {
    SysTick->LOAD = TIMER_LEN;
    SysTick->VAL = 1-1;
    g_timer_wrap = 0;
    analog_data_t out;
    for (volatile int i = 0; i < 1000; i++) {
      in.ax1 = INT_N_TO_AX((rand() & 255 - 128), 8);
      in.ax2 = INT_N_TO_AX((rand() & 255 - 128), 8);
      analoglib_process(&in, &out, false);
    }
    uint32_t duration = (TIMER_LEN - SysTick->VAL) + (g_timer_wrap * TIMER_LEN);
    int speed = (int)(24.0e9 /((float)duration));
    debug_print("%d\n\r", speed);
    HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    intf_out(AX_TO_INT16(out.ax1),AX_TO_INT16(out.ax2));
    thing++;
    if ((thing & 15) == 0) {
      intf_out(0,0);
      for (volatile int i = 0; i < 1000000; i++) {
      }
    }
  }
}
#endif

static inline void main_loop_body(bool settings_mode) {
  static int cnt = 0;
  analog_data_t in;
  analog_data_t out;
    // Read ADC
    adc_read(&hadc1, g_adc_res, false);

    // In Phobri stickboard emulation mode? Send out the raw ADC reading
    if (!intf_is_mode_analog()) {
      intf_out(g_adc_res[CHAN_X], g_adc_res[CHAN_Y]);
      return;
    }

    in.ax1 = UINT_N_TO_AX(g_adc_res[CHAN_X], 12);
    in.ax2 = UINT_N_TO_AX(g_adc_res[CHAN_Y], 12);

    // Process, either normal or we are in settings menu
    if (settings_mode) {
      bool btn_press = HAL_GPIO_ReadPin(STICK_BTN_GPIO_Port, STICK_BTN_Pin);
      menu_process(&in, &out, btn_press);
    } else {
      analoglib_process(&in, &out, false);
    } 

    intf_out(AX_TO_INT16(out.ax1), AX_TO_INT16(out.ax2)); 
    if ((cnt & 255) == 0) {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    }
    cnt++; 
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  MX_DAC1_Init();
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
  DAC1->DHR12RD = (2048) << 16 | (2048);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_LPTIM1_Init();
  /* USER CODE BEGIN 2 */
  // Initialize output interface - do the I2C test to see if we need to be in Phobri mode or not
  intf_init(&hdac1);

  // First (try to) load settings from flash. Interface will need this for output calibration when running in analog mode.
  bool needs_factory_init = intf_is_mode_analog() && settings_load();

  // Run any special benchmark modes if specified
#ifdef BENCHMARK_SPEED
  speed_benchmark();
  return 0;
  #elif BENCHMARK_POWER
  // TODO
  return 0;
  #endif

  // Stick procesing library should use our settings
  analoglib_init(&g_settings.calib_results, &g_settings.stick_config);
  
  // Run the factory calibration process if settings load says this is the first time
  if (needs_factory_init) {
    factory_init(&hadc1);
  }

  // Done initializing!
  debug_print("This is FrogBoard!\n\r");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  bool settings_mode = needs_factory_init || HAL_GPIO_ReadPin(STICK_BTN_GPIO_Port, STICK_BTN_Pin);
  
  LPTIM1->IER = LPTIM_IER_ARRMIE;
  HAL_LPTIM_Counter_Start(&hlptim1, 32768 / POLL_RATE_HZ);
  while (1) {
    if (!g_poll_rdy) {
      debug_print("Sanity check failed! Is the main loop taking too long..?\n");
      Error_Handler();
    }
    main_loop_body(false);
    g_poll_rdy = 0;

    HAL_ADC_DeInit(&hadc1);
    LL_ADC_DisableInternalRegulator(hadc1.Instance);
    HAL_SuspendTick();
    g_sleeping = 1;
    HAL_PWREx_EnterSTOP0Mode(PWR_SLEEPENTRY_WFI);
    g_sleeping = 0;
    HAL_ResumeTick();
    MX_ADC1_Init();
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 24;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */
  RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;

  // 2. (CRITICAL FOR ROBUSTNESS) Force a reset of the DAC peripheral
  RCC->APB1RSTR1 |= RCC_APB1RSTR1_DAC1RST; // Assert reset
  RCC->APB1RSTR1 &= ~RCC_APB1RSTR1_DAC1RST; // De-assert reset

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */
  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STICK_BTN_Pin */
  GPIO_InitStruct.Pin = STICK_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(STICK_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
PRINTF2UART2 {
  HAL_UART_Transmit(&huart2, (uint8_t *)&data, 1, 0xFFFF);
  return data;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: debug_print("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
