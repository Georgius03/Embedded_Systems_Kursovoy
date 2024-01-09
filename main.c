/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t setpoint = 0;
uint16_t ultrasonic_level = 0;

uint16_t system_state = 0;

uint16_t input_pump = 0;
uint16_t output_pump = 0;

uint16_t output_but_1 = 0;
uint16_t output_but_2 = 0;
uint16_t input_but_1 = 1;
uint16_t input_but_2 = 0;

uint16_t led_cascade = 0;

uint16_t rgb_red = 0;
uint16_t rgb_green = 0;
uint16_t rgb_blue = 0;

uint16_t emergency_level = 0;

uint16_t error = 0;

uint16_t current_level = 0;
extern uint16_t ultrasonic_level_reg;
extern uint16_t setpoint_reg;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint16_t system_first_safety_test()
{
	// Проверка датчиков
	if (emergency_level == 1) { error = 1; return 0; }
	if (ultrasonic_level > 210) { error = 2; return 0; }

	HAL_Delay(3000);

	// Проверка насосов
	// Проверка наполняющего насоса
	current_level = ultrasonic_level_reg;
	HAL_GPIO_WritePin(INPUT_PUMP_GPIO_Port, INPUT_PUMP_Pin, GPIO_PIN_SET);
	input_pump = 1;
	HAL_Delay(5000); // 35000
	for (int16_t i = 0; i < 1000; i++) {i = i + 1;}
	HAL_GPIO_WritePin(INPUT_PUMP_GPIO_Port, INPUT_PUMP_Pin, GPIO_PIN_RESET);
	input_pump = 0;
	if (ultrasonic_level_reg / current_level) { error = 4; return 0; }

	// Проверка сливающего насоса
	current_level = ultrasonic_level_reg;
	HAL_GPIO_WritePin(OUTPUT_PUMP_GPIO_Port, OUTPUT_PUMP_Pin, GPIO_PIN_SET);
	output_pump = 1;
	HAL_Delay(5000); // 35000
	for (int16_t i = 0; i < 1000; i++) {i = i + 1;}
	HAL_GPIO_WritePin(OUTPUT_PUMP_GPIO_Port, OUTPUT_PUMP_Pin, GPIO_PIN_RESET);
	output_pump = 0;
	if (current_level / ultrasonic_level_reg) { error = 5; return 0; }
	return 0;
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim4);
	// тест системы
	system_first_safety_test();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{if (error == 1) { // Критический уровень (крас)
		rgb_red = 1;
		rgb_green = 0;
		rgb_blue = 0;
	}
	if (error == 2) { // неисправность уз. датчика уровня (жёл)
		rgb_red = 1;
		rgb_green = 1;
		rgb_blue = 0;
	}
	if (error == 3) { // неисправность уз. датчика уровня (жёл)
		rgb_red = 1;
		rgb_green = 1;
		rgb_blue = 0;
	}
	if (error == 4) { // неисправность наполняющего насоса или уз датчика уровня (фиол)
		rgb_red = 1;
		rgb_green = 0;
		rgb_blue = 1;
	}
	if (error == 5) { // неисправность сливающего насоса или уз датчика уровня (син)
		rgb_red = 0;
		rgb_green = 0;
		rgb_blue = 1;
	}

	if (error != 0) { // Выключить всё если есть ошибка
		HAL_TIM_Base_Stop_IT(&htim3);

		HAL_GPIO_WritePin(INPUT_PUMP_GPIO_Port, INPUT_PUMP_Pin, RESET);
		input_pump = 0;
		HAL_GPIO_WritePin(OUTPUT_PUMP_GPIO_Port, OUTPUT_PUMP_Pin, RESET);
		output_pump = 0;

		if (HAL_GPIO_ReadPin(RESET_ERROR_GPIO_Port, RESET_ERROR_Pin)){ error = 0; }

	}
	else { // ошибок нет
		ultrasonic_level = 200 - (int32_t)(ultrasonic_level_reg * 200) / 4095;
		if (system_state) { // выбор уставки
			HAL_TIM_Base_Stop_IT(&htim3);
			rgb_red = 1;
			rgb_green = 1;
			rgb_blue = 1;

			if (setpoint >= 198) { led_cascade = 4; }
			else if (setpoint >= 150) { led_cascade = 3; }
			else if (setpoint >= 100) { led_cascade = 2; }
			else if (setpoint >= 50) { led_cascade = 1; }
			else if (setpoint >= 0) { led_cascade = 0; }
			setpoint = (int32_t)(setpoint_reg * 200) / 4095;
		}
		else { // нормальная работа

			rgb_red = 0;
			rgb_green = 1;
			rgb_blue = 0;

			HAL_TIM_Base_Start_IT(&htim3);

			if (ultrasonic_level >= 198) { led_cascade = 4; }
			else if (ultrasonic_level >= 150) { led_cascade = 3; }
			else if (ultrasonic_level >= 100) { led_cascade = 2; }
			else if (ultrasonic_level >= 50) { led_cascade = 1; }
			else if (ultrasonic_level >= 0) { led_cascade = 0; }

		}
	}

	if (led_cascade == 1) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
	}
	else if (led_cascade == 2) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
	}
	else if (led_cascade == 3) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);
	}
	else if (led_cascade == 4) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, SET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, SET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, SET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, SET);

	}
	else if (led_cascade == 0) {
		HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, RESET);
		HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, RESET);
		HAL_GPIO_WritePin(LED_3_GPIO_Port, LED_3_Pin, RESET);
		HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, RESET);

	}

	rgb_red ? HAL_GPIO_WritePin(RED_PIN_GPIO_Port, RED_PIN_Pin, SET) : HAL_GPIO_WritePin(RED_PIN_GPIO_Port, RED_PIN_Pin, RESET);
	rgb_green ? HAL_GPIO_WritePin(GREEN_PIN_GPIO_Port, GREEN_PIN_Pin, SET) : HAL_GPIO_WritePin(GREEN_PIN_GPIO_Port, GREEN_PIN_Pin, RESET);
	rgb_blue ? HAL_GPIO_WritePin(BLUE_PIN_GPIO_Port, BLUE_PIN_Pin, SET) : HAL_GPIO_WritePin(BLUE_PIN_GPIO_Port, BLUE_PIN_Pin, RESET);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 8000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_2_Pin|LED_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_1_Pin|LED_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUTPUT_PUMP_Pin|INPUT_PUMP_Pin|RED_PIN_Pin|GREEN_PIN_Pin
                          |BLUE_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_2_Pin LED_3_Pin */
  GPIO_InitStruct.Pin = LED_2_Pin|LED_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LED_4_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin|LED_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_ERROR_Pin */
  GPIO_InitStruct.Pin = RESET_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(RESET_ERROR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_BUT_1_Pin */
  GPIO_InitStruct.Pin = INPUT_BUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(INPUT_BUT_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_BUT_2_Pin OUTPUT_BUT_2_Pin EMERGENCY_PIN_Pin */
  GPIO_InitStruct.Pin = INPUT_BUT_2_Pin|OUTPUT_BUT_2_Pin|EMERGENCY_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OUTPUT_PUMP_Pin INPUT_PUMP_Pin RED_PIN_Pin GREEN_PIN_Pin
                           BLUE_PIN_Pin */
  GPIO_InitStruct.Pin = OUTPUT_PUMP_Pin|INPUT_PUMP_Pin|RED_PIN_Pin|GREEN_PIN_Pin
                          |BLUE_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SYSTEM_STATE_Pin */
  GPIO_InitStruct.Pin = SYSTEM_STATE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SYSTEM_STATE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OUTPUT_BUT_1_Pin */
  GPIO_InitStruct.Pin = OUTPUT_BUT_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OUTPUT_BUT_1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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

#ifdef  USE_FULL_ASSERT
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
