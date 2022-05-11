/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

	int R25 = 10000;
	int BETA = 3900;
	int TK25 = 298;
	int TC25 = 25;

	int pulsador = 0;
	int estado_anterior = 1;

	int nivel_alarma = 99;
	int alarma_state = 0;
	int cooldown = 0;

	unsigned int contador = 0;
	unsigned int prev_contador = 0;
	int ledState = 0;

	unsigned int contador_alarma = 0;
	unsigned int prevContador_alarma = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int readPOT(void) {

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	int potencia = (int)HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return (int) ((potencia*100)/4096);
}

int readLUZ(void) {

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	int luz = (int)HAL_ADC_GetValue(&hadc1);
	luz = (luz * -1) + 4095; // invertimos el valor del sensor
	HAL_ADC_Stop(&hadc1);
	return (int) ((luz*100)/4096);
}

int readTEMP(void) {

	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;

	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	int valueAD = (int)HAL_ADC_GetValue(&hadc1);
	float TNTC = BETA/(log((-10000.0*3.3/(valueAD*3.3/4095.9-3.3)-10000.0)/R25)+BETA/TK25)-273.18;
	float temp = TNTC - TC25;
	HAL_ADC_Stop(&hadc1);
	return (int) ((temp*100)/6);
}

void setLeds(int porcentaje) {
	if (porcentaje <= 0) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 0);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 0);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 0);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 0);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 13) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 0);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 0);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 0);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 25) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 0);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 0);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 38) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 0);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 50) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 0);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 63) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 0);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 85) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 1);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 0);
	} else if (porcentaje <= 99) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, 1);
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, 1);
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, 1);
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, 1);
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, 1);
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, 1);
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, 1);
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, 1);
	}
}

void setAlarm(int porcentaje) {

	if (contador - prev_contador >= 1500) {

		ledState = !ledState;
		prev_contador = contador;
	}

	if (porcentaje <= 0) {
		HAL_GPIO_WritePin(D1_GPIO_Port, D1_Pin, ledState);
	} else if (porcentaje <= 13 && porcentaje > 0) {
		HAL_GPIO_WritePin(D2_GPIO_Port, D2_Pin, ledState);
	} else if (porcentaje <= 25 && porcentaje > 13) {
		HAL_GPIO_WritePin(D3_GPIO_Port, D3_Pin, ledState);
	} else if (porcentaje <= 38 && porcentaje > 25) {
		HAL_GPIO_WritePin(D4_GPIO_Port, D4_Pin, ledState);
	} else if (porcentaje <= 50 && porcentaje > 38) {
		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, ledState);
	} else if (porcentaje <= 63 && porcentaje > 50) {
		HAL_GPIO_WritePin(D6_GPIO_Port, D6_Pin, ledState);
	} else if (porcentaje <= 85 && porcentaje > 63) {
		HAL_GPIO_WritePin(D7_GPIO_Port, D7_Pin, ledState);
	} else if (porcentaje <= 99 && porcentaje > 85) {
		HAL_GPIO_WritePin(D8_GPIO_Port, D8_Pin, ledState);
	}

	contador++;
}


void fireAlarm(int alarma_state) {

	// Primera vez
	if (alarma_state == 1 && cooldown == 0) { HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1); }

	// Resto de veces
	if (contador_alarma - prevContador_alarma >= 100000 && alarma_state == 1) {

		prevContador_alarma = contador_alarma;
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 1);
		cooldown = 0;
	}

	if (HAL_GPIO_ReadPin(PULSADOR2_GPIO_Port, PULSADOR2_Pin) == 0) {
		HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, 0);
		alarma_state = 0;
		cooldown = 1;
	}

	if (cooldown == 1) { contador_alarma++; }
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  int nivel_alarma = readPOT();
	  int porcentaje_temp = readTEMP();
	  int porcentaje_luz = readLUZ();

	  int estado = (int)HAL_GPIO_ReadPin(PULSADOR1_GPIO_Port, PULSADOR1_Pin);
	  if (estado != estado_anterior && estado == 0) { pulsador++; }
	  estado_anterior = estado;

	  if (pulsador%2 == 0) {

		  setLeds(porcentaje_luz);

		  if (porcentaje_luz > nivel_alarma ) {
			  alarma_state = 1;
		  } else {
			  alarma_state = 0;
		  }

	  } else if (pulsador%2 == 1) {

		  setLeds(porcentaje_temp);

		  if (porcentaje_temp > nivel_alarma ) {
			  alarma_state = 1;
		  } else {
			  alarma_state = 0;
		  }
	  }

	  setAlarm(nivel_alarma);
	  fireAlarm(alarma_state);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
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
  sConfig.Channel = ADC_CHANNEL_1;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D8_Pin|D6_Pin|Buzzer_Pin|D3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D7_Pin|D2_Pin|D5_Pin|D1_Pin
                          |D4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : D8_Pin D6_Pin Buzzer_Pin D3_Pin */
  GPIO_InitStruct.Pin = D8_Pin|D6_Pin|Buzzer_Pin|D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : D7_Pin D2_Pin D5_Pin D1_Pin
                           D4_Pin */
  GPIO_InitStruct.Pin = D7_Pin|D2_Pin|D5_Pin|D1_Pin
                          |D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PULSADOR1_Pin */
  GPIO_InitStruct.Pin = PULSADOR1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PULSADOR1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PULSADOR2_Pin */
  GPIO_InitStruct.Pin = PULSADOR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PULSADOR2_GPIO_Port, &GPIO_InitStruct);

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

