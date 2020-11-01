/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <config.h>
#include <ntc.h>
#include <rs485.h>
#include <modbus.h>


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

#define WORK_STATE_MANUAL 0
#define WORK_STATE_AUTO 1

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Peripheral handler variables */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* Types */

typedef enum
{
	GATE_IDLE,
	GATE_ACTIVE,
} gate_state_t;

typedef struct
{
	uint32_t gate_pin;
	const uint8_t temp_sensor_index;
	uint8_t work_state;
	int16_t setpoint;
	int16_t output_voltage_decpercent;
	uint32_t activation_delay_us; // time from zero-crossing to gate activation
	gate_state_t state;
} channel_t;

/* Flags */
bool update_working_parameters_pending_flag = false;
bool modbus_request_pending_flag = false;

/* Global variables */
uint32_t gate_pulse_delay_counter_us = 0;
uint32_t update_parameter_timer_counter_us = 0;
uint32_t rx_time_interval_counter = 0;
sensors_t sensor_values;
int16_t temperature_error_state = TEMPERATURE_STATUS_NO_ERROR;
uint8_t incoming_modbus_frame[RS_RX_BUFFER_SIZE];
uint16_t modbus_frame_byte_counter = 0;

static channel_t channel_array[OUTPUT_CHANNELS_NUMBER] = {
	{gate_1_Pin, 0, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE},
	{gate_2_Pin, 1, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE},
	{gate_3_Pin, 2, WORK_STATE_AUTO, INIT_CHANNEL_SETPOINT_C, 0, 0, GATE_IDLE}
};
uint8_t uart_rx_byte = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
extern void initialise_monitor_handles(void);
/* USER CODE BEGIN PFP */

/* FUNCTION PROTOTYPES */
void drive_fans(void);
uint32_t get_gate_delay_us(uint16_t output_power);
int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t target_temperature);
void set_gate_state(channel_t * fan, gate_state_t pulse_state);
void update_working_parameters(void);
void update_modbus_registers(void);
void update_app_data(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void update_working_parameters()
{
	printf("%s", "Updating working parameters\n");
	HAL_GPIO_TogglePin(GPIOD, LED_G_Pin);
	ntc_calculate_temperatures(&sensor_values);
//	for (int channel = 0; channel < ADC_SENSOR_NUMBER; channel++)
//	{
//	  printf("CH%d val: %d, temp: %d\n", channel, sensor_values.adc_values[channel], sensor_values.temperatures[channel]);
//	}

	temperature_error_state = check_temperatures(&sensor_values);

	for (uint8_t i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		if (channel_array[i].work_state == WORK_STATE_AUTO)
		{
			channel_array[i].output_voltage_decpercent = pi_regulator(i, sensor_values.temperatures[i], channel_array[i].setpoint);
		}
		channel_array[i].activation_delay_us = get_gate_delay_us(channel_array[i].output_voltage_decpercent);
	}

	update_working_parameters_pending_flag = false;

}


void drive_fans(void)
{
	for (uint8_t i = 0; i < OUTPUT_CHANNELS_NUMBER; i++)
	{
		if (channel_array[i].output_voltage_decpercent < MIN_OUTPUT_VOLTAGE_DECPERCENT)
		{
			set_gate_state(&channel_array[i], GATE_IDLE); // full off
		}

		else if (channel_array[i].output_voltage_decpercent >= MAX_OUTPUT_VOLTAGE_DECPERCENT)
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE); // full on
		}

		else if ( (gate_pulse_delay_counter_us >= channel_array[i].activation_delay_us) && (gate_pulse_delay_counter_us < (channel_array[i].activation_delay_us + GATE_PULSE_MIN_TIME_US)) )
		{
			set_gate_state(&channel_array[i], GATE_ACTIVE);
		}
		else
		{
			set_gate_state(&channel_array[i], GATE_IDLE);
		}
	}
}


uint32_t get_gate_delay_us(uint16_t output_voltage_percent)
{
	uint16_t mean_voltage = (output_voltage_percent*23)/100;
	double activation_angle_rad = acos(mean_voltage/230.0); // acos function input is double, value from -1 to 1
	uint32_t gate_delay = HALF_SINE_PERIOD_US*activation_angle_rad/(PI/2.0);

	if (gate_delay > MAX_GATE_DELAY_US)
		gate_delay = MAX_GATE_DELAY_US;

	if (gate_delay < MIN_GATE_DELAY_US)
		gate_delay = MIN_GATE_DELAY_US;

	return gate_delay - ZERO_CROSSING_DETECTION_OFFSET_US;
}


int16_t pi_regulator(uint8_t channel, int16_t current_temp, int16_t setpoint)
{
	int16_t error;
	static int16_t integral_error[3] = {0, 0, 0};
	int16_t output_voltage_decpercent;

	error = current_temp - setpoint;

	integral_error[channel] = integral_error[channel] + error;

	if ((error>0) && (integral_error[channel] <350*TIME_CONST))
		integral_error[channel] = 350*TIME_CONST;

	if (integral_error[channel] > INTEGRAL_ERROR_MAX)
		integral_error[channel] = INTEGRAL_ERROR_MAX;
	if (integral_error[channel] < INTEGRAL_ERROR_MIN)
		integral_error[channel] = INTEGRAL_ERROR_MIN;

	output_voltage_decpercent = PI_KP * error  + integral_error[channel]/TIME_CONST;

	if (output_voltage_decpercent > MAX_OUTPUT_VOLTAGE_DECPERCENT)
		output_voltage_decpercent = FULL_ON_OUTPUT_VOLTAGE_DECPERCENT;

	if (output_voltage_decpercent < MIN_OUTPUT_VOLTAGE_DECPERCENT)
		output_voltage_decpercent = FULL_OFF_OUTPUT_VOLTAGE_DECPERCENT;

	return output_voltage_decpercent;
};


void set_gate_state(channel_t * fan, gate_state_t pulse_state)
{
	if (fan->state == pulse_state)
	{
		return; // no state change
	}

	fan->state = pulse_state;

	if (pulse_state == GATE_ACTIVE)
	{
		HAL_GPIO_WritePin(GPIOC, fan->gate_pin, GPIO_PIN_RESET); // optotransistor is active low
	}

	if (pulse_state != GATE_ACTIVE)
	{
		HAL_GPIO_WritePin(GPIOC, fan->gate_pin, GPIO_PIN_SET);
	}
}


void update_modbus_registers(void)
{
	printf("Updating Modbus registers with data from app before processing request\n");
	modbus_set_reg_value(0, channel_array[0].work_state);
	modbus_set_reg_value(1, channel_array[1].work_state);
	modbus_set_reg_value(2, channel_array[2].work_state);
	modbus_set_reg_value(3, channel_array[0].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(4, channel_array[1].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(5, channel_array[2].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(6, channel_array[0].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(7, channel_array[1].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(8, channel_array[2].setpoint/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(9, sensor_values.temperatures[0]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(10, sensor_values.temperatures[1]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(11, sensor_values.temperatures[2]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(12, sensor_values.temperatures[3]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(13, sensor_values.temperatures[4]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(14, sensor_values.temperatures[5]/TEMPERATURE_PRECISION_MULTIPLIER);
	modbus_set_reg_value(15, temperature_error_state);
}


void update_app_data(void)
{
	printf("Updating app data with data from Modbus registers\n");
	channel_array[0].work_state = modbus_get_reg_value(0);
	channel_array[1].work_state = modbus_get_reg_value(1);
	channel_array[2].work_state = modbus_get_reg_value(2);

	if ((modbus_get_reg_value(3)) != channel_array[0].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[0].work_state = WORK_STATE_MANUAL;
		channel_array[0].output_voltage_decpercent = modbus_get_reg_value(3)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(4) != channel_array[1].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[1].work_state = WORK_STATE_MANUAL;
		channel_array[1].output_voltage_decpercent = modbus_get_reg_value(4)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(5) != channel_array[2].output_voltage_decpercent/VOLTAGE_PRECISION_MULTIPLIER) // check if value changed
	{
		channel_array[2].work_state = WORK_STATE_MANUAL;
		channel_array[2].output_voltage_decpercent = modbus_get_reg_value(5)*VOLTAGE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(6) != (channel_array[0].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[0].work_state = WORK_STATE_AUTO;
		channel_array[0].setpoint = modbus_get_reg_value(6)*TEMPERATURE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(7) != (channel_array[1].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[1].work_state = WORK_STATE_AUTO;
		channel_array[1].setpoint = modbus_get_reg_value(7)*TEMPERATURE_PRECISION_MULTIPLIER;
	}

	if (modbus_get_reg_value(8) != (channel_array[2].setpoint/TEMPERATURE_PRECISION_MULTIPLIER)) // check if value changed
	{
		channel_array[2].work_state = WORK_STATE_AUTO;
		channel_array[2].setpoint = modbus_get_reg_value(8)*TEMPERATURE_PRECISION_MULTIPLIER;
	}
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
	initialise_monitor_handles(); // needed for debug semihosting

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sensor_values.adc_values, 6);

	rs485_init(&huart1);
	update_working_parameters();

	// TEMPORARY, START RECEIVING BYTES
	HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	if (status != HAL_OK)
	{
	  printf ("%s \n", "Error, cannot start HAL_UART_Transmit_IT");
	}

	/* USER CODE END 2 */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

  while (1)
  {
	if (update_working_parameters_pending_flag == true)
	{
		update_working_parameters();
	}

	if (modbus_request_pending_flag == true)
	{
		uint8_t response_buffer[RS_TX_BUFFER_SIZE];
		uint16_t response_size;
		update_modbus_registers(); // update Modbus registers with data from app
		if (modbus_process_frame(incoming_modbus_frame, modbus_frame_byte_counter, response_buffer, &response_size))
		{
			rs485_transmit_byte_array(response_buffer, response_size);
		}
		update_app_data(); // update app with new data from processed Modbus frame (needed if it was write command)

		modbus_request_pending_flag = false;
		modbus_frame_byte_counter = 0;
	}

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

  ADC_InjectionConfTypeDef sConfigInjected = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 6;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configures for the selected ADC injected channel its corresponding rank in the sequencer and its sample time
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_0;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_3CYCLES;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONVEDGE_NONE;
  sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */ // currently set as 100ns (1Meg/100/100 = 100n)
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, gate_1_Pin|gate_2_Pin|gate_3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LED_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(rs_dir_GPIO_Port, rs_dir_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : zero_crossing_detection_Pin */
  GPIO_InitStruct.Pin = zero_crossing_detection_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(zero_crossing_detection_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : gate_1_Pin gate_2_Pin gate_3_Pin */
  GPIO_InitStruct.Pin = gate_1_Pin|gate_2_Pin|gate_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : rs_dir_Pin */
  GPIO_InitStruct.Pin = rs_dir_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(rs_dir_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */

/**
  * @brief This function handles EXTI line0 interrupt.
  */
// AC zero crossing detection handler
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

	if (gate_pulse_delay_counter_us > HALF_SINE_PERIOD_US - 500)
	{
		gate_pulse_delay_counter_us = 0;
	}
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}


/* Timer 2 overflow interrupt callback */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2)
	{
		drive_fans();

		if(update_parameter_timer_counter_us >= WORKING_PARAMETERS_UPDATE_PERIOD_US)
		{
			HAL_ADC_Start_DMA(&hadc1, (uint32_t*)sensor_values.adc_values, 6);
			update_parameter_timer_counter_us = 0;
		}

		gate_pulse_delay_counter_us += MAIN_TIMER_RESOLUTION_US;
		update_parameter_timer_counter_us += MAIN_TIMER_RESOLUTION_US;
		rx_time_interval_counter += MAIN_TIMER_RESOLUTION_US;
	}

	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_MODBUS_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_complete_frame(incoming_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
	}
}

/* ADC conversion finished callback */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
//  printf("%s\n", "ADC conversion finished");
  update_working_parameters_pending_flag = true;
}

/* UART RX finished callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
//	printf("Serial received a byte: %02x\n", uart_rx_byte);

	if ( (rx_time_interval_counter > MAX_TIME_BETWEEN_MODBUS_FRAMES_US) && (!rs485_rx_buffer_empty()) )
	{
		rs485_get_complete_frame(incoming_modbus_frame, RS_RX_BUFFER_SIZE);
		modbus_request_pending_flag = true;
	}
	else
	{
		rx_time_interval_counter = 0;

		if (modbus_request_pending_flag == false)
		{
			if (rs485_collect_byte_to_buffer(&uart_rx_byte))
			{
				modbus_frame_byte_counter++;
			}
			else
			{
				printf ("ERROR, cannot get byte to buffer (buffer full)\n");
			}
		}
	}

	// Prepare for next byte receiving
	HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
	if (status != HAL_OK)
	{
	  printf ("ERROR, cannot start HAL_UART_Transmit_IT\n");
	}
}

/* UART TX finished callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	//
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
