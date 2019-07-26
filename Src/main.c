/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "../Drivers/IMU.h"
#include "../Drivers/PID.h"
#include <stdbool.h>
#include "arm_math.h"
#include "../Drivers/MY_NRF24.h"
#include "../Drivers/dwt_delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//Min and max counter load value for timer4, which handles PWM generation. ESC_MIN = 125us pulse
//and ESC_MAX = 250us pulse (OneShot125 protocol)
#define ESC_MIN 1250
#define ESC_MAX 2500
//How many of same pulse to send before updating with new value
#define PULSE_DIV 4

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

int esc1_throttle = 200;
int esc2_throttle = 200;
int esc3_throttle = 200;
int esc4_throttle = 200;

int esc1_total = 0;
int esc2_total = 0;
int esc3_total = 0;
int esc4_total = 0;

bool getRPY_flag = 0;

float imu_roll;
float imu_pitch;
float imu_yaw;

float pid_output_roll = 0;
float pid_output_pitch = 0;
float pid_output_yaw = 0;

int pulse_count = 0;
int tim3_count = 0;

bool main_loop = 0;

uint64_t TxpipeAddrs = 0x11223344AA;
char RxData[64];

/** For debugging and tuning PID control, data storage buffer for
 *  printing afterwards to evaluate system response, can comment out later to save RAM
 *  if needed (approx 20kB needed?)
 */
#define PID_TUNE_DEBUG 0

#if PID_TUNE_DEBUG

int samples_to_take = 5000;

float PID_print_buffer[5000];
float IMU_print_buffer[5000];
int print_buffer_index = 0;

#endif

#define MOTORS 0

/*** Set up structures for PID control using DSP library ***/

arm_pid_instance_f32 pid_pitch_gains;
arm_pid_instance_f32 pid_roll_gains;
arm_pid_instance_f32 pid_yaw_gains;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

void ARM_ESCs();

//These 4 functions set the PWM duty cycles
void PWM1_Set(uint16_t value);
void PWM2_Set(uint16_t value);
void PWM3_Set(uint16_t value);
void PWM4_Set(uint16_t value);

void pulse_posedge_handler();

void print_data_to_pc();
void update_PID_values();

long map(long x, long in_min, long in_max, long out_min, long out_max);
int __io_putchar(int ch);
int _write(int file, char *ptr, int len);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void printToPC();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_DMA_Init();
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	/** Init MPU9250 **/
	if (imu_init(&hi2c2) == IMU_SUCCESS) {
		imu_calibrate();
	}

	/**** Init NRF24L01 Module ***/


	DWT_Init(); //Enable some of the MCUs special registers so we can get microsecond (us) delays
	NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi2);
	nrf24_DebugUART_Init(huart2);
	NRF24_setAutoAck(false);
	NRF24_openReadingPipe(1,TxpipeAddrs);
	NRF24_startListening();


	printRadioSettings();

	/** Init timers ***/

	/*//Start timer 3 in interrupt mode, used for integral calculations
	HAL_TIM_Base_Start_IT(&htim3);

	//Start up PWMs
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_4);*/

#if MOTORS
	ARM_ESCs();
#endif

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		main_loop = 1;

		if (NRF24_available()) {

			NRF24_read(RxData, 32);
			printf("Received data %s\n\r", RxData);

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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 400000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI2_Init(void) {

	/* USER CODE BEGIN SPI2_Init 0 */

	/* USER CODE END SPI2_Init 0 */

	/* USER CODE BEGIN SPI2_Init 1 */

	/* USER CODE END SPI2_Init 1 */
	/* SPI2 parameter configuration*/
	hspi2.Instance = SPI2;
	hspi2.Init.Mode = SPI_MODE_MASTER;
	hspi2.Init.Direction = SPI_DIRECTION_2LINES;
	hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi2.Init.NSS = SPI_NSS_SOFT;
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi2.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI2_Init 2 */

	/* USER CODE END SPI2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 2000;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 9;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = 20000;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim4) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */
	HAL_TIM_MspPostInit(&htim4);

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/** 
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {
	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;

	/* DMA interrupt init */
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, CE_Pin | CSN_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : CE_Pin CSN_Pin */
	GPIO_InitStruct.Pin = CE_Pin | CSN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PWM_RE_INT_Pin */
	GPIO_InitStruct.Pin = PWM_RE_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PWM_RE_INT_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void ARM_ESCs() {

	//HAL_Delay(2000); //Wait

	HAL_Delay(100);

	int delay_time = 10000;

	PWM1_Set(ESC_MAX); //Send max value (250us pulse)
	PWM2_Set(ESC_MAX); //Send max value (250us pulse)
	PWM3_Set(ESC_MAX); //Send max value (250us pulse)
	PWM4_Set(ESC_MAX); //Send max value (250us pulse)

	//Try setting a breakpoint here and switching on PWR supply here??
	//ESC needs a high signal switched on

	HAL_Delay(delay_time); //Wait

	PWM1_Set(ESC_MIN); //Send lowest value (125us pulse)
	PWM2_Set(ESC_MIN); //Send lowest value (125us pulse)
	PWM3_Set(ESC_MIN); //Send lowest value (125us pulse)
	PWM4_Set(ESC_MIN); //Send lowest value (125us pulse)

	HAL_Delay(delay_time); //Wait

}

//These 4 functions set the PWM duty cycles
void PWM1_Set(uint16_t value) {
	htim4.Instance->CCR1 = value;
}

void PWM2_Set(uint16_t value) {
	htim4.Instance->CCR2 = value;

}

void PWM3_Set(uint16_t value) {
	htim4.Instance->CCR3 = value;
}

void PWM4_Set(uint16_t value) {
	htim4.Instance->CCR4 = value;
}

//
void pulse_posedge_handler() {

	//Only want this to happen in main loop - not during init sequence
	if (main_loop) {

		tim3_count = htim3.Instance->CNT; //read TIM3 counter value
		calc_RollPitchYaw(tim3_count);

		//Pitch PID calculation
		imu_pitch = get_pitch();
		tim3_count = htim3.Instance->CNT;
		pid_output_pitch = pid_calculate_pitch(imu_pitch, tim3_count);
		//pid_output_pitch = arm_pid_f32(&pid_pitch_gains, imu_pitch);

		/*** For tuning PID, store in buffer to print to PC later ***/
#if PID_TUNE_DEBUG
		if (print_buffer_index < samples_to_take) {
			PID_print_buffer[print_buffer_index] = pid_output_pitch;
			IMU_print_buffer[print_buffer_index] = imu_pitch;

			print_buffer_index++;
		} else {
			printToPC();
		}
#endif

		//Calculate new pulse width values
		esc1_total = ESC_MIN + esc1_throttle + (int) pid_output_pitch;
		esc2_total = ESC_MIN + esc2_throttle + (int) pid_output_pitch;
		esc3_total = (ESC_MIN + esc3_throttle) - (int) pid_output_pitch;
		esc4_total = (ESC_MIN + esc4_throttle) - (int) pid_output_pitch;

		//Clip PWM values to make sure they don't go outside of range
		if (esc1_total < ESC_MIN) {
			esc1_total = ESC_MIN;
		}
		if (esc1_total > ESC_MAX) {
			esc1_total = ESC_MAX;
		}
		if (esc2_total < ESC_MIN) {
			esc2_total = ESC_MIN;
		}
		if (esc2_total > ESC_MAX) {
			esc2_total = ESC_MAX;
		}
		if (esc3_total < ESC_MIN) {
			esc3_total = ESC_MIN;
		}
		if (esc3_total > ESC_MAX) {
			esc3_total = ESC_MAX;
		}
		if (esc4_total < ESC_MIN) {
			esc4_total = ESC_MIN;
		}
		if (esc4_total > ESC_MAX) {
			esc4_total = ESC_MAX;
		}

		PWM1_Set(esc1_total); //PWM1 = Back left, CW
		PWM2_Set(esc2_total); //PWM2 = Front left, CCW
		PWM3_Set(esc3_total); //PWM3 = Back right, CCW
		PWM4_Set(esc4_total); //PWM4 = Front right, CW

	}

}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*
 *	Some functions to allow the program to use printf,
 *	from http://www.emcu.eu/how-to-implement-printf-for-send-message-via-usb-on-stm32-nucleo-boards-using-atollic/
 *
 */
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;

	HAL_UART_Transmit(&huart2, &*c, 1, 10);
	return ch;
}

int _write(int file, char *ptr, int len) {
	int DataIdx;
	for (DataIdx = 0; DataIdx < len; DataIdx++) {
		__io_putchar(*ptr++);
	}
	return len;
}

void printToPC() {
#if PID_TUNE_DEBUG
	//After n number of samples logged into buffer, print out to PC
	/* Set a breakpoint here */
	/** Print data to PC **/
	for (int i = 0; i < samples_to_take; ++i) {

		printf("%f,%f\r\n", PID_print_buffer[i],IMU_print_buffer[i]);

	}


	print_buffer_index = 0;
	/* Set another breakpoint here */

	reset_pid_pitch();
	arm_pid_init_f32(&pid_pitch_gains, 1);

#endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (GPIO_Pin == PWM_RE_INT_Pin && main_loop) {
		pulse_posedge_handler();
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
