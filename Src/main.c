/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * STM32F411RE Flight Controller Main file.
 * Nicholas Savva, 2019.
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

#include "../Drivers/PID.h"
#include "../Drivers/GPS.h"
#include <stdbool.h>
#include <stdio.h>
#include "arm_math.h"
#include "../Drivers/MY_NRF24.h"
#include "../Drivers/dwt_delay.h"
#include "../Drivers/IMU.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//Min and max counter load value for timer4, which handles PWM generation. ESC_MIN = 125us pulse
//and ESC_MAX = 250us pulse (OneShot125 protocol)
#define ESC_MIN 1250
#define ESC_MAX 2500
//How many of same pulse to send before updating with new value
#define PULSE_DIV 4
#define CRTL_LOOP_FREQ 500

//Value of ADC reading that corresponds to around 11.5V - will shut off to protect battery (3S)
#define ADC_BATTERY_SHUTOFF 3545

//Max angle on all axis
#define MAX_ANGLE 20

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

TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* USER CODE BEGIN PV */

int throttle = 0;

int esc1_total = 0;
int esc2_total = 0;
int esc3_total = 0;
int esc4_total = 0;

bool getRPY_flag = 0;

/* IMU reading variables */
float imu_roll;
float imu_pitch;
float imu_yaw;

/* PID */
float roll_setpoint = 0;
float pitch_setpoint = 0;
float yaw_setpoint = 0;
float pid_output_roll = 0;
float pid_output_pitch = 0;
float pid_output_yaw = 0;

int tim11_count = 0;
bool main_loop = 0;
int N = 0;

/* NRF24 Module */
uint64_t TxpipeAddrs = 0x11223344AA;
char RxData[32];

char AckPayload_0[32];
char AckPayload_1[32] = "Ack by Drone!";

//These hold the received joystick positions from the transmitter, left and right respectively
int16_t L_Joystick_XPos = 0;
int16_t L_Joystick_YPos = 1100;
int16_t R_Joystick_XPos;
int16_t R_Joystick_YPos;

uint32_t batteryLevel = 0;
uint16_t loop_counter = 0;
char airmode = 0;
char kill_rx = 0;
uint16_t lastAvgBatteryLevel;
int avgBatteryLevel = 0;

/** For debugging and tuning PID control, data storage buffer for
 *  printing afterwards to evaluate system response, can comment out later to save RAM
 *  if needed (approx 20kB needed?)
 */
#define PID_TUNE_DEBUG 0

#if PID_TUNE_DEBUG

#define SAMPLES 5000

float PID_print_buffer[SAMPLES];
float IMU_print_buffer[SAMPLES];
int print_buffer_index = 0;

#endif

//1 if motors to be used
#define MOTORS 1

//1 if NRF24 module connected
#define NRF24 0

//1 if using battery
#define BATTERY 1

//1 if using GPS module
#define GPS 0

//Enable ONLY 1 of the two below
//1 if using IMU (Kris winer MBED Library, fusion algorithm on MCU)
#define IMU 1

//1 if using MPU9250 with Ivense Motion Driver Library and DMP
#define IMU_DMP 0 //DMP offloads fusion

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM11_Init(void);
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

float map(int x, int in_min, int in_max, int out_min, int out_max);
int __io_putchar(int ch);
int _write(int file, char *ptr, int len);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void printToPC();

void unpackRxData();
void packAckPayData_0();
void packAckPayData_1();
void breakpoint();
void resetNRF24();
void lostConnection();
void kill();
unsigned int new_conversion_processing(unsigned int new_data);
uint16_t packetsLostCtr = 0;
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
	MX_USART2_UART_Init();
	MX_I2C2_Init();
	MX_TIM4_Init();
	MX_USART6_UART_Init();
	MX_ADC1_Init();
	MX_SPI2_Init();
	MX_TIM11_Init();
	/* USER CODE BEGIN 2 */

	/////////////////////////////////////////////////////////////////
	////////////////////// Init MPU9250 IMU /////////////////////////
	/////////////////////////////////////////////////////////////////
#if IMU

	//Start timer 11, used for integral calculations
	HAL_TIM_Base_Start(&htim11);


	if (imu_init(&hi2c2) == IMU_SUCCESS) {
		imu_calibrate();
	}

	for (int i = 0; i < 31; ++i) {
		AckPayload_0[i] = 0;
		AckPayload_1[i] = 0;

	}
#endif



	/////////////////////////////////////////////////////////////////
	/////////////////////////////// GPS /////////////////////////////
	/////////////////////////////////////////////////////////////////

#if GPS

	HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, 600);

	//Start timer 3 in interrupt mode, used for DMA timeout
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);

#endif

#if NRF24

	/////////////////////////////////////////////////////////////////
	////////////////////// Init NRF24L01 Module /////////////////////
	/////////////////////////////////////////////////////////////////

	DWT_Init(); //Enable some of the MCUs special registers so we can get microsecond (us) delays
	NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi2);
	nrf24_DebugUART_Init(huart6);
	NRF24_enableAckPayload();
	NRF24_setAutoAck(true);
	NRF24_openReadingPipe(1, TxpipeAddrs);
	NRF24_startListening();

	printRadioSettings();
#endif

	/////////////////////////////////////////////////////////////////
	////////////////////////// Init timers //////////////////////////
	/////////////////////////////////////////////////////////////////

//	HAL_TIM_Base_Start(&htim2);
//	HAL_ADC_Start_DMA(&hadc1, &batteryLevel, 1);

	//Start up PWMs
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	//ARM_ESCs(); //Force arming sequence

	PWM1_Set(2500);
	PWM2_Set(2500);
	PWM3_Set(2500);
	PWM4_Set(2500);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		main_loop = 1;

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
	hadc1.Init.ContinuousConvMode = ENABLE;
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
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
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
static void  (void) {

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
	hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
 * @brief TIM11 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM11_Init(void) {

	/* USER CODE BEGIN TIM11_Init 0 */

	/* USER CODE END TIM11_Init 0 */

	/* USER CODE BEGIN TIM11_Init 1 */

	/* USER CODE END TIM11_Init 1 */
	htim11.Instance = TIM11;
	htim11.Init.Prescaler = 100 - 1;
	htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim11.Init.Period = 65535 - 1;
	htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim11) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM11_Init 2 */

	/* USER CODE END TIM11_Init 2 */

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
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pin : PA5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : kill_Pin */
	GPIO_InitStruct.Pin = kill_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(kill_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PB12 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_14;
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
	HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

void ARM_ESCs() {

	HAL_Delay(2000); //Wait

	int delay_time = 8000;

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

int divider = 0;

/* ------------------------------ MAIN CONTROL LOOP --------------------------------------
 *
 * Calculate the required pulse widths to set individual motor speeds based off of IMU
 * readings and instructions from controller.
 *
 * For the QAV210 kit the orientation is as so:
 *
 * (2 CCW)    (4 CW)
 *		\	  /
 *		   |
 *		   |
 *		   |
 *		/     \
 * (1 CW)     (3 CCW)
 *
 * This function gets called by the GPIO_EXTI callback when the PWM_RE_INT_Pin triggers an interrupt,
 * which is on the rising edge of every PWM pulse.
 */
void pulse_posedge_handler() {

	//Only want this to happen in main loop - not during init sequence
	if (main_loop) {

#if NRF24
		//Pack acknowledge data 0 - sent every control loop
		packAckPayData_0();

		//Pack acknowledge data 1 - sent every second
		if (loop_counter == CRTL_LOOP_FREQ - 1) {
			packAckPayData_1();
			loop_counter = 0;
		} else {
			loop_counter++;
		}

		//	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);

		/* Get data from receiver */
		if (NRF24_available()) {
			NRF24_read(RxData, 32);

			packetsLostCtr = 0;

			//Write the acknowledge payload back to the transmitter/controller
			if (loop_counter == CRTL_LOOP_FREQ - 1) {
				NRF24_writeAckPayload(1, AckPayload_1, 32);

			} else {
				NRF24_writeAckPayload(1, AckPayload_0, 32);

			}
			//Unpack the 32 byte payload from controller
			unpackRxData();

			//Map throttle joystick reading to ESC range
			throttle = map(L_Joystick_YPos, 850, 3300, ESC_MIN, ESC_MAX);

			//Implement a deadzone for the bottom end of values
			if (throttle < ESC_MIN + 200) {
				throttle = ESC_MIN;
			}

			//Implement a deadzone for the top end of values
			if (throttle > ESC_MAX - 200) {
				throttle = ESC_MAX;
			}

			//Map right joystick X axis to roll set point
			roll_setpoint = map(R_Joystick_XPos, 340, 3960, -MAX_ANGLE, MAX_ANGLE);

			//Map right joystick Y axis to roll set point
			pitch_setpoint = map(R_Joystick_YPos, 350, 4000, -MAX_ANGLE, MAX_ANGLE);

		} else {
			packetsLostCtr++;
		}

		if (packetsLostCtr > 10) {
			lostConnection();
		}

#endif

#if IMU
		//Calculate roll, pitch & yaw using IMU readings
		tim11_count = htim11.Instance->CNT; //read TIM11 counter value, used for integral calculations

		int tim1 = tim11_count;

		calc_RollPitchYaw(tim11_count);


		tim11_count = htim11.Instance->CNT; //read TIM11 counter value, used for integral calculations
		int tim2 = tim11_count;

		volatile int deltat = tim2-tim1;


		imu_pitch = get_pitch();
		imu_roll = get_roll();
		imu_yaw = get_yaw();


		//Offset roll because IMU is upside down, comment out if not
		bool done = 0;
		if (imu_yaw > 0 && !done) {
			imu_yaw -= 180.0f;
			done = 1;
		}
		if (imu_yaw < 0 && !done) {
			imu_yaw += 180.0f;
			done = 1;
		}
#endif
		if (airmode) {
			/*******    Pitch PID calculation  ********/
			pid_output_pitch = pid_calculate_pitch(imu_pitch, 0, 0);

			/*******    Roll PID calculation  ********/

			pid_output_roll = pid_calculate_roll(imu_roll, 0, roll_setpoint);

			/*******    Yaw PID calculation  ********/

			pid_output_yaw = pid_calculate_yaw(imu_yaw, 0, yaw_setpoint);
		} else {
			pid_output_roll = 0;
			pid_output_pitch = 0;
			pid_output_yaw = 0;
			reset_pid_roll();
			reset_pid_pitch();
			reset_pid_yaw();
		}

		/*** For tuning PID, store in buffer to print to PC later ***/
#if PID_TUNE_DEBUG
		if (print_buffer_index < SAMPLES) {
			PID_print_buffer[print_buffer_index] = roll;
			IMU_print_buffer[print_buffer_index] = roll;

			print_buffer_index++;
		} else {
			printToPC();
		}
#endif

		//Calculate new pulse width values
		esc1_total = throttle - (int) pid_output_roll - (int) pid_output_pitch;
		esc2_total = throttle - (int) pid_output_roll + (int) pid_output_pitch;
		esc3_total = (throttle) + (int) pid_output_roll
				- (int) pid_output_pitch;
		esc4_total = (throttle) + (int) pid_output_roll
				+ (int) pid_output_pitch;

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
#if MOTORS
		//Load new pulse widths into ESCs
		PWM1_Set(esc1_total); //PWM1 = Back left, CW
		PWM2_Set(esc2_total); //PWM2 = Front left, CCW
		PWM3_Set(esc3_total); //PWM3 = Back right, CCW
		PWM4_Set(esc4_total); //PWM4 = Front right, CW

#endif

	}

}

/*
 *	Some functions to allow the program to use printf,
 *	from http://www.emcu.eu/how-to-implement-printf-for-send-message-via-usb-on-stm32-nucleo-boards-using-atollic/
 *
 */
int __io_putchar(int ch) {
	uint8_t c[1];
	c[0] = ch & 0x00FF;

	HAL_UART_Transmit(&huart6, &*c, 1, 10);
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
	for (int i = 0; i < SAMPLES; ++i) {

		printf("%f,%f\r\n", PID_print_buffer[i],IMU_print_buffer[i]);

	}


	print_buffer_index = 0;
	/* Set another breakpoint here */


#endif
}

//GPIO interrupt callback
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

//GPIO pin configured to capture rising edge interrupt of PWM signals
	if (GPIO_Pin == PWM_RE_INT_Pin && main_loop) {
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
		pulse_posedge_handler();
		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	}

	if (GPIO_Pin == kill_Pin && main_loop) {
		//kill();
	}

}

// Unpack received 32 byte payload from transmitter, see documentation for specification details
void unpackRxData() {

	L_Joystick_XPos = (RxData[0] & 0xFF) | (RxData[1] << 8);
	L_Joystick_YPos = (RxData[2] & 0xFF) | (RxData[3] << 8);
	R_Joystick_XPos = (RxData[4] & 0xFF) | (RxData[5] << 8);
	R_Joystick_YPos = (RxData[6] & 0xFF) | (RxData[7] << 8);

	airmode = (RxData[8] >> 0) & 1;

	kill_rx = (RxData[8] >> 1) & 1;

	if (kill_rx) {
		kill();
	}

	//Unpack PID data
//	uint16_t roll_p_rx = (RxData[9] & 0xFF) | (RxData[10] << 8);
//	uint16_t roll_i_rx = (RxData[11] & 0xFF) | (RxData[12] << 8);
//	uint16_t roll_d_rx = (RxData[13] & 0xFF) | (RxData[14] << 8);

//	//Remap
//	pitch_p_gain = (float) roll_p_rx / 100;
//	pitch_i_gain = (float) roll_i_rx / 100;
//	pitch_d_gain = (float) roll_d_rx / 100;

}

//	Pack acknowledge payload data 0 - sent every control loop, which will be sent back to controller once drone has successfully
//	received a payload from it
void packAckPayData_0() {

	//ID for packet 0
	AckPayload_0[0] = 0x00;

	//Next 2 bytes = Battery level
	AckPayload_0[1] = batteryLevel;
	AckPayload_0[2] = batteryLevel >> 8;

	//Next 4 bytes = IMU Roll
	int16_t roll_tx = round(imu_roll * 100);
	AckPayload_0[3] = roll_tx;
	AckPayload_0[4] = roll_tx >> 8;

	//Next  4 bytes = IMU Pitch
	int16_t pitch_tx = round(imu_pitch * 100);
	AckPayload_0[5] = pitch_tx;
	AckPayload_0[6] = pitch_tx >> 8;

	//Next  4 bytes = IMU Yaw
	int16_t yaw_tx = round(imu_yaw * 100);
	AckPayload_0[5] = yaw_tx;
	AckPayload_0[6] = yaw_tx >> 8;

}

//	Pack acknowledge payload data 1 - sent every second
void packAckPayData_1() {

	//ID for packet 1
	AckPayload_1[0] = 0xFF;

//	//Next few bytes are just 1 Byte values
//	AckPayload_1[1] = GPS.satellites;
//	AckPayload_1[2] = GPS.day;
//	AckPayload_1[3] = GPS.month;
//	AckPayload_1[4] = GPS.year;
//	AckPayload_1[5] = GPS.minute;
//	AckPayload_1[6] = GPS.hour;
//
//	//Next byte = GPS Speed
//	unsigned char temp[sizeof(float)];
//	memcpy(temp, &GPS.speed, sizeof(GPS.speed));
//
//	AckPayload_1[7] = temp[0];
//	AckPayload_1[8] = temp[1];
//	AckPayload_1[9] = temp[2];
//	AckPayload_1[10] = temp[3];
//
//	//Next byte = GPS Latitude
//	memcpy(temp, &GPS.latitude, sizeof(GPS.latitude));
//
//	AckPayload_1[11] = temp[0];
//	AckPayload_1[12] = temp[1];
//	AckPayload_1[13] = temp[2];
//	AckPayload_1[14] = temp[3];
//
//	//Next byte = GPS Longitude
//	memcpy(temp, &GPS.longitude, sizeof(GPS.longitude));
//
//	AckPayload_1[15] = temp[0];
//	AckPayload_1[16] = temp[1];
//	AckPayload_1[17] = temp[2];
//	AckPayload_1[18] = temp[3];
//
//	//Next byte = GPS Altitude
//	memcpy(temp, &GPS.altitude, sizeof(GPS.altitude));
//
//	AckPayload_1[19] = temp[0];
//	AckPayload_1[20] = temp[1];
//	AckPayload_1[21] = temp[2];
//	AckPayload_1[22] = temp[3];

}

#if GPS

////	Implementation of output compare elapsed callback for Timer 3.
////	As explained at top of code this will trigger when TIM3 OC has reached it's max counter value.
////	This is configured to happen each time the GPS module data has been fully sent (DMA timeout for UART 6)
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {

	//GPS Data UART Transmission complete
	if (htim->Instance == TIM3) {

		//Stop the DMA transfer as we know have reached end of message
		HAL_UART_AbortReceive_IT(&huart6);

		//Parse the received data
		parse_GPS_data();

		//Begin receiving again
		HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, 600);

		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
	}

}

#endif

void breakpoint() {

}

/*
 *  Kill function disables PWM outputs, turning off motors
 */
void kill() {
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
}

/*
 *  In case connection to transmitter is lost, slowly power down motors
 *  in order to gently lower quadcopter as opposed to suddenly shutting them off
 *
 */
void lostConnection() {

	airmode = 1;

	//Reset joystick positions to centre
	roll_setpoint = 0;
	pitch_setpoint = 0;

	//Decrease throttle down to min value
	if (throttle > ESC_MIN) {
		throttle--;
		NRF24_DelayMicroSeconds(5000);
	} else {
		//Then turn off motors fully to be sure
		kill();
	}

}

/*
 * Reset NRF24 module
 */
void resetNRF24() {

	NRF24_powerDown();
	DWT_Init(); //Enable some of the MCUs special registers so we can get microsecond (us) delays
	NRF24_begin(GPIOB, nrf_CSN_PIN, nrf_CE_PIN, hspi2);
	//nrf24_DebugUART_Init(huart2);
	NRF24_setAutoAck(true);
	NRF24_enableAckPayload();
	NRF24_openReadingPipe(1, TxpipeAddrs);
	NRF24_startListening();

}

#if BATTERY
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	batteryLevel = HAL_ADC_GetValue(&hadc1);

	//Simple exponential moving average filter where alpha = 2/(N+1)

	float alpha = 0.10;
	avgBatteryLevel = alpha * batteryLevel + (1 - alpha) * lastAvgBatteryLevel;
	lastAvgBatteryLevel = avgBatteryLevel;

	if (N > 20) {
		//Shut off PWM when battery level too low
		if (avgBatteryLevel < ADC_BATTERY_SHUTOFF) {
			kill();
		}
	} else {
		N++;
	}
}

#endif
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
