/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

#include "../Drivers/GPS.h"
#include  "controlLoop.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




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

//////////////////////////////////// Peripheral defines  //////////////////////////////

//1 if motors to be used
#define MOTORS 0


//1 if using battery
#define BATTERY 0

//1 if using GPS module
#define GPS 0

//Enable ONLY 1 of the two below
//1 if using IMU (Kris winer MBED Library, fusion algorithm on MCU)
#define IMU 1

#define UART_DEBUG 0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */



/* USER CODE END Variables */
osThreadId ControlLoopHandle;
osThreadId GPSUpdateHandle;
osMutexId GPSDataMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */


/* USER CODE END FunctionPrototypes */

void StartControlLoop(void const * argument);
void StartGPSUpdate(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
		StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize) {
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
	*ppxIdleTaskStackBuffer = &xIdleStack[0];
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
	/* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of GPSDataMutex */
  osMutexDef(GPSDataMutex);
  GPSDataMutexHandle = osMutexCreate(osMutex(GPSDataMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of ControlLoop */
  osThreadDef(ControlLoop, StartControlLoop, osPriorityRealtime, 0, 1024);
  ControlLoopHandle = osThreadCreate(osThread(ControlLoop), NULL);

  /* definition and creation of GPSUpdate */
  osThreadDef(GPSUpdate, StartGPSUpdate, osPriorityIdle, 0, 128);
  GPSUpdateHandle = osThreadCreate(osThread(GPSUpdate), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartControlLoop */
/**
 * @brief  Function implementing the ControlLoop thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartControlLoop */
void StartControlLoop(void const * argument)
{
    
    
    

  /* USER CODE BEGIN StartControlLoop */

	CL_init();

	/* Infinite loop */
	for (;;) {

		CL_main();
		osDelay(2);
	}
  /* USER CODE END StartControlLoop */
}

/* USER CODE BEGIN Header_StartGPSUpdate */
/**
 * @brief Function implementing the GPSUpdate thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartGPSUpdate */
void StartGPSUpdate(void const * argument)
{
  /* USER CODE BEGIN StartGPSUpdate */

#if GPS

	GPS_init();

#endif



	/* Infinite loop */
	for (;;) {
		osDelay(10);
	}
  /* USER CODE END StartGPSUpdate */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */












#if UART_DEBUG

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
#endif
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

//////GPIO interrupt callback
//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
//
//	//GPIO pin configured to capture rising edge interrupt of PWM signals
//	if (GPIO_Pin == PWM_INT_Pin && main_loop) {
//		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//
//		//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//	}
//
//	//	if (GPIO_Pin == kill_Pin && main_loop) {
//	//		//kill();
//	//	}
//
//}



#if GPS

//////	Implementation of output compare elapsed callback for Timer 3.
//////	As explained at top of code this will trigger when TIM3 OC has reached it's max counter value.
//////	This is configured to happen each time the GPS module data has been fully sent (DMA timeout for UART 6)
//void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
//
//		//GPS Data UART Transmission complete
//		if (htim->Instance == TIM3) {
//
//			//Stop the DMA transfer as we know have reached end of message
//			HAL_UART_DMAStop(&huart6);
//
//			//First buffer received will probably be somewhere in the middle of a message so discard it
//			if(discard){
//				discard = 0;
//			}else{
//				//Parse the received data
//				GPS_parse_data();
//			}
//
//			//Begin receiving again
//			HAL_UART_Receive_DMA(&huart6, GPS_RX_Buffer, 600);
//
//			//HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_3);
//		}
//
//}


#endif


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
