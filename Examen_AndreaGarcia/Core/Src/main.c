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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Demo includes. */
#include "supportingFunctions.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* Define the strings that will be passed in as the Supporting Functions parameters.
 * These are defined const and off the stack to ensure they remain valid when the
 * tasks are executing. */
const char *pcTextForMain     	= "Project: FZ - FreeRTOS\r\n";

const char *pcTaskParameters	= "  <=> Task Test - Parameters\r\n";

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
 * semaphore that is used to synchronize a task with other task. */
xSemaphoreHandle xBinarySemaphoreEntry;
xSemaphoreHandle xBinarySemaphoreExit;
xSemaphoreHandle xBinarySemaphoreContinue;

/* Declare a variable of type xSemaphoreHandle.  This is used to reference the
 * mutex type semaphore that is used to ensure mutual exclusive access to ........ */
xSemaphoreHandle xMutex;

/* Declare a variable of type QueueHandle_t */
QueueHandle_t xQueueVehicle;
QueueHandle_t xQueueVehicleDateTime;

/* Used to hold the Tasks handles. */
xTaskHandle vTaskTestHandle;
xTaskHandle vTaskMonitorHandle;
xTaskHandle vTaskAHandle;
xTaskHandle vTaskBHandle;

/* Task A & B Counter	*/
uint32_t	lTasksCnt;
defaultPlate = "PWR186";
defaultDateTime = "23/04/22 9:02";

/* Structures */
typedef struct
{
    char plate[6];				//config
    xTaskHandle * taskOn;	//variables
} tData;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
/* The three task functions. */
extern void vTaskA(void *pvParameters);
extern void vTaskB(void *pvParameters);
extern void vTaskTest(void *pvParameters);
extern void vTaskMonitor(void *pvParameters);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main( void )
{
	/* USER CODE BEGIN 1 */
    BaseType_t res;

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
	MX_USART3_UART_Init();

	/* USER CODE BEGIN 2 */
	/* Print out the name of this example. */
	vPrintString( pcTextForMain );

	/* USER CODE END 2 */

	/* Create the thread(s) */
	/* definition and creation of defaultTask */
	osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
	defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    /* Before a semaphore is used it must be explicitly created.  In this example
     * a binary semaphore is created. */
    vSemaphoreCreateBinary( xBinarySemaphoreEntry    );
    vSemaphoreCreateBinary( xBinarySemaphoreExit     );
    vSemaphoreCreateBinary( xBinarySemaphoreContinue );

    /* Before a semaphore is used it must be explicitly created.  In this example
     * a mutex type semaphore is created. */
    xMutex = xSemaphoreCreateMutex();

    /* Check the semaphore was created successfully. */
	configASSERT( xBinarySemaphoreEntry    !=  NULL );
	configASSERT( xBinarySemaphoreExit     !=  NULL );
	configASSERT( xBinarySemaphoreContinue !=  NULL );
	configASSERT( xMutex                   !=  NULL );

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	for (uint32_t i = 0; i<_Qty ; i++)
	{
		/* Task A thread at priority 1 */
		res = xTaskCreate( vTaskA,						/* Pointer to the function thats implement the task. */
				(const char *) "vTaskA",		/* Text name for the task. This is to facilitate debugging only. */
				configMINIMAL_STACK_SIZE,	/* Stack depth in words. 						*/
				(void *) i,				/* We are not using the task parameter.			*/
				(tskIDLE_PRIORITY + 1UL),	/* This task will run at priority 1. 			*/
				(xTaskHandle *) &vTaskAHandle );		/* We are not going to use the task handle. 	*/
		/* Check the task was created successfully. */
		configASSERT( res == pdPASS );

	}

    /* Task B thread at priority 1 */
    res = xTaskCreate( vTaskB,						/* Pointer to the function thats implement the task. */
    				   (const char *) "vTaskB",		/* Text name for the task. This is to facilitate debugging only. */
					   configMINIMAL_STACK_SIZE,	/* Stack depth in words. 						*/
					   (void *) NULL,				/* We are not using the task parameter.			*/
					   (tskIDLE_PRIORITY + 1UL),	/* This task will run at priority 1. 			*/
					   (xTaskHandle *) &vTaskBHandle	);		/* We are not going to use the task handle. 	*/
    /* Check the task was created successfully. */
    configASSERT( res == pdPASS );

	/* Task Test at priority 1, simply excites the other tasks */
    res = xTaskCreate( vTaskTest,					/* Pointer to the function thats implement the task. */
					   (const char *) "vTaskTest",	/* Text name for the task. This is to facilitate debugging only. */
					   configMINIMAL_STACK_SIZE,	/* Stack depth in words. 						*/
					   (void *) pcTaskParameters,	/* We are using text string as task parameter.	*/
					   (tskIDLE_PRIORITY + 1UL),	/* This task will run at priority 1. 			*/
					   (xTaskHandle *) &vTaskTestHandle );	/* We are using a variable as task handle.	*/
    /* Check the task was created successfully. */
    configASSERT( res == pdPASS );

    /* Task Monitor at priority 1, Adds DateTime to struct and saves it in a new one */
    res = xTaskCreate( vTaskMonitor,					/* Pointer to the function thats implement the task. */
    		(const char *) "vTaskMonitor",	/* Text name for the task. This is to facilitate debugging only. */
			configMINIMAL_STACK_SIZE,	/* Stack depth in words. 						*/
			(void *) pcTaskParameters,	/* We are using text string as task parameter.	*/
			(tskIDLE_PRIORITY + 1UL),	/* This task will run at priority 1. 			*/
			(xTaskHandle *) &vTaskMonitorHandle );	/* We are using a variable as task handle.	*/
    /* Check the task was created successfully. */
    configASSERT( res == pdPASS );

	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* If all is well we will never reach here as the scheduler will now be
	 * running.  If we do reach here then it is likely that there was insufficient
	 * heap available for the idle task to be created. */
	configASSERT( 0 );

	/* Should never arrive here */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

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

