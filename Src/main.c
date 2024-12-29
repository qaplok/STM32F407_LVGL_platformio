/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "main.h"
#include "rng.h"
#include "sdio.h"
#include "spi.h"
#include "usb_device.h"
#include "gpio.h"
#include "fsmc.h"

#include "../lib/FreeRTOS/CMSIS_RTOS_V2/cmsis_os.h"

#include "usb_device.h"
#include "usb_device.h"

#include "string.h"
#include "usbd_cdc_if.h"
#include "../lib/FreeRTOS/include/task.h"
#include <stdio.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"
#include <stdbool.h>
#include "../lvgl/lvgl.h"
#include "lv_examples/lv_examples.h"
#include "ILI9341_Touchscreen.h"
#include "touchpad.h"
#include "XPT2046_lv.h"
#include "stm32f4xx_hal_rtc.h"



/* USER CODE END Includes */
/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */
typedef struct 							// Queue for UARD
{
	char Buf[1024];
}QUEUE_t;

typedef struct							// Queue for Touch LCD
{
	char buff[100];
}LCDQUEUE;

volatile unsigned long ulHighFreqebcyTimerTicks;		// This variable using for calculate how many time all tasks was running.
char str_management_memory_str[1000] = {0};
int freemem = 0;

uint32_t tim_val = 0;
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
//I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi1;
RTC_HandleTypeDef hrtc;
DMA_HandleTypeDef hdma_spi2_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim10;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for RTC */
osThreadId_t RTCHandle;
uint32_t Blue_LED_BlinkBuffer[ 500 ];
osStaticThreadDef_t Blue_LED_BlinkControlBlock;
const osThreadAttr_t RTC_attributes = {
  .name = "RTC",
  .cb_mem = &Blue_LED_BlinkControlBlock,
  .cb_size = sizeof(Blue_LED_BlinkControlBlock),
  .stack_mem = &Blue_LED_BlinkBuffer[0],
  .stack_size = sizeof(Blue_LED_BlinkBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Show_Resources */
osThreadId_t Show_ResourcesHandle;
uint32_t Show_ResourcesBuffer[ 500 ];
osStaticThreadDef_t Show_ResourcesControlBlock;
const osThreadAttr_t Show_Resources_attributes = {
  .name = "Show_Resources",
  .cb_mem = &Show_ResourcesControlBlock,
  .cb_size = sizeof(Show_ResourcesControlBlock),
  .stack_mem = &Show_ResourcesBuffer[0],
  .stack_size = sizeof(Show_ResourcesBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
uint32_t UART_TaskBuffer[ 500 ];
osStaticThreadDef_t UART_TaskControlBlock;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .cb_mem = &UART_TaskControlBlock,
  .cb_size = sizeof(UART_TaskControlBlock),
  .stack_mem = &UART_TaskBuffer[0],
  .stack_size = sizeof(UART_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for LCD */
osThreadId_t LCDHandle;
uint32_t LCDBuffer[ 1024 ];
osStaticThreadDef_t LCDControlBlock;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .cb_mem = &LCDControlBlock,
  .cb_size = sizeof(LCDControlBlock),
  .stack_mem = &LCDBuffer[0],
  .stack_size = sizeof(LCDBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LCD_touchscreen */
osThreadId_t LCD_touchscreenHandle;
uint32_t LCD_touchscreenBuffer[ 128 ];
osStaticThreadDef_t LCD_touchscreenControlBlock;
const osThreadAttr_t LCD_touchscreen_attributes = {
  .name = "LCD_touchscreen",
  .cb_mem = &LCD_touchscreenControlBlock,
  .cb_size = sizeof(LCD_touchscreenControlBlock),
  .stack_mem = &LCD_touchscreenBuffer[0],
  .stack_size = sizeof(LCD_touchscreenBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UARTQueue */
osMessageQueueId_t UARTQueueHandle;
uint8_t UARTQueueBuffer[ 10 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t UARTQueueControlBlock;
const osMessageQueueAttr_t UARTQueue_attributes = {
  .name = "UARTQueue",
  .cb_mem = &UARTQueueControlBlock,
  .cb_size = sizeof(UARTQueueControlBlock),
  .mq_mem = &UARTQueueBuffer,
  .mq_size = sizeof(UARTQueueBuffer)
};
/* Definitions for LCDQueue */
osMessageQueueId_t LCDQueueHandle;
uint8_t LCDQueueBuffer[ 1 * sizeof( LCDQUEUE ) ];
osStaticMessageQDef_t LCDQueueControlBlock;
const osMessageQueueAttr_t LCDQueue_attributes = {
  .name = "LCDQueue",
  .cb_mem = &LCDQueueControlBlock,
  .cb_size = sizeof(LCDQueueControlBlock),
  .mq_mem = &LCDQueueBuffer,
  .mq_size = sizeof(LCDQueueBuffer)
};
/* USER CODE BEGIN PV */
DMA_HandleTypeDef hdma_memtomem_dma2_stream0;

  extern int16_t last_x;
  extern int16_t last_y;
  extern uint8_t last_state;
  
  extern uint8_t isTouched;
  uint16_t x = 0, y = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void user_delay_ms(uint32_t period)
{
  HAL_Delay(period);
}

// Function for generate dalay more than 10 us (using for AM2302 T and H sensor)
bool delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim10, 0);
	tim_val = us/10;
	HAL_TIM_Base_Start_IT(&htim10);
	while(tim_val != 0)
	{

	}
	HAL_TIM_Base_Stop_IT(&htim10);
	tim_val = 0;
	int s = 99;
	return true;
}
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM10_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
static void MX_DMA_Init(void);

void StartDefaultTask(void *argument);
void Start_RTC(void *argument);
void Start_Show_Resources(void *argument);
void Start_UART_Task(void *argument);
void Start_SD_CARD(void *argument);
void Start_LCD(void *argument);
void Start_LCD_touchscreen(void *argument);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  MX_TIM3_Init();

  MX_TIM2_Init();
  MX_TIM10_Init();
  MX_SPI1_Init();

  MX_TIM1_Init();
  MX_RTC_Init();

  MX_DMA_Init();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_FSMC_Init();
  MX_RNG_Init();
  MX_SDIO_SD_Init();
  MX_SPI2_Init();
  MX_USB_DEVICE_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);		//  This TIM3 using for calculate how many time all tasks was running.

  HAL_TIM_Base_Start_IT(&htim1);			// Blink Green LED

  
  lv_touchpad_init();
  HAL_Delay(100);
  //lv_demo_benchmark();
  //lv_demo_widgets();
  lv_ex_cpicker_2();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  
  /* Init scheduler */
  osKernelInitialize();
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of UARTQueue */
  UARTQueueHandle = osMessageQueueNew (10, sizeof(QUEUE_t), &UARTQueue_attributes);

  /* creation of LCDQueue */
  LCDQueueHandle = osMessageQueueNew (1, sizeof(LCDQUEUE), &LCDQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RTC */
  RTCHandle = osThreadNew(Start_RTC, NULL, &RTC_attributes);

  /* creation of Show_Resources */
  Show_ResourcesHandle = osThreadNew(Start_Show_Resources, NULL, &Show_Resources_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(Start_UART_Task, NULL, &UART_Task_attributes);


  /* creation of LCD */
  LCDHandle = osThreadNew(Start_LCD, NULL, &LCD_attributes);

  /* creation of LCD_touchscreen */
  LCD_touchscreenHandle = osThreadNew(Start_LCD_touchscreen, NULL, &LCD_touchscreen_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  while (1)
  {
    
   
//lcd_random_circles();
//lcd_random_lines();
//lcd_random_points();
//lcd_fill_rand_colors();
//lcd_random_rectangles();
//ILI9341_Draw_Filled_Circle(last_x + 20, 120 - last_y - 2, 4, RED);
//ILI9341_Draw_Filled_Circle(120, 160, 4, RED);
//lv_task_handler();
//HAL_Delay(5);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x23;
  sTime.Minutes = 0x59;
  sTime.Seconds = 0x45;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY;
  sDate.Month = RTC_MONTH_DECEMBER;
  sDate.Date = 0x28;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x10;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_WEEKDAY;
  sAlarm.AlarmDateWeekDay = RTC_WEEKDAY_MONDAY;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/* USER CODE BEGIN Header_Start_SD_CARD */
/**
* @brief Function implementing the SD_CARD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SD_CARD */
void Start_SD_CARD(void *argument)
{
  /* USER CODE BEGIN Start_SD_CARD */
  /* Infinite loop */

	Mount_SD("/");

	Create_File("test_data_1.txt");
	Update_File("test_data_1.txt","\n\rStart recording\r\n");	// Add data to the end of file

	// Create folders
	Create_Dir("test_folder_1");
	Create_Dir("test_folder_2");
	Create_Dir("test_folder_3");

	Unmount_SD("/");

	static int i = 0;											// Test data for write

  for(;;)
  {
	  // Log data ewery one second
	  osDelay(1000);
	  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_SET);			// LED ON

	  Mount_SD("/");

	  char data[10] = {0};
	  sprintf(data, "%d\n", i);
	  Update_File("test_data_1.txt", data);						// Add data to the end of file
	  i++;

	  Unmount_SD("/");

	  HAL_GPIO_WritePin(GPIOD, LD3_Pin, GPIO_PIN_RESET);		// LED OFF
  }
  /* USER CODE END Start_SD_CARD */
}

/* USER CODE BEGIN Header_Start_LCD */
/**
* @brief Function implementing the LCD thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD */
void Start_LCD(void *argument)
{
  /* USER CODE BEGIN Start_LCD */
  /* Infinite loop */
	LCDQUEUE msg;												// Make QUEUE

	// Init LCD
	lv_init();
  ILI9341_Init();
  ILI9341_Set_Rotation(1);

	for(;;)
	{
		char str_buf[60] = {0};
		// osMessageQueueGet waiting data on a queue (If data are in queue so print it)
		osMessageQueueGet(LCDQueueHandle, &msg, 0, osWaitForever);			// Write for data on queue
		strcat(str_buf, msg.buff);											// Copy data in buffer for print
		ILI9341_Draw_Text(str_buf,30,30, 10, 10, 13);										// Print data on LCD
		memset(str_buf, 0, sizeof(str_buf));

		osDelay(200);

//	  // LCD speed test
//	  speed_test();
//	  TFT9341_FillScreen(TFT9341_BLACK);
//	  osDelay(5000);
//	  //
  }
  /* USER CODE END Start_LCD */
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16800-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 10000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 10;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 839;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

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
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_RTC */
/**
* @brief Function implementing the RTC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RTC */
void Start_RTC(void *argument)
{
  /* USER CODE BEGIN Start_RTC */
  /* Infinite loop */
	// Task every seconds blink blue LED and send data in virtual com port.
		// Set up RTC
		/*	README <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
		 * STM32IDE  automatically generated setup code and fill in RTC structure fields seconds, minutes,
		 * hours and date. So, for avoid rewriting time, you need comment automated generated code.
		 *
		 * RTC needs BATTERY for count time when external power will be removed.
		 * For STM32F407 discovery dev board needs remove R26, and connect battery to VBAT (near R26).
		 * Also, need solder the LF Crystal and two capacitors.
		 */

		// 1. Set time
		  RTC_TimeTypeDef sTime = {0};
		  sTime.Hours = 9;
		  sTime.Minutes = 33;
	    sTime.Seconds = 00;
		  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		  // Set date

		  RTC_DateTypeDef sDate = {0};
		  sDate.Date = 29;
		  sDate.Month = RTC_MONTH_DECEMBER;
		  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
		  sDate.Year = 21;
		  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		  /////////////////////////////////////////////////////////////////////

	QUEUE_t msg;												// Make a queue

	char buff[50] = {0};
	char buf[5] = {0};
	char str_end_of_line[4] = {'\r','\n','\0'};

	static uint8_t i = 1;
	for(;;)
	{
		// Blue LED blink//
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_SET);
		osDelay(100);
		HAL_GPIO_WritePin(GPIOD, LD6_Pin, GPIO_PIN_RESET);
		osDelay(900);

		// RTC part
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);						// Get time (write in sDime struct)
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);						// Get data (write in sDime struct)

		memset(msg.Buf, 0, sizeof(msg.Buf));								// Fill in buff '\0'
		memset(buff, 0, sizeof(buff));

		strcat(msg.Buf, "RTC DATA AND TIME >>>>>>>    " );

		// Date
		itoa(sDate.Year, buf, 10);
		strcat(msg.Buf, buf);

		itoa(sDate.Month, buf, 10);
		strcat(msg.Buf, "-");
		strcat(msg.Buf, buf);

		itoa(sDate.Date, buf, 10);
		strcat(msg.Buf, "-");
		strcat(msg.Buf, buf);

		strcat(msg.Buf, " | ");

		// Time
		itoa(sTime.Hours, buf, 10);
		strcat(msg.Buf, buf);

		itoa(sTime.Minutes, buf, 10);
		strcat(msg.Buf, ":");
		strcat(msg.Buf, buf);

		itoa(sTime.Seconds, buf, 10);
		strcat(msg.Buf, ":");
		strcat(msg.Buf, buf);

		strcat(msg.Buf, str_end_of_line);
		osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);					// Write data on queue (In will print on StartUART_Task task)
	}
  /* USER CODE END Start_RTC */
}

/* USER CODE BEGIN Header_Start_Show_Resources */
/**
* @brief Function implementing the Show_Resources thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Show_Resources */
void Start_Show_Resources(void *argument)
{
  /* USER CODE BEGIN Start_Show_Resources */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(5000);												// Every 5 second task management will print data

	  char str_end_of_line[3] = {'\r','\n'};
	  char str_sig = '-';
	  char buff[10] = {0};

	  QUEUE_t msg;												// Make a queue
	  memset(msg.Buf, 0, sizeof(msg.Buf));						// Fill in buff '\0'
	  strcat(msg.Buf, ">>>>> Free heap memory: ");				// Add string to another (Total heap)

	  freemem = xPortGetFreeHeapSize();							// Function return how many free memory.
	  itoa(freemem, buff, 10);
	  strcat(msg.Buf, buff);
	  strcat(msg.Buf, str_end_of_line);

	  // add a hat
	  strcat(msg.Buf, "| TASK NAME           | STATUS |   PRIOR	|  STACK  |    NUM  |\n\r\0");

	  vTaskList(str_management_memory_str);						// Fill in str_management_memory_str array management task information

	  // Finding the  end of string
	  uint16_t buffer_size = 0;
	  while(msg.Buf[buffer_size] != '\0')
	  {
	  	buffer_size ++;
	  }

	  // Add str_management_memory_str to queue string
	  int i = 0;
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }

	  // add a hat
	  char str_line[] = {"-----------------------\n\r"};
	  char str_head_2[] = {"| TASK NAME           | ABS TIME |              TASK TIME% |\n\r"};
	  strcat(msg.Buf, str_line);
	  strcat(msg.Buf, str_head_2);

	  memset(str_management_memory_str, 0, sizeof(str_management_memory_str));	// Clean buffer

	  vTaskGetRunTimeStats(str_management_memory_str);							// Function return how much time all functions running.

	  buffer_size = buffer_size + i + (sizeof(str_line)-1) + (sizeof(str_head_2)-1);           // НЕ ВИВОДИТЬ СТРОКУ !!!!!!!!!!!!!!!!!! <<<<<<<<<<<<<<<<<<<
	  for(i = 0; str_management_memory_str[i] != '\0'; i++)
	  {
	  	// add data to queue
	  	msg.Buf[buffer_size + i] = str_management_memory_str[i];
	  }
	  strcat(msg.Buf, "#########################################\n\r");

	  osMessageQueuePut(UARTQueueHandle, &msg, 0, osWaitForever);					// Write data on queue (In will print on StartUART_Task task)
  }
  /* USER CODE END Start_Show_Resources */
}

/* USER CODE BEGIN Header_Start_UART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_UART_Task */
void Start_UART_Task(void *argument)
{
  /* USER CODE BEGIN Start_UART_Task */
  /* Infinite loop */
  QUEUE_t msg;
  for(;;)
  {
	// osMessageQueueGet waiting data on a queue (If data are in queue so print it)
	osMessageQueueGet(UARTQueueHandle, &msg, 0, osWaitForever);			// Write for data on queue
	// Counting how many characters will be transmitted
	uint16_t buffer_size = 0;
	while(msg.Buf[buffer_size] != '\0')
	{
		buffer_size ++;
	}
	// Transmit over virtual comport
	CDC_Transmit_FS(msg.Buf, buffer_size);						// Transmit data over virtual comport
    osDelay(1);
  }
  /* USER CODE END Start_UART_Task */
}
//---------------------------------------------------------------------------------------------------------------------
/* USER CODE BEGIN 4 */


/* USER CODE BEGIN Header_Start_LCD_touchscreen */
/**
* @brief Function implementing the LCD_touchscreen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD_touchscreen */
void Start_LCD_touchscreen(void *argument)
{
  /* USER CODE BEGIN Start_LCD_touchscreen */
  /* Infinite loop */
	LCDQUEUE msg;												// Make QUEUE
	memset(msg.buff, 0, sizeof(msg.buff));						// Fill in buff '\0'
	char buffer[50] = {0};

	for(;;)
  	 {
	  memset(msg.buff, 0, sizeof(msg.buff));						// Fill in buff '\0'
	  //СЕНСОР ЕКРАНУ
	  if(TP_Touchpad_Pressed() == TOUCHPAD_PRESSED)
	  {
		  strcat(buffer, "PRESED ");

		  uint16_t x_and_y[2] = {0};
		  uint8_t status_ts = TP_Read_Coordinates(x_and_y);
		  if(status_ts == TOUCHPAD_DATA_OK)
		  {
			  // Convert coordinate from uint16_t format in string format
			  // And save it in main buffer
			  char buff_x_coordinates[6] = {0};
			  char buff_y_coordinates[6] = {0};
			  char buff_coordinates[15] = {0};

			  strcat(buff_x_coordinates, "x: ");
			  itoa(x_and_y[0], buff_x_coordinates, 10);
			  strcat(buff_x_coordinates, " ");

			  strcat(buff_y_coordinates, "y: ");
			  itoa(x_and_y[1], buff_y_coordinates, 10);
			  strcat(buff_y_coordinates, " ");

			  strcat(buff_coordinates, buff_x_coordinates);
			  strcat(buff_coordinates, buff_y_coordinates);
			  strcat(buffer, buff_coordinates);
		  }
	  }
	  else
	  {
		  strcat(buffer, "NO PRESS                  ");
	  }

	  strcat(msg.buff, buffer);
	  osMessageQueuePut(LCDQueueHandle, &msg, 0, osWaitForever);  	// Write data on queue (In will print on StartUART_Task task)
	  memset(buffer, 0, sizeof(buffer));

	  osDelay(200);
    //osDelay(1);
  }
  /* USER CODE END Start_LCD_touchscreen */
}

//void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
//  switch(GPIO_Pin)
//  {
//    case TS_IRQ_Pin:
  //       if(xpt2046_getXY(&x, &y))
   //      {
           //lv_task_handler();                   
    //     }
    //     lv_task_handler();
    //break;
//    default:
  //  __NOP();
//  }
//}

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
