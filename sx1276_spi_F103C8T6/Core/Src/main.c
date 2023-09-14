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
#include <string.h>
#include <stdio.h>
#include "LoRa.h"
#include "DHT22.h"
#include "NMEA.h"
#include "uartRingBuffer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
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
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
/* USER CODE BEGIN PV */
xSemaphoreHandle DHT_Sem;
xSemaphoreHandle LoRa_Sem;
//--------LoRa--------
LoRa myLoRa;
uint16_t LoRa_stat;
uint8_t TxBuffer[24];
char message[50];
char pckt_str[50];
char pckt_buffer[2];

//-------Environment--------
float Temperature = 0;
float Humidity = 0;

//--------GPSR--------
char GGA[100];
char RMC[100];
GPSSTRUCT gpsData;
char lcdBuffer[50];
int VCCTimeout;
int flagGGA, flagRMC; // if you set this variable (1->0 and 2->1), it will crash.
float fix_latitude, fix_longitude;
float lat_afterPoint, lon_afterPoint;

//------BlueTooth------
uint8_t rx_data;

//------RTC------
RTC_TimeTypeDef gTime;
RTC_DateTypeDef gDate;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
void LEDTask(void const * argument);
void StartTask02(void const * argument);
void StartTask03(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void float2Bytes(float val,uint8_t* bytes_array);
void LORA_Task(void* pvParameter);
void DHT22_Task(void* pvParameter);
void GPSR_Task(void* pvParameter);
void BLE_Task(void* pvParameter);

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart3)
	{
		if(rx_data == 'R')
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)"Reset GST !\n", 12, 0xffff);
		}
		if(rx_data == 'A')
		{
			HAL_UART_Transmit(&huart3, (uint8_t *)"Packet: ", 8, 0xffff);
			HAL_UART_Transmit(&huart3, (uint8_t *)TxBuffer, strlen(TxBuffer), 0xffff);
		}

	}
	HAL_UART_Receive_IT(&huart3, &rx_data, 1);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==RF_BTN_Pin)
  {
		for(int i = 0;i<3;i++){ delay(60000);}

		if(HAL_GPIO_ReadPin(RF_BTN_GPIO_Port, RF_BTN_Pin) == GPIO_PIN_RESET){
			HAL_GPIO_WritePin(OUTPUT_BTN_GPIO_Port, OUTPUT_BTN_Pin, GPIO_PIN_SET);
			sprintf(message, "Button Interrupt\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 0xffff);
			memset(message, NULL, strlen(message));
			BaseType_t xHigherPriorityTaskWoken = pdFALSE;
			xSemaphoreGiveFromISR (LoRa_Sem, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}

  }

}

void float2Bytes(float val,uint8_t* bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overwrite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}

void LORA_Task(void* pvParameter){

	while(1){
		if(xSemaphoreTake(LoRa_Sem, 9700) != pdTRUE){
			char *info = pvPortMalloc(50*sizeof(char));
			sprintf(info, "Unable to acquire LoRa semaphore !!!\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 0xffff);
			vPortFree(info);
		}
		else{
			char *info = pvPortMalloc(50*sizeof(char));
			sprintf(info, "Button push\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)info, strlen(info), 0xffff);
			vPortFree(info);
//			TxBuffer[0] = 0x02; // mobile mode
//			TxBuffer[1] = 0x01; // ID
//			TxBuffer[2] = 0x21; // Data Type:

//			TxBuffer[3] = 0x00;	// 3-6 Time
//			TxBuffer[4] = gTime.Hours;
//			TxBuffer[5] = gTime.Minutes;
//			TxBuffer[6] = gTime.Seconds;
//			float2Bytes(Temperature, &TxBuffer[7]); // 7-14 Sensor Data
//			float2Bytes(Humidity, &TxBuffer[11]);
//			float2Bytes(fix_latitude, &TxBuffer[15]); // 15-22 GPS position Data
//			float2Bytes(fix_longitude, &TxBuffer[19]);

//			TxBuffer[23] = 0x00;// 23 Checksum
//			for(int i = 0;i < 23; i++){
//				TxBuffer[23] = TxBuffer[23] + TxBuffer[i];
//			}

			// make packet string, fail to make the string to show. I think the reason is
			// that strcat function is not compatible to FreeRTOS or HAL_UART..
	//		memset(pckt_str, NULL, strlen(pckt_str));
	//		for(int i = 0;i < 24; i++)
	//		{
	//			if (TxBuffer[i] < 16) strcat(pckt_str, "0");
	//			strcat(pckt_str, itoa(TxBuffer[i], pckt_buffer, 16));
	//			strcat(pckt_str, " ");
	//		}
			sprintf(TxBuffer, "HelloHelloHello");
			HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);	//PC13 High
			LoRa_transmit(&myLoRa, TxBuffer, 24, 2500);
			info = pvPortMalloc(100*sizeof(char));
			sprintf(info, "Packet size: %d ,Send packet.. \r\n", sizeof(TxBuffer)/sizeof(TxBuffer[0]));
			HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 0xffff);
			vPortFree(info);
			HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET);	//PC13 Low

			HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
			HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

			info = pvPortMalloc(50*sizeof(char));
			sprintf(info,"RTC time: %02d:%02d:%02d \r\n", gTime.Hours, gTime.Minutes, gTime.Seconds);
			HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 0xffff);
			vPortFree(info);

			info = pvPortMalloc(50*sizeof(char));
			sprintf(info,"RTC date: %02d-%02d-%04d \r\n", gDate.Month, gDate.Date, 2000+gDate.Year);
			HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 0xffff);
			vPortFree(info);

			vTaskDelay(500);
			HAL_GPIO_WritePin(OUTPUT_BTN_GPIO_Port, OUTPUT_BTN_Pin, GPIO_PIN_RESET);
		}

	}
}
void DHT22_Task(void* pvParameter){
	int index = 1;
	while(1){
		if(xSemaphoreTake(DHT_Sem, 2500) != pdTRUE){
			HAL_UART_Transmit(&huart1, (uint8_t*)"Unable to acquire DHT semaphore\r\n", 35, 100);
		}
		else{
			if(DHT22_Get_Data(&Temperature, &Humidity) == 1){
				char *info = pvPortMalloc(50*sizeof(char));
				sprintf (info, "%d. Temp = %.2f C\t RH = %.2f%% \r\n",index++, Temperature, Humidity);
				HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 0xffff);
				vPortFree(info);
			}
		}
	}
}

void GPSR_Task(void* pvParameter){
	while(1){
		if (Wait_for("GGA") == 1){

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the GGA is being received

			Copy_upto("*", GGA);
			if(decodeGGA(GGA, &gpsData.ggastruct) == 0) flagGGA = 2; // 2 indicates the data is valid
			else flagGGA = 1;
		}
		if (Wait_for("RMC") == 1){

			VCCTimeout = 5000; // Reset the VCC Timeout indicating the RMC is being received

			Copy_upto("*", RMC);
			if(decodeRMC(RMC, &gpsData.rmcstruct) == 0) flagRMC = 2; // 2 indicates the data is valid
			else flagRMC = 1;
		}
		if ((flagGGA = 2) | (flagRMC = 2)){
			// print the time format
			sprintf(lcdBuffer, "%02d:%02d:%02d, %02d-%02d-%04d\r\n", gpsData.ggastruct.tim.hour, \
				  gpsData.ggastruct.tim.min, gpsData.ggastruct.tim.sec, gpsData.rmcstruct.date.Mon, \
				  gpsData.rmcstruct.date.Day, 2000 + gpsData.rmcstruct.date.Yr);
			HAL_UART_Transmit(&huart1, (uint8_t *)lcdBuffer, strlen(lcdBuffer), 0xffff);
			memset(lcdBuffer, '\0', strlen(lcdBuffer));
			// Convert to google map format
			lat_afterPoint = gpsData.ggastruct.lcation.latitude - (int)(gpsData.ggastruct.lcation.latitude);
			fix_latitude = (int)(gpsData.ggastruct.lcation.latitude) + lat_afterPoint/60*100;
			lon_afterPoint = gpsData.ggastruct.lcation.longitude - (int)(gpsData.ggastruct.lcation.longitude);
			fix_longitude = (int)(gpsData.ggastruct.lcation.longitude) + lon_afterPoint/60*100;
			// print the location format
			sprintf(lcdBuffer, "LAT: %.4f%c, LON: %.4f%c\r\n", fix_latitude, \
				  gpsData.ggastruct.lcation.NS, fix_longitude, gpsData.ggastruct.lcation.EW);
			HAL_UART_Transmit(&huart1, (uint8_t *)lcdBuffer, strlen(lcdBuffer), 0xffff);
			memset(lcdBuffer, '\0', strlen(lcdBuffer));
			// Correct the RTC time
		}
		else if((flagGGA = 1) | (flagRMC = 1)){
			HAL_UART_Transmit(&huart1, (uint8_t *)"NO FIX YET !!\r\n", 20, 0xffff);
		}
		if (VCCTimeout <= 0){
			VCCTimeout = 5000;  // Reset the timeout

			//reset flags
			flagGGA =flagRMC =0;

			// You are here means the VCC is less, or maybe there is some connection issue
			// Check the VCC, also you can try connecting to the external 5V
			HAL_UART_Transmit(&huart1, (uint8_t *)"VCC Issue, Check Connection\r\n", 20, 0xffff);
		}
		vTaskDelay(2000);
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	// --LoRa initialization--
	myLoRa = newLoRa();

	myLoRa.CS_port         = NSS_GPIO_Port;
	myLoRa.CS_pin          = NSS_Pin;
	myLoRa.reset_port      = RST_GPIO_Port;
	myLoRa.reset_pin       = RST_Pin;
	myLoRa.DIO0_port       = DIO0_GPIO_Port;
	myLoRa.DIO0_pin        = DIO0_Pin;
	myLoRa.hSPIx           = &hspi1;

	myLoRa.frequency             = 924400;          // default = 433 MHz
	myLoRa.spredingFactor        = SF_12;           	// default = SF_7
	myLoRa.bandWidth             = BW_125KHz;       // default = BW_125KHz
	myLoRa.crcRate               = CR_4_5;          // default = CR_4_5
	myLoRa.power                 = POWER_20db;      // default = 20db
	myLoRa.overCurrentProtection = 130;             // default = 100 mA
	//myLoRa.preamble = 10;

	uint16_t LoRa_status = LoRa_init(&myLoRa);
	sprintf(message, "LoRa initialization ..");
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
	memset(message, NULL, strlen(message));
	if(LoRa_status == LORA_OK){
	  sprintf(message, " success !\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
	  memset(message, NULL, strlen(message));
	}

	LoRa_setSyncWord(&myLoRa, 0x34); // default: 0x12

	// --DS1307 initialization--
//	if(ds1307_init(&hi2c1)){
//		sprintf(message, "DS1307 initialization .. fail !\r\n");
//		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
//		memset(message, NULL, strlen(message));
//		while(1);
//	}
//	sprintf(message, "DS1307 initialization .. success !\r\n");
//	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
//	memset(message, NULL, strlen(message));
//	mytime.seconds = 0;
//	mytime.minutes = 0;
//	mytime.hours = 0;
//	ds1307_set_current_time(&mytime);

	// --GPSR--
	Ringbuf_init();

	DHT_Sem = xSemaphoreCreateBinary();
	LoRa_Sem = xSemaphoreCreateBinary();

	xTaskCreate(LORA_Task, "LORA", 1024, NULL, 2, NULL);
	xTaskCreate(DHT22_Task, "DHT22", 256, NULL, 3, NULL);
	xTaskCreate(GPSR_Task, "GPSR", 256, NULL, 2, NULL);

	HAL_TIM_Base_Start(&htim4);
	HAL_TIM_Base_Start_IT(&htim1);

	HAL_UART_Receive_IT(&huart3,&rx_data,1);

	vTaskStartScheduler();
  /* USER CODE END 2 */

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
  /* definition and creation of LED */

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x11;
  sTime.Minutes = 0x22;
  sTime.Seconds = 0x33;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JULY;
  DateToUpdate.Date = 0x18;
  DateToUpdate.Year = 0x23;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  htim1.Init.Prescaler = 36000-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4000-1;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0xffff-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  huart3.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUTPUT_BTN_GPIO_Port, OUTPUT_BTN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : GREEN_Pin */
  GPIO_InitStruct.Pin = GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GREEN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin OUTPUT_BTN_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|OUTPUT_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RF_BTN_Pin */
  GPIO_InitStruct.Pin = RF_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(RF_BTN_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_LEDTask */
/**
  * @brief  Function implementing the LED thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_LEDTask */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if (htim->Instance == TIM1){
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(DHT_Sem, &xHigherPriorityTaskWoken);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
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
