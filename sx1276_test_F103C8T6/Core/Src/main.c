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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "LoRa.h"
#include "DS1307.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
LoRa myLoRa;
char message[100];
uint16_t LoRa_status;
uint8_t TxBuffer[16];
uint8_t RxBuffer[16];
char pckt_str[50];
char pckt_buffer[2];
int rssi;
float snr;
int flag = 0;

RTC_time_t mytime;

FATFS fs;
FIL fil;
FATFS *pfs;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char info[50];

int count = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == DIO0_Pin){
	  LoRa_receive(&myLoRa, RxBuffer, 16);
	  rssi = LoRa_getRSSI(&myLoRa);
	  snr = LoRa_getSNR(&myLoRa);
	  memset(pckt_str, NULL, strlen(pckt_str));
	  for(int i =0;i<16;i++)
	  {
		  if(RxBuffer[i] < 16) strcat(pckt_str, "0");
		  strcat(pckt_str, itoa(RxBuffer[i], pckt_buffer, 16));
		  strcat(pckt_str, " ");
	  }
	  sprintf(message, " Receive message: %s, RSSI = %d, SNR = %.2f\r\n", pckt_str, rssi, snr);
	  HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
	  memset(message, NULL, strlen(message));
	  itoa(rssi, TxBuffer, 10);
	  flag = 1;
	  //LoRa_transmit(&myLoRa, TxBuffer, 16, 500);
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
  MX_I2C1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
  myLoRa = newLoRa();

  myLoRa.CS_port         = NSS_GPIO_Port;
  myLoRa.CS_pin          = NSS_Pin;
  myLoRa.reset_port      = RST_GPIO_Port;
  myLoRa.reset_pin       = RST_Pin;
  myLoRa.DIO0_port       = DIO0_GPIO_Port;
  myLoRa.DIO0_pin        = DIO0_Pin;
  myLoRa.hSPIx           = &hspi1;

  myLoRa.frequency             = 922400;             // default = 433 MHz
  myLoRa.spredingFactor        = SF_8;           // default = SF_7
  myLoRa.bandWidth             = BW_250KHz;       // default = BW_125KHz
  myLoRa.crcRate               = CR_4_8;          // default = CR_4_5
  myLoRa.power                 = POWER_20db;      // default = 20db
  myLoRa.overCurrentProtection = 150;             // default = 100 mA

  uint16_t LoRa_status = LoRa_init(&myLoRa);
  sprintf(message, "LoRa initialization ..");
  HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
  memset(message, NULL, strlen(message));
  if(LoRa_status == LORA_OK){
	  sprintf(message, " success !\r\n");
	  HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
	  memset(message, NULL, strlen(message));
  }

  LoRa_startReceiving(&myLoRa);


  if(ds1307_init(&hi2c1)){
  		sprintf(message, "DS1307 initialization .. fail !\r\n");
  		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
  		memset(message, NULL, strlen(message));
  		while(1);
  	}
  	sprintf(message, "DS1307 initialization .. success !\r\n");
  	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
  	memset(message, NULL, strlen(message));
  	mytime.seconds = 0;
  	mytime.minutes = 0;
  	mytime.hours = 0;
  	mytime.time_format = TIME_FORMAT_24HRS;
  	ds1307_set_current_time(&mytime);

  	// ---SD CARD---
  	HAL_Delay(500);
	f_mount(&fs, "", 0);
	f_open(&fil, "write.txt", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

	HAL_Delay(500);
	f_lseek(&fil, fil.fsize);
	f_puts("Hello from Sun 250 12\n", &fil);
	f_close(&fil);

	f_getfree("", &fre_clust, &pfs);
	totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
	sprintf(info,"TotalSpace = %d\tFreeSpace = %d\r\n", totalSpace, freeSpace);
	HAL_UART_Transmit(&huart1, info, strlen(info), 0xffff);

	char buffer[100];
	f_open(&fil, "write.txt", FA_READ);
	while(f_gets(buffer, sizeof(buffer), &fil))
	{
	// Can use the buffer for something useful
	sprintf(info,"%s\r",buffer);
	HAL_UART_Transmit(&huart1, info, strlen(info), 0xffff);
	memset(buffer,0,sizeof(buffer));
	}
	f_close(&fil);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	if(flag == 1){
		count++;
		uint8_t checksum = 0;
		for(int i = 0;i < 15; i++)
		{
			checksum += RxBuffer[i];
		}
		sprintf(message,"Checksum = %02x, Answer = %02x %s \r\n", checksum, RxBuffer[15],
				(checksum == RxBuffer[15])?"OK !":"Fail !");
		HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
		memset(message ,NULL ,sizeof(message));

		sprintf(info,"%d,%d,%02d:%02d:%02d\n", count, (checksum == RxBuffer[15])?1:0, mytime.hours, mytime.minutes, mytime.seconds);
		HAL_UART_Transmit(&huart1, (uint8_t *)info, strlen(info), 0xffff);

		f_open(&fil, "BW250_SF12_test.csv", FA_OPEN_ALWAYS | FA_WRITE | FA_READ);

		f_lseek(&fil, fil.fsize);
		f_puts(info, &fil);
		f_close(&fil);

		char buffer[100];
		f_open(&fil, "BW250_SF12_test.csv", FA_READ);
		while(f_gets(buffer, sizeof(buffer), &fil))
		{
			// Can use the buffer for something useful
			sprintf(info,"%s\r",buffer);
			HAL_UART_Transmit(&huart1, info, strlen(info), 0xffff);
			memset(buffer,0,sizeof(buffer));
		}
		f_close(&fil);

		memset(info ,NULL ,sizeof(info));
		flag = 0;
	}

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);	//PC13 High
	HAL_Delay(500);// Delay
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//PC13 Low
	HAL_Delay(500);// Delay
	ds1307_get_current_time(&mytime);
	sprintf(message, "Current time: %02d:%02d:%02d\r\n",mytime.hours, mytime.minutes, mytime.seconds);
	HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), 0xffff);
	memset(message, NULL, strlen(message));
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, NSS_Pin|RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDCARD_CS_GPIO_Port, SDCARD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NSS_Pin RST_Pin SDCARD_CS_Pin */
  GPIO_InitStruct.Pin = NSS_Pin|RST_Pin|SDCARD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
