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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 128	//Buffer para la lectura de memoria SD
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
uint8_t buttonPinFlag;		//Bandera para la interrupción externa.

FATFS fs;  					//Variable de tipo archivo de sistema
FIL fil;					//Variable de tipo archivo
FILINFO fno;				//Variable de tipo información de archivo
FRESULT fresult;  			//Variable para almacenar resultado de operaciones
UINT br, bw;  				//Variables contador para lectura/escritura

/**** Variables relacionadas a capacidad de almacenamiento *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

char buffer[BUFFER_SIZE];  	//Arreglo para almacenar cadenas
int i = 0;					//Contador para recorrer "buffer"

//RTC_AlarmTypeDef gAlarm;		//Estructura de alarma

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);	//Atención a interrupción
int bufsize (uint8_t *buf);						//Función para calcular dimensión de buffer
void clear_buffer (void);						//Función para limpiar buffer
void send_uart(char *string);					//Función para mandar cadenas UART

static void Set_Time(void);
static void Set_Date(void);
static void Set_Alarm(void);

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef * hrtc);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//******************************************************************************
/***** Atención a interrupción externa *****/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	buttonPinFlag = 1;
	return;
}
//******************************************************************************
/***** Obtener tamaño de buffer *****/
int bufsize (uint8_t *buf)
{
	int i = 0;
	while(*buf++ != '\0')
		i++;
	return i;
}
//******************************************************************************
/***** Limpiar buffer *****/
void clear_buffer (void)
{
	for(int i = 0; i < BUFFER_SIZE; i++)
		buffer[i] = '\0';
}
//******************************************************************************
/***** Enviar strings por UART *****/
void send_uart(char *string)
{
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, HAL_MAX_DELAY);  //Transmitir en modo bloqueo
}

//******************************************************************************
/*
 * Sets the time.
 */
static void Set_Time(void)
{

	RTC_TimeTypeDef sTime = {0};

	sTime.Hours = 0x12;
	sTime.Minutes = 0x00;
	sTime.Seconds = 0x00;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
	{
	  Error_Handler();
	}

	return;
}
//******************************************************************************
/*
 * Sets the date.
 */
static void Set_Date(void)
{
	RTC_DateTypeDef DateToUpdate = {0};

	DateToUpdate.WeekDay = RTC_WEEKDAY_THURSDAY;
	DateToUpdate.Month = RTC_MONTH_APRIL;
	DateToUpdate.Date = 0x30;
	DateToUpdate.Year = 0x20;

	if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
	{
	  Error_Handler();
	}

	return;
}
//******************************************************************************
/*
 * Sets the alarm in interrupt mode.
 */
static void Set_Alarm(void)
{
	RTC_AlarmTypeDef gAlarm = {0};

	gAlarm.Alarm = 1;

	gAlarm.AlarmTime.Hours = 0x12;
	gAlarm.AlarmTime.Minutes = 0x00;
	gAlarm.AlarmTime.Seconds = 0x15;

	if(HAL_RTC_SetAlarm_IT(&hrtc, &gAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
		Error_Handler();
	}

	return;
}

/*
 * Alarm event handler. Interrupt happens when actual time is equals to alarm
 */
void HAL_RTC_AlarmAEventCallback (RTC_HandleTypeDef * hrtc)
{
	char logName[17];
	RTC_DateTypeDef gDate;

	HAL_RTC_GetDate(hrtc, &gDate, RTC_FORMAT_BIN);
	sprintf(logName, "log_%02u-%02u-%02u.txt", gDate.Date, gDate.Month, gDate.Year);

	fresult = f_open(&fil, logName, FA_CREATE_ALWAYS | FA_WRITE);
	f_close(&fil);

	uint8_t alarmArray[] = "Alarm!\n";
	HAL_UART_Transmit(&huart1, alarmArray, sizeof(alarmArray), 200);

	return;
}
//******************************************************************************
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
  MX_USART1_UART_Init();
  MX_FATFS_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */

  //This sentence evaluates if time/date/alarm was already updated
//  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
//  {
//  //Set the time, date and alarm
//	  Set_Time();
//  	  Set_Date();
//  	  Set_Alarm();
//  	  //This is used to store value even when uC is restarted (using backup battery/supply)
//  	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
//  }

  Set_Time();
  Set_Date();
  Set_Alarm();

  /***** Montar tarjeta SD - función f_mount *****/
  fresult = f_mount(&fs, "/", 1);
  	if(fresult != FR_OK)
  		send_uart("Error in mounting SD CARD.\n\n");
  	else
  		send_uart("SD CARD mounted successfully.\n\n");

  	/*********************UPDATING an existing file ***************************/

  	//Open the file with write access
  	//fresult = f_open(&fil, "log3.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);
  	/*fresult = f_open(&fil, "file1.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);

  	//Move to offset to the end of the file
  	fresult = f_lseek(&fil, f_size(&fil));

  	if(fresult == FR_OK)
  		//send_uart("About to update the log3.txt\n");
  		send_uart("About to update the file1.txt\n");

  	//Write the string to the file
  	fresult = f_puts("\nActualización", &fil);
  	//fresult = f_puts("1", &fil);

  	f_close(&fil);

  	clear_buffer();

  	//Open to read the file
  	fresult = f_open (&fil, "file1.txt", FA_READ);
  	//fresult = f_open(&fil, "log3.txt", FA_READ);

  	//Read string from the file
  	fresult = f_read(&fil, buffer, f_size(&fil), &br);
  	if(fresult == FR_OK)
  		send_uart("Below is the data from updated file1.txt\n");

  	send_uart(buffer);
  	send_uart("\n\n");

  	//Close file
  	f_close(&fil);

  	clear_buffer();*/

  	/**************** The following operation is using f_write and f_read **************************/

  	//Create second file with read write access and open it
  	/*fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);

  	//Writing text
  	strcpy(buffer, "This is File2.txt, written using ...f_write... and it says Hello from Controllerstech\n");

	fresult = f_write(&fil, buffer, bufsize(buffer), &bw);

  	send_uart("File2.txt created and data is written\n");

  	//Close file
  	f_close(&fil);

  	//Clearing buffer to show that result obtained is from the file
  	clear_buffer();

  	//Open second file to read
  	fresult = f_open(&fil, "file2.txt", FA_READ);
  	if(fresult == FR_OK)send_uart ("file2.txt is open and the data is shown below\n");

  	//Read data from the file
  	//Please see the function details for the arguments
  	f_read(&fil, buffer, f_size(&fil), &br);
  	send_uart(buffer);
  	send_uart("\n\n");

  	//Close file
  	f_close(&fil);

  	clear_buffer();*/

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //  uint16_t adcSample = 0;
  //  void * padcSample = &adcSample;
  //  uint8_t sendData = 0;

  RTC_TimeTypeDef gTime;		//Estructura de hora
  RTC_DateTypeDef gDate;		//Estructura de fecha

  uint8_t time[10];				//Arreglo de hora
  uint8_t date[10];				//Arreglo de fecha
  uint8_t sendDataBuffer[23];	//Cadena para UART/MicroSD

  float adcSample = 0.0;		//Muestra de ADC
  buttonPinFlag = 0;			//Bandera de interrupción externa

  while (1)
  {

	  //Evaluar interrupción
	  if(buttonPinFlag == 1){
		  buttonPinFlag = 0;

		  /***** Conversión ADC *****/
		  HAL_ADC_Start(&hadc1);
		  HAL_ADC_PollForConversion(&hadc1, 200);
		  adcSample = HAL_ADC_GetValue(&hadc1) * (0.000805); //Conversión: (adcSample) * (3.3v/(2^12)) = adcSample * 0.000805 (aprox).
		  HAL_ADC_Stop(&hadc1);

		  //Copiar string "Voltaje..." + adcSample (ASCII) a sendDataBuffer.
		  uint8_t bufferSize = sprintf((char *)sendDataBuffer, "Voltaje: %1.2f voltios\n", adcSample);

//		  itoa((uint8_t)adcSample, (char *)buffer, 10);
//		  itoa((uint8_t)adcSample, (char *)sendDataBuffer, 10);
//		  HAL_UART_Transmit(&huart1, sendDataBuffer, sizeof(sendDataBuffer), 100);
//		  HAL_UART_Transmit(&huart1, padcSample, 1, 100);

		  //Enviamos cadena por UART
		  HAL_UART_Transmit(&huart1, sendDataBuffer, sizeof(sendDataBuffer), 100);
		  //
		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		  /***** Leemos hora y fecha, guardamos en strings *****/
		  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		  uint8_t timeSizeBuffer = sprintf((char *)time, "%02u:%02u:%02u ", gTime.Hours, gTime.Minutes, gTime.Seconds);
		  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		  uint8_t dateSizeBuffer = sprintf((char *)date, "%02u-%02u-%02u ", gDate.Date, gDate.Month, gDate.Year);

		  //Abrir archivo
		  fresult = f_open(&fil, "file1.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);

		  //Mover al final del archivo
		  fresult = f_lseek(&fil, f_size(&fil));
		  //Escribir información en la dirección apuntada por &fil
		  fresult = f_write(&fil, time, timeSizeBuffer, &bw);
		  fresult = f_write(&fil, date, dateSizeBuffer, &bw);
		  fresult = f_write(&fil, sendDataBuffer, bufferSize, &bw);

		  //fresult = f_write(&fil, psendData, bufsize(psendData), &bw);
		  //fresult = f_write(&fil, &sendData, 1, &bw);
		  //fresult = f_write(&fil, sendDataBuffer, sizeof(sendDataBuffer), &bw);
		  //fresult = f_puts("1", &fil);
		  //fresult = f_puts(" voltios\n", &fil);
		  //fresult = f_puts("Voltaje leído: ", &fil);

		  //Cerramos archivo
		  f_close(&fil);

		  //Limpiamos buffer
		  clear_buffer();

		  //Esperar
		  HAL_Delay(100);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  return 0;
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

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_HSE_DIV128;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

//  RTC_TimeTypeDef sTime = {0};
//  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
//  sTime.Hours = 0x5;
//  sTime.Minutes = 0x45;
//  sTime.Seconds = 0x0;
//
//  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  DateToUpdate.WeekDay = RTC_WEEKDAY_TUESDAY;
//  DateToUpdate.Month = RTC_MONTH_MAY;
//  DateToUpdate.Date = 0x5;
//  DateToUpdate.Year = 0x20;
//
//  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    Error_Handler();
//  }
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|alarmPin_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : buttonPin_Pin */
  GPIO_InitStruct.Pin = buttonPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(buttonPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : analogReadPin_Pin */
  GPIO_InitStruct.Pin = analogReadPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(analogReadPin_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 alarmPin_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|alarmPin_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

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
