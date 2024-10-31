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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../../USB_DEVICE/App/usbd_cdc_if.h"

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

/* USER CODE BEGIN PV */
uint8_t RxBuffer[APP_RX_DATA_SIZE];
uint8_t TxBuf[1];
volatile uint32_t ComBufferLength = 0;
uint32_t ComBufferPos = 0;
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void SdCardSoft ()
{

}

uint8_t sd_init();
void error();

uint8_t bFsReady = 0;
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
  MX_USB_DEVICE_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  if (sd_init())
    error ();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
#define CLIENT_RTS A1_Pin
#define CLIENT_RTS_PORT A1_GPIO_Port
#define CLIENT_RTR A0_Pin
#define CLIENT_RTR_PORT A0_GPIO_Port

#define ROM_EMULATION 0

  HAL_GPIO_WritePin (RXRDY_GPIO_Port, RXRDY_Pin | TXRDY_Pin, 1);
  //while ((GPIOB->IDR & (CLIENT_RTS | CLIENT_RTR)) != (CLIENT_RTS | CLIENT_RTR)) ;
  HAL_GPIO_WritePin (GPIOC, LED13_Pin, 0);
  int32_t counter = 0;
  GPIO_PinState key = HAL_GPIO_ReadPin (KEY_GPIO_Port, KEY_Pin);
#if !ROM_EMULATION
  uint8_t bFrom_nCS = 0;
  DATA_BUS_IN ();
#endif

  uint8_t bRkSd = key != 0;
  if (bRkSd)
    RkSd_main();
  MX_USB_DEVICE_Init();
  void InitWB_Ring();
  if (!bRkSd)
  {
    bFsReady = 1;
    InitWB_Ring();
  }

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
#if !ROM_EMULATION
#if 1
    uint8_t RkSd_Loop();
    static uint32_t addr = 0;
    //while (1)
    {
      static uint32_t lastAddr = 0;

      static uint32_t oldAddr = -1;
      //DATA_BUS_OUT();
      addr = READ_ADDR ();
      if (addr == 0x7E/* && oldAddr != addr*/)
      {
check_addr:
	DATA_BUS_OUT ();
	oldAddr = 0;
	while (1)
	{
	  addr = READ_ADDR ();
	  if (oldAddr == addr)
	    continue;
	  oldAddr = addr;
	  if (addr == 0x7F)
	  {
	    WRITE_DATA (0xAA);
	    lastAddr = addr;
	  }
	  else if (addr == 0x7E)
	  {
	    WRITE_DATA (0x55);
	  }
	  else if (addr == 0x44)
	  {
	    lastAddr = 0x44;
	  }
	  else if (addr == 0x40)
	  {
	    if (lastAddr == 0x44)
	      lastAddr = 0x40;
	    else
	    {
	      lastAddr = 0;
	      break;
	    }
	  }
	  else if (addr == 0)
	  {
	    if (lastAddr == 0x40)
	    {
	      //RkSd_main();
	      //MX_USB_DEVICE_Deinit ();
	      while (RkSd_Loop ())
		;
	      MX_USB_DEVICE_Init ();
	      break;
	    }
	    else if (lastAddr != 0x7F)
	    {
	      lastAddr = 0;
	      break;
	    }
	  }
	}
      }
      DATA_BUS_IN ();
    }
#endif
    // if nCS is high - release the bus
    void rb_write_rec();
    rb_write_rec();
    if (nCS_GPIO_Port->IDR & nCS_Pin)
    {
      if (!bFrom_nCS)
      {
	DATA_BUS_IN ();
	GPIO_InitTypeDef GPIO_InitStruct =
	  { 0 };
	GPIO_InitStruct.Pin = RXRDY_Pin | TXRDY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init (GPIOA, &GPIO_InitStruct);
      }
      bFrom_nCS = 1;
      continue;

    }
    if (bFrom_nCS)
    {
      bFrom_nCS = 0;
    }
    HAL_GPIO_WritePin (RXRDY_GPIO_Port, RXRDY_Pin | TXRDY_Pin, 1);
    if ((CLIENT_RTS_PORT->IDR & CLIENT_RTS) != 0 && (CLIENT_RTR_PORT->IDR & CLIENT_RTR) != 0)
    {
      //MX_USB_DEVICE_Deinit ();
      //RkSd_main();
      while (RkSd_Loop ())
	;
      MX_USB_DEVICE_Init();
      continue;
    }
    if ((CLIENT_RTS_PORT->IDR & CLIENT_RTS) != 0)
    {
      // Receive a byte from client
      HAL_GPIO_WritePin (TXRDY_GPIO_Port, TXRDY_Pin, 0);
      while ((CLIENT_RTS_PORT->IDR & CLIENT_RTS) != 0)
	; // Wait for client writes data
      TxBuf[0] = READ_DATA ();
      HAL_GPIO_WritePin (TXRDY_GPIO_Port, TXRDY_Pin, 1);
      uint8_t res = 0;
      //uint8_t nTry = 10;
      do
      {
	res = CDC_Transmit_FS (TxBuf, 1);
	addr = READ_ADDR ();
	if (addr == 0x7E)
	  goto check_addr;
      }
      while ((res != USBD_OK)/* && (--nTry != 0)*/);
    }
    else if ((CLIENT_RTR_PORT->IDR & CLIENT_RTR) != 0 && ComBufferLength != 0)
    {
      // Send a byte to client
      uint8_t val = RxBuffer[ComBufferPos++];
      ComBufferLength--;
      if (ComBufferLength == 0)
	USBD_CDC_ReceivePacket (&hUsbDeviceFS);
      DATA_BUS_OUT ();
      WRITE_DATA (val);
      HAL_GPIO_WritePin (RXRDY_GPIO_Port, RXRDY_Pin, 0);
      while ((CLIENT_RTR_PORT->IDR & CLIENT_RTR) != 0)
	;
      DATA_BUS_IN ();
    }
#else
		static uint32_t oldAddr = -1;
		int32_t pa = GPIOA->IDR;// & (0b1110));
		uint32_t pb = GPIOB->IDR;// & 0b11100011);
		//               D7,D6                D1                D0                      D5-D2
		//               B7,B6                B0                A7                      A4-A1
		uint32_t addr = (pb & 0b11000000) | ((pb & 1) << 1) | ((pa & 0b10000000)>>7) | ((pa & 0b11110)<<1);
		uint32_t val = 255-addr;
		if (oldAddr != addr)
		{
			//val |= 0b10000;
			uint32_t portVal = ((val & 0b11110011) << 8) | ((val & 0b1100) >> 1);
			GPIOB->ODR = portVal;
			oldAddr = addr;
		}
#endif
    if (counter++ % 100000 == 0)
      HAL_GPIO_TogglePin (GPIOC, LED13_Pin);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  //RTC_TimeTypeDef sTime = {0};
  //RTC_DateTypeDef sDate = {0};

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
  HAL_RTCEx_SetSmoothCalib(&hrtc, RTC_SMOOTHCALIB_PERIOD_32SEC, RTC_SMOOTHCALIB_PLUSPULSES_SET, 273);

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
/*
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
*/
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED13_GPIO_Port, LED13_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
//  HAL_GPIO_WritePin(GPIOB, D2_Pin|D3_Pin|D4_Pin|D5_Pin
//                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_CS_Pin|RXRDY_Pin|TXRDY_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED13_Pin */
  GPIO_InitStruct.Pin = LED13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_Pin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : A2_Pin A3_Pin A4_Pin A5_Pin
                           A0_Pin */
  GPIO_InitStruct.Pin = A2_Pin|A3_Pin|A4_Pin|A5_Pin
                          |A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : A1_Pin A6_Pin A7_Pin */
  GPIO_InitStruct.Pin = A1_Pin|A6_Pin|A7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D2_Pin D3_Pin D4_Pin D5_Pin
                           D6_Pin D7_Pin D0_Pin D1_Pin */
  GPIO_InitStruct.Pin = D2_Pin|D3_Pin|D4_Pin|D5_Pin
                          |D6_Pin|D7_Pin|D0_Pin|D1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : nCS_Pin */
  GPIO_InitStruct.Pin = nCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(nCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RXRDY_Pin TXRDY_Pin */
  GPIO_InitStruct.Pin = RXRDY_Pin|TXRDY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

#ifdef USE_DMA
  GPIO_InitStruct.Pin = A2_Pin; //DRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init (A2_GPIO_Port, &GPIO_InitStruct);
#endif

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
  __disable_irq ();
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
