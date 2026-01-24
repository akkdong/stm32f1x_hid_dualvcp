/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usb_hid_keys.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_hid.h"
#include "usbd_desc.h"
#include "usbd_composite_builder.h"

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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

#define MAX_VCP_RXBUFFER	128
#define MAX_VCP_TXBUFFER	128
#define MAX_DBG_RXBUFFER	64
#define MAX_DBG_TXBUFFFER	128

uint8_t bufRxVCP[MAX_VCP_COUNT][MAX_VCP_RXBUFFER], bufRxDebug[MAX_DBG_RXBUFFER];
uint8_t bufTxVCP[MAX_VCP_COUNT][MAX_VCP_TXBUFFER], bufTxDebug[MAX_DBG_TXBUFFFER];

RingBuffer rbRxVCP[MAX_VCP_COUNT], rbRxDebug;
RingBuffer rbTxVCP[MAX_VCP_COUNT], rbTxDebug;

UartState uartVCP[MAX_VCP_COUNT], uartDebug;


uint8_t hidReport[16];

uint16_t adcValue[2];
uint32_t adcAverage[2];

uint8_t mouseButton;
int8_t mouseMovement[2]; // [UP/DOWN, LEFT/RIGHT]



extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t HID_InstID, CDC_InstID0, CDC_InstID1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
extern void DualVCP_UartInit();
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
  memset(&hidReport, 0, sizeof(hidReport));
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();

  // initialize RingBuffer
  for (int i = 0; i < MAX_VCP_COUNT; ++i)
  {
	  RB_Init(&rbRxVCP[i], &bufRxVCP[0][0], MAX_VCP_RXBUFFER);
	  RB_Init(&rbTxVCP[i], &bufTxVCP[0][0], MAX_VCP_TXBUFFER);
  }

  RB_Init(&rbRxDebug, &bufRxDebug[0], MAX_VCP_RXBUFFER);
  RB_Init(&rbTxDebug, &bufTxDebug[0], MAX_VCP_TXBUFFER);

  // initialize UART
  for (int i = 0; i < MAX_VCP_COUNT; ++i)
	  UART_Init(&uartVCP[i], i == 0 ? &huart2 : &huart3, &rbRxVCP[i], &rbTxVCP[i]);

  UART_Init(&uartDebug, &huart1, &rbRxDebug, &rbTxDebug);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  //
  mouseButton = 0;
  memset(&adcAverage[0], 0, sizeof(adcAverage));
  memset(&mouseMovement[0], 0, sizeof(mouseMovement));

  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adcValue[0], 2);

  //
  uint32_t lastTick = HAL_GetTick();
  while (1)
  {
	  //
	for (int i = 0; i < 2; i++)
	{
		//  UART.rxBuf <--- received from UART device
		//  UART.txBuf <--- received from USB host
		//
		//  rx_available : rxBuf is not empty & txUsbLen is zero
		//    ---> CDC_Transmit()
		//  tx_available : txBuf is not empty & txUartLen is zero
		//    ---> HAL_UART_Transmit()
		//
		if (UART_RxAvailable(&uartVCP[i]) > 0)
		{
			uint16_t len;
			uint8_t *data = UART_PreserveRxBuffer(&uartVCP[i], USB_FS_MAX_PACKET_SIZE, &len);

			// uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len, uint32_t classId);
			if (CDC_Transmit_FS(data, len, i == 0 ? CDC_InstID0 : CDC_InstID1) != USBD_OK)
				UART_CheckoutRxBuffer(&uartVCP[i]);
		}

		if (UART_TxAvailable(&uartVCP[i]) > 0)
		{
			uint16_t len;
			uint8_t *data = UART_PreserveTxBuffer(&uartVCP[i], -1, &len);

			if (HAL_UART_Transmit_IT(uartVCP[i].pHandle, data, len) != HAL_OK)
				UART_CheckoutTxBuffer(&uartVCP[i]);
		}
	}

	//
#if 0
	if (UART_RxAvailable(&uartDebug) > 0)
	{
		uint16_t len;
		uint8_t *data = UART_PreserveRxBuffer(&uartDebug, 64, &len);
		//
		// parse incoming
		//
		UART_CheckoutRxBuffer(&uartDebug);
	}
#endif
	if (UART_TxAvailable(&uartDebug) > 0)
	{
		uint16_t len;
		uint8_t *data = UART_PreserveTxBuffer(&uartDebug, -1, &len);

		if (HAL_UART_Transmit_IT(uartDebug.pHandle, data, len) != HAL_OK)
			UART_CheckoutTxBuffer(&uartDebug);
	}

	//
	if (HAL_GetTick() - lastTick > 1000)
	{
		//printf("%u %u\r\n", adcValue[0], adcValue[1]);
		lastTick = HAL_GetTick();
	}

	/*
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_SET)
	{
		USBD_HID_SendReport(&hUsbDeviceFS, hid_report_buffer, 4, HID_InstID);
		//USBD_CDC_TransmitPacket(&hUsbDeviceFS, CDC_InstID0);
		//USBD_CDC_TransmitPacket(&hUsbDeviceFS, CDC_InstID1);
		HAL_Delay(100);
	}
	*/
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6; // 12MHz
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /*Configure GPIO pin : PC13 */
  //GPIO_InitStruct.Pin = GPIO_PIN_13;
  //GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  //GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  //HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if 1
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  // Transmit a single character via the specified UART handle (e.g., &huart2)
	UART_Transmit(&uartDebug, (uint8_t *)&ch, 1);
  return ch;
}
#else
int _write(int file, char *ptr, int len)
{
    // Transmit the string via the specified UART handle (e.g., &huart2)
    //HAL_UART_Transmit(&huart1, (uint8_t*)ptr, len, HAL_MAX_DELAY);
	UART_Transmit(&uartDebug, (uint8_t *)ptr, (uint16_t)len);
    return len;
}
#endif

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		UART_CheckoutTxBuffer(&uartDebug);
	}
	else if (huart == &huart2)
	{
		UART_CheckoutTxBuffer(&uartVCP[0]);
	}
	else if (huart == &huart3)
	{
		UART_CheckoutTxBuffer(&uartVCP[1]);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		UART_ProcessReceive(&uartDebug);
	}
	else if (huart == &huart2)
	{
		UART_ProcessReceive(&uartVCP[0]);
	}
	else if (huart == &huart3)
	{
		UART_ProcessReceive(&uartVCP[1]);
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{

}

void ReportKeyboard()
{

}

void ReportMouse(uint8_t button, uint8_t moveX, uint8_t moveY)
{
	hidReport[0] = 2;
	hidReport[1] = button;
	hidReport[2] = moveX;
	hidReport[3] = moveY;

	USBD_HID_SendReport(&hUsbDeviceFS, hidReport, 9, HID_InstID);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint32_t count = 0;

	adcAverage[0] += adcValue[0];
	adcAverage[1] += adcValue[1];

	if (++count >= 1000)
	{
		adcAverage[0] /= count;
		adcAverage[1] /= count;
		count = 0;

		//printf("%u %u\r\n", (unsigned int)adcAverage[0], (unsigned int)adcAverage[1]);
		int deltaY = ((int)adcAverage[0] - 1992) / 14;
		int deltaX = ((int)adcAverage[1] - 1988) / 14;

		if (deltaY < -100)
			deltaY = -100;
		if (deltaY > 100)
			deltaY = 100;
		if (deltaX < -100)
			deltaX = -100;
		if (deltaX > 100)
			deltaX = 100;

		deltaX = -deltaX;

		if (mouseMovement[0] != deltaY || mouseMovement[1] != deltaX)
		{
			mouseMovement[0] = deltaY;
			mouseMovement[1] = deltaX;

			ReportMouse(mouseButton, mouseMovement[1], mouseMovement[0]);
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	GPIO_PinState state = HAL_GPIO_ReadPin(GPIOC, GPIO_Pin);

	switch (GPIO_Pin)
	{
	case GPIO_PIN_15:
		/*
		hidReport[0] = 1;
		hidReport[1] = 0;
		hidReport[2] = 0;
		hidReport[3] = state == GPIO_PIN_RESET ? KEY_LEFT : 0;

		USBD_HID_SendReport(&hUsbDeviceFS, hidReport, 9, HID_InstID);
		*/
		if (state == GPIO_PIN_RESET)
			mouseButton |= 0x01;
		else
			mouseButton &= ~0x01;
		ReportMouse(mouseButton, mouseMovement[1], mouseMovement[0]);
		break;
	case GPIO_PIN_14:
		/*
		hidReport[0] = 1;
		hidReport[1] = 0;
		hidReport[2] = 0;
		hidReport[3] = state == GPIO_PIN_RESET ? KEY_ENTER : 0;

		USBD_HID_SendReport(&hUsbDeviceFS, hidReport, 9, HID_InstID);
		*/
		if (state == GPIO_PIN_RESET)
			mouseButton |= 0x04;
		else
			mouseButton &= ~0x04;
		ReportMouse(mouseButton, mouseMovement[1], mouseMovement[0]);
		break;
	case GPIO_PIN_13:
		/*
		hidReport[0] = 1;
		hidReport[1] = 0;
		hidReport[2] = 0;
		hidReport[3] = state == GPIO_PIN_RESET ? KEY_RIGHT : 0;

		USBD_HID_SendReport(&hUsbDeviceFS, hidReport, 9, HID_InstID);
		*/
		if (state == GPIO_PIN_RESET)
			mouseButton |= 0x02;
		else
			mouseButton &= ~0x02;
		ReportMouse(mouseButton, mouseMovement[1], mouseMovement[0]);
		break;

	case GPIO_PIN_12:
		//ReportMouse(state == GPIO_PIN_RESET ? 0x01 : 0, mouseMovement[1], mouseMovement[0]);
		break;
	}
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
