/**
  ******************************************************************************
  * @file    usbd_cdc_if_template.c
  * @author  MCD Application Team
  * @brief   Generic media access Layer.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2015 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* BSPDependencies
- "stm32xxxxx_{eval}{discovery}{nucleo_144}.c"
- "stm32xxxxx_{eval}{discovery}_io.c"
EndBSPDependencies */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "usbd_composite_builder.h"
#include "main.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */


/** @defgroup USBD_CDC
  * @brief usbd core module
  * @{
  */

/** @defgroup USBD_CDC_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Defines
  * @{
  */
/**
  * @}
  */


/** @defgroup USBD_CDC_Private_Macros
  * @{
  */

/**
  * @}
  */


/** @defgroup USBD_CDC_Private_FunctionPrototypes
  * @{
  */

static int8_t DualVCP_Init(uint32_t id);
static int8_t DualVCP_DeInit(uint32_t id);
static int8_t DualVCP_Control(uint32_t id, uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t DualVCP_Receive(uint32_t id, uint8_t *pbuf, uint32_t *Len);
static int8_t DualVCP_TransmitCplt(uint32_t id, uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
  DualVCP_Init,
  DualVCP_DeInit,
  DualVCP_Control,
  DualVCP_Receive,
  DualVCP_TransmitCplt
};


//extern UART_HandleTypeDef huart2;
//extern UART_HandleTypeDef huart3;

USBD_CDC_LineCodingTypeDef linecoding[MAX_VCP_COUNT] =
{
	{
	  115200, /* baud rate*/
	  0x00,   /* stop bits-1*/
	  0x00,   /* parity - none*/
	  0x08    /* nb. of bits 8*/
	},
	{
	  115200, /* baud rate*/
	  0x00,   /* stop bits-1*/
	  0x00,   /* parity - none*/
	  0x08    /* nb. of bits 8*/
	},
};

#define APP_RX_DATA_SIZE	64
#define APP_TX_DATA_SIZE	64


uint8_t UserRxBufferFS[MAX_VCP_COUNT][APP_RX_DATA_SIZE];
#if 0
uint8_t UserTxBufferFS[MAX_VCP_COUNT][APP_TX_DATA_SIZE];
#endif

extern UartState uartVCP[MAX_VCP_COUNT];
extern USBD_HandleTypeDef hUsbDeviceFS;



/* Private functions ---------------------------------------------------------*/


/**
  * @brief  TEMPLATE_Init
  *         Initializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DualVCP_Init(uint32_t id)
{
  uint32_t idx = USBD_CMPIT_GetInstNbr(&hUsbDeviceFS, id);

  //
  uint8_t *data = UART_PreserveRxBuffer(&uartVCP[idx], 0, NULL);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, data, 0, id);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS[idx]);

  return (USBD_OK);
}

/**
  * @brief  TEMPLATE_DeInit
  *         DeInitializes the CDC media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DualVCP_DeInit(uint32_t id)
{
  //
  uint32_t idx = USBD_CMPIT_GetInstNbr(&hUsbDeviceFS, id);
  UART_DeInit(&uartVCP[idx]);

  return (USBD_OK);
}


/**
  * @brief  TEMPLATE_Control
  *         Manage the CDC class requests
  * @param  Cmd: Command code
  * @param  Buf: Buffer containing command data (request parameters)
  * @param  Len: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DualVCP_Control(uint32_t id, uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  UNUSED(length);

  uint32_t idx = USBD_CMPIT_GetInstNbr(&hUsbDeviceFS, id);

  switch (cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:
      /* Add your code here */
      break;

    case CDC_GET_ENCAPSULATED_RESPONSE:
      /* Add your code here */
      break;

    case CDC_SET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_GET_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_CLEAR_COMM_FEATURE:
      /* Add your code here */
      break;

    case CDC_SET_LINE_CODING:
      linecoding[idx].bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) | \
                                         (pbuf[2] << 16) | (pbuf[3] << 24));
      linecoding[idx].format     = pbuf[4];
      linecoding[idx].paritytype = pbuf[5];
      linecoding[idx].datatype   = pbuf[6];

      //
      UART_Config(&uartVCP[idx],
    		  linecoding[idx].bitrate,
			  linecoding[idx].format,
			  linecoding[idx].paritytype,
			  linecoding[idx].datatype);
      break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = (uint8_t)(linecoding[idx].bitrate);
      pbuf[1] = (uint8_t)(linecoding[idx].bitrate >> 8);
      pbuf[2] = (uint8_t)(linecoding[idx].bitrate >> 16);
      pbuf[3] = (uint8_t)(linecoding[idx].bitrate >> 24);
      pbuf[4] = linecoding[idx].format;
      pbuf[5] = linecoding[idx].paritytype;
      pbuf[6] = linecoding[idx].datatype;

      /* Add your code here */
      break;

    case CDC_SET_CONTROL_LINE_STATE:
      /* Add your code here */
      break;

    case CDC_SEND_BREAK:
      /* Add your code here */
      break;

    default:
      break;
  }

  return (0);
}

/**
  * @brief  TEMPLATE_Receive
  *         Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DualVCP_Receive(uint32_t id, uint8_t *Buf, uint32_t *Len)
{
  //
  //
  uint32_t idx = USBD_CMPIT_GetInstNbr(&hUsbDeviceFS, id);
#if TEST_LOOPBACK
  memcpy(&UserTxBufferFS[idx][0], Buf, *Len);
  CDC_Transmit_FS(&UserTxBufferFS[idx][0], *Len, id);
#else
  UART_Transmit(&uartVCP[idx], Buf, *Len);
#endif
  //
  //

  //
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  return (USBD_OK);
}

/**
  * @brief  TEMPLATE_TransmitCplt
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t DualVCP_TransmitCplt(uint32_t id, uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  //
  uint32_t idx = USBD_CMPIT_GetInstNbr(&hUsbDeviceFS, id);
  UART_CheckoutRxBuffer(&uartVCP[idx]);

  return (USBD_OK);
}

/**
  * @}
  */

uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len, uint32_t classId)
{
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassDataCmsit[classId];
  if (hcdc->TxState != 0){
	return USBD_BUSY;
  }

  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len, classId);
  uint8_t result = USBD_CDC_TransmitPacket(&hUsbDeviceFS, classId);

  return result;
}

/**
  * @}
  */

/**
  * @}
  */

