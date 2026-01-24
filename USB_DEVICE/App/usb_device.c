/*
 * usb_device.c
 *
 *
 *
 */

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"


//
//
//

USBD_HandleTypeDef hUsbDeviceFS;

uint8_t HID_EpAdd_Inst = HID_EPIN_ADDR;								/* HID Endpoint Address array */
uint8_t CDC_EpAdd_Inst0[3] = {CDC_IN_EP, CDC_OUT_EP, CDC_CMD_EP}; 	/* CDC Endpoint Addresses array */
uint8_t CDC_EpAdd_Inst1[3] = {CDC_IN_EP+2, CDC_OUT_EP+1, CDC_CMD_EP+2}; 	/* CDC Endpoint Addresses array */
uint8_t HID_InstID = 0, CDC_InstID0 = 0, CDC_InstID1 = 0;


//
//
//

void MX_USB_DEVICE_Init(void)
{
	/*
	if (USBD_Init(&hUsbDeviceFS, &Dev_Desc, DEVICE_FS) != USBD_OK)
	{
		Error_Handler();
	}
	if (USBD_RegisterClassComposite(&hUsbDeviceFS, &USBD_CDC, CLASS_TYPE_CDC, 0) != USBD_OK)
	{
		Error_Handler();
	}
	if (USBD_RegisterClassComposite(&hUsbDeviceFS, &USBD_CDC, CLASS_TYPE_CDC, 0) != USBD_OK)
	{
		Error_Handler();
	}
	if (USBD_CMPSIT_SetClassID(&hUsbDeviceFS, CLASS_TYPE_CDC, 0) != 0xFF)
	{
		if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops) != USBD_OK)
		{
			Error_Handler();
		}
	}
	if (USBD_CMPSIT_SetClassID(&hUsbDeviceFS, CLASS_TYPE_CDC, 1) != 0xFF)
	{
		if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops) != USBD_OK)
		{
			Error_Handler();
		}
	}
	if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
	{
		Error_Handler();
	}
	*/

	/* Initialize the USB Device Library */
	if(USBD_Init(&hUsbDeviceFS, &Dev_Desc, 0) != USBD_OK)
		Error_Handler();

	/* Store HID Instance Class ID */
	HID_InstID = hUsbDeviceFS.classId;

	/* Register the HID Class */
#if 1
	if(USBD_RegisterClassComposite(&hUsbDeviceFS, USBD_HID_CLASS, CLASS_TYPE_HID, &HID_EpAdd_Inst) != USBD_OK)
		Error_Handler();
#endif

	/* Store CDC#0 Instance Class ID */
	CDC_InstID0 = hUsbDeviceFS.classId;

	/* Register CDC Class First Instance */
	if(USBD_RegisterClassComposite(&hUsbDeviceFS, USBD_CDC_CLASS, CLASS_TYPE_CDC, CDC_EpAdd_Inst0) != USBD_OK)
		Error_Handler();

	/* Store CDC#1 Instance Class ID */
	CDC_InstID1 = hUsbDeviceFS.classId;

	/* Register CDC Class First Instance */
	if(USBD_RegisterClassComposite(&hUsbDeviceFS, USBD_CDC_CLASS, CLASS_TYPE_CDC, CDC_EpAdd_Inst1) != USBD_OK)
		Error_Handler();


	/* Add CDC Interface Class */
	if (USBD_CMPSIT_SetClassID(&hUsbDeviceFS, CLASS_TYPE_CDC, 0) != 0xFF)
	{
		USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops);
	}

	/* Add CDC Interface Class */
	if (USBD_CMPSIT_SetClassID(&hUsbDeviceFS, CLASS_TYPE_CDC, 1) != 0xFF)
	{
		USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_CDC_fops);
	}

	USBD_Start(&hUsbDeviceFS);
}
