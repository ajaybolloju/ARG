/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file            : usb_host.c
  * @version         : v1.0_Cube
  * @brief           : This file implements the USB Host
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

#include "usb_host.h"
#include "usbh_core.h"
#include "usbh_msc.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */
extern uint8_t usb_exp_disk;

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USB Host core handle declaration */
USBH_HandleTypeDef hUsbHostFS;
ApplicationTypeDef Appli_state = APPLICATION_IDLE;

/*
 * -- Insert your variables declaration here --
 */
/* USER CODE BEGIN 0 */
uint8_t MX_USB_HOST_App_state(void)
{
  return Appli_state;
}

/* USER CODE END 0 */

/*
 * user callback declaration
 */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);

/*
 * -- Insert your external function declaration here --
 */
/* USER CODE BEGIN 1 */

/**
   @brief  Displays disk content.
   @param  path: Pointer to root path
   @param  recu_level: Disk content level
   @retval Operation result
*/
//FRESULT Explore_Disk(char *path, uint8_t recu_level)
//{
//  FRESULT res = FR_OK;
//  FILINFO fno;
//  DIR dir;
//  char *fn;
//  char tmp[14];
//  uint8_t line_idx = 0;
//
//#if _USE_LFN
//  static char lfn[_MAX_LFN + 1]; /* Buffer to store the LFN */
//  fno.lfname = lfn;
//  fno.lfsize = sizeof lfn;
//#endif
//
//  res = f_opendir(&dir, path);
//  if (res == FR_OK)
//  {
//    //    while(USBH_MSC_IsReady(&hUSBHost))
//    while (1)
//    {
//      res = f_readdir(&dir, &fno);
//      if (res != FR_OK || fno.fname[0] == 0)
//      {
//        break;
//      }
//      if (fno.fname[0] == '.')
//      {
//        continue;
//      }
//
//#if _USE_LFN
//      fn = *fno.lfname ? fno.lfname : fno.fname;
//#else
//      fn = fno.fname;
//#endif
//      strcpy(tmp, fn);
//
//      line_idx++;
//      //          if (line_idx > 9)
//      //          {
//      //              line_idx = 0;
//      //              APP_DEBUG_STR("> Press [Key] To Continue.\n");
//
//      //              /* KEY Button in polling */
//      //              while (BSP_PB_GetState(BUTTON_KEY) != RESET)
//      //              {
//      //                  /* Wait for User Input */
//      //              }
//      //          }
//
//      if (recu_level == 1)
//      {
//        printf("|__");
//      }
//      else if (recu_level == 2)
//      {
//    	  printf("|__|__");
//      }
//      if ((fno.fattrib & AM_MASK) == AM_DIR)
//      {
//        strcat(tmp, "\n");
//        printf((void *) tmp);
//        Explore_Disk(fn, 2);
//      }
//      else
//      {
//        strcat(tmp, "\n");
//        printf((void *) tmp);
//      }
//
//      if (((fno.fattrib & AM_MASK) == AM_DIR) && (recu_level == 2))
//      {
//        Explore_Disk(fn, 2);
//      }
//    }
//    f_closedir(&dir);
//    //  APP_DEBUG_STR("> Select an operation to Continue.\n" );
//  }
//  else
//  {
//    //      APP_DEBUG_STR("\n\r!! Error res is %d",res);
//  }
//  return res;
//}

/* USER CODE END 1 */

/**
  * Init USB host library, add supported class and start the library
  * @retval None
  */
void MX_USB_HOST_Init(void)
{
  /* USER CODE BEGIN USB_HOST_Init_PreTreatment */

  /* USER CODE END USB_HOST_Init_PreTreatment */

  /* Init host Library, add supported class and start the library. */
  if (USBH_Init(&hUsbHostFS, USBH_UserProcess, HOST_FS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_RegisterClass(&hUsbHostFS, USBH_MSC_CLASS) != USBH_OK)
  {
    Error_Handler();
  }
  if (USBH_Start(&hUsbHostFS) != USBH_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_HOST_Init_PostTreatment */

  /* USER CODE END USB_HOST_Init_PostTreatment */
}

/*
 * Background task
 */
void MX_USB_HOST_Process(void)
{
  /* USB Host Background task */
  USBH_Process(&hUsbHostFS);
}
/*
 * user callback definition
 */
static void USBH_UserProcess  (USBH_HandleTypeDef *phost, uint8_t id)
{
  /* USER CODE BEGIN CALL_BACK_1 */
  switch(id)
  {
  case HOST_USER_SELECT_CONFIGURATION:
	  printf("\n USB Config");
  break;

  case HOST_USER_DISCONNECTION:
	  printf("\n USB Disconnected");
	  usb_exp_disk = 1;
	  Appli_state = APPLICATION_DISCONNECT;
  break;

  case HOST_USER_CLASS_ACTIVE:
	  printf("\n USB READY");
  Appli_state = APPLICATION_READY;
  break;

  case HOST_USER_CONNECTION:
	  printf("\n USB Connected");
  Appli_state = APPLICATION_START;
  break;

  default:
  break;
  }
  /* USER CODE END CALL_BACK_1 */
}

/**
  * @}
  */

/**
  * @}
  */

