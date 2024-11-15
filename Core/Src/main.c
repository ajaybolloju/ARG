/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "fatfs.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
//extern uint8_t retUSBH;    /* Return value for USBH */
//extern char USBHPath[4];   /* USBH logical drive path */
//extern FATFS USBHFatFS;    /* File system object for USBH logical drive */
//extern FIL USBHFile;       /* File object for USBH */
FRESULT RES;
uint32_t BytesWritten = 0;
int Dir_Flag;
char hold[100] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t aRxBuffer;
uint8_t SerialTxReady = RESET;
uint8_t ModemTxReady = SET;
uint8_t g_buff[100] = {0};
uint8_t wr_ptr = 0;
uint8_t usb_exp_disk = 1;

#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
  set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define ONBOARD_LED_OFF()  HAL_GPIO_WritePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin, GPIO_PIN_RESET)
#define ONBOARD_LED_ON() HAL_GPIO_WritePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin, GPIO_PIN_SET)

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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  if (f_mount(&USBHFatFS, (TCHAR const*) USBHPath, 0) != FR_OK)
  {
    printf("\n\n ERROR : USBHFatFS Initialization");
  }

  HAL_UART_Receive_IT(&huart1, (uint8_t *) &aRxBuffer, 1);
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &aRxBuffer, 1);

  // Test data to write to flash
      uint8_t write_data[16] = "Hello, W25QXX!";
      uint8_t read_data[16] = {0};
      ONBOARD_LED_OFF();

      // Write data to address 0x000000
      printf("\n SPI FLASH DATA WRITE STARTED");
      W25QXX_WriteData(0x000000, write_data, sizeof(write_data));
      printf("\n SPI FLASH DATA WRITE COMPLETED");

      // Read data back from address 0x000000
      printf("\n SPI FLASH DATA READ STARTED");
      W25QXX_ReadData(0x000000, read_data, sizeof(read_data));
      printf("\n SPI FLASH DATA READ COMPLETED");

//      // Compare read data with write data
//      if (memcmp(write_data, read_data, sizeof(write_data)) == 0) {
//          // Success! Data matches
//          ONBOARD_LED_ON();
//          printf("\n SPI FLASH READ DATA SUCCESS: \t");
//          for(uint8_t i = 0; i < 16;i++)
//          {
//              printf("%c", read_data[i]);
//          }
////          HAL_GPIO_WritePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin, GPIO_PIN_SET); // Turn on an LED or indicate success
//      } else {
//          // Failure! Data mismatch
//          printf("\n SPI FLASH READ DATA FAILED");
//          ONBOARD_LED_OFF();
////          HAL_GPIO_WritePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin, GPIO_PIN_RESET); // Turn on a different LED or indicate failure
//      }

      FATFS USBFatFS;     // File system object for USB
      FIL MyFile;
      char buffer[100];   // Buffer to hold read data
      FRESULT res;    // FATFS function common result variable
      UINT bytesWritten, bytesRead;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//      printf("\n SPI FLASH READ DATA SUCCESS: \t");
//      for(uint8_t i = 0; i < 16;i++)
//      {
//          printf("%c", read_data[i]);
//      }
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */

    if ((MX_USB_HOST_App_state() == APPLICATION_READY) && usb_exp_disk)
    {
      usb_exp_disk = 0;
      printf("\n USB DETECTED");
//      printf("\n usb  =  %d", Explore_Disk(USBHPath, 1));

      res = f_mount(&USBFatFS, "", 1);
          if (res != FR_OK) {
              printf("Failed to mount USB drive. Error: %d\n", res);
          }
          printf("USB Drive mounted successfully.\n");

          // 4. Create and Write to a File on USB
              res = f_open(&MyFile, "test.txt", FA_CREATE_ALWAYS | FA_WRITE);
              if (res == FR_OK) {
                  const char *data = "Hello, USB World!";
                  res = f_write(&MyFile, data, strlen(data), &bytesWritten);
                  if (res == FR_OK && bytesWritten == strlen(data)) {
                      printf("Data written successfully to USB.\n");
                  } else {
                      printf("Failed to write data to USB. Error: %d\n", res);
                  }
                  f_close(&MyFile);  // Close file after writing
              } else {
                  printf("Failed to open file for writing. Error: %d\n", res);
              }

              // 5. Read the Data Back from the File
              res = f_open(&MyFile, "test.txt", FA_READ);
              if (res == FR_OK) {
                  res = f_read(&MyFile, buffer, sizeof(buffer) - 1, &bytesRead);
                  if (res == FR_OK) {
                      buffer[bytesRead] = '\0';  // Null-terminate the read data
                      printf("Data read from USB: %s\n", buffer);
                  } else {
                      printf("Failed to read data from USB. Error: %d\n", res);
                  }
                  f_close(&MyFile);  // Close file after reading
              } else {
                  printf("Failed to open file for reading. Error: %d\n", res);
              }

              // 6. Unmount the USB Drive after Operations
              f_mount(NULL, "", 1);
              printf("USB Drive unmounted.\n");
   }
	  // Send AT command
//	      char *command = "AT\r\n";

//	      printf("AT\r\n");
//	      HAL_UART_Transmit(&huart1, (uint8_t*)command, strlen(command), HAL_MAX_DELAY);

//	      HAL_Delay(2000);
	      // Wait for and print response
//	      uint8_t response[100];
//	      if (HAL_UART_Receive(&huart1, response, sizeof(response), 1000) == HAL_OK) {
//	          // Process or print the response here
//	    	  printf("\n Response Rxd %s",(char *)response);
//	      }
//
//	      HAL_Delay(1000); // Wait before sending the next command
////	  if(HAL_GPIO_ReadPin(OnBoardKey_GPIO_Port,OnBoardKey_Pin) == GPIO_PIN_SET)
////	  {
////	  if(ModemTxReady == SET)
//	  {
//		  memset(g_buff,'\0',sizeof(g_buff));
//		  wr_ptr = 0;
////
//		  if (HAL_UART_Transmit(&huart2, (uint8_t *) "AT+CGMM\r\n",strlen("AT+CGMM\r\n"),HAL_MAX_DELAY) != HAL_OK)
//		  {
//			  printf("\n Txn Failed");
//		  }
//		  HAL_Delay(5000);
////
//		  if (strstr((char *) g_buff, "OK"))
//		  {
//			  printf("\n Response OK: %s",(char *)g_buff);
//		  }
//	  }
//	  HAL_Delay(5000);
//	  ModemTxReady = RESET;
//	  }
			//		  HAL_GPIO_TogglePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : OnBoardLED_Pin */
  GPIO_InitStruct.Pin = OnBoardLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OnBoardLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RainGuagePulseInput_Pin */
  GPIO_InitStruct.Pin = RainGuagePulseInput_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RainGuagePulseInput_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
   @brief  Rx reception completed callback
   @param  UartHandle: UART handle.
   @note   This callback executes once defined bytes of reception completed
   @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//	  if (UartHandle->Instance == USART1)
//	  {
////		    HAL_UART_Receive_IT(&huart1, (uint8_t *) &aRxBuffer, 1);
//	  }
	  if (UartHandle->Instance == USART2)
	  {
		    HAL_UART_Receive_IT(&huart2, (uint8_t *) &aRxBuffer, 1);
		    g_buff[wr_ptr++] = aRxBuffer;
//		    printf("%c", aRxBuffer);
	  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	  if (UartHandle->Instance == USART1)
	  {
	    /* Set transmission flag: transfer complete */
		  SerialTxReady = SET;
	  }
	  if (UartHandle->Instance == USART2)
	  {
	    /* Set transmission flag: transfer complete */
		  ModemTxReady = SET;
	  }
}

/**
   @brief EXTI line detection callbacks
   @param GPIO_Pin: Specifies the pins connected EXTI line
   @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

  if (GPIO_Pin == RainGuagePulseInput_Pin)
  {
    if (HAL_GPIO_ReadPin(RainGuagePulseInput_GPIO_Port, RainGuagePulseInput_Pin)
         == GPIO_PIN_SET)
    {
    	HAL_GPIO_TogglePin(OnBoardLED_GPIO_Port, OnBoardLED_Pin);
    }
  }
}

/**
   @brief  Retargets the C library APP_DEBUG_STR function to the USART.
   @param  None
   @retval None
*/
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *) &ch, 1, 0XFFFF);
  return ch;
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
