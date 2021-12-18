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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <string.h>
#include "bootloader.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum
{
	DATA_TYPE_8=0,
	DATA_TYPE_16,
	DATA_TYPE_32,
}DataTypeDef;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void bootloaderInit(void);
void errorBlink(void);
void successBlink(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t MY_SectorAddrs;
uint8_t MY_SectorNum;

 void MY_FLASH_EraseSector(void)
{
	HAL_FLASH_Unlock();
	//Erase the required Flash sector
	FLASH_Erase_Sector(MY_SectorNum, FLASH_VOLTAGE_RANGE_3);
	HAL_FLASH_Lock();
}


void MY_FLASH_SetSectorAddrs(uint8_t sector, uint32_t addrs)
{
	MY_SectorNum = sector;
	MY_SectorAddrs = addrs;
}

// https://github.com/MYaqoobEmbedded/STM32-Tutorials/blob/master/Tutorial%2030%20-%20FLASH%20Memory/MY_FLASH.c
void MY_FLASH_WriteN(uint32_t idx, void *wrBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = MY_SectorAddrs + idx;

	//Erase sector before write
	MY_FLASH_EraseSector();

	//Unlock Flash
	HAL_FLASH_Unlock();
	//Write to Flash
	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress , ((uint8_t *)wrBuf)[i]);
					flashAddress++;
				}
			break;

		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress , ((uint16_t *)wrBuf)[i]);
					flashAddress+=2;
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress , ((uint32_t *)wrBuf)[i]);
					flashAddress+=4;
				}
			break;
	}
	//Lock the Flash space
	HAL_FLASH_Lock();
}


void MY_FLASH_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = MY_SectorAddrs + idx;

	switch(dataType)
	{
		case DATA_TYPE_8:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint8_t *)rdBuf + i) = *(uint8_t *)flashAddress;
					flashAddress++;
				}
			break;

		case DATA_TYPE_16:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint16_t *)rdBuf + i) = *(uint16_t *)flashAddress;
					flashAddress+=2;
				}
			break;

		case DATA_TYPE_32:
				for(uint32_t i=0; i<Nsize; i++)
				{
					*((uint32_t *)rdBuf + i) = *(uint32_t *)flashAddress;
					flashAddress+=4;
				}
			break;
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  // when trying to program the board without this st link fails with
  // error DEV_TARGET_NOT_HALTED, as it tries to erase all the flash
  // memory and could not as the flash controller is busy with this
  // little procedure
  HAL_Delay(5000);

  char tw[50];
  memset(tw, 0, sizeof(tw));
  strcpy(tw, "Testing");

  size_t tl = strlen(tw);

  MY_FLASH_SetSectorAddrs(7, 0x8060000);
  // MY_FLASH_WriteN(0, (uint8_t *) tw, (uint32_t) tl, DATA_TYPE_8);

  char tr[50];
  memset(tr, 0, sizeof(tr));

  bootloaderInit();

  // MY_FLASH_ReadN(0, (uint8_t *) tr, (uint32_t) tl, DATA_TYPE_8);
  /* USER CODE END 2 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void bootloaderInit()
{
	Flashed_offset = 0;
	flashStatus = Unerased;
	BootloaderMode bootloaderMode;

	// based on pin value, determine bootloader mode
	if(HAL_GPIO_ReadPin(BOOT1_GPIO_Port, BOOT1_Pin) == GPIO_PIN_SET) {
		bootloaderMode = FlashMode;

		// here we need to enable uart (or actually read all the data???)

	} else {
		bootloaderMode = JumpMode;
	}

	if(bootloaderMode == JumpMode)
	{
		char tr[4];
		memset(tr, 0, sizeof(tr));

		uint8_t emptyCellCount = 0;
		uint32_t idx = 0;

		// TODO: ??? how many iterations should we conduct there?
		for(uint8_t i=0; i<10; i++)
		{
			MY_FLASH_ReadN(idx, (uint8_t *) tr, 4, DATA_TYPE_8);
			uint32_t word = tr[0] | (tr[1] << 8) | (tr[2] << 16) | (tr[3] << 24);
			if(word == 0xffffffff) {
				emptyCellCount++;
			}

			idx += 4;
		}

		if(emptyCellCount != 10) {
			// jumpToApp(APP1_START);
			successBlink();
		} else {
			errorBlink();
		}
	}
}

void errorBlink() {
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
}

void successBlink() {
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
