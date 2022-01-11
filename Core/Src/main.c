/*USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *<h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 *All rights reserved.</center></h2>
 *
 *This software component is licensed by ST under Ultimate Liberty license
 *SLA0044, the "License"; You may not use this file except in compliance with
 *the License. You may obtain a copy of the License at:
 *                            www.st.com/SLA0044
 *
 ******************************************************************************
 */
/*USER CODE END Header */
/*Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


/*Private includes ----------------------------------------------------------*/
/*USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/*USER CODE END Includes */

/*Private typedef -----------------------------------------------------------*/
/*USER CODE BEGIN PTD */

/*USER CODE END PTD */

/*Private define ------------------------------------------------------------*/
/*USER CODE BEGIN PD */
// 70 kb for programm
#define MAIN_PROGRAM_START_ADDRESS 0x8060000
#define MAIN_PROGRAM_END_ADDRESS 0x807FFFF
#define FW_START 5
#define FW_READ 1000
#define FW_WRITE 2000
#define FW_FINISH 10000
#define FW_ERROR 100000

typedef enum
{
	DATA_TYPE_8 = 0,
		DATA_TYPE_16,
		DATA_TYPE_32,
}
DataTypeDef;

typedef void (application_t)(void);

typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

const char fileName[] = "FIRMWARE.bin";

/*USER CODE END PD */

/*Private macro -------------------------------------------------------------*/
/*USER CODE BEGIN PM */

/*USER CODE END PM */

/*Private variables ---------------------------------------------------------*/

/*USER CODE BEGIN PV */

/*USER CODE END PV */

/*Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/*USER CODE BEGIN PFP */
void myprintf(const char *fmt);
void ExecMainFW(void);
/*USER CODE END PFP */

/*Private user code ---------------------------------------------------------*/
/*USER CODE BEGIN 0 */

#define myprintf(...) sprintf((char*) msg, __VA_ARGS__);\
		HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg), HAL_MAX_DELAY);

static char msg[256];

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

	//Unlock Flash
	HAL_FLASH_Unlock();
	//Write to Flash
	switch (dataType)
	{
		case DATA_TYPE_8:
			for (uint32_t i = 0; i < Nsize; i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, flashAddress, ((uint8_t*) wrBuf)[i]);
				flashAddress++;
			}
			break;

		case DATA_TYPE_16:
			for (uint32_t i = 0; i < Nsize; i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, flashAddress, ((uint16_t*) wrBuf)[i]);
				flashAddress += 2;
			}
			break;

		case DATA_TYPE_32:
			for (uint32_t i = 0; i < Nsize; i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, flashAddress, ((uint32_t*) wrBuf)[i]);
				flashAddress += 4;
			}
			break;
	}
	//Lock the Flash space
	HAL_FLASH_Lock();
}

void MY_FLASH_ReadN(uint32_t idx, void *rdBuf, uint32_t Nsize, DataTypeDef dataType)
{
	uint32_t flashAddress = MY_SectorAddrs + idx;

	switch (dataType)
	{
		case DATA_TYPE_8:
			for (uint32_t i = 0; i < Nsize; i++)
			{ 	*((uint8_t*) rdBuf + i) = *(uint8_t*) flashAddress;
				flashAddress++;
			}
			break;

		case DATA_TYPE_16:
			for (uint32_t i = 0; i < Nsize; i++)
			{ 	*((uint16_t*) rdBuf + i) = *(uint16_t*) flashAddress;
				flashAddress += 2;
			}
			break;

		case DATA_TYPE_32:
			for (uint32_t i = 0; i < Nsize; i++)
			{ 	*((uint32_t*) rdBuf + i) = *(uint32_t*) flashAddress;
				flashAddress += 4;
			}
			break;
	}
}

void errorBlink()
{
	HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_SET);
}

void successBlink()
{
	HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);
}
/*USER CODE END 0 */

/**
 *@brief  The application entry point.
 *@retval int
 */
int main(void)
{
	/*USER CODE BEGIN 1 */

	/*USER CODE END 1 */

	/*MCU Configuration--------------------------------------------------------*/

	/*Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/*USER CODE BEGIN Init */

	/*USER CODE END Init */

	/*Configure the system clock */
	SystemClock_Config();

	/*USER CODE BEGIN SysInit */
	HAL_Delay(2000);
	/*USER CODE END SysInit */

	/*Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_SPI1_Init();
	MX_FATFS_Init();
	MX_USART2_UART_Init();
	/*USER CODE BEGIN 2 */

	uint32_t t;
	uint32_t fw_step = FW_READ;	// finite state machine currenst state

	uint32_t fw_buf[512];

	// index will be added to offset produced by the beginning sector
	uint32_t idx = 0;
	uint32_t firmware_size = 0;

	FATFS FS;
	FIL F;
	/*USER CODE END 2 */

	/*Infinite loop */
	/*USER CODE BEGIN WHILE */
	while (1)
	{
		/*USER CODE END WHILE */

		/*USER CODE BEGIN 3 */
		switch (fw_step)
		{
			case FW_READ:
				{
				 		// opens the sd card
					if (f_mount(&FS, "", 0) == FR_OK)
					{
						if (f_open(&F, fileName, FA_READ) == FR_OK)
						{
							f_lseek(&F, 0);
							myprintf("Updating firmware\n");

							myprintf("Erasing the flash segment\n");
							myprintf("------------------------------------\n");

							// we choose the last sector, so to make this as safe as possible
							MY_FLASH_SetSectorAddrs(7, MAIN_PROGRAM_START_ADDRESS);
							MY_FLASH_EraseSector();

							// TODO: do we need to add here 4 (for vector table relocation);
							// idx = MAIN_PROGRAM_START_ADDRESS;
							idx = 0;
							fw_step = FW_READ + 20;

							firmware_size = f_size(&F);
							myprintf("Size of firmware: %ld\n", firmware_size);
						}
						else
						{
							myprintf("There is no firmware file\n");
							fw_step = FW_FINISH;
						}
					}
					else
					{
						myprintf("There is no sd-card\n");
						fw_step = FW_FINISH;
					}
					break;
				}


			case FW_READ + 20:	// Flash Firmware
				{
					myprintf("Writing the firmware into flash\n");

					if (idx > firmware_size)
					{
						myprintf("Done!\n");
						f_unlink(fileName);
						// demount the device
						f_mount(NULL, "", 0);
						fw_step = FW_FINISH;
						break;
					}

					myprintf("Current index: %ld; chunk to write: %d\n", idx, sizeof(fw_buf));

					f_read(&F, &fw_buf, sizeof(fw_buf), (UINT *) &t);
					if (t != sizeof(fw_buf))
					{
						if (idx + t + sizeof(fw_buf) > firmware_size) {
							myprintf("Writing last chunk...\n");
						} else {
							myprintf("Error while reading from sd, chunk to small: %ld\n", t);
							fw_step = FW_ERROR;
							break;
						}
					}


					// for(t = 0; t < sizeof(fw_buf); t += 4)
					//    FLASH_ProgramWord(idx+t, aes_buf[t/4]);

					MY_FLASH_WriteN(idx, fw_buf, sizeof(fw_buf) / sizeof(fw_buf[0]), DATA_TYPE_32);

					// as index is counted in bytes
					idx += sizeof(fw_buf);

					break;
				}

			case FW_ERROR:
				{
					break;
				}

			case FW_FINISH:
				{
					ExecMainFW();

					break;
				}
		}
	}
	/*USER CODE END 3 */
}

/**
 *@brief System Clock Configuration
 *@retval None
 */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
	/**Initializes the RCC Oscillators according to the specified parameters
	 *in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
	/**Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
		RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
	{
		Error_Handler();
	}
}

/*USER CODE BEGIN 4 */
void ExecMainFW()
{
	// setting jump address, +4 bytes as we need to put the
	// irq table in the very beginning
	// uint32_t jumpAddress = *(__IO uint32_t *)(MAIN_PROGRAM_START_ADDRESS + 4);

	// deinit usart
	HAL_UART_DeInit(&huart2);

	// deinit spi and fatfs
	HAL_SPI_DeInit(&hspi1);

	// HAL_USART2_UART_DeInit(&huart2);
	__HAL_RCC_GPIOC_CLK_DISABLE();
	__HAL_RCC_GPIOD_CLK_DISABLE();
	__HAL_RCC_GPIOB_CLK_DISABLE();
	__HAL_RCC_GPIOA_CLK_DISABLE();
	HAL_RCC_DeInit();
	HAL_DeInit();
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;


	__disable_irq();
	// NVIC_SetVectorTable(NVIC_VectTab_FLASH, MAIN_PROGRAM_START_ADDRESS);
	// SCB->VTOR = MAIN_PROGRAM_START_ADDRESS;
	// __set_MSP(*(__IO uint32_t *) MAIN_PROGRAM_START_ADDRESS);

	const JumpStruct* vector_p = (JumpStruct*) MAIN_PROGRAM_START_ADDRESS;
	asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
}

/*USER CODE END 4 */

/**
 *@brief  This function is executed in case of error occurrence.
 *@retval None
 */
void Error_Handler(void)
{
	/*USER CODE BEGIN Error_Handler_Debug */
	/*User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {}
	/*USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
 *@brief  Reports the name of the source file and the source line number
 *        where the assert_param error has occurred.
 *@param  file: pointer to the source file name
 *@param  line: assert_param error line source number
 *@retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
	/*USER CODE BEGIN 6 */
	/*User can add his own implementation to report the file name and line number,
	   ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/*USER CODE END 6 */
}

#endif /*USE_FULL_ASSERT */

/************************(C) COPYRIGHT STMicroelectronics *****END OF FILE****/

