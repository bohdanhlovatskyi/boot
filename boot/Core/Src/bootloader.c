/*
 * bootloader.c
 *      https://github.com/viktorvano/STM32-Bootloader/blob/master/STM32F103C8T6_Bootloader/Core/Src/bootloader.c
 */

#include "bootloader.h"
#include "usart.h"

void jumpToApp(const uint32_t address)
{
	const JumpStruct* vector_p = (JumpStruct*)address;

	deinitEverything();

	/* let's do The Jump! */
    /* Jump, used asm to avoid stack optimization */
    asm("msr msp, %0; bx %1;" : : "r"(vector_p->stack_addr), "r"(vector_p->func_p));
}

void deinitEverything()
{
	//-- reset peripherals to guarantee flawless start of user application

	// TODO: deinit everything properly
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
}
