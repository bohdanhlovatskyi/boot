/*
 * bootloader.h
 */

#ifndef INC_BOOTLOADER_H_
#define INC_BOOTLOADER_H_

#include "main.h"
#include <string.h>

#define APP1_START (0x8005000)			//Origin + Bootloader size (20kB)

typedef enum
{
    JumpMode,
	FlashMode
} BootloaderMode;

typedef enum
{
    Unerased,
	Erased,
	Unlocked,
	Locked
} FlashStatus;

typedef void (application_t)(void);

typedef struct
{
    uint32_t		stack_addr;     // Stack Pointer
    application_t*	func_p;        // Program Counter
} JumpStruct;

uint32_t Flashed_offset;
FlashStatus flashStatus;

void jumpToApp();
void deinitEverything();

#endif /* INC_BOOTLOADER_H_ */
