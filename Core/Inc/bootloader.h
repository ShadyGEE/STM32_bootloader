/*
 * bootloader.h
 * for STM32
 * author : Shady Ghonim
 *
 */

#ifndef BOOTLOADER_H_
#define BOOTLOADER_H_


#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "crc.h"
#include "usart.h"

#define CRC_VERIFYING_FAILED  0x00U
#define CRC_VERIFYING_PASS  0x01U

#define SEND_NACK  0xAB
#define SEND_ACK  0xCD

#define CBL_GET_VER_CMD   0x10U
#define CBL_GET_HELP_CMD  0x11U
#define CBL_GET_CID_CMD  0x12U
#define CBL_GO_TO_ADDER_CMD  0x14U
#define CBL_FLASH_ERASE_CMD  0x15U
#define CBL_MEM_WRITE_CMD  0x16U

#define CBL_VENDOR_ID         100U
#define CBL_SW_MAJOR_VERSION    1U
#define CBL_SW_MINOR_VERSION    1U
#define CBL_SW_PATCH_VERSION    0U

#define INVALID_PAGE_NUMBER         0x00U
#define VALID_PAGE_NUMBER           0x01U
#define UNSUCCESSFUL_ERASE          0x02U
#define SUCCESSFUL_ERASE            0x03U

#define CBL_FLASH_MAX_PAGE_NUMBER 16U
#define CBL_FLASH_MASS_ERASE    0xFFU
#define HAL_SUCCESSFUL_ERASE  0xFFFFFFFFU

#define ADDRESS_IS_INVALID       0x00U
#define ADDRESS_IS_VALID         0x01U


#define STM32F103_SRAM_SIZE (20*1024)
#define STM32F103_FLASH_SIZE (64*1024)
#define STM32F103_SRAM_END (SRAM_BASE + STM32F103_SRAM_SIZE )
#define STM32F103_FLASH_END (FLASH_BASE + STM32F103_SRAM_SIZE )

#define FLASH_PAYLOAD_WRITE_FAILED 0x00U
#define FLASH_PAYLOAD_WRITE_SUCESS 0x01U



#define HOST_MAX_SIZE 200U
typedef enum
{
	BL_NACK =0U ,
	BL_ACK =1U
}BL_STATUS;
void vBL_SendMsg(char * format , ...);
BL_STATUS BL_FetchHostCommand();



#endif


