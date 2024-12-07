/*
 * bootloader.c
 * for STM32
 * author : Shady Ghonim
 *
 */

#include "bootloader.h"
static uint32_t u8BL_CRC_Verify(uint8_t * pdata , uint32_t DataLen , uint32_t HostCRC);
static void vBL_SendACK(uint8_t dataLen);
static void vBL_SendNACK();
static void vBL_GetVer(uint8_t * HostBuffer );
static void vBL_GetHelp(uint8_t * HostBuffer );
static void vBL_GetChipID(uint8_t * HostBuffer );
static void vBL_FlashErase(uint8_t * HostBuffer );
static void vBL_Write_Data(uint8_t * HostBuffer );
static uint8_t u8BL_AdressVerification(uint32_t Address);
static void vBL_JumpToAddress(uint8_t *HostBuffer);
static uint8_t u8BL_FlashMemoryPayloadWrite(uint16_t * pdata,uint32_t StartAddress, uint8_t PayloadLen);

static uint32_t Perform_Flash_Erase(uint32_t PageAddress , uint8_t PageNumber);
uint16_t counter_;
void vBL_SendMsg(char * format , ...)
{
	char message [100U]={0U};
	va_list args ;
	va_start(args,format);
	vsprintf(message,format,args);
	HAL_UART_Transmit(&huart2,(uint8_t*)message,sizeof(message),HAL_MAX_DELAY);
	va_end(args);

}
static uint8_t Host_buffer[HOST_MAX_SIZE];
BL_STATUS BL_FetchHostCommand()
{

   BL_STATUS status = BL_NACK;

   HAL_StatusTypeDef HAL_status = HAL_ERROR;
   uint8_t DataLen = 0U;
   memset(Host_buffer,0U,HOST_MAX_SIZE);  // setting host buffer to zeros
   HAL_status = HAL_UART_Receive(&huart2,&Host_buffer,1,HAL_MAX_DELAY); // at first getting command size from host in hal status
   if (HAL_status!=HAL_OK) // check
   {
	   status = BL_NACK; //not ok
   }
   else
   {
	   // status is ok
	   DataLen = Host_buffer[0U]; // assign data length to the input from host
	   HAL_status = HAL_UART_Receive(&huart2,&Host_buffer[1],DataLen,HAL_MAX_DELAY);
	   if (HAL_status!=HAL_OK) // check
	      {
	   	   status = BL_NACK; //not ok
	      }
	      else
	      {

	    	  switch (Host_buffer[1])
	    	  {
	    	  case CBL_GET_VER_CMD : vBL_GetVer(Host_buffer); break;
	    	  case CBL_GET_HELP_CMD :vBL_GetHelp(Host_buffer); break;
	    	  case CBL_GET_CID_CMD : vBL_GetChipID(Host_buffer);break;
	    	  case CBL_GO_TO_ADDER_CMD :vBL_JumpToAddress(Host_buffer); break;
	    	  case CBL_FLASH_ERASE_CMD :vBL_FlashErase(Host_buffer); break;
	    	  case CBL_MEM_WRITE_CMD : vBL_Write_Data(Host_buffer);break;
	    	  default: status = BL_NACK; ;break;

	    	  }

	      }
	   }

   }

static uint32_t u8BL_CRC_Verify(uint8_t * pdata , uint32_t DataLen , uint32_t HostCRC)
{
	uint8_t CRC_status = CRC_VERIFYING_FAILED   ;
	uint32_t MCU_CRC = 0U;
	uint32_t dataBuffer = 0U;
	for (uint16_t count = 0U; count < DataLen ; count ++)
	{
		dataBuffer = (uint32_t)pdata[count];
		MCU_CRC = HAL_CRC_Accumulate(&hcrc ,dataBuffer ,1U);
	}
	__HAL_CRC_DR_RESET(&hcrc);
	if(HostCRC == MCU_CRC )
	{
		CRC_status = CRC_VERIFYING_PASS;

	}
	else
	{
		CRC_status = CRC_VERIFYING_FAILED;
	}
	return CRC_status;
}
static void vBL_SendACK(uint8_t dataLen)
{
	uint8_t u8Local_AckValue[2U] = {0U};
	u8Local_AckValue[0]=SEND_ACK;


	u8Local_AckValue[1]=dataLen;

	HAL_UART_Transmit(&huart2,(uint8_t*)u8Local_AckValue[1U],2U,HAL_MAX_DELAY);



}
static void vBL_GetHelp(uint8_t * HostBuffer )
{
	uint8_t BL_supported_CMDs[] = {
			CBL_GET_VER_CMD,
			CBL_GET_HELP_CMD,
			CBL_GET_CID_CMD,
			CBL_GO_TO_ADDER_CMD,
			CBL_FLASH_ERASE_CMD,
			CBL_MEM_WRITE_CMD};
	uint16_t u16Local_host_packet_length =0U;
	uint32_t u32Local_CRC_value = 0U;
	u16Local_host_packet_length = HostBuffer[0U]+1U;
	u32Local_CRC_value =(uint32_t *) (HostBuffer + u16Local_host_packet_length - 4U );
	if (CRC_VERIFYING_PASS == u8BL_CRC_Verify ((uint8_t*)&HostBuffer[0U] ,  u16Local_host_packet_length - 4U , u32Local_CRC_value ))
	{
		vBL_SendACK(6);
		HAL_UART_Transmit(&huart2,(uint8_t*)BL_supported_CMDs,4 ,HAL_MAX_DELAY);

	}
	else
	{
		vBL_SendNACK();// NACK
	}
}
static void vBL_SendNACK()
{
	uint8_t u8Local_AckValue = SEND_NACK;

		HAL_UART_Transmit(&huart2,&u8Local_AckValue,sizeof(u8Local_AckValue),HAL_MAX_DELAY);

}

static void vBL_GetVer(uint8_t * HostBuffer )
{
	uint8_t u8Local_version[4U]= {CBL_VENDOR_ID,CBL_SW_MAJOR_VERSION,CBL_SW_MINOR_VERSION,CBL_SW_PATCH_VERSION};
	uint16_t u16Local_host_packet_length =0U;
	uint32_t u32Local_CRC_value = 0U;
	u16Local_host_packet_length = HostBuffer[0U]+1U;
	u32Local_CRC_value =(uint32_t *) (HostBuffer + u16Local_host_packet_length - 4U );
	if (CRC_VERIFYING_PASS == u8BL_CRC_Verify ((uint8_t*)&HostBuffer[0U] ,  u16Local_host_packet_length - 4U , u32Local_CRC_value ))
	{
		vBL_SendACK(4);
		HAL_UART_Transmit(&huart2,(uint8_t*)u8Local_version,4 ,HAL_MAX_DELAY);

	}
	else
	{
		vBL_SendNACK();// NACK
	}
}

static void vBL_GetChipID(uint8_t * HostBuffer )
{
	uint16_t Chip_ID = 0;
	uint16_t u16Local_host_packet_length =0U;
	uint32_t u32Local_CRC_value = 0U;
	u16Local_host_packet_length = HostBuffer[0U]+1U;
	u32Local_CRC_value =(uint32_t *) (HostBuffer + u16Local_host_packet_length - 4U );
	if (CRC_VERIFYING_PASS == u8BL_CRC_Verify ((uint8_t*)&HostBuffer[0U] ,  u16Local_host_packet_length - 4U , u32Local_CRC_value ))
	{
		Chip_ID = (uint16_t)(DBGMCU -> IDCODE & 0x0000FFFFU);
		vBL_SendACK(2);
		HAL_UART_Transmit(&huart2,(uint8_t*)&Chip_ID,2 ,HAL_MAX_DELAY);

	}
	else
	{
		vBL_SendNACK();// NACK
	}

}
static void vBL_FlashErase(uint8_t * HostBuffer )
{
	uint32_t u32Local_EraseStatus = UNSUCCESSFUL_ERASE ;
	uint16_t u16Local_host_packet_length =0U;
	uint32_t u32Local_CRC_value = 0U;
	u16Local_host_packet_length = HostBuffer[0U]+1U;
	u32Local_CRC_value =(uint32_t *) (HostBuffer + u16Local_host_packet_length - 4U );
	if (CRC_VERIFYING_PASS == u8BL_CRC_Verify ((uint8_t*)&HostBuffer[0U] ,  u16Local_host_packet_length - 4U , u32Local_CRC_value ))
	{
		u32Local_EraseStatus=Perform_Flash_Erase(*((uint32_t*)&HostBuffer[7]) , HostBuffer[6]);
		vBL_SendACK(4);
		HAL_UART_Transmit(&huart2,(uint32_t*)&u32Local_EraseStatus,4,HAL_MAX_DELAY);

	}
	else
	{
		vBL_SendNACK();// NACK
	}


}

static uint32_t Perform_Flash_Erase(uint32_t PageAddress , uint8_t PageNumber)
{
	FLASH_EraseInitTypeDef pEraseInit;
	HAL_StatusTypeDef HAL_status =HAL_ERROR;
	uint32_t PageError = 0U;
	uint8_t u8Local_PageStatus = INVALID_PAGE_NUMBER;
	if (PageNumber > CBL_FLASH_MAX_PAGE_NUMBER)
	{
		u8Local_PageStatus =INVALID_PAGE_NUMBER;
	}
	else
	{
		u8Local_PageStatus =VALID_PAGE_NUMBER;

		if(PageNumber <= (CBL_FLASH_MAX_PAGE_NUMBER -1 )||PageAddress==CBL_FLASH_MASS_ERASE)
		{
			if(PageAddress == CBL_FLASH_MASS_ERASE)
			{
				pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
				pEraseInit.Banks = FLASH_BANK_1;
				pEraseInit.PageAddress = 0x8008000;
				pEraseInit.NbPages = 12;

			}
			else
			{
				pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
				pEraseInit.Banks = FLASH_BANK_1;
				pEraseInit.PageAddress = PageAddress;
				pEraseInit.NbPages = PageNumber;

			}
			HAL_FLASH_Unlock();
			HAL_status=HAL_FLASHEx_Erase(&pEraseInit,&PageError);
			HAL_FLASH_Lock();
			if(PageError==HAL_SUCCESSFUL_ERASE)
			{
				u8Local_PageStatus = SUCCESSFUL_ERASE;

			}
			else
			{
				u8Local_PageStatus = UNSUCCESSFUL_ERASE;
			}
		}

		else
		{
			u8Local_PageStatus = INVALID_PAGE_NUMBER;
		}


	}
	return u8Local_PageStatus;
}
static void vBL_Write_Data(uint8_t * HostBuffer )
{
	uint8_t u8Local_AddressVerify = ADDRESS_IS_INVALID;
	uint32_t Address_Host =0U;
	uint8_t Data_Length=0U;
	uint16_t u16Local_host_packet_length =0U;
	uint32_t u32Local_CRC_value = 0U;
	uint8_t payload_status= FLASH_PAYLOAD_WRITE_FAILED;
	u16Local_host_packet_length = HostBuffer[0U]+1U;
	u32Local_CRC_value =(uint32_t *) (HostBuffer + u16Local_host_packet_length - 4U );
	if (CRC_VERIFYING_PASS == u8BL_CRC_Verify ((uint8_t*)&HostBuffer[0U] ,  u16Local_host_packet_length - 4U , u32Local_CRC_value ))
	{
		vBL_SendACK(1);
		Address_Host =*((uint32_t*)&Host_buffer[2])+(64 * counter_);
		counter_++;
		Data_Length = Host_buffer[6];
		u8Local_AddressVerify=u8BL_AdressVerification(Address_Host);
		if(u8Local_AddressVerify == ADDRESS_IS_VALID )
		{
			payload_status = u8BL_FlashMemoryPayloadWrite((uint16_t)&HostBuffer[7],Address_Host,Data_Length);
			HAL_UART_Transmit(&huart2,(uint8_t*)&payload_status,1,HAL_MAX_DELAY);

		}
		else
		{
			HAL_UART_Transmit(&huart2,(uint8_t*)&u8Local_AddressVerify,1,HAL_MAX_DELAY);
		}
		vBL_SendACK(4);


	}
	else
	{
		vBL_SendNACK();// NACK
	}



}
static uint8_t u8BL_AdressVerification(uint32_t Address)
{
	uint8_t u8Local_AddressVerify = ADDRESS_IS_INVALID;
	if(Address >= FLASH_BASE  && Address<= STM32F103_FLASH_END)
	{
		uint8_t u8Local_AddressVerify = ADDRESS_IS_VALID;//valid address
	}
	else if (Address >= SRAM_BASE  && Address<= STM32F103_SRAM_END)
	{
		uint8_t u8Local_AddressVerify = ADDRESS_IS_VALID; //valid address
	}
	else
	{
		uint8_t u8Local_AddressVerify = ADDRESS_IS_INVALID;
	}
}
static uint8_t u8BL_FlashMemoryPayloadWrite(uint16_t * pdata,uint32_t StartAddress, uint8_t PayloadLen)
{
    uint32_t Address=0U;
    uint8_t UpdateDataAdress=0U;
    HAL_StatusTypeDef Hal_status = HAL_ERROR;
    uint8_t payload_status= FLASH_PAYLOAD_WRITE_FAILED;



	HAL_FLASH_Unlock();
	for (uint8_t payload_count =0 , UpdateDataAdress = 0 ; payload_count<(PayloadLen/2);payload_count++ , UpdateDataAdress+2)
	{
		Address = StartAddress + UpdateDataAdress;
		Hal_status =HAL_FLASH_PROGRAM(FLASH_TYPEPROGRAM_HALFWORD,StartAddress , pdata[payload_count] );
		if (Hal_status != HAL_OK )
		{
			payload_status = FLASH_PAYLOAD_WRITE_FAILED;
		}
		else
		{
			payload_status = FLASH_PAYLOAD_WRITE_SUCESS;
		}

	}
	return payload_status;
}
static void vBL_JumpToApplication(uint8_t *HostBuffer)
{
    uint32_t jumpAddress;
    void (*jumpToApplication)(void);
    uint16_t u16Local_host_packet_length = 0U;
    uint32_t u32Local_CRC_value = 0U;

    // Get packet length and CRC value
    u16Local_host_packet_length = HostBuffer[0U] + 1U;
    u32Local_CRC_value = *((uint32_t *)(HostBuffer + u16Local_host_packet_length - 4U));

    // Validate CRC
    if (CRC_VERIFYING_PASS == u8BL_CRC_Verify((uint8_t *)&HostBuffer[0U], u16Local_host_packet_length - 4U, u32Local_CRC_value))
    {
        vBL_SendACK(1);

        // Extract the address from the HostBuffer
        jumpAddress = *((uint32_t *)&HostBuffer[2]);

        // Verify the address
        if (u8BL_AdressVerification(jumpAddress) == ADDRESS_IS_VALID)
        {
            // Deinitialize all used peripherals
            HAL_UART_DeInit(&huart2);
            HAL_CRC_DeInit(&hcrc);
            HAL_NVIC_DisableIRQ(SysTick_IRQn);

            // Set the vector table to the application start address
            SCB->VTOR = jumpAddress;

            // Set jump address and jump to application
            jumpToApplication = (void (*)(void))(*((uint32_t *)(jumpAddress + 4U)));
            __set_MSP(*((uint32_t *)jumpAddress)); // Set Main Stack Pointer
            jumpToApplication();
        }
        else
        {
            vBL_SendNACK();
        }
    }
    else
    {
        vBL_SendNACK();
    }
}
