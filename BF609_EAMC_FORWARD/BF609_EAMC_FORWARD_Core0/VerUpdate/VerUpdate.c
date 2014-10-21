/*
 * update.c
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

/*
 * version update module
 */
#include "BF609_Update_Protocol.h"  //update program protocol
#include "common/dpia.h"
#include "common/flash_errors.h"
#include "flash/pc28f128P33.h"
#include "Flash/Flash_Init.h"
#include "VerUpdate.h"

#include <services/gpio/adi_gpio.h>
#include <stdio.h>
#include <string.h>
#include <adi_osal.h>

#define FLASH_START_ADDR 0xB0000000

#include <stdlib.h>
uint8_t *g_pLdrDataBuff = NULL;//Global parameter, pointer of ldr data buffer

/* init the LED service
 * used for indicating the update process
  */
void static  myLED_KEY_Init(void)
 {
	 ADI_GPIO_RESULT result;
	 static uint8_t gpioMemory[32];
	 uint32_t gpioMaxCallbacks;

	 result = adi_gpio_Init(
			(void*)gpioMemory,
			32,
			&gpioMaxCallbacks);
		if (result != ADI_GPIO_SUCCESS) {
			printf("%s failed\n", result);
		}
	/* set GPIO output LED 1 */
	result = adi_gpio_SetDirection(
		ADI_GPIO_PORT_G,
		ADI_GPIO_PIN_13,
		ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS) {
		printf("%s failed\n", result);
	}
	/* LED1 */
	result = adi_gpio_Set(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
	if (result != ADI_GPIO_SUCCESS) {
		printf("%s failed\n", result);
	}
 }
/*
 * CRC16 calculate (int start, char *p, int n)
 */
uint16_t GetCrc16(int start,unsigned char *p, int n)
{
  int crc = start;
  register int r;
  static uint16_t crc_16_table[16] = {
	  0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
	  0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
	};

   /* while there is more data to process */
   while (n-- > 0)
   {
     /* compute checksum of lower four bits of *p */
     r = crc_16_table[crc & 0xF];
     crc = (crc >> 4) & 0x0FFF;
     crc = crc ^ r ^ crc_16_table[*p & 0xF];

     /* now compute checksum of upper four bits of *p */
     r = crc_16_table[crc & 0xF];
     crc = (crc >> 4) & 0x0FFF;
     crc = crc ^ r ^ crc_16_table[(*p >> 4) & 0xF];

     /* next... */
     p++;
   }

   return(crc);
}
/*
 * calculate the CheckSum
 */
uint8_t GetCheckSum8(int start, uint8_t *pucData, int len)
{
	uint8_t CheckSum = 0;
	int i = 0;
	for(i = start; i < (start + len); i++)
	{
		CheckSum += pucData[i];
	}

	return CheckSum;
}
/*
 * flash erase use buffer
 */
void FlashErase(uint8_t *pBuffer, size_t len)
{
	 COMMAND_STRUCT CmdBuffer;
	 ERROR_CODE Result = NO_ERR;

	 Flash_Init();//setup the pin mux and init flash
	 myLED_KEY_Init();

	 adi_gpio_Clear(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);/* reset the indicated LED */

	/*
	 * 0-0x20000 (16K*2Byte*4Block),
	 * 0x20000- 7FFFFF(64K*2Byte*127Block)
     * calculate the number of block to be erased.
     */
	 CmdBuffer.SEraseSect.ulFlashStartAddr = FLASH_START_ADDR;
	 for(int block = 0; block < (len /(128*1024) + 4); block++)
	 {
		 CmdBuffer.SEraseSect.nSectorNum       = block;
		 Result = pc28f128P33_Control(CNTRL_ERASE_SECT, &CmdBuffer);
	 }

	 if(Result == NO_ERR) //write the buffer to 16-width flash
		 Result = pc28f128P33_Write((uint16_t *)pBuffer, FLASH_START_ADDR, len / 2);

	 adi_gpio_Set(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);/*indicate the erase process.*/

	if(Result == NO_ERR) //Software Reset
	{
		 Flash_Reset();
		*pREG_RCU0_CTL |= 0x01;
	}
}
/*
 * update the version
 * @ *pBuf: the pointer of BF609 control frame
 */
BF609_COMM_ACK_CODE VerUpdate(void *pBuf)
{
	BF609_COMM_ACK_CODE ret;

	BF609_ETHE_FRAME *pRxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrData;	//control data ptr
	LDR_FRAME        *pLdrFrame;	//LDR
	uint8_t 		 *pLdrPartData;

	uint8_t  CheckSum;

	uint8_t Ctr_Cmd;
	uint32_t DataLen;
	uint32_t LDR_Index;
	uint32_t LDR_Len;
	uint16_t CRC16 ;

	pRxData   = (BF609_ETHE_FRAME *)pBuf;
	uint8_t  BF609_Cmd = pRxData->CtrCmd;
	switch(BF609_Cmd)
	{
		case BF609_CTR://0x00£¬¿ØÖÆÐÅÏ¢Ö¡
		{
			 pCtrData = (CONTROL_FRAME *)(pRxData->Data);
			 Ctr_Cmd  = pCtrData->CtrCmd;
			 if(Ctr_Cmd == VERSION_UPDATE) 		// Update version
			 {
				 // get the ldr info in frame
				 pLdrFrame    = (LDR_FRAME *)(pCtrData->CtrData);
				 DataLen     = pLdrFrame->DataLen;
				 LDR_Index   = pLdrFrame->LDR_Index;

				 // get the length of ldr file, and malloc memory to save
				 LDR_Len = pLdrFrame->LDR_Len;

				 if( LDR_Index > LDR_Len )
				 {
					 return NAK_ERROR_INDEX;
				 }

				 if(g_pLdrDataBuff == NULL)  // malloc only in the first time
				 {
					 g_pLdrDataBuff = (uint8_t *)heap_malloc(LDR_DATA_BUFFER_LOCATE_HEAP, LDR_Len);

					 if(g_pLdrDataBuff == NULL)
					 {
						 printf("alloc memory for 'g_pLdrDataBuff: %u bytes'£º failed!\n", LDR_Len);
					 }
				 }

				 //check the checksum,from MU_addr to last byte of data field.
				 pLdrPartData = (uint8_t *)(pLdrFrame->PartData);
				 CheckSum   = GetCheckSum8(0, (uint8_t *)pLdrFrame, sizeof(*pLdrFrame)) +\
                                       GetCheckSum8(0, pLdrPartData, DataLen);
				 CheckSum = CheckSum + pCtrData->MU_Addr + pCtrData->CtrCmd;

				 //copy frame data to update buffer if CheckSum is correct.
				 if( CheckSum == pLdrPartData[DataLen])
				 {
					 memcpy( g_pLdrDataBuff + LDR_Index, pLdrPartData, DataLen);
				 }
				 else
				 {
					 return NAK_ERROR_FRM_CHKSUM;
				 }

				 //last frame,completed receive LDR file data,erase the flash
				 if(DataLen == (LDR_Len-LDR_Index))
				 {
					 CRC16 = GetCrc16(0, g_pLdrDataBuff, LDR_Len);
					 if(CRC16 == (pLdrFrame->LDR_CRC)) //erase flash if CRC correct
					 {
						 adi_osal_EnterCriticalRegion();
						 FlashErase(g_pLdrDataBuff, LDR_Len);
						 adi_osal_ExitCriticalRegion();
					 }
					 else
					 {
						 return NAK_ERROR_FILE_CRC;
					 }
				 }
			}//VERSION_UPDATE


			break;
		}//BF609_CTR
		default:
			ret = NAK_ERROR_UNKNOWN_COMMAND;
			break;
	}//Ctr_Cmd

	return ret;
}
