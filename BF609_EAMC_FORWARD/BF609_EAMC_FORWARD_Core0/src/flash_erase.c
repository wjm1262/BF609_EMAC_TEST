/*
 * update.c
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

/*
 * version update module
 */
#include "flash_erase.h"

#include "xl-6004_forward_protocol.h"  //update program protocol
#include "common/dpia.h"
#include "common/flash_errors.h"
#include "flash/pc28f128P33.h"
#include "Flash/Flash_Init.h"

#include <services/gpio/adi_gpio.h>
#include <drivers/ethernet/adi_ether.h>
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

