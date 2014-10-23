/*
 * flash_erase.h
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

#ifndef FLASH_ERASE_H_
#define FLASH_ERASE_H_

#include "stdint.h"
#include "stddef.h"

extern uint8_t *g_pLdrDataBuff;

void FlashErase(uint8_t *pBuffer, size_t len);

#endif /* VERUPDATE_H_ */
