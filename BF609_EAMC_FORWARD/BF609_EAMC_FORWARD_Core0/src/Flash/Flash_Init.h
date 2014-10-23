/*
 * Flash_Init.h
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

#ifndef FLASH_INIT_H_
#define FLASH_INIT_H_


#define myCLKIN 16.384  /* MHz, define the frequency of OSC used in fact */
/*
 *   Function:    Init_ParallelFlash
 *   Description: This function initializes parallel flash.
 */
void Flash_Init(void);
void Flash_Reset(void);

#endif /* FLASH_INIT_H_ */
