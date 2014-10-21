/*
 *  (C) Copyright 2012 - Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential.  By using this software
 * you agree to the terms of the associated Analog Devices License Agreement.
 *
 * Project Name:  	Power_On_Self_Test
 *
 * Hardware:		ADSP-BF609 EZ-Board
 *
 * Description:	This file tests the parallel flash on the EZ-Board.
 */

/*******************************************************************
*  include files
*******************************************************************/
#include <cdefBF609.h>
#include <ccblkfn.h>
#include <stdlib.h>
#include <math.h>
#include "../common/dpia.h"
#include "../common/flash_errors.h"
#include "pc28f128p33.h"
#include "Flash_Init.h"
typedef enum
{
  ASYNC_SRAM,
  ASYNC_MODE,
  ASYNC_PAGE_MODE,
  SYNC_MODE,
} flash_mode_t;

/*******************************************************************
*  function prototypes
*******************************************************************/
ERROR_CODE SetupForFlash(flash_mode_t mode);

/*
 *   Function:    Init_ParallelFlash
 *   Description: This function initializes parallel flash.
 */
void Flash_Init(void)
{
	/*
	 * We need first configure it as asynchronous mode
	 * before configure it as synchronous mode
	*/
	SetupForFlash(ASYNC_MODE);
	SetupForFlash(SYNC_MODE);

}
void Flash_Reset(void)
{
	/*
	 * We need Reset configure it as asynchronous mode
	*/
	COMMAND_STRUCT CmdBuffer;
	CmdBuffer.SReset.ulFlashStartAddr = 0xB0000000;
	pc28f128P33_Control(CNTRL_RESET, &CmdBuffer);
	SetupForFlash(ASYNC_MODE); //Must setup to asynchronous mode before Reset DSP core.

}
/*
 * FREQ is in MHz, CYCLE is in nanosecond
 */
ERROR_CODE myGetSCLK0(float *freq, float *cycle)
{
  ERROR_CODE ErrorCode = NO_ERR;
  int msel, csel, syssel, s0sel, df;
  uint32_t cgu_ctl, cgu_div;

  cgu_ctl = *pREG_CGU0_CTL;
  cgu_div = *pREG_CGU0_DIV;

  msel = (cgu_ctl & BITM_CGU_CTL_MSEL) >> BITP_CGU_CTL_MSEL;
  df = (cgu_ctl & BITM_CGU_CTL_DF) >> BITP_CGU_CTL_DF;
  csel = (cgu_div & BITM_CGU_DIV_CSEL) >> BITP_CGU_DIV_CSEL;
  syssel = (cgu_div & BITM_CGU_DIV_SYSSEL) >> BITP_CGU_DIV_SYSSEL;
  s0sel = (cgu_div & BITM_CGU_DIV_S0SEL) >> BITP_CGU_DIV_S0SEL;

  *freq = myCLKIN * msel / (1 + df) / syssel / s0sel;
  *cycle = 1000 / *freq;

  return ErrorCode;
}

/*
 *----------- S e t u p F o r F l a s h ( ) ----------
 *
 *  PURPOSE
 *   		Perform necessary setup for the processor to talk to the
 *   		flash such as external memory interface registers, etc.
 *
 *   RETURN VALUE
 *   	ERROR_CODE - value if any error occurs during Opcode scan
 *   	NO_ERR     - otherwise
 */
ERROR_CODE SetupForFlash(flash_mode_t mode)
{

	ERROR_CODE ErrorCode = NO_ERR;			/* return error code */
	float freq, cycle;
	int bclk;
	int wst, wht, wat, rst, rht, rat;
	int prest, preat, tt, it, pgws;
	uint32_t smc_b0ctl, smc_b0tim, smc_b0etim;
	/* For Read Configuration Register */
	int latency_count;
	float nor_clk;

	*pREG_PORTA_FER = 0x0000FFFF;
	asm(" ssync ; ");

	*pREG_PORTA_MUX = 0x00000000;
	asm(" ssync ; ");

	*pREG_PORTB_INEN = 0x00000000;
	asm(" ssync ; ");

	*pREG_PORTB_FER = 0x000005CD;
	asm(" ssync ; ");

	*pREG_PORTB_MUX = 0x00000000;
	asm(" ssync ; ");


	ErrorCode = myGetSCLK0(&freq, &cycle);

	if (ErrorCode != NO_ERR)
		return ErrorCode;

	/* calculate for PC28F128P33BF60 */

	if (mode == SYNC_MODE)
	  {
	    /* BCLK for synchronous burst read */
	    for (bclk = 0; bclk <= 3; bclk++)
	      if (freq / (bclk + 1) <= 52)
		break;

	    /* SCLK0 is too high, we can't get a valid BCLK by dividing it */
	    if (bclk == 4)
	      return SETUP_ERROR;

	    nor_clk = freq / (bclk + 1);

	    pc28f128P33_ConfigSyncModeRCR (nor_clk, &latency_count);
	  }
	else if(mode == ASYNC_MODE)
	  {
		  pc28f128P33_ConfigAsyncModeRCR(nor_clk, &latency_count);
	  }

	/* Write setup time */
	wst = 2;

	/* Write hold time */
	wht = 0;

	/* Write access time >= 50ns */
	wat = ceilf (50 / cycle);

	if (mode == SYNC_MODE)
	{
		int j;

		/* See SMC section of the HW Reference Manual */
		j = 1;
		while (j * (bclk + 1) * cycle < 60 - 25)
			j++;
		rst = j;

		rat = (bclk + 1) * latency_count + 1 - rst;

		rht = bclk;
	}
	else
	{
		/* Read access time >= 25ns */
		rat = ceilf (25 / cycle);

		/* Read setup time */
		rst = ceilf (60 / cycle) - rat;
		if (rst < 1)
			rst = 1;

		/* Read hold time >= 20ns */
		rht = ceilf (20 / cycle);
	}

	/* Pre setup time */
	prest = 0;

	/* Pre access time */
	preat = 0;

	/* Memory transition time */
	tt = 1;

	/* Memory idle time */
	it = 1;

	/* Page mode wait states */
	/* ceilf ((tCO + tAPA + tDS) / cycle) */
	/* tAPA is 25ns */
	pgws = ceilf (25 / cycle);
	if (pgws < 2)
	  pgws = 2;

	smc_b0tim = (((wst << BITP_SMC_B0TIM_WST) & BITM_SMC_B0TIM_WST)
		     | ((wht << BITP_SMC_B0TIM_WHT) & BITM_SMC_B0TIM_WHT)
		     | ((wat << BITP_SMC_B0TIM_WAT) & BITM_SMC_B0TIM_WAT)
		     | ((rst << BITP_SMC_B0TIM_RST) & BITM_SMC_B0TIM_RST)
		     | ((rht << BITP_SMC_B0TIM_RHT) & BITM_SMC_B0TIM_RHT)
		     | ((rat << BITP_SMC_B0TIM_RAT) & BITM_SMC_B0TIM_RAT));

	smc_b0etim = (((prest << BITP_SMC_B0ETIM_PREST) & BITM_SMC_B0ETIM_PREST)
		      | ((preat << BITP_SMC_B0ETIM_PREAT) & BITM_SMC_B0ETIM_PREAT)
		      | ((tt << BITP_SMC_B0ETIM_TT) & BITM_SMC_B0ETIM_TT)
		      | ((it << BITP_SMC_B0ETIM_IT) & BITM_SMC_B0ETIM_IT)
		      | ((pgws << BITP_SMC_B0ETIM_PGWS) & BITM_SMC_B0ETIM_PGWS));

	smc_b0ctl = ((1 << BITP_SMC_B0CTL_EN)
		     | (mode << BITP_SMC_B0CTL_MODE));
	switch (mode)
	  {
	  case ASYNC_MODE:
	    break;

	  case ASYNC_PAGE_MODE:
	    /* Set page size to 8 bytes according to the flash datasheet */
	    smc_b0ctl |= (1 << BITP_SMC_B0CTL_PGSZ);
	    break;

	  case SYNC_MODE:
	    smc_b0ctl |= (
			  /* Enable READY */
			  (1 << BITP_SMC_B0CTL_RDYEN)
			  | (0 << BITP_SMC_B0CTL_RDYPOL)
			  /* RDYABTEN can be used for debugging purpose */
			  /* | (1 << BITP_SMC_B0CTL_RDYABTEN) */
			  /* Set page size to 16 bytes */
			  | (2 << BITP_SMC_B0CTL_PGSZ)
			  | (bclk << BITP_SMC_B0CTL_BCLK)
			  /* BTYPE should be same as BW in flash RCR */
			  | (1 << BITP_SMC_B0CTL_BTYPE)
			  );
	    break;
	  default:
	    return SETUP_ERROR;
	  }

	*pREG_SMC0_B0TIM = smc_b0tim;
	asm(" ssync ; ");
	*pREG_SMC0_B0ETIM = smc_b0etim;
	asm(" ssync ; ");
	*pREG_SMC0_B0CTL = smc_b0ctl;
	asm(" ssync ; ");

	return NO_ERR;
}
