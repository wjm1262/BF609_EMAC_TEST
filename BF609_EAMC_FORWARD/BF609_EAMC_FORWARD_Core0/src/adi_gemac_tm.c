/*!
*********************************************************************************
 *
 * @file:    adi_gemac.c
 *
 * @brief:   Ethernet GEMAC driver source file
 *
 * @version: $Revision: 13282 $
 *
 * @date:    $Date: 2013-03-05 13:46:03 -0500 (Tue, 05 Mar 2013) $
 * ------------------------------------------------------------------------------
 *
 * Copyright (c) 2011 Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Modified versions of the software must be conspicuously marked as such.
 * - This software is licensed solely and exclusively for use with processors
 *   manufactured by or for Analog Devices, Inc.
 * - This software may not be combined or merged with other code in any manner
 *   that would cause the software to become subject to terms and conditions
 *   which differ from those listed here.
 * - Neither the name of Analog Devices, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 * - The use of this software may or may not infringe the patent rights of one
 *   or more patent holders.  This license does not release you from the
 *   requirement that you obtain separate licenses from these patent holders
 *   to use this software.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
 * NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
 * PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/** \defgroup GEMAC_Driver BF60x On-Chip EMAC Driver
 *  @{
 */
#include "BF609_EMAC_Test_Core0.h"

#include <string.h>
#include <stdio.h>
#include <cdefbf609.h>
#include <drivers/ethernet/adi_ether.h>
#include <services/int/adi_int.h>
#include "adi_gemac_int.h"

#include "adi_dp83848_phy_int.h"

#include <adi_osal.h>
#include <ccblkfn.h>

#include "post_debug.h"

//handle time
#include "arith.h"
#include "servo.h"

#define NANO_SECOND_UNIT (1000000000)

#define MAX_ADJ_TIME_IN_PI (100000)



//E Z-KIT
#define ADDEND_100M_25M_200M (0X80000000)
//
#define ADDEND_100M_16384K_163840K (0X9C400000)

#define ADDEND_100M ADDEND_100M_16384K_163840K
#define PPS_WIDTH_100M  (0x2FAF080)
#define PPS_INTERVAL_100M (0x5F5E100)

#define ADDEND_50M (ADDEND_100M *0.5)
#define PPS_WIDTH_50M (PPS_WIDTH_100M*0.5)
#define PPS_INTERVAL_50M (PPS_INTERVAL_100M*0.5)

// 系统晶振频率
#define SYS_CLKIN (1684000)

//PTP模块输出频率
#define PTP_REF_CLK (50000000)

#define ADDEND_VAL ADDEND_50M
#define PPS_WIDTH_VAL PPS_WIDTH_50M
#define PPS_INTERVAL_VAL PPS_INTERVAL_50M


//external variables
extern ADI_ETHER_HANDLE g_hDev[MAX_NETWORK_IF] ;
extern int g_AuxiTMIsFirstUpdated ;
extern PIservo PI;


#ifdef _DEBUG
TimeInternal g_StartTime ={0,0};
TimeInternal g_EndTime ={0,0};
#endif

unsigned int g_CurAuxiFIFOCounter = 0;
TimeInternal g_AuxiTimeStamp[4]={0};


/*!  EMAC0 and EMAC1 Driver instances */
//capture the rx time stamp and enable the auxiliary time stamp.
static ADI_EMAC_DEVICE gEMAC0 = {0,0,0,{0},0,0,0,0,
									1,
									1,
									0,0,0,0,0,0,0,
									0,0,{0},0
									};
static ADI_EMAC_DEVICE gEMAC1 = {0};

/*!  EMAC0 Driver Entrypoints */
extern ADI_ETHER_DRIVER_ENTRY  GEMAC0DriverEntry;
/*!  EMAC1 Driver Entrypoints */
extern ADI_ETHER_DRIVER_ENTRY  GEMAC1DriverEntry;

#define ETH_PTPAUXIN_PORTC_MUX  ((uint32_t) ((uint32_t) 2<<22))
#define ETH_PTPAUXIN_PORTC_FER  ((uint32_t) ((uint32_t) 1<<11))

static TimeInternal s_StdPPSTime ={-1,0};//
static TimeInternal s_PreTmInternal ={-1,0};//

static int s_MaxAdjTimeInPI = MAX_ADJ_TIME_IN_PI;
static int s_MaxAdjTimeCount = 0;


void InitSysTime(ADI_EMAC_REGISTERS *pEmacRegs )
{
	//(a)
	pEmacRegs->EMAC_TM_SECUPDT = 0;
	pEmacRegs->EMAC_TM_NSECUPDT = 0;
	//(b)
	//*pREG_EMAC0_TM_CTL |= BITM_EMAC_TM_CTL_TSINIT;//ENUM_EMAC_TM_CTL_EN_TS_INIT;//system time init
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSINIT;//ENUM_EMAC_TM_CTL_EN_TS_INIT;//system time init
	//(c)
	//*pREG_EMAC0_TM_CTL |= ENUM_EMAC_TM_CTL_RO_SUBSEC_RES;
	pEmacRegs->EMAC_TM_CTL |= ENUM_EMAC_TM_CTL_RO_NANO_RES;

	//When EMAC_TM_CTL.TSCTRLSSR is clear, the EMAC_TM_NSEC register has a resolution of ~0.465ns.
	//Binary rollover mode, When reset, the roll over value of EMAC_TM_NSEC register is 0x7FFF_FFFF.

	pEmacRegs->EMAC_TM_ADDEND = ADDEND_VAL ;

	//*pREG_EMAC0_TM_SUBSEC = 0xA;
	pEmacRegs->EMAC_TM_SUBSEC = NANO_SECOND_UNIT/PTP_REF_CLK; //0x14;//20ns, PTP Refer CLK = 50M

	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG |ENUM_EMAC_TM_CTL_EN_FINE_UPDT;
	asm ( "SSYNC;" );
}


void ResetSysTime(ADI_ETHER_HANDLE phDevice)
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	//reset the system time
	pEmacRegs->EMAC_TM_SECUPDT = 0; //
	pEmacRegs->EMAC_TM_NSECUPDT = 0;//
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSINIT;
	asm ( "SSYNC;" );
}
void SetFlexiblePPSOutput( ADI_ETHER_HANDLE phDevice, PPS_TYPE ePPSType, TimeInternal tmStart,
									unsigned int uPPSInterval,unsigned int uPPSWidth)
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	// b. Programming Flexible Pulse-Per-Second Output
	//b.1 Enable the PTP module by setting the EMAC_TM_CTL.TSENA
	pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ;
	//*pREG_EMAC0_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ;

	// b.2 EMAC_TM_PPSCTL.PPSEN bit to enable flexible PPS output,// b.3 TRGTMODSEL bits with 11
	pEmacRegs->EMAC_TM_PPSCTL = BITM_EMAC_TM_PPSCTL_PPSEN | BITM_EMAC_TM_PPSCTL_TRGTMODSEL;

	/* b.4 Program the start time value when the PPS output should start using the EMAC_TM_TGTM and EMAC_TM_
	NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
	time registers again.
	*/
	while(pEmacRegs->EMAC_TM_NTGTM & BITM_EMAC_TM_NTGTM_TSTRBUSY )
	{
		;
	}
	pEmacRegs->EMAC_TM_TGTM = tmStart.seconds;
	pEmacRegs->EMAC_TM_NTGTM = tmStart.nanoseconds;

	/* b.5 Program the period of the PPS signal output using the EMAC_TM_PPSINTVL register for pulse train
		output, and the width of the PPS signal output in the EMAC_TM_PPSWIDTH register for single pulse or
		pulse train output.
	*/
	if ( phDevice == g_hDev[0] )
	{
		*pREG_EMAC0_TM_PPSINTVL = uPPSInterval;//100M //0x2FAF080; //50M
		*pREG_EMAC0_TM_PPSWIDTH = uPPSWidth;//0x2625A0;// 50ms
	}
	else
	{
		*pREG_EMAC1_TM_PPSINTVL = uPPSInterval;//100M //0x2FAF080; //50M
		*pREG_EMAC1_TM_PPSWIDTH = uPPSWidth;//0x2625A0;// 50ms
	}


	/* b.6 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program the bits to 0001 to start
		single pulse, or to 0010 to start pulse train at programmed start time (Step 4).
	  (ADDITIONAL INFORMATION: The PPS pulse train is free-running unless stopped by a STOP pulse train at
		time command (EMAC_TM_PPSCTL.PPSCTL = 0100) or STOP pulse train immediately command EMAC_
		TM_PPSCTL.PPSCTL = 0101).)
	*/
	while( pEmacRegs->EMAC_TM_PPSCTL & 0xF != 0)
	{
		;
	}

	if( ePPSType == PULSE_SINGLE )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x1;//0001,start SINGLE pulse
	}
	else if( ePPSType == PULSE_TRAIN )
	{
		pEmacRegs->EMAC_TM_PPSCTL |= 0x2;//0010,start pulse train
	}


	/* b.7 The start of pulse generation can be cancelled by giving the cancel start command (EMAC_TM_PPSCTL.
		PPSCTL = 0011) before the programmed start time (Step 4) elapses
	*/

	/* b.8 Program the stop time value when the PPS output should stop using the EMAC_TM_TGTM and EMAC_TM_
		NTGTM registers. Ensure that the EMAC_TM_NTGTM.TSTRBUSY bit is reset before programming the target
		time registers again.
	*/


	/* b.9 Ensure that the EMAC_TM_PPSCTL.PPSCTL bits are cleared and then program them to 0100. This stops
		the train of pulses on PPS signal output after the programmed stop time (Step 8) elapses.
	*    (ADDITIONAL INFORMATION: The pulse train can be stopped immediately by giving the STOP pulse train
		immediately command (EMAC_TM_PPSCTL.PPSCTL = 0101). Similarly, the stop pulse train command
		(given in Step 9) can be cancelled by programming the EMAC_TM_PPSCTL.PPSCTL bits to 0110 before
		the programmed stop time (Step 8) elapses.)
	*/

}


void Init_PTPAuxin(void)
{
	 *pREG_PORTC_MUX |= ETH_PTPAUXIN_PORTC_MUX ;
	 ssync();

	  /* PORTx_FER registers */
	 *pREG_PORTC_FER |= ETH_PTPAUXIN_PORTC_FER ;
	 ssync();

	 // disable Time Stamp Interrupt
	 *pREG_EMAC0_IMSK  =  BITM_EMAC_IMSK_TS;
	 *pREG_EMAC1_IMSK  =  BITM_EMAC_IMSK_TS;
	 ssync();

}

void Enable_Time_Stamp_Auxin_Interrupt(void)
{
	if( gEMAC0.AuxiTMEnabled )
		*pREG_EMAC0_IMSK  =  0x0;

	if( gEMAC1.AuxiTMEnabled )
		*pREG_EMAC1_IMSK  =  0x0;

	ssync();
}



/* returns the pointer to gemac registers */
#pragma inline
ADI_EMAC_REGISTERS *get_gemac_regptr ( ADI_ETHER_HANDLE *const phDevice )
{
	return ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
}

/* return gemac version */
#pragma inline
uint32_t gemac_version ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	return ( pEmacRegs->EMAC_VER );
}

/* set media clock range */
#pragma inline
void gemac_set_mdcclk ( ADI_ETHER_HANDLE phDevice,
						const uint32_t mdcClkRange )
{
	ADI_EMAC_DEVICE *pDev = ( ADI_EMAC_DEVICE * ) phDevice;
	pDev->MDCClockRange   = mdcClkRange;
}

/* set dma operation mode */
#pragma inline
void gemac_set_dmaopmode ( ADI_ETHER_HANDLE phDevice,
						   const uint32_t opMode )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_OPMODE = opMode;
}

/* clear gemac interrupts */
#pragma inline
uint32_t gemac_clr_interrupts ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	
	uint32_t status = pEmacRegs->EMAC_DMA_STAT;
	pEmacRegs->EMAC_DMA_STAT = status;
	return ( status );
}

/* returns false if finished receive descriptor has any errors */
#pragma inline
bool gemac_rx_desc_valid ( const uint32_t status )
{
	return ( ( status & ENUM_DS_DESC_ERR ) == 0 ) &&
		   ( ( status & ENUM_DS_RXFIRST_DESC ) == ENUM_DS_RXFIRST_DESC ) &&
		   ( ( status & ENUM_DS_RXLAST_DESC ) == ENUM_DS_RXLAST_DESC ) ;
}

/* returns if descriptor is valid */
#pragma inline
bool gemac_is_desc_valid ( const uint32_t status )
{
	return ( ( status & ENUM_DS_DESC_ERR ) == 0 );
}

/* returns if tx is suspended */
#pragma inline
bool gemac_is_txbuf_unavail ( const uint32_t status )
{
	return ( ( status & BITM_EMAC_DMA_STAT_TU ) );
}

/* returns if rx is suspended */
#pragma inline
bool gemac_is_rxbuf_unavail ( const uint32_t status )
{
	return ( ( status & BITM_EMAC_DMA_STAT_RU ) );
}

/* set mac configuration */
#pragma inline
void gemac_set_maccfg ( ADI_ETHER_HANDLE phDevice, const uint32_t macCfg )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_MACCFG = macCfg;
}

/* set mac interrupt mask */
#pragma inline
void gemac_set_macimask ( ADI_ETHER_HANDLE phDevice, const uint32_t iMask )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_IMSK = iMask;
}

/* reset given queue */
#pragma inline
void reset_queue ( ADI_EMAC_FRAME_Q *pQueue )
{
	pQueue->pQueueHead = NULL;
	pQueue->pQueueTail = NULL;
	pQueue->ElementCount = 0;
}

/* reset all queues in the ethernet device */
#pragma inline
void reset_all_queues ( ADI_EMAC_DEVICE *const pDev )
{
	/* rest all queue structures */
	reset_queue ( &pDev->Rx.Active );
	reset_queue ( &pDev->Rx.Pending );
	reset_queue ( &pDev->Rx.Queued );
	reset_queue ( &pDev->Rx.Completed );
	pDev->Rx.Recv    = true;
	
	reset_queue ( &pDev->Tx.Active );
	reset_queue ( &pDev->Tx.Pending );
	reset_queue ( &pDev->Tx.Queued );
	reset_queue ( &pDev->Tx.Completed );
}

/* copy queue elements from source to destination */
#pragma inline
void copy_queue_elements ( ADI_EMAC_FRAME_Q *pDstQ,
						   ADI_EMAC_FRAME_Q *pSrcQ )
{
	pDstQ->pQueueHead = pSrcQ->pQueueHead;
	pDstQ->pQueueTail = pSrcQ->pQueueTail;
	pDstQ->ElementCount = pSrcQ->ElementCount;
}

/* reset dma lists of a channel */
#pragma inline
void reset_dma_lists ( ADI_EMAC_CHANNEL *pChannel )
{
	pChannel->pDmaDescHead = NULL;
	pChannel->pDmaDescTail = NULL;
	pChannel->NumAvailDmaDesc = 0;
}

/* resume tx dma channel */
#pragma inline
void resume_tx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	uint32_t TxDmaStatus = pEmacRegs->EMAC_DMA_STAT & BITM_EMAC_DMA_STAT_TS;
	
	if ( TxDmaStatus == ENUM_EMAC_DMA_STAT_TS_SUSPENDED )
		pEmacRegs->EMAC_DMA_TXPOLL = 0x1;
		
	else if ( TxDmaStatus == ENUM_EMAC_DMA_STAT_TS_STOPPED )
		pEmacRegs->EMAC_DMA_OPMODE |= BITM_EMAC_DMA_OPMODE_ST;
}

/* stop receive dma */
#pragma inline
void gemac_stop_rx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_OPMODE &= ~ ( BITM_EMAC_DMA_OPMODE_SR );
}

/* stop transmit dma */
#pragma inline
void gemac_stop_tx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_OPMODE &= ~ ( BITM_EMAC_DMA_OPMODE_ST );
}

/* resume receive dma */
#pragma inline
void resume_rx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	uint32_t RxDmaStatus = pEmacRegs->EMAC_DMA_STAT & BITM_EMAC_DMA_STAT_RS;
	
	if ( RxDmaStatus == ENUM_EMAC_DMA_STAT_RS_SUSPENDED )
		pEmacRegs->EMAC_DMA_RXPOLL = 0x1;
		
	else if ( RxDmaStatus == ENUM_EMAC_DMA_STAT_RS_STOPPED )
		pEmacRegs->EMAC_DMA_OPMODE |= BITM_EMAC_DMA_OPMODE_SR; // stopped
}

/* enable rx dma */
//#pragma inline
void enable_rx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_OPMODE |= BITM_EMAC_DMA_OPMODE_SR;
}

/* enable tx dma */
//#pragma inline
void enable_tx ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_OPMODE |= BITM_EMAC_DMA_OPMODE_ST;
}

/* set dma bus mode register */
#pragma inline
void gemac_set_dmabusmode ( ADI_ETHER_HANDLE phDevice, const uint32_t BusModeValue )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_BUSMODE = BusModeValue;
	
}

/* mask ethernet interrupts */
#pragma inline
void mask_gemac_ints ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_IEN = 0;
}

/* umask ethernet interrupts */
#pragma inline
void unmask_gemac_ints ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = get_gemac_regptr ( phDevice );
	pEmacRegs->EMAC_DMA_IEN = ADI_EMAC_AIS_NIS_INTERRUPTS;
}


//enable emac tx,rx
void enable_emac_tx_rx ( ADI_ETHER_HANDLE phDevice )
{
	uint32_t reg_data;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice ;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;

	reg_data = pEmacRegs->EMAC_MACCFG;
	reg_data  |=  ( BITM_EMAC_MACCFG_TE | BITM_EMAC_MACCFG_RE ); //

	pEmacRegs->EMAC_MACCFG = reg_data;
}

void stop_transfers ( ADI_ETHER_HANDLE phDevice )
{
	uint32_t reg_data;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice ;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;

	/* stop transmit and receive */
	ENTER_CRITICAL_REGION();
	gemac_stop_tx ( phDevice );
	EXIT_CRITICAL_REGION() ;

	//Wait for any previous frame transmissions to complete
	while(pEmacRegs->EMAC_DBG & BITM_EMAC_DBG_TXFIFONE)
	{
		asm("nop;");
	}

	ENTER_CRITICAL_REGION();
	reg_data = pEmacRegs->EMAC_MACCFG;
	reg_data  &=  ~( BITM_EMAC_MACCFG_TE | BITM_EMAC_MACCFG_RE ); //
	pEmacRegs->EMAC_MACCFG = reg_data;
	EXIT_CRITICAL_REGION() ;

	//after ensuring that the data in the receive FIFO is transferred
	//to the system memory by reading the EMAC_DBG register.
	while( (pEmacRegs->EMAC_DBG & BITM_EMAC_DBG_RXFIFOST) != ENUM_EMAC_DBG_FIFO_EMPTY)
	{
		asm("nop;");
	}

	ENTER_CRITICAL_REGION();
	gemac_stop_rx ( phDevice );
	EXIT_CRITICAL_REGION();
}
//To re-start the operation, first start the DMA,
//and then enable the MAC transmitter and receiver.
void restart_transfers ( ADI_ETHER_HANDLE phDevice )
{
	uint32_t reg_data;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice ;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;

	pEmacRegs->EMAC_DMA_OPMODE |= (BITM_EMAC_DMA_OPMODE_SR| BITM_EMAC_DMA_OPMODE_ST);

	reg_data = pEmacRegs->EMAC_MACCFG;
	reg_data  |=  ( BITM_EMAC_MACCFG_TE | BITM_EMAC_MACCFG_RE ); //

	pEmacRegs->EMAC_MACCFG = reg_data;

}

/**
 * @brief       Configure EMAC for full duplex operation
 *
 * @details     gemac_fullduplex function is responsible for configuring the MAC for
 *              duplex configuration. Typically this function gets called from the
 *              PHYInterruptHandler once auto-negotiation is successfully completed.
 *              MAC configuration is setup in configuration register (MACCFG) frame
 *              filter register (MACFRMFILT) and flow control register (MACFLOWCTL)
 *
 *              This routine sets the MACCFG register with the following bits
 *
 *              # enable jabber
 *              # burst enable
 *              # disable jumbo frame
 *              # enable multicast hash function
 *              # full duplex mode
 *
 *              The routine sets the following frame filter register bits
 *
 *              # multicast hash bits
 *              # unicast hash bits
 *
 *              The routine sets the following flow control register bits
 *
 *              # receive flow control enable
 *              # transmit flow control enable
 *
 * @param [in]  hDevice  Device Handle.
 *
 * @param [in]  port10   True for 10Mbps port and false for 100Mpbs port
 */
static void gemac_fullduplex ( ADI_ETHER_HANDLE hDevice, const bool Port10 )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	volatile uint32_t phyRegData;
	
	/* MAC configuration */
	phyRegData =  pEmacRegs->EMAC_MACCFG;
	
	/* jabber enable */
	phyRegData |=  BITM_EMAC_MACCFG_JB;
	
	phyRegData &=  ~ ( ( 1UL << BITP_EMAC_MACCFG_DO )  | /* disable receive own */
					   ( 1UL << BITP_EMAC_MACCFG_LM )  | /* disable loopback    */
					   ( 1UL << BITP_EMAC_MACCFG_DR )  | /* enable retry        */
					   ( 1UL << BITP_EMAC_MACCFG_ACS ) | /* automatic pad stripping */
					   ( 1UL << BITP_EMAC_MACCFG_DC )    /* disable defferal check */
					 );
	/* set the duplex mode */
	phyRegData |= BITM_EMAC_MACCFG_DM;
	
	if ( Port10 )
		phyRegData &= ~BITM_EMAC_MACCFG_FES;
		
	else
		phyRegData |= BITM_EMAC_MACCFG_FES;
		
	/* Update MACCFG with the new value */
	pEmacRegs->EMAC_MACCFG = phyRegData;
	
	/* MAC Frame filter configuration */
	phyRegData = pEmacRegs->EMAC_MACFRMFILT;
	
	phyRegData  |=  ( BITM_EMAC_MACFRMFILT_HMC  | /* enable multicast hash filter */
					  BITM_EMAC_MACFRMFILT_HUC    /* enable unicast hash filter   */
					);
					
	pEmacRegs->EMAC_MACFRMFILT = phyRegData;
	
	/* rx,tx flow control */
	pEmacRegs->EMAC_FLOWCTL |= ( BITM_EMAC_FLOWCTL_RFE | BITM_EMAC_FLOWCTL_TFE );
}

/**
 * @brief       Configure EMAC for half duplex operation
 *
 * @details     gemac_fullduplex function is responsible for configuring the MAC for
 *              duplex configuration. Typically this function gets called from the
 *              PHYInterruptHandler once auto-negotiation is successfully completed.
 *              MAC configuration is setup in configuration register (MACCFG) frame
 *              filter register (MACFRMFILT) and flow control register (MACFLOWCTL)
 *
 *              This routine sets the MACCFG register with the following bits
 *
 *              # enable jabber
 *              # burst enable
 *              # disable jumbo frame
 *              # enable multicast hash function
 *              # half duplex mode
 *
 *              The routine sets the following frame filter register bits
 *
 *              # multicast hash bits
 *              # unicast hash bits
 *
 *              The routine disables the following flow control register bits
 *
 *              # receive flow control enable
 *              # transmit flow control enable
 *
 * @param [in]  hDevice  Device Handle.
 *
 * @param [in]  port10   True for 10Mbps port and false for 100Mpbs port
 */
static void gemac_halfduplex ( ADI_ETHER_HANDLE hDevice, const bool Port10 )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	uint32_t phyRegData;
	
	phyRegData =  pEmacRegs->EMAC_MACCFG;
	
	/* jabber enable */
	phyRegData |=  BITM_EMAC_MACCFG_JB;
	
	phyRegData &=  ~ ( ( 1UL << BITP_EMAC_MACCFG_DO )  | /* disable receive own */
					   ( 1UL << BITP_EMAC_MACCFG_LM )  | /* disable loopback    */
					   ( 1UL << BITP_EMAC_MACCFG_DR )  | /* enable retry        */
					   ( 1UL << BITP_EMAC_MACCFG_ACS ) | /* automatic pad stripping */
					   ( 1UL << BITP_EMAC_MACCFG_DC )    /* disable defferal check */
					 );
	/* set the duplex mode */
	phyRegData &= ~BITM_EMAC_MACCFG_DM;
	
	if ( Port10 )
		phyRegData &= ~BITM_EMAC_MACCFG_FES;
		
	else
		phyRegData |= BITM_EMAC_MACCFG_FES;
		
	/* Update MACCFG with the new value */
	pEmacRegs->EMAC_MACCFG = phyRegData;
	
	/* MAC Frame filter configuration */
	phyRegData = pEmacRegs->EMAC_MACFRMFILT;
	
	phyRegData  |=  ( BITM_EMAC_MACFRMFILT_HMC  | /* enable multicast hash filter */
					  BITM_EMAC_MACFRMFILT_HUC    /* enable unicast hash filter   */
					);
					
	pEmacRegs->EMAC_MACFRMFILT = phyRegData;
	
	/* rx,tx flow control - disable */
	pEmacRegs->EMAC_FLOWCTL &= ~ ( BITM_EMAC_FLOWCTL_RFE | BITM_EMAC_FLOWCTL_TFE );
}

/**
 * @brief       PHY interrupt handler
 *
 * @details     PHY Interrupt handler gets invoked because of a PHY event. PHY events includes
 *              link up, link down, auto-negotiation completion etc.
 *
 *              Upon enabling EMAC (adi_Ether_EnableMAC) the EMAC and phy are by default
 *              configured for auto negotiation mode. Once auto negotiation is complete PHY
 *              interrupt gets generated which will invoke this handler.
 *
 *              In this handler the operational mode (speed, duplex) connectivity is
 *              determined and the EMAC block is configured accordingly.
 *
 *              Callback is invoked with ADI_ETHER_EVENT_PHY_INTERRUPT event along with
 *              the PHY status.The status information indicates the speed and the duplex
 *              nature of the connections.Potential bit enumerations for the status are given below
 *
 *              - ADI_ETHER_PHY_LINK_DOWN
 *              - ADI_ETHER_PHY_LINK_UP
 *              - ADI_ETHER_PHY_10T_FULL_DUPLEX
 *              - ADI_ETHER_PHY_10T_HALF_DUPLEX
 *              - ADI_ETHER_PHY_100T_FULL_DUPLEX
 *              - ADI_ETHER_PHY_100T_HALF_DUPLEX
 *              - ADI_ETHER_PHY_AN_COMPLETE
 *              - ADI_ETHER_PHY_LOOPBACK
 *
 * @param [in]  IID       Interrupt Identifier.
 *
 * @param [in]  pCBParm   Callback parameter which contains the EMAC device pointer (&gEMACx)
 */

//void PHYInterruptHandler(uint32_t IID, void *pCBParm)
//{
//    ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)pCBParm;
//    ADI_EMAC_REGISTERS* const  pEmacRegs = pDev->pEMAC_REGS;
//    volatile uint32_t  phyReg,phyStatus;
//
//    /* read the phy status */
//    phyReg = dp83848_phy_read((ADI_ETHER_HANDLE)pDev,pDev->PhyAddress,REG_PHY_STS);
//
//    /* check whether the link is up or down */
//    if (phyReg & PHY_STS_LINK_STATUS)
//    {
//        /*
//         * configure GEMAC with the auto-negotiation results
//         */
//        if (phyReg & PHY_STS_AUTO_NEG_COMPLETE)
//        {
//            bool port10 = phyReg & PHY_STS_SPEED_STATUS;
//
//            /* configure EMACs MACCFG register as per duplex,10/100Mpbs */
//            if (phyReg & PHY_STS_DUPLEX_STATUS)
//               gemac_fullduplex(pDev,port10);
//            else
//               gemac_halfduplex(pDev,port10);
//        }
//
//    }
//    else /* link down */
//    {
//    }
//
//    /* get the phy status */
//    phyStatus =  dp83848_phy_get_status((ADI_ETHER_HANDLE)pDev,pDev->PhyAddress);
//
//    /* invoke the callback */
//    pDev->pEtherCallback(pDev,ADI_ETHER_EVENT_PHY_INTERRUPT,(void*)phyStatus);
//
//    dp83848_ack_phyint((ADI_ETHER_HANDLE)pDev,pDev->PhyAddress);
//}

/**
 * @brief       Processes incoming or outgoing frames.
 *
 * @details     This function gets invoked from the Ethernet interrupt service routine in case
 *              of receive or transmit complete interrupt occurs. This function checks the
 *              associated active queue for receive or transmit channels and processes it.
 *              processing include removing the association of dma descriptor from the processed
 *              buffer and placing the descriptor in the free list. Also the buffer that is
 *              processed is returned via the callback handler. The dma descriptors are now
 *              avaialble for the resepective channels.
 *
 *              Once buffers are returned to the application, the frame completed queue is reset
 *              to zero.
 *
 * @param [in]  hDevice    Device Handle.
 *
 * @param [in]  pChannel   Receive or transmit channel
 *
 * @note                   This function may return more than one buffer in the callback.
 */
static void process_int ( ADI_EMAC_DEVICE *pDev, ADI_EMAC_CHANNEL *pChannel )
{
	ADI_ETHER_BUFFER *pProcessedBuffer = pChannel->Active.pQueueHead;
	ADI_EMAC_DMADESC *pCurDmaDesc = NULL;
	ADI_ETHER_EVENT   Event;
	short *pLength;
	
	while ( pProcessedBuffer )
	{
		pCurDmaDesc  = ( ( ADI_EMAC_BUFINFO * ) pProcessedBuffer )->pDmaDesc;
		
		/* data cache is enabled then flush and invalidate the descriptor */
		if ( pDev->Cache )
		{
			SIMPLEFLUSHINV ( pCurDmaDesc );
		}
		
		/* if any descriptor is owned by host we will break */
		if ( pCurDmaDesc->Status & ADI_EMAC_DMAOWN )
			break;
			
		/* we have atleast one finished buffer */
		pChannel->Active.pQueueHead = pProcessedBuffer->pNext;
		pChannel->Active.ElementCount--;
		pProcessedBuffer->pNext = NULL;
		pCurDmaDesc->pNextDesc  = NULL;
		
		/* pProcessedBuffer is the currently processed buffer and the pCurDmaDesc is
		 * current descriptor. The length of the frame is stored in DESC0 status area.
		 */
		if ( pChannel->Recv )
		{
			pProcessedBuffer->ProcessedElementCount = ( ( pCurDmaDesc->Status >> 16 ) & 0x3FFF );
			pLength = ( short * ) pProcessedBuffer->Data;
			*pLength = pProcessedBuffer->ProcessedElementCount + 6;
			pProcessedBuffer->ProcessedElementCount += 6;
			pProcessedBuffer->ProcessedFlag = ( uint32_t ) true;
			
			//get the time stamp, modified by wjm @2014-7-21

			RX_TX_TIME_STAMP_BUFFER *pTmp = ( RX_TX_TIME_STAMP_BUFFER * ) pProcessedBuffer;

			unsigned int flag = 1 * ( ( pCurDmaDesc->Status & 0x100 ) == 0x100 ) + 2 * ( ( pCurDmaDesc->Status & 0x80 ) == 0x80 )  ;
			
			switch ( flag )
			{
				case 0:
					pTmp->RxTimeStamp.TimeStampLo = flag;
					pTmp->RxTimeStamp.TimeStampHi = 0xeeeeeeee;
					break;
					
				case 1:
					pTmp->RxTimeStamp.TimeStampLo = flag;
					pTmp->RxTimeStamp.TimeStampHi = 0xeeeeeeee;
					break;
					
				case 2:
					pTmp->RxTimeStamp.TimeStampLo = flag;
					pTmp->RxTimeStamp.TimeStampHi = 0xeeeeeeee;
					break;
					
				case 3:
					pTmp->RxTimeStamp.TimeStampLo = pCurDmaDesc->RxTimeStampLo;
					pTmp->RxTimeStamp.TimeStampHi = pCurDmaDesc->RxTimeStampHi;
					break;
			}
			
		}

		/* put the buffer in the completed queue */
		insert_queue ( &pChannel->Completed, pProcessedBuffer );
		
		/* see if next buffer in the active list is also done */
		pProcessedBuffer = pChannel->Active.pQueueHead;
		
		/* place the finished dma descriptor in the available list for the channel */
		if ( pChannel->pDmaDescTail != NULL )
		{
			pChannel->pDmaDescTail->pNextDesc =  pCurDmaDesc;
			pChannel->pDmaDescTail = pCurDmaDesc;
		}
		
		else
		{
			pChannel->pDmaDescHead = pChannel->pDmaDescTail =  pCurDmaDesc;
		}
		
		pChannel->NumAvailDmaDesc += 1;
	}
	
	/* determine the event */
	Event = pChannel->Recv ? ADI_ETHER_EVENT_FRAME_RCVD : ADI_ETHER_EVENT_FRAME_XMIT;
	
	/* return the processed buffers to the application */
	//maybe have more than one buffers in the Completed, by wjm @2014-7-21
	pDev->pEtherCallback ( pDev, Event, pChannel->Completed.pQueueHead );
	
	/* reset the completed queue */
	reset_queue ( &pChannel->Completed );
	
	/* if no more active elements in the queue reset the queue */
	if ( pChannel->Active.pQueueHead == NULL )
		reset_queue ( &pChannel->Active );
		
	return;
}

/**
 * @brief       Transfers buffers on either pending on queued
 *
 * @details     This routine gets called from the EMACInterruptHandler once transmission
 *              or reception completes. Because of completion of transmit or receive packet
 *              associated descriptors gets available for the next transfer. This routine
 *              checks for any queued buffers or buffers that are waiting for descriptors
 *              to be available, binds them and schedules for transfer.
 *
 * @param [in]  phDevice   Device handle
 *
 * @param [in]  pChannel   Device channel
 */
static void transfer_queued_bufs_debug ( ADI_ETHER_HANDLE phDevice, ADI_EMAC_CHANNEL *pChannel )
{
	/* if buffers are queued and descriptors are available or
	 * if buffers are pending - already binded with descriptors
	 * we activate the respective channel
	 */
	if ( ( ( pChannel->Queued.pQueueHead != NULL ) && pChannel->NumAvailDmaDesc > 1 ) ||
			( pChannel->Pending.pQueueHead != NULL ) )
	{
		bind_buf_with_desc ( phDevice, pChannel );
		activate_channel ( phDevice, pChannel );
	}

	if( ( pChannel->Queued.pQueueHead == NULL  && pChannel->NumAvailDmaDesc > 1 ) )
	{
		DEBUG_STATEMENT("Queued == NULL \n\n ");
	}

	if( ( pChannel->Queued.pQueueHead != NULL  && pChannel->NumAvailDmaDesc <= 1 ) )
	{
		DEBUG_STATEMENT("DmaDesc <= 1 \n\n " );
	}

	if( ( pChannel->Queued.pQueueHead == NULL  && pChannel->NumAvailDmaDesc <= 1 ) )
	{
		DEBUG_STATEMENT("Queued == NULL && DmaDesc <= 1 \n\n ");
	}

}
static void transfer_queued_bufs ( ADI_ETHER_HANDLE phDevice, ADI_EMAC_CHANNEL *pChannel )
{
	/* if buffers are queued and descriptors are available or
	 * if buffers are pending - already binded with descriptors
	 * we activate the respective channel
	 */
	if ( ( ( pChannel->Queued.pQueueHead != NULL ) && pChannel->NumAvailDmaDesc > 1 ) ||
			( pChannel->Pending.pQueueHead != NULL ) )
	{
		bind_buf_with_desc ( phDevice, pChannel );
		activate_channel ( phDevice, pChannel );
	}
}
/**
 * @brief       Handles abnomral ethernet interrupts or events
 *
 * @details     This function gets called only from the etherent interrupt handler because of an
 *              abnormal event. Appropriate action is taken dependeing on the status.
 *
 *              Handles the following errr conditions
 *
 *              # transmit process stopped
 *              # transmit jabber timeout
 *              # receive fifo overflow
 *              # transmit underflow
 *              # receive buffer unavailable
 *              # receive buffer stopped
 *              # receive watchdog timeout
 *              # early transmit interrupt
 *              # fatel bus error
 *
 * @param [in]  pDev       Device Handle.
 *
 * @param [in]  DmaStatus  DMA status
 */

static void handle_abnormal_interrupts ( ADI_EMAC_DEVICE *pDev, const uint32_t DmaStatus )
{
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	short value;
	
	/* transmit process stopped */
	//EMAC_DMA_IEN.TPS
	if ( DmaStatus & BITM_EMAC_DMA_STAT_TPS )
	{
		STATS_INC ( pDev->Stats.TxProcessStopCnt );
		DEBUG_STATEMENT("transmit process stopped \n\n ");
	}
	
	/* transmit jabber timeout */
	// EMAC_DMA_IEN.TJT,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_TJT )
	{
		STATS_INC ( pDev->Stats.TxJabberTimeOutCnt );
		DEBUG_STATEMENT("transmit jabber timeout \n\n ");
	}
	
	/* receive buffer unavailable */
	/* The EMAC_DMA_STAT.OVF bit indicates that the Receive Buffer had an
Overflow during frame reception. If the partial frame is transferred
to application, the overflow status is set in RDES0[11].

	Overflow Error. When set, this bit indicates that the received frame was
damaged due to buffer overflow in MFL.
	 * */
	//receive fifo overflow,EMAC_DMA_IEN.OVF,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_OVF )
	{
		STATS_INC ( pDev->Stats.RxOvfCnt );
		DEBUG_STATEMENT("Receive FIFO Overflow \n\n ");
	}
	
	/* transmit buffer Underflow */
	/* The EMAC_DMA_STAT.UNF bit indicates that the Transmit Buffer had
an Underflow during frame transmission. Transmission is suspended
and an Underflow Error TDES0[1] is set.

	Underflow Error. When set, this bit indicates that the EMAC aborted the
frame because data arrived late from the application memory. Underflow
Error indicates that the DMA encountered an empty transmit buffer while
transmitting the frame.
	 * */
	//EMAC_DMA_IEN.UNF,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_UNF )
	{
		//DEBUG_PRINT("EMAC:%X, transmit buffer Underflow \n\n ", pDev);
		STATS_INC ( pDev->Stats.TxUnfCnt );
		transfer_queued_bufs ( pDev, &pDev->Tx );
	}
	
	/* no receive buffer available -rx suspeneded */
	// EMAC_DMA_IEN.RU,
	if (  DmaStatus & BITM_EMAC_DMA_STAT_RU )
	{
		DEBUG_PRINT("EMAC:%X, no receive buffer available \n\n ", pDev);
		//transfer_queued_bufs ( pDev, &pDev->Rx );
		transfer_queued_bufs_debug( pDev, &pDev->Rx );
	}

	/* receiver process stopped */
	//EMAC_DMA_IEN.RPS,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_RPS )
	{
		STATS_INC ( pDev->Stats.RxProcessStopCnt );
		DEBUG_STATEMENT("receiver process stopped \n\n ");
	}

	/* received watchdog timeout */
	//EMAC_DMA_IEN.RWT,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_RWT )
	{
		STATS_INC ( pDev->Stats.RxWDTimeoutCnt );
		DEBUG_STATEMENT("received watchdog timeout \n\n ");
	}
	
	/* early transmit interrupt */
	/*indicates that the frame to be transmitted was fully transferred to the MFL Transmit FIFO.
	 * */
	//EMAC_DMA_IEN.ETI,
	if ( DmaStatus & BITM_EMAC_DMA_STAT_ETI )
	{
		STATS_INC ( pDev->Stats.EarlyTxIntCnt );
		//DEBUG_STATEMENT("early transmit interrupt \n\n ");
	}

	/* fatal buss error */
	//EMAC_DMA_IEN.FBI.
	/*
	Fatal Bus Error
	The EMAC SCB asserts the error interrupt (EMAC_DMA_STAT.FBI) when the corresponding fatal bus error
	interrupt is enabled in the DMA interrupt enable register. The application has to reset the core to restart
	the DMA.
	*/

	if ( DmaStatus & BITM_EMAC_DMA_STAT_FBI )
	{
		STATS_INC ( pDev->Stats.BusErrorCnt );
		DEBUG_STATEMENT("fatal buss error \n\n ");
	}
	

}
static void handle_auxiliary_tm_interrupt(	ADI_ETHER_HANDLE phDevice, const TimeInternal *pAuxiTimeStamp )
{

	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;

	TimeInternal tmInternal = {0,0};
	double deltaNanoSec = 0;
	unsigned int addend = 0;
	int adjAddend = 0;
	TimeInternal startTime ={10,0};

	/* the First time for external trigger PPS arrived,
	 * reInit the system time and output system PPS
	*/

	if(g_AuxiTMIsFirstUpdated == 1)
	{
		/* reset the system time should be done before the SetFlexiblePPSOutput,
		 * for the current time may exceed the startTime,
		 * then cause a Time Stamp Target Time Programming Error.
		 * And this situation is testified in our TESTS.
			by wjm@2014-8-16 AM9:30
		 * */
		//reset the system time
		ResetSysTime( phDevice );

		DEBUG_PRINT("aux:< %10d.%-9d >  sys:(reset)\n\n",pAuxiTimeStamp->seconds,  pAuxiTimeStamp->nanoseconds );

		//startTime.seconds = g_AuxiTimeStamp[i-1].seconds+1;
		//startTime.nanoseconds = g_AuxiTimeStamp[i-1].nanoseconds;
		SetFlexiblePPSOutput( phDevice, PULSE_TRAIN, startTime, PPS_INTERVAL_VAL-1, PPS_WIDTH_VAL-1);

		// 第一次更新标志
		g_AuxiTMIsFirstUpdated = 0;

	}
	else
	{
		//
		tmInternal.nanoseconds = pAuxiTimeStamp->nanoseconds ;

		if( tmInternal.nanoseconds > (NANO_SECOND_UNIT/2) )
		{
			tmInternal.nanoseconds =  tmInternal.nanoseconds - NANO_SECOND_UNIT ;
		}

		if( (tmInternal.nanoseconds <= MAX_ADJ_TIME_IN_PI) && (tmInternal.nanoseconds >= -MAX_ADJ_TIME_IN_PI) )
		{

			//Fine Correction
			deltaNanoSec = runPIservo ( &PI, tmInternal.nanoseconds );

			//
			adjAddend = calcAdjAddend ( ADDEND_VAL, deltaNanoSec, PI.dt );//

			//
			addend = ADDEND_VAL + adjAddend;

			pEmacRegs->EMAC_TM_ADDEND =  addend;
			pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSADDREG |ENUM_EMAC_TM_CTL_EN_FINE_UPDT;

		}
		else
		{
			//reset the system time
			ResetSysTime( phDevice );

//			s_MaxAdjTimeCount++;
//
//			if( s_MaxAdjTimeCount >= 2 )
//			{
//				s_MaxAdjTimeInPI =  s_MaxAdjTimeInPI + (s_MaxAdjTimeInPI >> 1);
//
//				s_MaxAdjTimeCount = 0;
//			}

		}//	if( 0 == tmInternal.seconds  ...)


		//add by wjm@2014-9-4 to verify whether have one more tm in the TM FIFO.
#if 0
		if(g_CurAuxiFIFOCounter > 1)
		{
			for( int i = 0; i < g_CurAuxiFIFOCounter; i++)
			{
				DEBUG_PRINT( "i: %d \t < %10d.%-9d >\n\n ", i, g_AuxiTimeStamp[i].seconds,  g_AuxiTimeStamp[i].nanoseconds);
			}

		}
#endif
		DEBUG_PRINT("aux:< %10d.%-9d > int:(%10d %-9d) adj: %d \n\n", pAuxiTimeStamp->seconds, pAuxiTimeStamp->nanoseconds, tmInternal.seconds, tmInternal.nanoseconds, adjAddend);
	}//if(g_AuxiTMIsFirstUpdated == 1)


}



void TimeStampStatusInterruptHandler ( 	ADI_ETHER_HANDLE phDevice, uint32_t tm_status )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	int i = 0;
	TimeInternal startTime ={1,0};

	/* Auxiliary Time Stamp Snapshot Trigger Missed */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_ATSSTM )
	{
		DEBUG_STATEMENT("Auxiliary Time Stamp Snapshot Trigger Missed \n\n ");

#ifdef _DEBUG
		g_CurAuxiFIFOCounter = (tm_status & BITM_EMAC_TM_STMPSTAT_ATSNS) >> BITP_EMAC_TM_STMPSTAT_ATSNS;
		// get auxiliary stamp
		for( i = 0; i < g_CurAuxiFIFOCounter; i++)
		{
			g_AuxiTimeStamp[i].nanoseconds = pEmacRegs->EMAC_TM_AUXSTMP_NSEC;
			g_AuxiTimeStamp[i].seconds = pEmacRegs->EMAC_TM_AUXSTMP_SEC ;
			DEBUG_PRINT( "i: %d \t < %10d.%-9d >\n\n ", i, g_AuxiTimeStamp[i].seconds,  g_AuxiTimeStamp[i].nanoseconds);

		}
#endif
 		//Set the EMAC_TM_CTL.ATSFC bit to clear the FIFO
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_ATSFC;

		asm ( "SSYNC;" );

		return;
	}

	 /* Time Stamp Target Time Programming Error */
	/*The EMAC_TM_STMPSTAT.TSTRGTERR bit is set when the target time,
			which is being programmed in the EMAC_TM_SEC and EMAC_TM_NSEC
			registers, has already elapsed. This bit is cleared when read by the
			application.
	 * */
	//R/W
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSTRGTERR )
	{
		DEBUG_STATEMENT("Time Stamp Target Time Programming Error \n\n ");
	}

	 /* Auxiliary Time Stamp Trigger Snapshot */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_ATSTS)
	{
		/*get auxiliary stamp, auxiliary stamp occurs 1 times/per second,
		 * so only get one auxiliary time at every time
		 * by wjm@2014-8-15 PM 17:36
		 * */

		g_CurAuxiFIFOCounter = (tm_status & BITM_EMAC_TM_STMPSTAT_ATSNS) >> BITP_EMAC_TM_STMPSTAT_ATSNS;
		for( i = 0; i < g_CurAuxiFIFOCounter; i++)
		{
			g_AuxiTimeStamp[i].nanoseconds = pEmacRegs->EMAC_TM_AUXSTMP_NSEC;
			g_AuxiTimeStamp[i].seconds = pEmacRegs->EMAC_TM_AUXSTMP_SEC ;

		}
		handle_auxiliary_tm_interrupt(phDevice, &g_AuxiTimeStamp[i-1]);

		//PPS out is ERROR, it will be done in late day. by wjm@2014-9-4
		//startTime.seconds = g_AuxiTimeStamp[i-1].seconds+1;
		//startTime.nanoseconds = g_AuxiTimeStamp[i-1].nanoseconds;

		//SetFlexiblePPSOutput( phDevice, PULSE_SINGLE, startTime, 0x5F5E0FF, 0x2FAF07f);

	}

	 /* Time Stamp Target Time Reached */
	//RC/NW
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSTARGT )
	{
		DEBUG_STATEMENT("Time Stamp Target Time Reached \n\n ");
	}

	 /* Time Stamp Seconds Overflow */
	if( tm_status & BITM_EMAC_TM_STMPSTAT_TSSOVF )
	{
		DEBUG_STATEMENT("Time Stamp Seconds Overflow\n\n ");
	}

}
/**
 * @brief       EMAC Interrupt handler
 *
 * @details     EMAC interrupt handler is responsible for processing the all EMAC interrupts.
 *              EMAC interrupts include DMA receive, transmit complete interrupts, DMA error
 *              interrupts, Overflow, underflow interrupts, MMC counter and timestamp interrupts.
 *
 *              After processing EMAC interrupt handler acknowledges the interrupt at the EMAC
 *              source, by writing to the W1C bits of status registers.
 *
 *              By the time this interrupt handler is called all required actions at the SEC
 *              are taken care of. Upon returning from this handler the dispatcher acknowledges
 *              the interrupt via writing to the SEC_END register.
 *
 *              If rx and tx dma is suspended or stopped and there is a queued buffer that is
 *              ready then the interrupt handler will trigger the DMA by writing to the poll
 *              registers.
 *
 *
 * @param [in]  IID      Interrupt Identifier.
 *
 * @param [in]  pCBParm  Callback parameter which contains the EMAC device pointer (&gEMACx)
 *
 * @note                 This function may return more than one buffer in the callback.
 */
void EMACInterruptHandler ( uint32_t IID, void *pCBParm )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) pCBParm;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	uint32_t dma_status;
	uint32_t int_status;
	uint32_t tm_status;
	short value;

	asm ( "SSYNC;" );
	dma_status =  pEmacRegs->EMAC_DMA_STAT;
	
	/* Auxiliary interrupts */
	int_status = pEmacRegs->EMAC_ISTAT;
	tm_status = pEmacRegs->EMAC_TM_STMPSTAT;

	//Time Stamp Status Interrupt
	if( (pDev->AuxiTMEnabled ) && (int_status & BITM_EMAC_ISTAT_TS) )
	{
		TimeStampStatusInterruptHandler ((ADI_ETHER_HANDLE)pCBParm, tm_status );
	}

	/* check if we got any interrupt */
	if ( dma_status == 0 )
		return;
		
	/* acknowledge dma interrupts */
	pEmacRegs->EMAC_DMA_STAT = dma_status & 0x1FFFF;
	
	/* receive frame interrupt */
	if ( dma_status & BITM_EMAC_DMA_STAT_RI )
	{
		STATS_INC ( pDev->Stats.RxIntCnt );
		process_int ( pDev, &pDev->Rx );
	}
	
	/* transmit complete interrupt */
	if ( dma_status &  BITM_EMAC_DMA_STAT_TI )
	{
		STATS_INC ( pDev->Stats.TxIntCnt );
		process_int ( pDev, &pDev->Tx );
	}
	
	/* TU, no buffer to transmit - tx in suspended state check queued buffers */
	/* The EMAC_DMA_STAT.TU bit indicates that the Next Descriptor in the
	Transmit List is owned by the application and cannot be acquired by
	the DMA. Transmission is suspended. The value in the EMAC_DMA_STAT.TS
	bits explain the Transmit Process state transitions. To
	resume processing transmit descriptors, the application should
	change the ownership of the bit of the descriptor and then issue a
	Transmit Poll Demand command.
	 * */
	if ( gemac_is_txbuf_unavail ( dma_status ) )
	{
		//DEBUG_PRINT("EMAC:%X, no buffer to transmit \n\n ", pDev);
		transfer_queued_bufs ( pDev, &pDev->Tx );
	}

	/* early Receive interrupt */
	/* indicates that the DMA had filled the
	first data buffer of the packet. The EMAC_DMA_STAT.RI bit
	automatically clears this bit.
	 * */
	if ( dma_status & BITM_EMAC_DMA_STAT_ERI )
	{
		STATS_INC ( pDev->Stats.EarlyRxIntCnt );
	}

	/* abnormal interrupts - errors */
	if ( dma_status & BITM_EMAC_DMA_STAT_AIS )
	{
		//DEBUG_STATEMENT("abnormal interrupts errors\n\n ");
		handle_abnormal_interrupts ( pDev, dma_status );
	}

	/* IEEE-1588 time stamp trigger interrupt */
	if ( dma_status & BITM_EMAC_DMA_STAT_TTI )
	{
	}
	
	/* memory management counter interrupts */
//	if ( dma_status & BITM_EMAC_DMA_STAT_MCI )
//	{
//
//	}

	if ( dma_status & BITM_EMAC_DMA_STAT_MCI )
	{
		STATS_INC ( pDev->Stats.MMCIntCnt );
		value = pEmacRegs->EMAC_ISTAT;
		pEmacRegs->EMAC_ISTAT = 0;
		pEmacRegs->EMAC_MMC_CTL = 0x1;
		value = pEmacRegs->EMAC_MMC_RXINT;
		value = pEmacRegs->EMAC_MMC_TXINT;
	}
	
}

/**
 * @brief      Configures phy interrupt
 *
 * @details    PHY_INT is generated by the PHY when any of the PHY events occurs. Typical
 *             phy events include link up or down, auto-negotiation completion.
 *
 *             PHYInterruptHandler handles all the PHY events
 *
 * @param [in] hDevice   Device Handle.
 */
static void config_phyint ( ADI_ETHER_HANDLE *const phDevice )
{
#if BF609_EZ_BRD

	if ( phDevice == ( ADI_ETHER_HANDLE * ) &gEMAC0 )
	{
		/* configure gpio  for EMAC0*/
		*pREG_PORTB_FER |= 0x0000e000;
		*pREG_PORTB_MUX = 0x0;
		
		*pREG_PORTC_FER |= 0x000002ff;
		*pREG_PORTC_MUX = 0x0;
		
		*pREG_PORTD_MUX = 0x0;
		*pREG_PORTD_FER |= 0x00000040;
		
		// configuration
		*pREG_PINT2_ASSIGN   |= ( BITM_PINT_ASSIGN_B0MAP );
		*pREG_PINT2_MSK_SET  |= BITM_PINT_MSK_SET_PIQ6 ;
		*pREG_PINT2_INV_SET  |= BITM_PINT_INV_SET_PIQ6 ;
		
		*pREG_PORTD_FER_CLR  |= BITM_PORT_FER_CLR_PX6  ;
		*pREG_PORTD_DIR_CLR  |= BITM_PORT_DIR_CLR_PX6  ;
		*pREG_PORTD_INEN_SET |= BITM_PORT_INEN_SET_PX6 ;
		*pREG_PORTD_POL_SET  |= BITM_PORT_POL_SET_PX6  ;
		
		*pREG_SEC0_GCTL      |= ENUM_SEC_GCTL_EN;
		*pREG_SEC0_CCTL0     |= ENUM_SEC_CCTL_EN;
		
		*pREG_PINT2_LATCH    = BITM_PINT_LATCH_PIQ6;
		adi_int_InstallHandler ( INTR_PINT2_BLOCK, PHYInterruptHandler, ( void * ) phDevice, true );
		//adi_int_InstallHandler(INTR_PINT2_BLOCK,PHYInterruptHandler,(void*)phDevice,false);
	}
	
#endif
}


/**
 * @brief      Sets GEMAC handle
 *
 * @details    Depending on the device id this function will set the GEMAC handle
 *             and associated interrupt
 *
 * @param [in] phDevice   pointer to device handle
 *
 * @param [in] devID      Device ID
 *
 * @note       This function may required to change with different hardware platforms.
 */
#pragma inline
void set_gemac_handle ( const ADI_ETHER_DRIVER_ENTRY *pEntryPoint, ADI_ETHER_HANDLE  *const phDevice )
{
	if ( pEntryPoint == &GEMAC0DriverEntry )
	{
		*phDevice = &gEMAC0;
		gEMAC0.Interrupt = ( uint32_t ) INTR_EMAC0_STAT;
	}
	
	else if ( pEntryPoint == &GEMAC1DriverEntry )
	{
		*phDevice = &gEMAC1;
		gEMAC1.Interrupt = ( uint32_t ) INTR_EMAC1_STAT;
	}
}

/**
 * @brief       Sets the default init parameters
 *
 * @details     Sets up the driver with default initialization parameters
 *
 * @param [in]  phDevice    pointer to device handle
 *
 * @param [in]  pDeviceInit pointer to device initialization structure
 *
 * @note        This function may required to change with different hardware platforms.
 *
 */
#pragma inline
void set_init_params ( ADI_ETHER_HANDLE *phDevice, ADI_ETHER_DEV_INIT *const pDeviceInit )
{
	ADI_EMAC_DEVICE *const  pDev = ( ADI_EMAC_DEVICE * ) phDevice;
	
	/* populate with default parameters */
	pDev->AutoNegotiate = true;
	pDev->Port10        = false;//100M
	pDev->FullDuplex    = true;//false
	pDev->LoopBackMode  = false;//false
	pDev->Cache         = pDeviceInit->Cache;
}

/**
 * @brief       Initializes EMAC
 *
 * @details     This function configures the EMAC Configuration register, filter register
 *              and flow control register using the default configuration supplied by the
 *              user.
 *
 * @param [in]  phDevice   Device Handle.
 *
 * @return      Status
 *                         - ADI_ETHER_RESULT_SUCCESS  successfully opened the device
 *
 * @note        Internal Driver function.
 */
static uint32_t gemac_init ( ADI_ETHER_HANDLE *const phDevice )
{
	uint32_t reg_data;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	
	reg_data = pEmacRegs->EMAC_MACCFG;
	
	/* initialize mac configuration register */
	if ( pDev->LoopBackMode )
		reg_data = BITM_EMAC_MACCFG_LM;
		
	if ( pDev->FullDuplex )
		reg_data |= BITM_EMAC_MACCFG_DM;
		
	if ( pDev->Port10 )
		reg_data &= ~BITM_EMAC_MACCFG_FES;
		
	else
		reg_data |= BITM_EMAC_MACCFG_FES;
		
		
	//reg_data  |= (BITM_EMAC_MACCFG_TE | BITM_EMAC_MACCFG_RE ) ;// NOTE: this is added by wjm @2014-7-18
	//reg_data  |=  ( BITM_EMAC_MACCFG_TE | BITM_EMAC_MACCFG_RE | BITM_EMAC_MACCFG_CST ); // NOTE: this is the original code
	reg_data  |=  ( BITM_EMAC_MACCFG_CST ); // modify by wjm@7.28
	gemac_set_maccfg ( phDevice, reg_data );
	
	/* initialize frame filter configuration */
	
	//    pEmacRegs->EMAC_MACFRMFILT = BITM_EMAC_MACFRMFILT_RA  | 0x1 |
	//                                 BITM_EMAC_MACFRMFILT_PM  |
	//                                 BITM_EMAC_MACFRMFILT_HMC ;
	
	pEmacRegs->EMAC_MACFRMFILT =  BITM_EMAC_MACFRMFILT_DBF ;
	if( phDevice == g_hDev[0])
	{
		pEmacRegs->EMAC_MACFRMFILT = BITM_EMAC_MACFRMFILT_PM ;
	}
	
	/* setup flow control options */
	if ( pDev->FullDuplex )
		pEmacRegs->EMAC_FLOWCTL = BITM_EMAC_FLOWCTL_TFE |
								  BITM_EMAC_FLOWCTL_RFE;
								  
	return ( ADI_ETHER_RESULT_SUCCESS );
}
/**
 * @brief       Opens the given ethernet device
 *
 * @details     Every ethernet device driver exports its functionality using an entrypoint.
 *              Entrypoint consists of set of driver functions like, open, read, write, close etc.
 *
 *              adi_ether_GemacOpen function is responsible for initializing the ethernet device.
 *              This routine sets up EMAC and platform specific configuration.adi_ether_GemacOpen
 *              is called by routing layer function adi_ether_Open() using the supplied entrypoint.
 *              Applications open the ethernet drivers and supply the driver handle to the stack.
 *
 *              pDeviceInit parameter supplies memory for the driver. It uses the memory
 *              to construct receive and transmit dma descriptor chains. These descriptors
 *              will be used in conjuction with transmit and receive buffers to schedule DMA
 *              transfers.
 *
 *              It is adviced to increase the number of descriptors for applications having
 *              large data transfers.
 *
 *              This routine merely configures the MAC with default settings. Applications
 *              can change any defaults before actually enabling the MAC.
 *
 * @param [in]  pEntryPoint     Device driver entry point. For GEMAC driver the entry point
 *                              can be GEMAC0DriverEntry or GEMAC1DriverEntry.
 *
 *
 * @param[in]    pDeviceInit     Pointer to Initialization Data Structure. The init
 *                               structure is used to supply memory to the driver as
 *                               well operation of the cache. The supplied memory is
 *                               used for storing the transmit and receive descriptors.
 *                               Cache element is used to specifify whether data cache
 *                               is enable or disabled. If data cache is enabled driver
 *                               performs additional functions for the cache-coherency.
 *
 * @param[in]    pfCallback      Pointer to Ethernet callback Function. This callback
 *                               is used by the driver asynchronously to return the
 *                               received packets as well as transmitted packets. With
 *                               lwip tcp/ip stack this callback has to be adi_lwip_StackcallBack.
 *                               adi_lwip_StackcallBack is defined in lwip. Applications
 *                               that directly use the driver can hookup their own callback
 *                               routines.
 *
 * @param [out] phDevice        Pointer to a location where the handle to the
 *                              opened device is written.
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  successfully opened the device
 *                              - ADI_ETHER_RESULT_INVALID_PARAM invalid input parameter
 *                              - ADI_ETHER_RESULT_INVALID_DEVICE_ENTRY invalid entry point
 *
 * @sa          adi_Ether_Close()
 * @sa          adi_Ether_EnableMAC()
 */
ADI_ETHER_RESULT adi_ether_GemacOpen (
	ADI_ETHER_DRIVER_ENTRY *const pEntryPoint,
	ADI_ETHER_DEV_INIT   *const pDeviceInit,
	ADI_ETHER_CALLBACK_FN const pfCallback,
	ADI_ETHER_HANDLE     *const phDevice
)
{
	ADI_EMAC_DEVICE *pDev;
	ADI_ETHER_RESULT  Result =  ADI_ETHER_RESULT_FAILED;
	
	gEMAC0.pEMAC_REGS = ( ( ADI_EMAC_REGISTERS * ) ADI_EMAC0_BASE_ADDRESS );
	gEMAC1.pEMAC_REGS = ( ( ADI_EMAC_REGISTERS * ) ADI_EMAC1_BASE_ADDRESS );
	
#if defined(ADI_DEBUG)
	
	if ( ( pfCallback == NULL ) || ( phDevice == NULL )  || ( pDeviceInit == NULL ) )
	{
		return ( ADI_ETHER_RESULT_INVALID_PARAM );
	}
	
	if ( ( pEntryPoint != &GEMAC0DriverEntry ) && ( pEntryPoint != &GEMAC1DriverEntry ) )
	{
		return ADI_ETHER_RESULT_INVALID_DEVICE_ENTRY;
	}
	
	if ( ( pDeviceInit->pEtherMemory->RecvMemLen < ADI_EMAC_MIN_DMEM_SZ )  ||
			( pDeviceInit->pEtherMemory->TransmitMemLen < ADI_EMAC_MIN_DMEM_SZ ) )
	{
		return ADI_ETHER_RESULT_INVALID_PARAM;
	}
	
#endif
	/* set GEMAC handle */
	set_gemac_handle ( pEntryPoint, phDevice );
	
	pDev = ( ADI_EMAC_DEVICE * ) *phDevice;
	
	ENTER_CRITICAL_REGION();
	
	/* check if the device is already opened */
	if ( !pDev->Opened )
	{
		pDev->Opened                = true;
		pDev->PhyAddress            = 0x1;
		
		pDev->pEtherCallback = pfCallback;
		pDev->MdcClk  =  0x4; //depends on SCLK0 100 - 150Mhz
		//pDev->MdcClk  =  0x8;
		
		/* reset the dma descriptor lists */
		reset_all_queues ( *phDevice );
		reset_dma_lists ( &pDev->Rx );
		reset_dma_lists ( &pDev->Tx );
		
		/* initialize device from supplied parameters */
		set_init_params ( *phDevice, pDeviceInit );
		
		/* configure phy interrupt */
#if BF609_EZ_BRD
		config_phyint ( *phDevice ); //wjm@phy
#endif
		
		/* gemac version */
		pDev->Version = gemac_version ( *phDevice );
		
		/* setup rx descriptor list */
		Result = init_descriptor_list ( pDev, ( uint8_t * ) pDeviceInit->pEtherMemory->pRecvMem,
										pDeviceInit->pEtherMemory->RecvMemLen,
										&pDev->Rx );
										
		/* setup tx descriptor list */
		if ( Result == ADI_ETHER_RESULT_SUCCESS )
		{
			Result = init_descriptor_list ( pDev, ( uint8_t * ) pDeviceInit->pEtherMemory->pTransmitMem,
											pDeviceInit->pEtherMemory->TransmitMemLen,
											&pDev->Tx );
		}
	}
	
	else
	{
		*phDevice = NULL;
	}
	
	EXIT_CRITICAL_REGION();
	
	return ( Result );
}

/**
 * @brief      Wait until gemac reset is complete
 *
 * @details    Waits for the Soft reset bit in DMA busmode register to get cleared.
 *             Upon successful reset this bit gets cleared automatically. If this bit is
 *             not cleared with in MAC_LOOPCOUNT number of times then the function result
 *             false or else it returns true.
 *
 * @param [in] hDevice   Device Handle.
 *
 * @note       This function may required to change with different hardware platforms.
 *
 */
static bool gemac_reset_complete ( ADI_ETHER_HANDLE *const phDevice )
{
	int32_t loopcount = MAC_LOOPCOUNT;
	ADI_EMAC_REGISTERS    *const  pEmacRegs      = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	
	/* wait for the bus mode register to reset */
	do
	{
		if ( ! ( pEmacRegs->EMAC_DMA_BUSMODE & BITM_EMAC_DMA_BUSMODE_SWR ) )
			return ( true );
	}
	while ( --loopcount > 0 );
	
	return ( false );
}

/**
 * @brief       Supply buffers to the ethernet driver to receive packets
 *
 * @details     adi_ether_GemacRead is responsible for reading the ethernet frames from the
 *              network. Buffers supplied via this API are queued and used one for each
 *              received frame. Only frames without errors will be received.
 *
 *              This routine can take single or linked list of buffers, it is adviced to
 *              supply list of buffers for optimal operation.
 *
 *              The supplied buffers are associated with a dma descriptor and scheduled for
 *              dma transfer. As number of buffers can vary independent of number of descriptors
 *              if descriptors are not available it will keep the buffers in queued list and
 *              schedules for DMA only when descriptors are available.Depending on the application
 *              requirements configure the number of descriptors.
 *
 *              If device is already started the routine will activate the channel so that
 *              the receive dma channel continue to operate. Once a buffer or buffer list is
 *              supplied to the driver upper layers should not attempt to access these buffers.
 *              Once buffer processing is completed driver will return the buffer via the callback
 *              handler supplied in the adi_ether_GemacOpen function.
 *
 *              All read and write operations are DMA driven. Programmed I/O is not supported.
 *
 *
 * @param [in]  phDevice        Handle to the ethernet device
 *
 * @param [in]  pBuffer         Pointer to a single buffer or list of buffers
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  successfully supplied the buffers
 *
 * @sa          adi_Ether_Write()
 */
ADI_ETHER_RESULT adi_ether_GemacRead ( ADI_ETHER_HANDLE const phDevice,
									   ADI_ETHER_BUFFER *pBuffer
									 )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	
	/* first insert the incoming buffer(s) in to the queued list */
	if ( insert_queue ( &pDev->Rx.Queued, pBuffer ) == ADI_ETHER_RESULT_SUCCESS )
	{
		if ( pDev->Started )
		{
			/* Bind buffers with descriptors if successful activate */
			if ( ( bind_buf_with_desc ( phDevice, &pDev->Rx ) == ADI_ETHER_RESULT_SUCCESS ) )
				// activate_channel(pDev,&pDev->Rx);//by wjm 2014.7.16
				return activate_channel ( pDev, &pDev->Rx );
		}
	}
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/**
 * @brief       Transmits packet over the network
 *
 * @details     adi_ether_GemacWrite is responsible for transmitting ethernet frames over
 *              network. Single or list of frames can be supplied by the upper layers.
 *
 *              The write operation immediately return by queuing up the transfer, this does
 *              not indicate successful transfer of the supplied frame. Actual tranfer of
 *              frame is indicated by returning the buffer via callback.
 *
 *              This routine will bind the incoming buffer with the avaiable transmit channel
 *              descriptor and schedules for DMA transfer. If descriptors are not avaiable
 *              for the transmit channel the buffer gets queued for transmission. As soon as
 *              more descriptors are available buffers will be associated with descriptors and
 *              scheduled for DMA transfer.
 *
 *              Once buffer is supplied to the driver applications should not access the
 *              buffer or buffer contents. With data cache enabled systems any such action
 *              will lead to cache coherency problems. For applications performing large transfers
 *              it is adviced to increase the number of DMA descriptors.
 *
 *              All read and write operations are DMA driven. Programmed I/O is not available.
 *
 * @param [in]  phDevice        Handle to the ethernet device
 *
 * @param [in]  pBuffer         Pointer to a single buffer or list of buffers
 *
 * @return      Status
 *                              -ADI_ETHER_RESULT_SUCCESS  successfully transmitted the packet
 *
 * @sa          adi_Ether_Read()
 */
ADI_ETHER_RESULT adi_ether_GemacWrite ( ADI_ETHER_HANDLE const phDevice,
										ADI_ETHER_BUFFER *pBuffer )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	
	/* first insert the incoming buffer(s) in to the queued list */
	if ( insert_queue ( &pDev->Tx.Queued, pBuffer ) == ADI_ETHER_RESULT_SUCCESS )
	{
		/* Bind buffers with descriptors if successful activate */
		if ( pDev->Started )
		{
			if ( ( bind_buf_with_desc ( phDevice, &pDev->Tx ) ==  ADI_ETHER_RESULT_SUCCESS ) )
				//activate_channel(pDev,&pDev->Tx);//by wjm 2014.7.16
				return activate_channel ( pDev, &pDev->Tx );
		}
	}
	
	return ( ADI_ETHER_RESULT_FAILED );
}


/**
 * @brief       Returns the link status
 *
 * @details     Returns the link status, if link is up it returns true else false. This
 *              API can be used by the applications to check whether network link is present
 *              or not.
 *
 * @param [in]  phDevice        Handle to the ethernet device
 *
 * @return      Boolean
 *                              - True if the link is up, False if the link is down
 */
bool adi_ether_GemacGetLinkStatus ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	uint32_t Status = 0;
	uint32_t interruptStatus  = cli();
	
	//    Status = dp83848_phy_get_status(phDevice,pDev->PhyAddress);
	Status = ADI_ETHER_PHY_LINK_UP;
	sti ( interruptStatus );
	return ( ( Status & ADI_ETHER_PHY_LINK_UP ) ? true : false );
}

/**
 * @brief       Enables the Ethernet.
 *
 * @details     adi_ether_GemacEnableMAC enables the MAC unit. One MAC is enabled driver
 *              configuration can not be changed. Once enable is issued reception and
 *              transmission of frames starts with active ethernet and PHY interrupts.
 *
 *              Any previously sumbitted transmit and receive buffers will also be processed.
 *
 * @param [in]  phDevice        Handle to the ethernet device
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 *                              - ADI_ETHER_RESULT_FAILED [D]  in case of failure
 *                              - ADI_ETHER_RESULT_RESET_FAILED Unable to reset MAC
 */
ADI_ETHER_RESULT adi_ether_GemacEnableMAC ( ADI_ETHER_HANDLE phDevice )
{
	ADI_INT_STATUS      intResult;
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = ( ( ADI_EMAC_DEVICE * ) phDevice )->pEMAC_REGS;
	uint32_t            Status;
	ADI_EMAC_DMADESC    *pDmaDesc;



#if defined(ADI_DEBUG)
	
	/* If device is already enabled then we throw an error in debug build */
	if ( pDev->Started )
	{
		return ( ADI_ETHER_RESULT_FAILED );
	}
	
#endif
	
	/* reset emac bus mode register */
	/*The EMAC_DMA_BUSMODE.SWR bit, when set, directs the MAC DMA
		Controller to reset all MAC Subsystem internal registers and logic. It
		is cleared automatically after the reset operation has completed in all
		of the core clock domains. Read a 0 value in this bit before reprogramming
		any register of the core.
	 * */
	gemac_set_dmabusmode ( phDevice, BITM_EMAC_DMA_BUSMODE_SWR );
	
	/* wait for rest to finsih */
	if ( !gemac_reset_complete ( phDevice ) )
	{
		ETHER_DEBUG ( "rest failed \n", *pREG_EMAC0_DMA_BUSMODE );
		return ( ADI_ETHER_RESULT_RESET_FAILED );
	}
	
	pEmacRegs->EMAC_IMSK = BITM_EMAC_IMSK_TS; // DISABLE Time Stamp Interrupt
	Status = pEmacRegs->EMAC_ISTAT;
	Status = pEmacRegs->EMAC_TM_STMPSTAT;

	/* clear any pending interrupts - write to clear bits in status reg */
	Status = pEmacRegs->EMAC_DMA_STAT;
	
	/* only lower 16-bits has interrupts and w1c */
	pEmacRegs->EMAC_DMA_STAT = Status & 0x1FFFF;
	
	pEmacRegs->EMAC_MMC_RXIMSK = 0x00ffffff;
	pEmacRegs->EMAC_MMC_TXIMSK = 0x00ffffff;
	
	/*
	gemac_set_dmabusmode(phDevice,  (1UL << BITP_EMAC_DMA_BUSMODE_RPBL) |
	                 (1UL << BITP_EMAC_DMA_BUSMODE_PBL)  |
	   (1UL << BITP_EMAC_DMA_BUSMODE_ATDS));
	   */
	
	gemac_set_dmabusmode ( phDevice, ( 1UL << BITP_EMAC_DMA_BUSMODE_ATDS ) );
	
	
	/* initialize dma interrupt mask register
	 * normal mode enables the following interrupts
	 * - Transmit interrupt
	 * - Transmit buffer unavailable
	 * - Receive interrupt
	 * - Early receive interrupt
	 */
	pEmacRegs->EMAC_DMA_IEN = ( 1UL << BITP_EMAC_DMA_IEN_NIS ) |
							  ( 1UL << BITP_EMAC_DMA_IEN_AIS ) |
							  ( 1UL << BITP_EMAC_DMA_IEN_RU )  |
							  ( 1UL << BITP_EMAC_DMA_IEN_RI )  |
							  ( 1UL << BITP_EMAC_DMA_IEN_UNF ) |
							  ( 1UL << BITP_EMAC_DMA_IEN_OVF)   |
							  ( 1UL << BITP_EMAC_DMA_IEN_TU )  |
							  ( 1UL << BITP_EMAC_DMA_IEN_TI );
							  
	/* setup descriptor lists */
	
	/* setup appropriate filtering options GEMAC 1,2,3 */
	/* enable GEMAC register 0 for trnamsit and receive modes */
	gemac_init ( phDevice );
	
	
	/* Enable DMA for rx and tx if pakcets are present */
	gemac_set_dmaopmode ( phDevice, // BITM_EMAC_DMA_OPMODE_OSF    |
						  BITM_EMAC_DMA_OPMODE_FEF    |
						  BITM_EMAC_DMA_OPMODE_FUF    |
						  ENUM_EMAC_DMA_OPMODE_TTC_64 |
						  ENUM_EMAC_DMA_OPMODE_RTC_64 );
						  
	/* Register Ehernet Interrupt Handler and enable it */
	if ( ( intResult = adi_int_InstallHandler (
						   pDev->Interrupt,
						   EMACInterruptHandler,
						   ( void * ) pDev,
						   false
					   ) ) != ADI_INT_SUCCESS )
	{
		return ( ADI_ETHER_RESULT_FAILED );
	}
	
	adi_ether_SetMACAddress ( pDev, ( const uint8_t * ) pDev->MacAddress );
	
	pEmacRegs->EMAC_DMA_TXDSC_ADDR = ( uint32_t ) pDev->Tx.pDmaDescHead;
	pEmacRegs->EMAC_DMA_RXDSC_ADDR = ( uint32_t ) pDev->Rx.pDmaDescHead;
	
	if (pDev->TxTimeStamped || pDev->RxTimeStamped )
	{
		//*pREG_EMAC0_TM_CTL = 0;
		pEmacRegs->EMAC_TM_CTL = 0;

		// init steps for TASK:
		// a. Capture the ETHER FRAMES Time Stamp
		// c. Programming for Auxiliary Timestamps

		//a.1 set the TTSE bit in the TDES0 register of the corresponding frame
		//a.2  Extend the descriptor word length from 4 words to 8 words by setting the EMAC_DMA_BUSMODE.ATDS bit
		
		//a.3 detects and/or timestamps only specific types of received frames
		//*pREG_EMAC0_TM_CTL |= BITM_EMAC_TM_CTL_TSENALL; //ENUM_EMAC_TM_CTL_E_TSALL_FRAMES;
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENALL; //ENUM_EMAC_TM_CTL_E_TSALL_FRAMES;
		
		//a.4 select clk source
		*pREG_PADS0_EMAC_PTP_CLKSEL = 0x5;//SELECT SCLK0/SCLK1, EMAC0:SCLK0(0x1) AND EMAC1:SCLK0(0x4) ，SCLK(0X01)

		//c.1 Set the EMAC_IMSK.TS bit to enable PTP interrupts. THIS is done after ENABLE the EMAC.
//
//		*pREG_EMAC0_IMSK  =  TM_AUXI_INT_EN;//BITM_EMAC_IMSK_TS;
//		asm ( "SSYNC;" );

		//a.5 Enable the PTP module
		//c.2 Set the EMAC_TM_CTL.TSENA bit to enable the PTP module
		pEmacRegs->EMAC_TM_CTL |= BITM_EMAC_TM_CTL_TSENA ; //ENUM_EMAC_TM_CTL_TS;//TSENA: Enable PTP Module


		//a.6 init system time
			//c.3 init system time, all initiations are over for TASK C.
		InitSysTime( pEmacRegs );

		
		/*a.7 Verify the RDES4 register for the status of the received frame and the RDES6 and RDES7 registers for
			timestamp nanoseconds and seconds value.
		 */
		asm ( "SSYNC;" );

	}
	
	pDev->Started = true;
	
	/* setup the phy */
#if  BF609_EZ_BRD
	dp83848_phy_init ( phDevice, 1 ); //wjm@phy
#endif
	ENTER_CRITICAL_REGION();
	/* activate rx channel */
	bind_buf_with_desc ( phDevice, &pDev->Rx );
	activate_channel ( pDev, &pDev->Rx );
	//      enable_rx(pDev);
	
	/* activate tx channel */
	bind_buf_with_desc ( phDevice, &pDev->Tx );
	activate_channel ( pDev, &pDev->Tx );
	//      enable_tx(pDev);
	EXIT_CRITICAL_REGION();
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}


/**
 * @brief       Add's supplied multicast group address in the MAC
 *
 * @details     Enables the relevant multicast bits for the supplied multicast group address.
 *              Multicast based applications typically use IGMP protocol and enable multicast
 *              functionality using BSD socket API call ioctlsocket().
 *
 * @param [in]  phDevice            Handle to the ethernet device
 *
 * @param [in]  MultiCastGroupAddr  Multicast group address
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
ADI_ETHER_RESULT adi_ether_GemacAddMulticastFilter ( ADI_ETHER_HANDLE phDevice,
		const uint32_t MultiCastGroupAddr )
{
	return add_multicastmac_filter ( phDevice, MultiCastGroupAddr, true );
}

/**
 * @brief       Deletes the supplied multicast group address from the EMAC
 *
 * @details     Disables the relevant multicast bits for the supplied multicast group address.
 *              Multicast based applications typically use IGMP protocol and disable multicast
 *              functionality using BSD socket API ioctlsocket() which will in-turn call the
 *              driver routine.
 *
 * @param [in]  phDevice             Handle to the ethernet device
 *
 * @param [in]  MultiCastGroupAddr   Multicast group address
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
ADI_ETHER_RESULT adi_ether_GemacDelMulticastFilter ( ADI_ETHER_HANDLE phDevice,
		const uint32_t MultiCastGroupAddr )
{
	return add_multicastmac_filter ( phDevice, MultiCastGroupAddr, false );
}

/**
 * @brief       Returns the current MAC address
 *
 * @details     This API returns the MAC address present in the EMAC.
 *
 * @param [in]  phDevice           Handle to the ethernet device
 *
 * @param [out] pMacAddress        Pointer to the memory area where MAC address will be stored. It
 *                                 has to be atleast 6 bytes long.
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
ADI_ETHER_RESULT adi_ether_GemacGetMACAddress ( ADI_ETHER_HANDLE phDevice, uint8_t *pMacAddress )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	uint32_t  mac;
	int32_t   i;
	
	mac = pEmacRegs->EMAC_ADDR0_LO;
	
	for ( i = 0; i <= 3 ; i++ )
	{
		pMacAddress[i] = mac & 0xFF;
		mac >>= 8;
	}
	
	mac = pEmacRegs->EMAC_ADDR0_HI;
	
	for ( i = 4; i <= 5; i++ )
	{
		pMacAddress[i] = mac & 0xFF;
		mac >>= 8;
	}
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/**
 * @brief       Sets the given MAC address in the EMAC
 *
 * @details     adi_ether_GemacSetMACAddress sets the MAC address in the EMAC. It will not
 *              update the non-volatile block where ez-kits store the MAC address.
 *
 * @param [in]  phDevice           Handle to the ethernet device
 *
 * @param [in] pMacAddress         Pointer to the memory where MAC address is present
 *
 * @return      Status
 *                              - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
ADI_ETHER_RESULT adi_ether_GemacSetMACAddress ( ADI_ETHER_HANDLE phDevice, const uint8_t *pMacAddress )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	uint32_t  mac;
	int32_t   i;
	
	mac = 0;
	
	/* store the mac address */
	memcpy ( pDev->MacAddress, pMacAddress, 6 );
	
	for ( i = 3; i >= 0; i-- )
	{
		mac = ( mac << 8 ) | pMacAddress[i];
	}
	
	pEmacRegs->EMAC_ADDR0_LO = mac;
	
	mac = 0;
	
	for ( i = 5; i >= 4; i-- )
	{
		mac = ( mac << 8 ) | pMacAddress[i];
	}
	
	pEmacRegs->EMAC_ADDR0_HI = mac;
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}


/**
 * @brief       Initializes the descriptor list for a channel
 *
 * @details     This functions constructs the descriptor linked list from the supplied memory.
 *              It also aligns the start address of the descriptor memory area to 32-byte. This
 *              is required to maintain the cache coherencey. By the end of this function
 *              pChannel->pDmaDescHead will point to the start of the dma descriptor chain and
 *              pChannel->pDmaDescTail will point to the last descriptor in the chain. It also
 *              updates the pChannel->NumAvailDmaDesc to the number of available descriptors.
 *
 * @param [in]  phDevice      Handle to the ethernet device
 *
 * @param [in]  pMemory       Start adress of memory area for descriptors
 *
 * @param [in]  Length        Length of the pMemory block in bytes
 *
 * @param [in]  pChannel      Pointer to the channel
 *
 * @return      Status
 *                            - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
static ADI_ETHER_RESULT init_descriptor_list ( ADI_ETHER_HANDLE phDevice,
		const uint8_t *pMemory,
		const uint32_t Length,
		ADI_EMAC_CHANNEL *pChannel )
{
	uint8_t *pCurBase, *pOriginalBase;
	int32_t ActualLength, MaxNumDesc = 0, i;
	ADI_EMAC_DEVICE    *const  pDev = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_DMADESC    *pLast = NULL, *pFirst = NULL;
	
	pOriginalBase =  ( uint8_t * ) pMemory;
	
	/* align the start address to 32-byte */
	pCurBase = ( uint8_t * ) ( ( ( ( uint32_t ) pMemory ) + 31 ) & ( ~0x1F ) );
	//pCurBase = (uint8_t*)((((uint32_t)pMemory) + 3) & (~0x3));
	
	/* Adjust length according to the alignment */
	ActualLength = Length - ( pCurBase - pOriginalBase );
	
	/* construct the descriptor list */
	pChannel->pDmaDescHead    = ( ADI_EMAC_DMADESC * ) pCurBase;
	
	/* compute the number of descriptors */
	pChannel->NumAvailDmaDesc = MaxNumDesc = ActualLength / sizeof ( ADI_EMAC_DMADESC );
	
	/* set the first descriptor */
	pFirst = pChannel->pDmaDescHead;
	
	for ( i = 1 ; i < MaxNumDesc; i++ )
	{
		/* set pLast to the next descriptor */
		pLast =  pFirst + 1;
		
		/* reset descriptor memory to zero */
		memset ( pFirst, 0, sizeof ( ADI_EMAC_DMADESC ) );
		
		/* set the next pointer of first descriptor */
		pFirst->pNextDesc =  pLast;
		
		/* move the first to next */
		pFirst++;
	}
	
	// pLast->pNextDesc = pChannel->pDmaDescHead;
	pLast->pNextDesc = NULL;
	
	/* set tail to point to the last descriptor */
	pChannel->pDmaDescTail = pLast;
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/*
 * Inserts the buffer in the given queue
 */
static ADI_ETHER_RESULT insert_queue ( ADI_EMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER *pBuffer )
{
	int32_t NumInputBuffers = 0;
	ADI_ETHER_BUFFER *pTempBuffer = pBuffer, *pLastBuffer = NULL;
	
#ifdef ADI_DEBUG
	
	if ( pBuffer == NULL ) return ADI_ETHER_RESULT_NULL_BUFFER;
	
#endif
	
	/* typically the number of incoming buffers are small */
	do
	{
		NumInputBuffers++;
		pLastBuffer = pTempBuffer;
		pTempBuffer = pTempBuffer->pNext;
		
	}
	while ( pTempBuffer != NULL );
	
	ENTER_CRITICAL_REGION();
	
	/* Now insert and update the queue */
	if ( ( pQueue->pQueueHead == NULL ) && ( pQueue->pQueueTail == NULL ) )
		pQueue->pQueueHead = pBuffer;
		
	else
		pQueue->pQueueTail->pNext = pBuffer;
		
	pQueue->pQueueTail    = pLastBuffer;
	pQueue->ElementCount += NumInputBuffers;
	
	EXIT_CRITICAL_REGION();
	
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/**
 * @brief       Sets descriptor values
 *
 * @details     Depending on the channel the decriptors were configured with default values.
 *
 * @param [in]  hDevice       Handle to the device
 * @param [in]  pDmaDesc      Pointer to the dma descriptor
 * @param [in]  pBindedBuf    Pointer to the buffer to be associated with the descriptor
 * @param [in]  pChannel      Pointer to the channel
 *
 * @return      void
 *
 * @note        used by bind_buf_with_desc()
 */
static void  set_descriptor ( ADI_ETHER_HANDLE hDevice,
							  ADI_EMAC_DMADESC *pDmaDesc,
							  ADI_ETHER_BUFFER *pBindedBuf,
							  ADI_EMAC_CHANNEL *pChannel )
{
	ADI_EMAC_DEVICE    *const  pDev = ( ADI_EMAC_DEVICE * ) hDevice;
	
	/* set the descriptor parameters */
	pDmaDesc->ControlDesc = pBindedBuf->ElementCount;
	pDmaDesc->Status      = 0;
	
	if ( pChannel->Recv )
	{
#if 0
		/*****added by wjm@2014-10-20, reserved 14 bytes for the forward
		 *  header (FORWARD_ETHER_FRAME )*****/
		pDmaDesc->StartAddr   = ( uint32_t ) ( ( uint8_t * ) pBindedBuf->Data + 2 + 14 );

		if( hDevice == g_hDev[1]  )
		{
			pDmaDesc->StartAddr   = ( uint32_t ) ( ( uint8_t * ) pBindedBuf->Data + 2 );
		}
		/**********/
#endif
		pDmaDesc->StartAddr   = ( uint32_t ) ( ( uint8_t * ) pBindedBuf->Data + 2 );

		pDmaDesc->ControlDesc |= ( 1UL << 14 );
		
		/* data cache is enabled flush and invalidate the cache for entire data area */
		if ( pDev->Cache )
		{
			flushinv_area ( ( uint8_t * ) pBindedBuf->Data,
							( uint32_t ) ( pBindedBuf->ElementWidth * pBindedBuf->ElementCount ) );
		}
	}
	
	else
	{
		pDmaDesc->StartAddr   = ( uint32_t ) ( ( uint8_t * ) pBindedBuf->Data + 2 );
		pDmaDesc->ControlDesc -= 2;
		pDmaDesc->Status |= ( ( 1UL << 30 ) | ( 1UL << 29 ) |
							  ( 1UL << 28 ) | ( 1UL << 20 ) );
							  
		/* data cache is enabled flush the cache for entire data area */
		if ( pDev->Cache )
		{
			flush_area ( ( uint8_t * ) pBindedBuf->Data,
						 ( uint32_t ) ( pBindedBuf->ElementWidth * pBindedBuf->ElementCount ) );
		}
	}
	
	/* now give owner ship of the descriptor to dma */
	pDmaDesc->Status     |= ADI_EMAC_DMAOWN;
	
	/* if descriptors are cached flush and invalidate the descriptors */
	if ( pDev->Cache )
	{
		SIMPLEFLUSHINV ( pDmaDesc );
	}
}

/**
 * @brief       Bind buffers with descriptors.
 *
 * @details     This function fetches an available dma descriptor from the respective
 *              channel and associates with a buffer in Queued list then moves the buffer to
 *              the Pending queue. The buffers in Pending queue are ready to be go to the
 *              active dma chain. Number of dma descriptors are independent of the number
 *              of buffers in the system. So if there is no available dma descriptor to
 *              associate a buffer then buffer is left in Queued list.
 *
 *              Depending on the channel the decriptors were configured with default values.
 *
 * @param [in]  hDevice       Handle to the ethernet device
 * @param [in]  pChannel      Pointer to the channel
 *
 * @return      Status
 *                            - ADI_ETHER_RESULT_SUCCESS  if successfully enables the EMAC
 */
static ADI_ETHER_RESULT bind_buf_with_desc ( ADI_ETHER_HANDLE hDevice, ADI_EMAC_CHANNEL *pChannel )
{
	ADI_EMAC_DMADESC *pAvailDesc;
	ADI_ETHER_BUFFER *pQueuedBuf;
	
	/* Mask off etherent interrupt alone */
	ENTER_CRITICAL_REGION();
	
	pAvailDesc = pChannel->pDmaDescHead;
	pQueuedBuf = pChannel->Queued.pQueueHead;
	
#ifdef ADI_DEBUG
	
	if ( ( pAvailDesc == NULL ) || ( pQueuedBuf == NULL ) )
	{
		EXIT_CRITICAL_REGION();
		return ADI_ETHER_RESULT_NULL_BUFFER;
	}
	
#endif /* ADI_DEBUG */
	
	
	/* We will leave the last descriptor without associating with a buffer. This is
	 * because the DMA will fetch the descriptor and enter into suspend state. We do
	 * not want to stop and restart DMA but rather use the last descriptor to continue
	 * from the suspended state.
	 */
	while ( pAvailDesc && ( pChannel->NumAvailDmaDesc > 1 ) && pQueuedBuf )
	{
		/* remove the queued buffer from the queue */
		pChannel->Queued.pQueueHead = pChannel->Queued.pQueueHead->pNext;
		pChannel->Queued.ElementCount--;
		pQueuedBuf->pNext = NULL;
		
		/* remove the descriptor from the avail list */
		pChannel->pDmaDescHead = pChannel->pDmaDescHead->pNextDesc;
		pChannel->NumAvailDmaDesc--;
		
		/* now bind the descriptor with the buffer */
		( ( ADI_EMAC_BUFINFO * ) pQueuedBuf )->pDmaDesc = pAvailDesc;
		
		set_descriptor ( hDevice, pAvailDesc, pQueuedBuf, pChannel );
		
		/* now put the buffer in the pending list */
		if ( pChannel->Pending.pQueueHead == NULL )
		{
			pChannel->Pending.pQueueHead = pQueuedBuf;
			pChannel->Pending.pQueueTail = pQueuedBuf;
			pChannel->Pending.ElementCount = 1;
		}
		
		else
		{
			/* link the descriptors */
			( ( ADI_EMAC_BUFINFO * ) pChannel->Pending.pQueueTail )->pDmaDesc->pNextDesc = pAvailDesc;
			pChannel->Pending.pQueueTail->pNext = pQueuedBuf;
			pChannel->Pending.pQueueTail = pQueuedBuf;
			pChannel->Pending.ElementCount++;
		}
		
		/* next available descriptor will be always at the end */
		pAvailDesc->pNextDesc = pChannel->pDmaDescHead;
		
		pAvailDesc = pChannel->pDmaDescHead;
		pQueuedBuf = pChannel->Queued.pQueueHead;
		
		if ( pQueuedBuf == NULL )
		{
			pChannel->Queued.pQueueTail = NULL;
		}
	}
	
	EXIT_CRITICAL_REGION();
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/*
 * Moves binded buffers from the pending queue and places them in the
 * active queue. It also enables the appropriate config and dma opmode bits.
 */

static ADI_ETHER_RESULT activate_channel ( ADI_ETHER_HANDLE hDevice, ADI_EMAC_CHANNEL *pChannel )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	ADI_ETHER_BUFFER *pPendQFirstBuf, *pActiveQLastBuf;
	ADI_EMAC_DMADESC *pLastDmaDesc, *pNextDmaDesc;
	
	ENTER_CRITICAL_REGION();
	
	
	/* if no buffers to be processed return */
	if ( pChannel->Pending.pQueueHead == NULL )
	{
		EXIT_CRITICAL_REGION();
		return ADI_ETHER_RESULT_SUCCESS;
	}
	
	/* check if there are any buffers in the active list */
	if ( pChannel->Active.pQueueHead == NULL )
	{
		copy_queue_elements ( &pChannel->Active, &pChannel->Pending );
		reset_queue ( &pChannel->Pending );
		
		pNextDmaDesc = ( ( ADI_EMAC_BUFINFO * ) pChannel->Active.pQueueHead )->pDmaDesc;
		pLastDmaDesc = ( ( ADI_EMAC_BUFINFO * ) pChannel->Active.pQueueTail )->pDmaDesc;
		
		if ( pChannel->Recv )
		{
			pLastDmaDesc->ControlDesc |= ( 1UL << 14 );
			
			/* if data cache is enabled flush and invalidate the descriptor */
			if ( pDev->Cache )
			{
				SIMPLEFLUSHINV ( pLastDmaDesc );
			}
			
			if ( ( pEmacRegs->EMAC_DMA_STAT & BITM_EMAC_DMA_STAT_RS ) == ENUM_EMAC_DMA_STAT_RS_STOPPED )
			{
				pEmacRegs->EMAC_DMA_RXDSC_ADDR = ( uint32_t ) pNextDmaDesc;
			}
			
			resume_rx ( hDevice );
		}
		
		else
		{
			pLastDmaDesc->Status |= ( 1UL << 20 );
			pLastDmaDesc->Status |= ( 1UL << 30 ) ;
			
			/* if data cache is enabled flush and invalidate the descriptor */
			if ( pDev->Cache )
			{
				SIMPLEFLUSHINV ( pLastDmaDesc );
			}
			
			if ( ( pEmacRegs->EMAC_DMA_STAT & BITM_EMAC_DMA_STAT_TS ) == ENUM_EMAC_DMA_STAT_TS_STOPPED )
			{
				pEmacRegs->EMAC_DMA_TXDSC_ADDR = ( uint32_t ) pNextDmaDesc;
			}
			
			resume_tx ( hDevice );
			ssync();
		}
	}
	
	else
	{
		/* if there are sufficient buffers to process we append to the
		 * currently active list
		 */
		if ( pChannel->Active.ElementCount >= ADI_EMAC_THERSHOLD )
		{
			pActiveQLastBuf = pChannel->Active.pQueueTail;
			pPendQFirstBuf  = pChannel->Pending.pQueueHead;
			
			pLastDmaDesc = ( ( ADI_EMAC_BUFINFO * ) pActiveQLastBuf )->pDmaDesc;
			pNextDmaDesc = ( ( ADI_EMAC_BUFINFO * ) pPendQFirstBuf )->pDmaDesc;
			
			if ( pChannel->Recv )
				pLastDmaDesc->ControlDesc |= ( 1ul << 14 );
				
			else
			{
				pLastDmaDesc->Status |=  ( 1ul << 20 ) ;
				pLastDmaDesc->Status |= ( 1ul << 30 ) ;
			}
			
			/* now link the descriptors */
			pLastDmaDesc->pNextDesc = pNextDmaDesc;
			
			if ( pDev->Cache )
			{
				SIMPLEFLUSHINV ( pLastDmaDesc );
			}
			
			/* link the buffers */
			pActiveQLastBuf->pNext = pPendQFirstBuf;
			pChannel->Active.pQueueTail = pChannel->Pending.pQueueTail;
			pChannel->Active.ElementCount += pChannel->Pending.ElementCount;
			
			/* get the newly set last descriptor in the active list */
			pLastDmaDesc = ( ( ADI_EMAC_BUFINFO * ) pChannel->Active.pQueueTail )->pDmaDesc;
			
			/* adjust the last descriptor */
			reset_queue ( &pChannel->Pending );
		}
		
		if ( pChannel->Recv )
			resume_rx ( hDevice );
			
		else
			resume_tx ( hDevice );
	}
	
	EXIT_CRITICAL_REGION();
	
	return ADI_ETHER_RESULT_SUCCESS;
}


/**
* @brief        Closes the driver and releases any memory
*
* @details      Closes the MAC driver. Buffers that are pending will not be returned.
*
* @param[in]    hEtherDevice    Ethernet Device Handler
*
* @return       Status
*                   - ADI_ETHER_RESULT_SUCCESS on Success
*                   - ADI_ETHER_RESULT_INVALID_PARAM on Parameter Error
*                   - ADI_ETHER_RESULT_FAILED on Error
*
* @details
*               Function closes the driver and releases any memory.
*               If there is any current frames then it will wait for it
*               to complete.
*
* @sa           adi_ether_GemacOpen
*/

ADI_ETHER_RESULT adi_ether_GemacClose ( ADI_ETHER_HANDLE hEtherDevice )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) hEtherDevice;
	ADI_ETHER_RESULT    Result = ADI_ETHER_RESULT_FAILED;
	
	ENTER_CRITICAL_REGION();
	
	if ( pDev->Opened == true )
	{
		/* stop transmit and receive */
		gemac_stop_rx ( hEtherDevice );
		gemac_stop_tx ( hEtherDevice );
		mask_gemac_ints ( hEtherDevice );
		
		/* reset driver internal variables */
		reset_all_queues ( hEtherDevice );
		reset_dma_lists ( &pDev->Rx );
		reset_dma_lists ( &pDev->Tx );
		
		pDev->Opened = false;
		Result = ADI_ETHER_RESULT_SUCCESS;
	}
	
	adi_int_UninstallHandler ( INTR_PINT2_BLOCK );
	adi_int_UninstallHandler ( pDev->Interrupt );
	
	EXIT_CRITICAL_REGION();
	
	return ( Result );
}

/**
* @brief        Return the Buffer Prefix for the Driver
*
* @param[in]    hEtherDevice   Ethernet Device Handle
*
* @param[out]   pBufferPrefix  Pointer to Buffer Prefix
*
* @return       Status
*                  - ADI_ETHER_RESULT_SUCCESS on Success
*                  - ADI_ETHER_RESULT_INVALID_PARAM on Parameter Error
*                  - ADI_ETHER_RESULT_FAILED on Error
*
*/
ADI_ETHER_RESULT adi_ether_GemacGetBufferPrefix (
	ADI_ETHER_HANDLE    hEtherDevice,
	uint32_t *const     pBufferPrefix
)
{

#if defined (ADI_DEBUG)
	ADI_ETHER_HANDLE      hDev;
	
	/* Map the Handle to Internal Handle */
	hDev = hEtherDevice;
	
#endif
	
	/* The Buffer prefix used is 2 for GEMAC driver */
	( *pBufferPrefix ) = 2;
	
	/* Return Success */
	return ( ADI_ETHER_RESULT_SUCCESS );
}

/***************************************************************************//*!
* @brief        Flush a Memory Area
*
* @param[in]    start   Pointer to the start of the memory Segment
* @param[in]    bytes   Number of bytes to flush
*
* @return       None
*
* @details
*               Function flush the given memory segment by flushing out each
*               memory element. If the memory is cached and dirty, the data will
*               be written back to the next higher memory else it act as NOP.
*
* @sa           flushinv_area
*******************************************************************************/
static void flush_area ( void *start, uint32_t bytes )
{
	uint32_t count;
	uint32_t x;
	
	count = ( bytes + 31U + ( ( ( uint32_t ) start ) & 31U ) ) / 32U;
	
	/* System Sync */
	ssync();
	
	/* Flush given cache area */
	for ( x = 0U; x < count; x++ )
	{
		/* Flush the cache line at location start and increment start to next
		   cache line */
		FLUSH ( start );
	}
	
	/* System Sync */
	ssync();
}

/***************************************************************************//*!
* @brief        Flush & Invalidate a Memory Area
*
* @param[in]    start   Pointer to the start of the memory Segment
* @param[in]    bytes   Number of bytes to flush
*
* @return       None
*
* @details
*               Function flush the given memory segment by flushing out each
*               memory element and it also invalidate the cache. If the memory
*               is cached and dirty, the data will be written back to the next
*               higher memory else it act as NOP
*
* @sa           flush_area
*******************************************************************************/
static void flushinv_area ( void *start, uint32_t bytes )
{
	uint32_t count;
	uint32_t x;
	
	count = ( bytes + 31U + ( ( ( uint32_t ) start ) & 31U ) ) / 32U;
	
	/* System sync */
	ssync();
	
	/* Invalidate cache in the given area */
	for ( x = 0U; x < count; x++ )
	{
		/* Inverse Flush the location start and move to next cache line */
		FLUSHINV ( start );
	}
	
	/* System Sync */
	ssync();
}

/********************/
void clear_queue ( ADI_EMAC_FRAME_Q *pQueue )
{
	pQueue->pQueueHead = NULL;
	pQueue->pQueueTail = NULL;
	pQueue->ElementCount = 0;
}


ADI_ETHER_BUFFER *pop_queue ( ADI_EMAC_FRAME_Q *pQueue )
{
	ADI_ETHER_BUFFER *p = NULL;

	ENTER_CRITICAL_REGION();


	p = pQueue->pQueueHead;

	if ( !p )
	{
		EXIT_CRITICAL_REGION();

		return NULL;
	}

	if ( pQueue->pQueueHead  == pQueue->pQueueTail  )
	{
		pQueue->pQueueTail = NULL;
		pQueue->pQueueHead  = NULL;
	}

	else
	{
		pQueue->pQueueHead = p->pNext;
	}


	pQueue->ElementCount--;

	p->pNext = NULL;

	EXIT_CRITICAL_REGION();

	return p;
}

int push_queue ( ADI_EMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pBuffer )
{

	int32_t NumInputBuffers = 0;
	ADI_ETHER_BUFFER *pTempBuffer = pBuffer, *pLastBuffer = NULL;

	if ( !pQueue || !pBuffer )
	{
		return 0;
	}

	/* typically the number of incoming buffers are small */
	do
	{
		NumInputBuffers++;
		pLastBuffer = pTempBuffer;
		pTempBuffer = pTempBuffer->pNext;

	}while ( pTempBuffer != NULL );

	ENTER_CRITICAL_REGION();

	/* Now insert and update the queue */
	if ( ( pQueue->pQueueHead == NULL ) && ( pQueue->pQueueTail == NULL ) )
		pQueue->pQueueHead = pBuffer;

	else
		pQueue->pQueueTail->pNext = pBuffer;

	pQueue->pQueueTail    = pLastBuffer;
	pQueue->ElementCount += NumInputBuffers;

	EXIT_CRITICAL_REGION();

	return NumInputBuffers;
}

#ifdef ADI_DEBUG
#include <stdio.h>
/* Print the addresses of the EMAC registers */
void PrintEMACRegisterAddresses ( ADI_ETHER_HANDLE phDevice )
{
	ADI_EMAC_DEVICE    *const  pDev      = ( ADI_EMAC_DEVICE * ) phDevice;
	ADI_EMAC_REGISTERS *const  pEmacRegs = pDev->pEMAC_REGS;
	
	ETHER_DEBUG ( "MACCONFIG = 0x%08X\n", &pEmacRegs->EMAC_MACCFG );
	ETHER_DEBUG ( "EMAC_MACFRMFILT = %08X\n", &pEmacRegs->EMAC_MACFRMFILT );
	ETHER_DEBUG ( "EMAC_HASHTBL_HI = %08X\n", &pEmacRegs->EMAC_HASHTBL_HI );
	ETHER_DEBUG ( "EMAC_HASHTBL_LO = %08X\n", &pEmacRegs->EMAC_HASHTBL_LO );
	ETHER_DEBUG ( "EMAC_GMII_ADDR = %08X\n", &pEmacRegs->EMAC_GMII_ADDR );
	ETHER_DEBUG ( "EMAC_GMII_DATA = %08X\n", &pEmacRegs->EMAC_GMII_DATA );
	ETHER_DEBUG ( "EMAC_FLOWCTL = %08X\n", &pEmacRegs->EMAC_FLOWCTL );
	ETHER_DEBUG ( "EMAC_VLANTAG = %08X\n", &pEmacRegs->EMAC_VLANTAG );
	ETHER_DEBUG ( "EMAC_VER = %08X\n", &pEmacRegs->EMAC_VER );
	ETHER_DEBUG ( "EMAC_DBG = %08X\n", &pEmacRegs->EMAC_DBG );
	ETHER_DEBUG ( "EMAC_RMTWKUP = %08X\n", &pEmacRegs->EMAC_RMTWKUP );
	ETHER_DEBUG ( "EMAC_PMT_CTLSTAT = %08X\n", &pEmacRegs->EMAC_PMT_CTLSTAT );
	ETHER_DEBUG ( "EMAC_ISTAT = %08X\n", &pEmacRegs->EMAC_ISTAT );
	ETHER_DEBUG ( "EMAC_IMSK = %08X\n", &pEmacRegs->EMAC_IMSK );
	ETHER_DEBUG ( "EMAC_ADDR0_HI = %08X\n", &pEmacRegs->EMAC_ADDR0_HI );
	ETHER_DEBUG ( "EMAC_ADDR0_LO = %08X\n", &pEmacRegs->EMAC_ADDR0_LO );
	ETHER_DEBUG ( "EMAC_MMC_CTL = %08X\n", &pEmacRegs->EMAC_MMC_CTL );
	ETHER_DEBUG ( "EMAC_MMC_RXINT = %08X\n", &pEmacRegs->EMAC_MMC_RXINT );
	ETHER_DEBUG ( "EMAC_MMC_TXINT = %08X\n", &pEmacRegs->EMAC_MMC_TXINT );
	ETHER_DEBUG ( "EMAC_MMC_RXIMSK = %08X\n", &pEmacRegs->EMAC_MMC_RXIMSK );
	ETHER_DEBUG ( "EMAC_MMC_TXIMSK = %08X\n", &pEmacRegs->EMAC_MMC_TXIMSK );
	ETHER_DEBUG ( "EMAC_TXOCTCNT_GB = %08X\n", &pEmacRegs->EMAC_TXOCTCNT_GB );
	ETHER_DEBUG ( "EMAC_TXFRMCNT_GB = %08X\n", &pEmacRegs->EMAC_TXFRMCNT_GB );
	ETHER_DEBUG ( "EMAC_TXBCASTFRM_G = %08X\n", &pEmacRegs->EMAC_TXBCASTFRM_G );
	ETHER_DEBUG ( "EMAC_TXMCASTFRM_G = %08X\n", &pEmacRegs->EMAC_TXMCASTFRM_G );
	ETHER_DEBUG ( "EMAC_TX64_GB = %08X\n", &pEmacRegs->EMAC_TX64_GB );
	ETHER_DEBUG ( "EMAC_TX65TO127_GB = %08X\n", &pEmacRegs->EMAC_TX65TO127_GB );
	ETHER_DEBUG ( "EMAC_TX128TO255_GB = %08X\n", &pEmacRegs->EMAC_TX128TO255_GB );
	ETHER_DEBUG ( "EMAC_TX256TO511_GB = %08X\n", &pEmacRegs->EMAC_TX256TO511_GB );
	ETHER_DEBUG ( "EMAC_TX512TO1023_GB = %08X\n", &pEmacRegs->EMAC_TX512TO1023_GB );
	ETHER_DEBUG ( "EMAC_TX1024TOMAX_GB = %08X\n", &pEmacRegs->EMAC_TX1024TOMAX_GB );
	ETHER_DEBUG ( "EMAC_TXUCASTFRM_GB = %08X\n", &pEmacRegs->EMAC_TXUCASTFRM_GB );
	ETHER_DEBUG ( "EMAC_TXMCASTFRM_GB = %08X\n", &pEmacRegs->EMAC_TXMCASTFRM_GB );
	ETHER_DEBUG ( "EMAC_TXBCASTFRM_GB = %08X\n", &pEmacRegs->EMAC_TXBCASTFRM_GB );
	ETHER_DEBUG ( "EMAC_TXUNDR_ERR = %08X\n", &pEmacRegs->EMAC_TXUNDR_ERR );
	ETHER_DEBUG ( "EMAC_TXSNGCOL_G = %08X\n", &pEmacRegs->EMAC_TXSNGCOL_G );
	ETHER_DEBUG ( "EMAC_TXMULTCOL_G = %08X\n", &pEmacRegs->EMAC_TXMULTCOL_G );
	ETHER_DEBUG ( "EMAC_TXDEFERRED = %08X\n", &pEmacRegs->EMAC_TXDEFERRED );
	ETHER_DEBUG ( "EMAC_TXLATECOL = %08X\n", &pEmacRegs->EMAC_TXLATECOL );
	ETHER_DEBUG ( "EMAC_TXEXCESSCOL = %08X\n", &pEmacRegs->EMAC_TXEXCESSCOL );
	ETHER_DEBUG ( "EMAC_TXCARR_ERR = %08X\n", &pEmacRegs->EMAC_TXCARR_ERR );
	ETHER_DEBUG ( "EMAC_TXOCTCNT_G = %08X\n", &pEmacRegs->EMAC_TXOCTCNT_G );
	ETHER_DEBUG ( "EMAC_TXFRMCNT_G = %08X\n", &pEmacRegs->EMAC_TXFRMCNT_G );
	ETHER_DEBUG ( "EMAC_TXEXCESSDEF = %08X\n", &pEmacRegs->EMAC_TXEXCESSDEF );
	ETHER_DEBUG ( "EMAC_TXPAUSEFRM = %08X\n", &pEmacRegs->EMAC_TXPAUSEFRM );
	ETHER_DEBUG ( "EMAC_TXVLANFRM_G = %08X\n", &pEmacRegs->EMAC_TXVLANFRM_G );
	ETHER_DEBUG ( "EMAC_RXFRMCNT_GB = %08X\n", &pEmacRegs->EMAC_RXFRMCNT_GB );
	ETHER_DEBUG ( "EMAC_RXOCTCNT_GB = %08X\n", &pEmacRegs->EMAC_RXOCTCNT_GB );
	ETHER_DEBUG ( "EMAC_RXOCTCNT_G = %08X\n", &pEmacRegs->EMAC_RXOCTCNT_G );
	ETHER_DEBUG ( "EMAC_RXBCASTFRM_G = %08X\n", &pEmacRegs->EMAC_RXBCASTFRM_G );
	ETHER_DEBUG ( "EMAC_RXMCASTFRM_G = %08X\n", &pEmacRegs->EMAC_RXMCASTFRM_G );
	ETHER_DEBUG ( "EMAC_RXCRC_ERR = %08X\n", &pEmacRegs->EMAC_RXCRC_ERR );
	ETHER_DEBUG ( "EMAC_RXALIGN_ERR = %08X\n", &pEmacRegs->EMAC_RXALIGN_ERR );
	ETHER_DEBUG ( "EMAC_RXRUNT_ERR = %08X\n", &pEmacRegs->EMAC_RXRUNT_ERR );
	ETHER_DEBUG ( "EMAC_RXJAB_ERR = %08X\n", &pEmacRegs->EMAC_RXJAB_ERR );
	ETHER_DEBUG ( "EMAC_RXUSIZE_G = %08X\n", &pEmacRegs->EMAC_RXUSIZE_G );
	ETHER_DEBUG ( "EMAC_RXOSIZE_G = %08X\n", &pEmacRegs->EMAC_RXOSIZE_G );
	ETHER_DEBUG ( "EMAC_RX64_GB = %08X\n", &pEmacRegs->EMAC_RX64_GB );
	ETHER_DEBUG ( "EMAC_RX65TO127_GB = %08X\n", &pEmacRegs->EMAC_RX65TO127_GB );
	ETHER_DEBUG ( "EMAC_RX128TO255_GB = %08X\n", &pEmacRegs->EMAC_RX128TO255_GB );
	ETHER_DEBUG ( "EMAC_RX256TO511_GB = %08X\n", &pEmacRegs->EMAC_RX256TO511_GB );
	ETHER_DEBUG ( "EMAC_RX512TO1023_GB = %08X\n", &pEmacRegs->EMAC_RX512TO1023_GB );
	ETHER_DEBUG ( "EMAC_RX1024TOMAX_GB = %08X\n", &pEmacRegs->EMAC_RX1024TOMAX_GB );
	ETHER_DEBUG ( "EMAC_RXUCASTFRM_G = %08X\n", &pEmacRegs->EMAC_RXUCASTFRM_G );
	ETHER_DEBUG ( "EMAC_RXLEN_ERR = %08X\n", &pEmacRegs->EMAC_RXLEN_ERR );
	ETHER_DEBUG ( "EMAC_RXOORTYPE = %08X\n", &pEmacRegs->EMAC_RXOORTYPE );
	ETHER_DEBUG ( "EMAC_RXPAUSEFRM = %08X\n", &pEmacRegs->EMAC_RXPAUSEFRM );
	ETHER_DEBUG ( "EMAC_RXFIFO_OVF = %08X\n", &pEmacRegs->EMAC_RXFIFO_OVF );
	ETHER_DEBUG ( "EMAC_RXVLANFRM_GB = %08X\n", &pEmacRegs->EMAC_RXVLANFRM_GB );
	ETHER_DEBUG ( "EMAC_RXWDOG_ERR = %08X\n", &pEmacRegs->EMAC_RXWDOG_ERR );
	ETHER_DEBUG ( "EMAC_IPC_RXIMSK = %08X\n", &pEmacRegs->EMAC_IPC_RXIMSK );
	ETHER_DEBUG ( "EMAC_IPC_RXINT = %08X\n", &pEmacRegs->EMAC_IPC_RXINT );
	ETHER_DEBUG ( "EMAC_RXIPV4_GD_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV4_GD_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV4_HDR_ERR_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV4_HDR_ERR_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV4_NOPAY_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV4_NOPAY_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV4_FRAG_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV4_FRAG_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV4_UDSBL_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV4_UDSBL_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV6_GD_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV6_GD_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV6_HDR_ERR_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV6_HDR_ERR_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV6_NOPAY_FRM = %08X\n", &pEmacRegs->EMAC_RXIPV6_NOPAY_FRM );
	ETHER_DEBUG ( "EMAC_RXUDP_GD_FRM = %08X\n", &pEmacRegs->EMAC_RXUDP_GD_FRM );
	ETHER_DEBUG ( "EMAC_RXUDP_ERR_FRM = %08X\n", &pEmacRegs->EMAC_RXUDP_ERR_FRM );
	ETHER_DEBUG ( "EMAC_RXTCP_GD_FRM = %08X\n", &pEmacRegs->EMAC_RXTCP_GD_FRM );
	ETHER_DEBUG ( "EMAC_RXTCP_ERR_FRM = %08X\n", &pEmacRegs->EMAC_RXTCP_ERR_FRM );
	ETHER_DEBUG ( "EMAC_RXICMP_GD_FRM = %08X\n", &pEmacRegs->EMAC_RXICMP_GD_FRM );
	ETHER_DEBUG ( "EMAC_RXICMP_ERR_FRM = %08X\n", &pEmacRegs->EMAC_RXICMP_ERR_FRM );
	ETHER_DEBUG ( "EMAC_RXIPV4_GD_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV4_GD_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV4_HDR_ERR_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV4_HDR_ERR_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV4_NOPAY_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV4_NOPAY_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV4_FRAG_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV4_FRAG_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV4_UDSBL_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV4_UDSBL_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV6_GD_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV6_GD_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV6_HDR_ERR_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV6_HDR_ERR_OCT );
	ETHER_DEBUG ( "EMAC_RXIPV6_NOPAY_OCT = %08X\n", &pEmacRegs->EMAC_RXIPV6_NOPAY_OCT );
	ETHER_DEBUG ( "EMAC_RXUDP_GD_OCT = %08X\n", &pEmacRegs->EMAC_RXUDP_GD_OCT );
	ETHER_DEBUG ( "EMAC_RXUDP_ERR_OCT = %08X\n", &pEmacRegs->EMAC_RXUDP_ERR_OCT );
	ETHER_DEBUG ( "EMAC_RXTCP_GD_OCT = %08X\n", &pEmacRegs->EMAC_RXTCP_GD_OCT );
	ETHER_DEBUG ( "EMAC_RXTCP_ERR_OCT = %08X\n", &pEmacRegs->EMAC_RXTCP_ERR_OCT );
	ETHER_DEBUG ( "EMAC_RXICMP_GD_OCT = %08X\n", &pEmacRegs->EMAC_RXICMP_GD_OCT );
	ETHER_DEBUG ( "EMAC_RXICMP_ERR_OCT = %08X\n", &pEmacRegs->EMAC_RXICMP_ERR_OCT );
	ETHER_DEBUG ( "EMAC_TM_CTL = %08X\n", &pEmacRegs->EMAC_TM_CTL );
	ETHER_DEBUG ( "EMAC_TM_SUBSEC = %08X\n", &pEmacRegs->EMAC_TM_SUBSEC );
	ETHER_DEBUG ( "EMAC_TM_SEC = %08X\n", &pEmacRegs->EMAC_TM_SEC );
	ETHER_DEBUG ( "EMAC_TM_NSEC = %08X\n", &pEmacRegs->EMAC_TM_NSEC );
	ETHER_DEBUG ( "EMAC_TM_SECUPDT = %08X\n", &pEmacRegs->EMAC_TM_SECUPDT );
	ETHER_DEBUG ( "EMAC_TM_NSECUPDT = %08X\n", &pEmacRegs->EMAC_TM_NSECUPDT );
	ETHER_DEBUG ( "EMAC_TM_ADDEND = %08X\n", &pEmacRegs->EMAC_TM_ADDEND );
	ETHER_DEBUG ( "EMAC_TM_TGTM = %08X\n", &pEmacRegs->EMAC_TM_TGTM );
	ETHER_DEBUG ( "EMAC_TM_NTGTM = %08X\n", &pEmacRegs->EMAC_TM_NTGTM );
	ETHER_DEBUG ( "EMAC_TM_HISEC = %08X\n", &pEmacRegs->EMAC_TM_HISEC );
	ETHER_DEBUG ( "EMAC_TM_STMPSTAT = %08X\n", &pEmacRegs->EMAC_TM_STMPSTAT );
	ETHER_DEBUG ( "EMAC_TM_PPSCTL = %08X\n", &pEmacRegs->EMAC_TM_PPSCTL );
	ETHER_DEBUG ( "EMAC_TM_AUXSTMP_NSEC = %08X\n", &pEmacRegs->EMAC_TM_AUXSTMP_NSEC );
	ETHER_DEBUG ( "EMAC_TM_AUXSTMP_SEC = %08X\n", &pEmacRegs->EMAC_TM_AUXSTMP_SEC );
	ETHER_DEBUG ( "EMAC_DMA_BUSMODE = %08X\n", &pEmacRegs->EMAC_DMA_BUSMODE );
	ETHER_DEBUG ( "EMAC_DMA_TXPOLL = %08X\n", &pEmacRegs->EMAC_DMA_TXPOLL );
	ETHER_DEBUG ( "EMAC_DMA_RXPOLL = %08X\n", &pEmacRegs->EMAC_DMA_RXPOLL );
	ETHER_DEBUG ( "EMAC_DMA_RXDSC_ADDR = %08X\n", &pEmacRegs->EMAC_DMA_RXDSC_ADDR );
	ETHER_DEBUG ( "EMAC_DMA_TXDSC_ADDR = %08X\n", &pEmacRegs->EMAC_DMA_TXDSC_ADDR );
	ETHER_DEBUG ( "EMAC_DMA_STAT = %08X\n", &pEmacRegs->EMAC_DMA_STAT );
	ETHER_DEBUG ( "EMAC_DMA_OPMODE = %08X\n", &pEmacRegs->EMAC_DMA_OPMODE );
	ETHER_DEBUG ( "EMAC_DMA_IEN = %08X\n", &pEmacRegs->EMAC_DMA_IEN );
	ETHER_DEBUG ( "EMAC_DMA_MISS_FRM = %08X\n", &pEmacRegs->EMAC_DMA_MISS_FRM );
	ETHER_DEBUG ( "EMAC_DMA_RXIWDOG = %08X\n", &pEmacRegs->EMAC_DMA_RXIWDOG );
	ETHER_DEBUG ( "EMAC_DMA_BMMODE = %08X\n", &pEmacRegs->EMAC_DMA_BMMODE );
	ETHER_DEBUG ( "EMAC_DMA_BMSTAT = %08X\n", &pEmacRegs->EMAC_DMA_BMSTAT );
	ETHER_DEBUG ( "EMAC_DMA_TXDSC_CUR = %08X\n", &pEmacRegs->EMAC_DMA_TXDSC_CUR );
	ETHER_DEBUG ( "EMAC_DMA_RXDSC_CUR = %08X\n", &pEmacRegs->EMAC_DMA_RXDSC_CUR );
	ETHER_DEBUG ( "EMAC_DMA_TXBUF_CUR = %08X\n", &pEmacRegs->EMAC_DMA_TXBUF_CUR );
	ETHER_DEBUG ( "EMAC_DMA_RXBUF_CUR = %08X\n", &pEmacRegs->EMAC_DMA_RXBUF_CUR );
	ETHER_DEBUG ( "EMAC_HWFEAT = %08X\n", &pEmacRegs->EMAC_HWFEAT );
}
#endif

/* The Entry Point Structure to the GEMAC Driver */
ADI_ETHER_DRIVER_ENTRY  GEMAC0DriverEntry =
{
	adi_ether_GemacOpen,
	adi_ether_GemacRead,
	adi_ether_GemacWrite,
	adi_ether_GemacClose,
	adi_ether_GemacGetLinkStatus,
	adi_ether_GemacAddMulticastFilter,
	adi_ether_GemacDelMulticastFilter,
	adi_ether_GemacGetBufferPrefix,
	adi_ether_GemacGetMACAddress,
	adi_ether_GemacSetMACAddress,
	adi_ether_GemacEnableMAC
};

ADI_ETHER_DRIVER_ENTRY  GEMAC1DriverEntry =
{
	adi_ether_GemacOpen,
	adi_ether_GemacRead,
	adi_ether_GemacWrite,
	adi_ether_GemacClose,
	adi_ether_GemacGetLinkStatus,
	adi_ether_GemacAddMulticastFilter,
	adi_ether_GemacDelMulticastFilter,
	adi_ether_GemacGetBufferPrefix,
	adi_ether_GemacGetMACAddress,
	adi_ether_GemacSetMACAddress,
	adi_ether_GemacEnableMAC
};

/*@}*/
