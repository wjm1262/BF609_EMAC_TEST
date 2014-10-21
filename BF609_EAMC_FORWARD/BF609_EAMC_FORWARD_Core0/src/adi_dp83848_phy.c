/*!
*********************************************************************************
 *
 * @file:    adi_dp83848_phy.c
 *
 * @brief:   DP83848VYB phy driver source
 *
 * @version: $Revision: 10617 $
 *
 * @date:    $Date: 2012-07-26 14:20:43 -0400 (Thu, 26 Jul 2012) $
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
#include <cdefbf609.h>
#include "adi_dp83848_phy_int.h"
#include <stdlib.h>
#include <cycle_count_bf.h>

/** \defgroup PHY Driver DP83848VYB PHY Driver
 *  @{
 */

/**
 * @brief       Wait until busy bit is cleared
 * 
 * @param [in]  phDevice    Device driver handle
 *
 * @return      void
 */
static int32_t poll_mii_done(ADI_ETHER_HANDLE *phDevice)
{
    ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)phDevice;
    ADI_EMAC_REGISTERS* const  pEmacRegs = pDev->pEMAC_REGS;

    int32_t  loop_count = PHY_LOOP_COUNT;

    /* poll for the GMII busy bit */
    do 
    {
       if (!(pEmacRegs->EMAC_GMII_ADDR &  BITM_EMAC_SMI_ADDR_SMIB) )
               break;
    } while (--loop_count > 0);

    return ( (loop_count == 0) ? -1 : 0);
}

/**
 * @brief       Writes data to the phy register without checking the busy state
 *
 * @param [in]  phDevice     Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * @param [in]  RegAddr      Register Address in the PHY
 *
 * @param [in]  Data         Data to be written
 *
 * @return      void
 *
 */
static uint32_t raw_phy_write(ADI_ETHER_HANDLE* const phDevice, 
                          const uint16_t PhyAddr,
                          const uint16_t RegAddr,
                          const uint32_t Data)
{
    ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)phDevice;
    ADI_EMAC_REGISTERS* const  pEmacRegs = pDev->pEMAC_REGS;
    uint32_t result;

    pEmacRegs->EMAC_GMII_DATA = Data;

    pEmacRegs->EMAC_GMII_ADDR = SET_PHYAD(PhyAddr)      | 
                                SET_REGAD(RegAddr)      | 
                                BITM_EMAC_SMI_ADDR_SMIB | 
                                BITM_EMAC_SMI_ADDR_SMIW | 
                                SET_CR(pDev->MdcClk);
    result = poll_mii_done(phDevice);
    return result;
}

/**
 * @brief       Reads data from a PHY register
 *
 * @param [in]  phDevice     Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * @param [in]  RegAddr      Register Address in the PHY
 *
 * @return      16-bit register data
 *
 */
uint16_t dp83848_phy_read(ADI_ETHER_HANDLE* const phDevice,const uint16_t PhyAddr, 
                          const uint16_t RegAddr)
{
    ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)phDevice;
    ADI_EMAC_REGISTERS* const  pEmacRegs = pDev->pEMAC_REGS;

    poll_mii_done(phDevice);

    pEmacRegs->EMAC_GMII_ADDR = SET_PHYAD(PhyAddr)     | 
                                SET_REGAD(RegAddr)     |  
                                BITM_EMAC_SMI_ADDR_SMIB | 
                                SET_CR(pDev->MdcClk);

    poll_mii_done(phDevice);
    return (uint16_t)pEmacRegs->EMAC_GMII_DATA;
}

/**
 * @brief       Writes data to the phy register after checking the busy state
 *
 * @param [in]  phDevice     Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * @param [in]  RegAddr      Register Address in the PHY
 *
 * @param [in]  Data         Data to be written
 *
 * @return      void
 *
 */
uint32_t dp83848_phy_write(ADI_ETHER_HANDLE *phDevice,
                       const uint16_t PhyAddr, 
                       const uint16_t RegAddr, 
                       const uint32_t Data)
{
    ADI_EMAC_DEVICE*    const  pDev      = (ADI_EMAC_DEVICE*)phDevice;
    ADI_EMAC_REGISTERS* const  pEmacRegs = pDev->pEMAC_REGS;
    uint32_t result;

    poll_mii_done(phDevice);

    raw_phy_write(phDevice,PhyAddr,RegAddr,Data);

    pEmacRegs->EMAC_GMII_DATA = Data;

    pEmacRegs->EMAC_GMII_ADDR = SET_PHYAD(PhyAddr)     | 
                                SET_REGAD(RegAddr)     |
                                BITM_EMAC_SMI_ADDR_SMIB | 
                                BITM_EMAC_SMI_ADDR_SMIW | 
                                SET_CR(pDev->MdcClk);
    result = poll_mii_done(phDevice);

    return result;
}


/**
 * @brief     Waits for a milli second
 */
void wait_millisec(void)
{
volatile unsigned long long int cur,nd;

    _GET_CYCLE_COUNT(cur);

    nd = cur + (__PROCESSOR_SPEED__/1000);
    while (cur < nd) {
    _GET_CYCLE_COUNT(cur);
    }
}
/**
 * @brief     Sleeps for specified number of milli-seconds
 */
void sleep(uint32_t msec)
{
    while (msec != 0) {
           wait_millisec();
           msec--;
    }
    return;
}


/**
 * @brief       Initializes the PHY 
 *
 * @param [in]  phDevice     EMAC device handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 */
uint32_t dp83848_phy_init(ADI_ETHER_HANDLE* const phDevice, const uint16_t PhyAddr)
{
    uint16_t phy_data,phy_id1,phy_id2;
    ADI_EMAC_DEVICE *pDev = (ADI_EMAC_DEVICE*)phDevice;
    uint32_t result =0;

    /* reset the phy */     
    raw_phy_write(phDevice,PhyAddr, REG_PHY_MODECTL, PHY_MODECTL_RESET);

    /* wait 1 second, DP83848 spec recommends 3us wait time after reset */
    sleep(1000);

    /* read mode control register */
    phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_MODECTL);

    if (phy_data & PHY_MODECTL_RESET)
    {
        ETHER_DEBUG("Phy reset failed\n",phy_data);
        return (ADI_ETHER_RESULT_PHYINIT_FAILED);
    }

    /* read phy identifier registers */
    phy_id1 = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_PHYID1);

    phy_id2 = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_PHYID2);

    /* advertise flow control supported */
    phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_ANAR);

    phy_data |= PHY_ANAR_PAUSE_OPERATION;

    dp83848_phy_write(phDevice,PhyAddr,REG_PHY_ANAR, phy_data );
    phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_ANAR);

    phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_MODECTL);
    phy_data &= ~PHY_MODECTL_AUTO_NEGO_ENA;
    dp83848_phy_write(phDevice,PhyAddr,REG_PHY_MODECTL, phy_data );

    phy_data  = 0;

    /* loopback mode - enable rx <-> tx loopback */
    if(pDev->LoopBackMode)
        phy_data |= PHY_MODECTL_LOOPBACK;

    /* check if auto-negotiation is enabled */
    if (pDev->AutoNegotiate)
    {
        phy_data |= PHY_MODECTL_AUTO_NEGO_ENA | PHY_MODECTL_RESTART_A_NEGO;
    }
    else /* configure as specified by the driver */
    {
        /* configure duplex mode */
        if (pDev->FullDuplex)
            phy_data |= PHY_MODECTL_DUPLEX_MODE;
        else
            phy_data &= ~PHY_MODECTL_DUPLEX_MODE;

        /* configure port speed  */
        if (pDev->Port10)
            phy_data &= ~PHY_MODECTL_SPEED_SELECT; 
        else
            phy_data |= PHY_MODECTL_SPEED_SELECT;
    }

    /* acknowledge any latched interrupts */
    dp83848_ack_phyint(phDevice,PhyAddr);

    dp83848_phy_write(phDevice,PhyAddr, REG_PHY_MICR, PHY_MICR_INTEN | PHY_MICR_INT_OE);

    /* enable phy interrupts  */
    dp83848_phy_write(phDevice,PhyAddr, REG_PHY_MISR, 
                                        PHY_MISR_LINKINT_EN |  /* link established */
                                        PHY_MISR_ANCINT_EN);   /* auto-negotiation completed */
    /* start the auto-negotiation */
    dp83848_phy_write(phDevice,PhyAddr, REG_PHY_MODECTL, phy_data);
    /*
     * By default PHY interrupt is active. By enabling ADI_ETHER_POLLING this routine
     * will not return until auto-negotiation is complete. So if a cable is not plugged-in
     * the code may block infinitely.
     */
    //sleep(3000);
#if defined(ADI_ETHER_POLLING)
    /* read the mode status register */
    do {
       phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_MODESTAT);
    } while(!(phy_data & 0x0020));

    phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_CR);

    /* clear  pending PHY interrupts */
    phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_MISR);
    phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_MICR);
#endif /* ADI_ETHER_POLLING */

    return(result);
}
/**
 * @brief       Acknowledge phy interrupt
 *
 * @param [in]  phDevice    Device driver handle
 *
 * @param [in]  PhyAddr     Physical address of the ethernet PHY
 *
 * FIXIT: use the GPIO/INT service
 *
 */
void dp83848_ack_phyint(ADI_ETHER_HANDLE* const phDevice, const uint16_t PhyAddr)
{
    uint16_t phy_data;
    phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_MISR);
    phy_data = dp83848_phy_read(phDevice,PhyAddr, REG_PHY_MICR);

    *pREG_PINT2_REQ      = BITM_PINT_REQ_PIQ6 ;
}

/**
 * @brief       Power down PHY
 *
 * @param [in]  phDevice    Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * @param [in]  bPowerDown   If true power down the PHY else power up the PHY
 *
 * @return      true if operation is successful
 * 
 */
uint32_t dp83848_phy_pwrdown(ADI_ETHER_HANDLE* const phDevice,
                         const uint16_t          PhyAddr, 
                         const bool              bPowerDown)
{
    uint16_t phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_MODECTL);
    uint32_t result;

    if(bPowerDown) 
        phy_data |= PHY_MODECTL_POWER_DOWN;
    else
        phy_data &= ~PHY_MODECTL_POWER_DOWN;

    result = dp83848_phy_write(phDevice,PhyAddr,REG_PHY_MODECTL,phy_data);

    return result;
}

/**
 * @brief       Returns PHY powerdown status
 *
 * @param [in]  phDevice    Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * return       True if phy is in power down state. False if it is powered.
 */
bool    dp83848_phy_get_pwrdown(ADI_ETHER_HANDLE* const phDevice,const uint16_t PhyAddr)
{
    uint16_t phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_MODECTL);

    return(! (phy_data & PHY_MODECTL_POWER_DOWN));
}

/**
 * @brief       Get the PHY status
 *
 * @param [in]  phDevice    Device driver handle
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * return       Status
 */
uint32_t    dp83848_phy_get_status(ADI_ETHER_HANDLE * const phDevice, const uint16_t PhyAddr)
{
    uint16_t phy_data = dp83848_phy_read(phDevice,PhyAddr,REG_PHY_STS);
    uint32_t status = 0;

    if (phy_data & PHY_STS_LINK_STATUS)
          status |= (ADI_ETHER_PHY_LINK_UP);

    if ( phy_data & PHY_STS_SPEED_STATUS )
    {
        if (phy_data & PHY_STS_DUPLEX_STATUS) 
            status |= ADI_ETHER_PHY_10T_FULL_DUPLEX;
        else
            status |= ADI_ETHER_PHY_10T_HALF_DUPLEX;
    }
    else
    {
        if (phy_data & PHY_STS_DUPLEX_STATUS) 
            status |= ADI_ETHER_PHY_100T_FULL_DUPLEX;
        else
            status |= ADI_ETHER_PHY_100T_HALF_DUPLEX;
    }
            
    if (phy_data & PHY_STS_AUTO_NEG_COMPLETE)
          status |= (ADI_ETHER_PHY_AN_COMPLETE);

    if (phy_data & PHY_STS_LOOPBACK_STATUS)
          status |= (ADI_ETHER_PHY_LOOPBACK);

    return status;
}


#ifdef ADI_DEBUG
#include <stdio.h>
/**
 * @brief       Returns all PHY register contents
 *
 * @param [in]  PhyAddr      Physical Address of the PHY
 *
 * @param [out] pRegister    Constant pointer to the memory where register
 *                           values will be saved
 * 
 * @param [in]  numRegs      Number of registers
 *
 * @return      void  // FIXIT
 *
 * @note        valid only in debug build
 *
 */
void  dp83848_phy_get_regs(ADI_ETHER_HANDLE* const phDevice,
                           const uint16_t PhyAddr,
                           int32_t* const pRegisters, 
                           int32_t numRegs)
{
    int32_t register_indx;

    for(register_indx = 0; register_indx < NUM_PHY_REGS; register_indx++ )
    {
        pRegisters[register_indx] = dp83848_phy_read(phDevice,PhyAddr,register_indx);
    }  
}

#endif /* ADI_DEBUG */

/*@}*/
