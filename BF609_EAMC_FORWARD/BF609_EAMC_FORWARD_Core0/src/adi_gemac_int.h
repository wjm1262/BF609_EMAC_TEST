/*!
*********************************************************************************
 *
 * @file:    adi_gemac_int.h
 *
 * @brief:   Ethernet GEMAC driver internal header file
 *
 * @version: $Revision: 13222 $
 *
 * @date:    $Date: 2013-02-28 11:50:31 -0500 (Thu, 28 Feb 2013) $
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
#ifndef _ADI_GEMAC_INT_H_
#define _ADI_GEMAC_INT_H_
#include <stdlib.h>
#include <adi_types.h>
#include <drivers/ethernet/adi_ether.h>
#include <drivers/ethernet/adi_ether_gemac.h>
#include <drivers/ethernet/adi_ether_misra.h>
#include <string.h>

/*! Statistical collection is enabled if ADI_ETHER_DEBUG macro is defined */
#if defined(ADI_ETHER_DEBUG)
#include <stdio.h>
#define ETHER_DEBUG(str,val)  (printf((str),(val))) /*!< Enable debug prints */
#define ADI_ETHER_STATISTICS            /*!< enable statistics             */
#define STATS_INC(x) (x++)              /*!< increment statistical counters */
#else
#define ETHER_DEBUG(str,val)            /*!< disable debug prints */
#define STATS_INC(x)                    /*!< disable statistic collection */
#endif  /* ADI_ETHER_DEBUG */

#define ADI_EMAC0_BASE_ADDRESS (0xFFC20000)     /*!< EMAC0 Base address for BF560x */
#define ADI_EMAC1_BASE_ADDRESS (0xFFC22000)     /*!< EMAC1 Base address for BF560x */

/*! EMAC DMA thershold above which descriptors get added to live active dma queue */
#define ADI_EMAC_THERSHOLD         (3)
#define ADI_EMAC_MIN_DMEM_SZ       (4 *32)  /*!< minimum number of dma descriptors per channel */
#define ADI_EMAC_DMAOWN            (1UL << 31) /*!< Set when DMA owns the descriptor */
#define ADI_EMAC_NUM_MCAST_BINS    (64)   /*!< number of multicast bins supported by controller */

#define MAC_LOOPCOUNT    1000000          /*!< counter used as timeout for MAC operations */

#if defined(ADI_ETHER_STATISTICS)
/*! EMAC statistics */
typedef struct ADI_EMAC_STATS
{
	int32_t TxIntCnt;
	int32_t TxProcessStopCnt;
	int32_t TxBufferUnAvailCnt;
	int32_t TxJabberTimeOutCnt;
	int32_t RxOvfCnt;
	int32_t TxUnfCnt;
	int32_t RxIntCnt;
	int32_t RxBufferUnAvailCnt;
	int32_t RxProcessStopCnt;
	int32_t RxWDTimeoutCnt;
	int32_t EarlyTxIntCnt;
	int32_t BusErrorCnt;
	int32_t EarlyRxIntCnt;
	int32_t MMCIntCnt;
	
} ADI_EMAC_STATS;
#endif  /* ADI_ETHER_STATISTICS */

/*! EMAC Register map including the padding */
typedef volatile struct ADI_EMAC_REGISTERS
{
	uint32_t EMAC_MACCFG;                 /*!< mac configuration register */
	uint32_t EMAC_MACFRMFILT;             /*!< mac frame filter register  */
	uint32_t EMAC_HASHTBL_HI;             /*!< hash table register low    */
	uint32_t EMAC_HASHTBL_LO;             /*!< has table register high    */
	uint32_t EMAC_GMII_ADDR;              /*!< MII address register       */
	uint32_t EMAC_GMII_DATA;              /*!< MII data register          */
	uint32_t EMAC_FLOWCTL;                /*!< flow control register      */
	uint32_t EMAC_VLANTAG;                /*!< vlan tag register          */
	uint32_t EMAC_VER;                    /*!< version register           */
	uint32_t EMAC_DBG;                    /*!< debug register             */
	uint32_t EMAC_RMTWKUP;                /*!< remote wakeup register     */
	uint32_t EMAC_PMT_CTLSTAT;            /*!< PMT control status register*/
	uint8_t  PAD1[8];                     /*!< pad 1 */
	uint32_t EMAC_ISTAT;                  /*!< mac interrupt status       */
	uint32_t EMAC_IMSK;                   /*!< mac imask                  */
	uint32_t EMAC_ADDR0_HI;               /*!< MAC address low            */
	uint32_t EMAC_ADDR0_LO;               /*!< MAC address high           */
	uint8_t  PAD2[184];                   /*!< pad 2 */
	uint32_t EMAC_MMC_CTL;                /*!< mmc control register       */
	uint32_t EMAC_MMC_RXINT;              /*!< mmc rx interrupt           */
	uint32_t EMAC_MMC_TXINT;              /*!< mmc tx interrupt           */
	uint32_t EMAC_MMC_RXIMSK;             /*!< rx imask register          */
	uint32_t EMAC_MMC_TXIMSK;             /*!< tx imask register          */
	uint32_t EMAC_TXOCTCNT_GB;            /*!< no. tx bytes good and bad frames */
	uint32_t EMAC_TXFRMCNT_GB;            /*!< no. tx frames transmitted  */
	uint32_t EMAC_TXBCASTFRM_G;           /*!< no. of tx broadcast frames */
	uint32_t EMAC_TXMCASTFRM_G;           /*!< no. of tx multicast frames */
	uint32_t EMAC_TX64_GB;                /*!< no. of tx frames with length = 64 */
	uint32_t EMAC_TX65TO127_GB;           /*!< no. of tx frames with length between 65 and 127 */
	uint32_t EMAC_TX128TO255_GB;          /*!< no. of tx frames with length between 128 and 255 */
	uint32_t EMAC_TX256TO511_GB;          /*!< no. of tx frames with length between 256 and 512 */
	uint32_t EMAC_TX512TO1023_GB;         /*!< no. of tx frames with length between 512 and 1023 */
	uint32_t EMAC_TX1024TOMAX_GB;         /*!< no. of tx frames with length between 1024 and max */
	uint32_t EMAC_TXUCASTFRM_GB;          /*!< no. of tx unicast frames */
	uint32_t EMAC_TXMCASTFRM_GB;          /*!< no. of tx multicast frames */
	uint32_t EMAC_TXBCASTFRM_GB;          /*!< no. of tx broadcast frames */
	uint32_t EMAC_TXUNDR_ERR;             /*!< no. of tx frames with under runs  */
	uint32_t EMAC_TXSNGCOL_G;             /*!< no. of tx frames with single collision */
	uint32_t EMAC_TXMULTCOL_G;            /*!< no. of tx frames with multiple collisions */
	uint32_t EMAC_TXDEFERRED;             /*!< no. of deferred tx frames */
	uint32_t EMAC_TXLATECOL;              /*!< no. of tx frames with late collision */
	uint32_t EMAC_TXEXCESSCOL;            /*!< no. of tx frames with excessive collision */
	uint32_t EMAC_TXCARR_ERR;             /*!< no. of tx frames with carrier error */
	uint32_t EMAC_TXOCTCNT_G;             /*!< tx octet count - good frames only */
	uint32_t EMAC_TXFRMCNT_G;             /*!< tx frame count - good frames only */
	uint32_t EMAC_TXEXCESSDEF;            /*!< no. of tx frames aborted due to excess collision */
	uint32_t EMAC_TXPAUSEFRM;             /*!< no. of good pause frames */
	uint32_t EMAC_TXVLANFRM_G;            /*!< no. of tx virtual lan frames */
	uint8_t  PAD3[8];                     /*!< pad 3 */
	uint32_t EMAC_RXFRMCNT_GB;            /*!< no. of rx frames */
	uint32_t EMAC_RXOCTCNT_GB;            /*!< no. of rx octets good and bad frames */
	uint32_t EMAC_RXOCTCNT_G;             /*!< no. of rx octets only good frames */
	uint32_t EMAC_RXBCASTFRM_G;           /*!< no. of rx good broadcast frames */
	uint32_t EMAC_RXMCASTFRM_G;           /*!< no. of rx good multicast frames */
	uint32_t EMAC_RXCRC_ERR;              /*!< no. of rx frames with crc errors */
	uint32_t EMAC_RXALIGN_ERR;            /*!< no. of rx frames with alignment errors */
	uint32_t EMAC_RXRUNT_ERR;             /*!< no. of rx frame with runt(<64bytes & crc) error */
	uint32_t EMAC_RXJAB_ERR;              /*!< no. of rx frames with jabber error */
	uint32_t EMAC_RXUSIZE_G;              /*!< no. of rx frames with length < 64 & no errors */
	uint32_t EMAC_RXOSIZE_G;              /*!< no. of rx frames with > 1518 bytes without errors */
	uint32_t EMAC_RX64_GB;                /*!< no. of rx frames with 64 byte length */
	uint32_t EMAC_RX65TO127_GB;           /*!< rx frames with length between 65 and 127 */
	uint32_t EMAC_RX128TO255_GB;          /*!< rx frames with length between 128 and 255 */
	uint32_t EMAC_RX256TO511_GB;          /*!< rx frames with length between 256 and 511 */
	uint32_t EMAC_RX512TO1023_GB;         /*!< rx frames with length between 512 and 1023 */
	uint32_t EMAC_RX1024TOMAX_GB;         /*!< rx frames with length between 1024 and max */
	uint32_t EMAC_RXUCASTFRM_G;           /*!< no. of rx unicast frames */
	uint32_t EMAC_RXLEN_ERR;              /*!< no. of rx frames with length errors */
	uint32_t EMAC_RXOORTYPE;              /*!< no. of rx frames with invalid frame size */
	uint32_t EMAC_RXPAUSEFRM;             /*!< no. of received pause frames */
	uint32_t EMAC_RXFIFO_OVF;             /*!< no. of missed received frames due to FIFO overflow*/
	uint32_t EMAC_RXVLANFRM_GB;           /*!< no. of good and bad vlan frames */
	uint32_t EMAC_RXWDOG_ERR;             /*!< no. of frames received with watchdog timeout */
	uint8_t  PAD4[32];                    /*!< pad 4 */
	uint32_t EMAC_IPC_RXIMSK;             /*!< IPC receive interrupt mask */
	uint8_t  PAD5[4];                     /*!< pad 5 */
	uint32_t EMAC_IPC_RXINT;              /*!< IPC receive interrupt */
	uint8_t  PAD6[4];                     /*!< pad 6 */
	uint32_t EMAC_RXIPV4_GD_FRM;          /*!< good ipv4 frames */
	uint32_t EMAC_RXIPV4_HDR_ERR_FRM;     /*!< ipv4 frames with header or version errors */
	uint32_t EMAC_RXIPV4_NOPAY_FRM;       /*!< ipv4 frames with no payload */
	uint32_t EMAC_RXIPV4_FRAG_FRM;        /*!< ipv4 frames with fragmentation */
	uint32_t EMAC_RXIPV4_UDSBL_FRM;       /*!< good ipv4 frames with disabled checksum for udp */
	uint32_t EMAC_RXIPV6_GD_FRM;          /*!< no. of ipv6 frames */
	uint32_t EMAC_RXIPV6_HDR_ERR_FRM;     /*!< ipv6 frames with header errors */
	uint32_t EMAC_RXIPV6_NOPAY_FRM;       /*!< ipv6 frames with no payload */
	uint32_t EMAC_RXUDP_GD_FRM;           /*!< ip datagrams with good udp payload */
	uint32_t EMAC_RXUDP_ERR_FRM;          /*!< good ip datagrams with udp checksum errors */
	uint32_t EMAC_RXTCP_GD_FRM;           /*!< ip datagrams with tcp payload */
	uint32_t EMAC_RXTCP_ERR_FRM;          /*!< ip datagrames with tcp payload checksum errors */
	uint32_t EMAC_RXICMP_GD_FRM;          /*!< ip datagrams with  icmp payload */
	uint32_t EMAC_RXICMP_ERR_FRM;         /*!< ip datagrams with icmp checksum errors */
	uint8_t  PAD7[8];                     /*!< pad 7 */
	uint32_t EMAC_RXIPV4_GD_OCT;          /*!< ipv4 octets */
	uint32_t EMAC_RXIPV4_HDR_ERR_OCT;     /*!< ipv4 frames with header errors */
	uint32_t EMAC_RXIPV4_NOPAY_OCT;       /*!< ipv4 frames with no payload */
	uint32_t EMAC_RXIPV4_FRAG_OCT;        /*!< ipv4 frames with fragmentation */
	uint32_t EMAC_RXIPV4_UDSBL_OCT;       /*!< ipv4 udp segments with checksum disabled */
	uint32_t EMAC_RXIPV6_GD_OCT;          /*!< ipv6 good frames */
	uint32_t EMAC_RXIPV6_HDR_ERR_OCT;     /*!< ipv6 with header errors */
	uint32_t EMAC_RXIPV6_NOPAY_OCT;       /*!< ipv6 frames with no payload */
	uint32_t EMAC_RXUDP_GD_OCT;           /*!< good udp frames */
	uint32_t EMAC_RXUDP_ERR_OCT;          /*!< udp frames with errors */
	uint32_t EMAC_RXTCP_GD_OCT;           /*!< good tcp frames */
	uint32_t EMAC_RXTCP_ERR_OCT;          /*!< tcp frames with errors */
	uint32_t EMAC_RXICMP_GD_OCT;          /*!< good icmp frames */
	uint32_t EMAC_RXICMP_ERR_OCT;         /*!< icmp frames with errors */
	uint8_t  PAD8[1144];                  /*!< pad 8 */
	uint32_t EMAC_TM_CTL;                 /*!< timestamp control register */
	uint32_t EMAC_TM_SUBSEC;              /*!< timestamp subsecond register */
	uint32_t EMAC_TM_SEC;                 /*!< timestamp second register */
	uint32_t EMAC_TM_NSEC;                /*!< timestamp nano-second register */
	uint32_t EMAC_TM_SECUPDT;             /*!< timestamp second update register */
	uint32_t EMAC_TM_NSECUPDT;            /*!< timestamp nano-second update register */
	uint32_t EMAC_TM_ADDEND;              /*!< timestamp addend register */
	uint32_t EMAC_TM_TGTM;                /*!< target time seconds register */
	uint32_t EMAC_TM_NTGTM;               /*!< target time nano-second register */
	uint32_t EMAC_TM_HISEC;               /*!< system time high register */
	uint32_t EMAC_TM_STMPSTAT;            /*!< timestamp status register */
	uint32_t EMAC_TM_PPSCTL;              /*!< timestamp pps control register */
	uint32_t EMAC_TM_AUXSTMP_NSEC;        /*!< auxiliary timestamp nanosecond register */
	uint32_t EMAC_TM_AUXSTMP_SEC;         /*!< auxiliary timestamp seconds register */
	uint8_t  PAD9[2248];                  /*!< pad 9 */
	uint32_t EMAC_DMA_BUSMODE;            /*!< emac dma busmode register */
	uint32_t EMAC_DMA_TXPOLL;             /*!< emac dma tx poll register */
	uint32_t EMAC_DMA_RXPOLL;             /*!< emac dma rx poll register */
	uint32_t EMAC_DMA_RXDSC_ADDR;         /*!< emac dma rx descriptor address register */
	uint32_t EMAC_DMA_TXDSC_ADDR;         /*!< emac dma tx descriptor address register */
	uint32_t EMAC_DMA_STAT;               /*!< emac dma status register */
	uint32_t EMAC_DMA_OPMODE;             /*!< emac dma opmode register */
	uint32_t EMAC_DMA_IEN;                /*!< emac dma interrupt enable register */
	uint32_t EMAC_DMA_MISS_FRM;           /*!< emad dma missed frame register */
	uint32_t EMAC_DMA_RXIWDOG;            /*!< dma watchdog register */
	uint32_t EMAC_DMA_BMMODE;             /*!< dma busmode register */
	uint32_t EMAC_DMA_BMSTAT;             /*!< dma busmode status register */
	uint8_t  PAD10[24];                   /*!< pad 10 */
	uint32_t EMAC_DMA_TXDSC_CUR;          /*!< dma current tx descriptor register */
	uint32_t EMAC_DMA_RXDSC_CUR;          /*!< dma current rx descriptor register */
	uint32_t EMAC_DMA_TXBUF_CUR;          /*!< dma current tx buffer register */
	uint32_t EMAC_DMA_RXBUF_CUR;          /*!< dma current rx buffer register */
	uint32_t EMAC_HWFEAT;                 /*!< hardware feature register */
	
} ADI_EMAC_REGISTERS;

/*! EMAC DMA Descriptor */
typedef struct ADI_EMAC_DMADESC
{
	uint32_t                   Status;         /*!< descriptor status       */
	uint32_t                   ControlDesc;    /*!< descriptor control bits */
	uint32_t                   StartAddr;      /*!< dma start address       */
	struct ADI_EMAC_DMADESC   *pNextDesc;      /*!< next descriptor pointer */
	uint32_t                   ExtendedStatus; /*!< extended status         */
	uint32_t                   Reserved;       /*!< reserved                */
	uint32_t                   RxTimeStampLo;  /*!< ieee-1588 timestamp low */
	uint32_t                   RxTimeStampHi;  /*!< ieee-1588 timestamp high */
	
} ADI_EMAC_DMADESC;

/*! Dma descriptor size */
#define ADI_EMAC_DMADESC_SIZE  (sizeof(ADI_EMAC_DMADESC))

/*! EMAC buffer information */
typedef struct ADI_EMAC_BUFINFO
{
	ADI_EMAC_DMADESC           *pDmaDesc;   /*!< pointer to DMA descriptor */
	
} ADI_EMAC_BUFINFO;


/*! Frame queue */
typedef struct ADI_EMAC_FRAME_Q
{
	ADI_ETHER_BUFFER  *pQueueHead;        /*!< frame queue head */
	ADI_ETHER_BUFFER  *pQueueTail;        /*!< frame queue tail */
	int32_t            ElementCount;      /*!< number of buffers in a queue */
	
} ADI_EMAC_FRAME_Q;

/*! Emac channel */
typedef struct ADI_EMAC_CHANNEL
{
	ADI_EMAC_FRAME_Q   Active;           /*!< active dma queue */
	ADI_EMAC_FRAME_Q   Queued;           /*!< queued dma queue - not bound with descriptor  */
	ADI_EMAC_FRAME_Q   Pending;          /*!< pending queue - buffers bound with descriptor */
	ADI_EMAC_FRAME_Q   Completed;        /*!< completed buffer queue */
	
	ADI_EMAC_DMADESC  *pDmaDescHead;     /*!< free dma descriptor head */
	ADI_EMAC_DMADESC  *pDmaDescTail;     /*!< free dma descriptor tail */
	int32_t           NumAvailDmaDesc;   /*!< number of available dma descriptors */
	bool              Recv;              /*!< receive channel identifier */
	
} ADI_EMAC_CHANNEL;


/*! EMAC device   */
typedef struct ADI_EMAC_DEVICE
{
	ADI_EMAC_REGISTERS  *pEMAC_REGS;   /*!< pointer to the EMAC registers */
	uint32_t            Version;       /*!< EMAC version information      */
	uint32_t            Interrupt;     /*!< EMAC interrupt                */
	uint8_t             MacAddress[6]; /*!< MAC address                   */
	bool                Started;       /*!< driver is started or not      */
	bool                Opened;        /*!< driver is opened or not       */
	uint32_t            MdcClk;        /*!< MDC clock to the PHY          */
	bool				TxTimeStamped; /* capture tx time stamp or not. by wjm@2014-8-14*/
	bool				RxTimeStamped; /* capture rx time stamp or not. by wjm@2014-8-14*/
	bool				AuxiTMEnabled; /* enabled the auxiliary time stamp. by wjm@2014-8-15 */
	
	/* PHY configuration */
	uint32_t            PhyAddress;    /*!< PHY address                   */
	bool                AutoNegotiate; /*!< AutoNegotiation enabled or not */
	bool                Port10;        /*!< 10Mbps port or 100Mbps port   */
	bool                FullDuplex;    /*!< full-duplex or half-duplex    */
	bool                LoopBackMode;  /*!< loopback mode on              */
	bool                Cache;         /*!< data cache on                 */
	uint32_t            MDCClockRange; /*!< MDC clock range               */
	
	/* Channels */
	ADI_EMAC_CHANNEL    Rx;            /*!< receive channel              */
	ADI_EMAC_CHANNEL    Tx;            /*!< transmit channel             */
	int32_t MulticastBinCount[ADI_EMAC_NUM_MCAST_BINS]; /*!< 64 bin hash entires */
	ADI_ETHER_CALLBACK_FN  pEtherCallback;  /*!< driver callback         */
	
#ifdef ADI_ETHER_STATISTICS
	ADI_EMAC_STATS     Stats;         /*!< statistics collection */
#endif
	
} ADI_EMAC_DEVICE;

/*! EMAC Dma Descriptor status */
typedef enum ADI_EMAC_DMADESC_STATUS
{
	ENUM_DS_DAFILT_FAIL = 0x40000000,
	ENUM_DS_FRMLEN_MASK = 0x3FFF0000,
	ENUM_DS_DESC_ERR    = 0x00008000,
	ENUM_DS_RX_TRUNCATED = 0x00004000,
	ENUM_DS_SAFILT_FAIL = 0x00002000,
	ENUM_DS_RXLEN_ERR   = 0x00001000,
	ENUM_DS_RXOVF_ERR   = 0x00000800,
	ENUM_DS_RXVLAN_TAG  = 0x00000400,
	ENUM_DS_RXFIRST_DESC = 0x00000200,
	ENUM_DS_RXLAST_DESC = 0x00000100,
	ENUM_DS_RXLONG_FRAME = 0x00000080,
	ENUM_DS_RXETH_FRAME = 0x00000020,
	ENUM_DS_RXWATCHDOG  = 0x00000010,
	ENUM_DS_RXMII_ERR   = 0x00000008,
	ENUM_DS_RXDRIBBLE   = 0x00000004,
	ENUM_DS_RXCRC_ERR   = 0x00000002,
	ENUM_DS_RXEXT_STAT  = 0x00000001,
	
	ENUM_DS_TXINT_ENA   = 0x40000000,
	ENUM_DS_TXLAST_DESC = 0x20000000,
	ENUM_DS_TXFIRST_DESC = 0x10000000,
	ENUM_DS_TXDIS_CRC   = 0x08000000,
	ENUM_DS_TXDIS_PAD   = 0x04000000,
	ENUM_DS_TXCSUM_MASK = 0x00C00000,
	ENUM_DS_TXCSUM_BYPASS = 0x00000000,
	ENUM_DS_TXCSUM_TCP  = 0x00800000,
	ENUM_DS_TXCSUM_IPV4   = 0x00400000,
	ENUM_DS_TXCSUM_PESUDO = 0x00C00000,
	ENUM_DS_TXENDOF_RING  = 0x00200000,
	ENUM_DS_TXDESC_CHAIN  = 0x00100000,
	ENUM_DS_TXIPHDR_ERR   = 0x00010000,
	ENUM_DS_TXJAB_TIMEOUT = 0x00004000,
	ENUM_DS_TXFRAME_FLUSH = 0x00002000,
	ENUM_DS_TXPAYLOAD_ERR = 0x00001000,
	ENUM_DS_TXLOST_CARRIER = 0x00000800,
	ENUM_DS_TXNO_CARRIER  = 0x00000400,
	ENUM_DS_TXLATE_COLL   = 0x00000200,
	ENUM_DS_TXEXS_COLL    = 0x00000100,
	ENUM_DS_TXVLAN_FRAME  = 0x00000080,
	ENUM_DS_TXCOLL_MASK   = 0x00000078,
	ENUM_DS_TXEXC_DEF     = 0x00000004,
	ENUM_DS_TX_UNDFLOW    = 0x00000002,
	ENUM_DS_TX_DEF        = 0x00000001
	
} ADI_EMAC_DMADESC_STATUS;


/***********************/
/* user extend define*/
typedef struct WJM_TIME_STAMP
{
	uint32_t        TimeStampLo;  /*!< ieee-1588 timestamp low */
	uint32_t       TimeStampHi;  /*!< ieee-1588 timestamp high */
} WJM_TIME_STAMP;

typedef struct RX_TX_TIME_STAMP_BUFFER
{
	WJM_TIME_STAMP    RxTimeStamp;  /*!< ieee-1588 timestamp low */
	WJM_TIME_STAMP    TxTimeStamp;  /*!< ieee-1588 timestamp high */
} RX_TX_TIME_STAMP_BUFFER;

typedef enum PPS_TYPE
{
	PULSE_SINGLE = 0x1, //0001
	PULSE_TRAIN = 0x2, //0010,start pulse train
}PPS_TYPE;
/***********************/

/*! Abnormal and Normal Interrupts */
#define ADI_EMAC_AIS_NIS_INTERRUPTS   ((1UL << BITP_EMAC_DMA_IEN_NIS)|  \
									   (1UL << BITP_EMAC_DMA_IEN_AIS) |  \
									   (1UL << BITP_EMAC_DMA_IEN_RU)  |  \
									   (1UL << BITP_EMAC_DMA_IEN_RI)  |  \
									   (1UL << BITP_EMAC_DMA_IEN_UNF) |  \
									   (1UL << BITP_EMAC_DMA_IEN_TU)  |  \
									   (1UL << BITP_EMAC_DMA_IEN_TI))

/*
 * Cache utility macros
 */

/*!
 * If the cached data line is dirty then this macro writes the line out
 * and marks the cache line a clean. The macro increments the supplied
 * address by cache line size.
 */
#define FLUSH(P)                                   \
	do {                                           \
		asm volatile("FLUSH[%0++];" : "+p"(P));    \
	} while (0)

/*!
 * This macro flushes and invalidates the cache line. So the cache line
 * will be loaded again from next level of memory.The macro increments
 * the supplied cache line pointer by the cache size.
 */
#define FLUSHINV(P)                                \
	do {                                           \
		asm volatile("FLUSHINV[%0++];" : "+p"(P)); \
	} while (0)

/*!
 * This macro also performs flush and invalidate operation for single cache
 * line.
 */
#define SIMPLEFLUSHINV(P)                          \
	do {                                           \
		ssync();                                   \
		asm volatile("FLUSHINV[%0];"::"#p"(P));    \
		ssync();                                   \
	} while (0)

/* function prototypes */
static ADI_ETHER_RESULT activate_channel ( ADI_ETHER_HANDLE hDevice,
		ADI_EMAC_CHANNEL *pChannel );

static ADI_ETHER_RESULT insert_queue ( ADI_EMAC_FRAME_Q *pQueue,
									   ADI_ETHER_BUFFER *pBuffer );

static ADI_ETHER_RESULT bind_buf_with_desc ( ADI_ETHER_HANDLE hDevice, ADI_EMAC_CHANNEL *pChannel );

static ADI_ETHER_RESULT init_descriptor_list ( ADI_ETHER_HANDLE phDevice,
		const uint8_t *pMemory,
		const uint32_t Length,
		ADI_EMAC_CHANNEL *pChannel );

static uint32_t emac_init ( ADI_ETHER_HANDLE *const phDevice );

static void sleep ( uint32_t time_sec );

ADI_ETHER_RESULT add_multicastmac_filter ( ADI_ETHER_HANDLE *const phDevice,
		uint32_t GroupIpAddress,
		bool bAddAddress );

static void flush_area ( void *start, uint32_t bytes );
static void flushinv_area ( void *start, uint32_t bytes );
#endif /* _ADI_GEMAC_INT_H_ */
