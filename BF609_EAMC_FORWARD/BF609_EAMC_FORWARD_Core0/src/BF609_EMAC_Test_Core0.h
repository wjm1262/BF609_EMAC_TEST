/*****************************************************************************
 * BF609_EMAC_Test_Core0.h
 *****************************************************************************/

#ifndef __BF609_EMAC_TEST_CORE0_H__
#define __BF609_EMAC_TEST_CORE0_H__


/* Add your custom header content here */
#include <sys/exception.h>
#include <drivers/ethernet/adi_ether.h>

#include <stdio.h>
#include <string.h>
#include <cplb.h>

#include <stddef.h>

#include "adi_gemac_int.h"

#include "queue.h"


#ifdef __cplusplus
extern "C"  {
#endif


	
	/*********************************************/
	
	// check for four byte alignment
#define MY_CHECK_ALIGNMENT(x) (((x+3)&(~0x3)) == x)
#define MAX_NETWORK_IF 2
	
	/* \struct ADI_ETHER_FRAME_BUFFER
	 *
	 * Structure map the ethernet MAC frame
	 */
	typedef struct ADI_ETHER_FRAME_BUFFER
	{
		uint16_t     NoBytes;               /*!< Number of Bytes */
		uint8_t      Dest[6];               /*!< destination MAC address  */
		uint8_t      Srce[6];               /*!< source MAC address   */
		uint8_t      LTfield[2];            /*!< length/type field    */
		uint8_t      Data[1];               /*!< payload bytes */
		
	} ADI_ETHER_FRAME_BUFFER;
	
	
	typedef struct eth_cfg_info
	{
		size_t                    buff_overhead;
		
		short                     rx_buffs;
		short                     tx_buffs;
		// the maximum data size that each receive and transmit buffer must support
		short                     rx_buff_datalen;
		short                     tx_buff_datalen;
		
		short 					rx_len_align;
		short					tx_len_align;
		
		// the address and size of the area from which buffers are to be allocated
		// (must be 32-bit aligned, uncached and accessible by the controller)
		char                     *buff_area;
		int                       buff_area_size;
		
		// interface's individual (MAC) address
		unsigned char             hwaddr[6];
		
		// netif's device driver handle
		void                     *handle;
		
		// lists of received/transmitted buffers awaiting to be submit to Dev'RX/TX Channel
		ADI_ETHER_BUFFER         *rcv_list;
		//ADI_ETHER_BUFFER*         xmt_list;
		ADI_EMAC_FRAME_Q xmt_queue;
		
		// keeps track if there is a already a post. if its zero stack callback handler will
		// post the message, else it will be skipped.
		//
		int                       txmsg_processed;
		int                       rxmsg_processed;
		// lists of received/transmitted  buffers have been completed by EMAC awaiting APP processing/disposal
		QType rx_completed_q;
		QType tx_completed_q;
		
	} ETH_CFG_INFO;
	


	ETH_CFG_INFO user_net_config_info[MAX_NETWORK_IF] =
	{
		{
				0,
				8000,
				100,
				1600,
				1548,
				0,
				0,
				0,
				0,
				{0x00u, 0x0au, 0x1au, 0x00u, 0x05u, 0x0bu},
				0,
				0,
				{NULL, NULL, 0},
				0,
				0,
			},
		{
			0,
 			100,
			8000,
			1600,
			1548,
			0,
			0,
			0,
			0,
	  		{0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u},
			0,
			0,
			{NULL, NULL, 0},
			0,
			0,
		},

	};
	
	unsigned long int user_net_num_ifces = sizeof ( user_net_config_info ) / sizeof ( ETH_CFG_INFO );
	
	
	/***************************************************************
	 *
	 * ADSP-BF60x family configuration
	 */
#if defined( __ADSPBF60x__)
#include <drivers/ethernet/adi_ether_gemac.h>
#include <services/int/adi_int.h>
#include <services/int/adi_sec.h>
	
#if defined(__ADSPBF609__)
#include <cdefbf609.h>
#include <defbf609.h>
#elif defined(__ADSPBF608__)
#include <cdefbf608.h>
#include <defbf608.h>
#elif defined(__ADSPBF607__)
#include <cdefbf607.h>
#include <defbf607.h>
#elif defined(__ADSPBF606__)
#include <cdefbf606.h>
#include <defbf606.h>
#endif
	
	//   #define EMAC_DRIVER_ENTRY (GEMAC1DriverEntry) /* EMAC0 Driver Entry point */
#define EMAC0_NUM_RECV_DESC    (1200)  /*! Number of receive DMA descriptors  */
#define EMAC0_NUM_XMIT_DESC    (100)  /*! Number of transmit DMA descriptors */
#define EMAC1_NUM_RECV_DESC    (100)  /*! Number of receive DMA descriptors  */
#define EMAC1_NUM_XMIT_DESC    (1200)  /*! Number of transmit DMA descriptors */
	
#pragma alignment_region (32)
	uint8_t BaseMemSize0[32];
	uint8_t MemRcve0[EMAC0_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */
	uint8_t MemXmit0[EMAC0_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */

	uint8_t BaseMemSize1[32];
	uint8_t MemRcve1[EMAC1_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */
	uint8_t MemXmit1[EMAC1_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */
#pragma alignment_region_end
#endif /* __ADSPBF60x__ */
	/***********************************************************/
	
	
	
	/* Initialize the Memory Table */
	ADI_ETHER_MEM memtable[MAX_NETWORK_IF] =
	{
		{
			MemRcve0, sizeof ( MemRcve0 ),
			MemXmit0, sizeof ( MemXmit0 ),
			BaseMemSize0, sizeof ( BaseMemSize0 )
		},
		{
			MemRcve1, sizeof ( MemRcve1 ),
			MemXmit1, sizeof ( MemXmit1 ),
			BaseMemSize1, sizeof ( BaseMemSize1 )
		}
	};
	
	
	
	//
	inline void reverse_mac ( char *ptr );
	
	
	/*********************** user-defined Functions  ******************************************/
	typedef  void ( *ETHERNET_CALLBACK_FUN ) ( void *arg1, unsigned int event, void *FrameBuffers );
	
	void Ethernet0_Callback ( void *arg1, unsigned int event, void *FrameBuffers );
	void Ethernet1_Callback ( void *arg1, unsigned int event, void *FrameBuffers );
	
	
	
	/*******************************************************************************************/
	
	
	
	
	
	
	
#ifdef __cplusplus
}
#endif

#endif /* __BF609_EMAC_TEST_CORE0_H__ */
