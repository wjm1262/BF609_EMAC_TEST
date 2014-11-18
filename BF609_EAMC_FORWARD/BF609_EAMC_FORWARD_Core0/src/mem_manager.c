/*
 * mem_manager.c
 *
 *  Created on: 2014-10-21
 *      Author: Administrator
 */

#include "stddef.h"
#include "post_debug.h"
#include "mem_manager.h"
/* bf51x_hw reference v1.2 P22-18
 * The length of each RX DMA buffer must be at least 1556 (0x614) bytes.
 * This is the maximum number of bytes that the MAC can deliver by DMA
 * on any receive frame. Frames longer than the 1556-byte hardware limit
 * are truncated by the MAC.The 1556-byte hardware limit accommodates the
 * longest legal Ethernet frames (1518 bytes for untagged frames, or
 * 1522 bytes for tagged 802.1Q frames) plus a small margin to accommodate
 * future standards extensions.
 * The MAC does not support RX DMA data buffers composed of more
 * than one descriptor.
 * */

/* bf51x_hw reference v1.2 P22-25
 * End of frame – At the end of the frame, the MAC issues a finish command to the DMA controller,
 * causing it to advance to the next (status) descriptor.
 * If the TX frame exceeds the maximum length limit (1560 bytes, or 0x618),
 * the frame’s DMA transfer is truncated.
 * Only 1543 (0x607) are transmitted on the MII.
 * */
// 1600,1548
ETH_CFG_INFO user_net_config_info[MAX_NETWORK_IF] =
{
	{
			0,
			8000,
			10,
			1560,
			1560,
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
			{NULL, NULL, 0},
	},
	{
		0,
		200,
		6000,
		1560,
		1560,
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
		{NULL, NULL, 0},
	},

};

unsigned long int user_net_num_ifces = sizeof ( user_net_config_info ) / sizeof ( ETH_CFG_INFO );


/****************************************************************
 * Initialises, assigns Tx and Rx buffers to each
 * network interface.
 *
 * Buffer structure
 *
 *   ----------------  ---------------   -----------------------
 *   ADI_ETHER_BUFFER| buffer_overhead|  ADI_ETHER_FRAME_BUFFER
 *   ----------------  ----------------  -----------------------
 ****************************************************************/
static int InitFrameBuff ( ADI_ETHER_HANDLE  const hDevice, ETH_CFG_INFO *bsInfo )
{
	ADI_ETHER_BUFFER *p;

	int rx_len, tx_len, count;
	int i;
	ADI_ETHER_BUFFER *rx_head = NULL;
	ADI_ETHER_BUFFER *tx_head = NULL;

	char *buf;

	if ( !bsInfo || bsInfo->rx_buff_datalen < MIN_ETHER_FRAME_LEN ||
			bsInfo->tx_buff_datalen < MIN_ETHER_FRAME_LEN )
		return -1;

	if ( !bsInfo->buff_area || 0 >= bsInfo->buff_area_size )
		return -1;

	clear_queue ( &bsInfo->xmt_buffers_queue );
	clear_queue ( &bsInfo->rx_completed_queue );


	// calculate total requirement for each rx and tx buffer

	rx_len = bsInfo->rx_len_align;
	tx_len = bsInfo->tx_len_align;

	rx_len = ( ( rx_len + 3 ) / 4 ) * 4;
	tx_len = ( ( tx_len + 3 ) / 4 ) * 4;

	rx_len = ( rx_len + 31 ) & ~31;
	tx_len = ( tx_len + 31 ) & ~31;

	buf = ( char * ) ( ( ( unsigned int ) bsInfo->buff_area + 31 ) & ~31 );

	// allocate buffers in required ratio from supplied memory area
	//while (bsInfo->buff_area_size > rx_len || bsInfo->buff_area_size > tx_len)
	if ( bsInfo->buff_area_size > rx_len || bsInfo->buff_area_size > tx_len )
	{
		int n;

		for ( n = 0; n < bsInfo->rx_buffs; n += 1 )
		{
			if ( bsInfo->buff_area_size < rx_len )
				break;

			p = ( ADI_ETHER_BUFFER * ) bsInfo->buff_area;

			bsInfo->buff_area += rx_len;

			bsInfo->buff_area_size -= rx_len;

			p->pNext = rx_head;
			rx_head = p;
		}

		for ( n = 0; n < bsInfo->tx_buffs; n += 1 )
		{
			if ( bsInfo->buff_area_size < tx_len )
				break;

			p = ( ADI_ETHER_BUFFER * ) bsInfo->buff_area;

			bsInfo->buff_area += tx_len;

			bsInfo->buff_area_size -= tx_len;

			p->pNext = tx_head;
			tx_head = p;

			if ( 0 == n )
			{
				bsInfo->xmt_buffers_queue.pQueueTail = p;
			}
		}
	}

	// initialise each buffer's ADI_ETHER_BUFFER descriptor
	p = rx_head;
	count = 0;

	while ( p )
	{
		p->Data = ( char * ) p + sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead;

		p->ElementCount = bsInfo->rx_buff_datalen;
		p->ElementWidth = 1;
		p->CallbackParameter = p;
		p->ProcessedElementCount = 0;
		p->ProcessedFlag = 0;
		p->PayLoad = 0;
		p->x = 0;

		count += 1;
		p = p->pNext;
	}

	bsInfo->rx_buffs = count;

	p = tx_head;
	count = 0;

	while ( p )
	{
		p->Data = ( char * ) p + sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead;

		p->ElementCount = bsInfo->tx_buff_datalen;
		p->ElementWidth = 1;
		p->CallbackParameter = p;
		p->x = 0;
		p->PayLoad = 0;

		count += 1;
		p = p->pNext;
	}

	bsInfo->tx_buffs = count;

	// give all the rx buffers to the Ethernet device driver
	adi_ether_Read ( hDevice , ( ADI_ETHER_BUFFER * ) rx_head );
	bsInfo->rcv_list = rx_head;

	// save the list of tx buffers until they are needed

	bsInfo->xmt_buffers_queue.pQueueHead = tx_head;
	bsInfo->xmt_buffers_queue.ElementCount = bsInfo->tx_buffs;

	return 1;
}

int InitBuff ( const unsigned int inMemSize, char *MemArea, ADI_ETHER_HANDLE hDevice, ETH_CFG_INFO *bsInfo )
{
	int i, overhead = 0;
	int nw_ifce_buffer_length = 0;
	unsigned int BUFF_AREA_START = 0;
	unsigned int BUFF_AREA_LEN = 0;
	int rx_len, tx_len;
	short buffs;

	BUFF_AREA_START = ( unsigned int ) ( ( ( unsigned int ) ( MemArea + 32 ) ) & ~31 );
	BUFF_AREA_LEN   = ( unsigned int ) inMemSize;

	// get driver specific overhead
	adi_ether_GetBufferPrefix ( hDevice, ( void * ) &overhead );
	bsInfo->buff_overhead  = overhead;

	// Caluculate the total buffer space for rx/tx and for all n/w interfaces.
	nw_ifce_buffer_length = 0;

	rx_len = sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead + bsInfo->rx_buff_datalen;
	tx_len = sizeof ( ADI_ETHER_BUFFER ) + bsInfo->buff_overhead + bsInfo->tx_buff_datalen;

	/* make rx and tx lengths multiple of 32 byte cache lines */
	rx_len = ( rx_len + 31 ) & ~31;
	tx_len = ( tx_len + 31 ) & ~31;

	bsInfo->rx_len_align = rx_len;
	bsInfo->tx_len_align = tx_len;

	nw_ifce_buffer_length  = bsInfo->rx_buffs * rx_len;
	nw_ifce_buffer_length  += bsInfo->tx_buffs * tx_len;

	if ( BUFF_AREA_LEN  < nw_ifce_buffer_length )
	{
		//先满足最小的buff需求
		buffs = (bsInfo->rx_buffs < bsInfo->tx_buffs) ? bsInfo->rx_buffs:bsInfo->tx_buffs;

		nw_ifce_buffer_length  = buffs * ( rx_len + tx_len );

		if ( BUFF_AREA_LEN  < nw_ifce_buffer_length )
		{
			DEBUG_STATEMENT ( "InitBuff: NO enough memory!\n\n" );
			return -1;
		}

		if( bsInfo->rx_buffs == buffs )
		{
			bsInfo->tx_buffs = buffs + (BUFF_AREA_LEN  - nw_ifce_buffer_length)/tx_len;
		}
		else if( bsInfo->tx_buffs == buffs )
		{
			bsInfo->rx_buffs = buffs + (BUFF_AREA_LEN  - nw_ifce_buffer_length)/rx_len;
		}

		nw_ifce_buffer_length  = bsInfo->rx_buffs * rx_len + bsInfo->tx_buffs * tx_len;

	}

	bsInfo->buff_area = ( char * ) ( BUFF_AREA_START  );
	bsInfo->buff_area_size = nw_ifce_buffer_length;

	//
	return InitFrameBuff ( hDevice, bsInfo );
}


