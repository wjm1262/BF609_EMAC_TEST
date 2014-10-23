/*
 * mem_manager.h
 *
 *  Created on: 2014-10-21
 *      Author: Administrator
 */

#ifndef MEM_MANAGER_H_
#define MEM_MANAGER_H_

#include "BF609_EMAC_Test_Core0.h"
#include <drivers/ethernet/adi_ether.h>
#include "adi_gemac_int.h"

#include "queue.h"



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

extern ETH_CFG_INFO user_net_config_info[MAX_NETWORK_IF];
extern unsigned long int user_net_num_ifces;

int InitBuff ( const unsigned int inMemSize, char *MemArea, ADI_ETHER_HANDLE hDevice, ETH_CFG_INFO *bsInfo );


#endif /* MEM_MANAGER_H_ */
