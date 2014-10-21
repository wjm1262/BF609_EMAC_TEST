/*****************************************************************************
 * BF609_EMAC_Test_Core0.c
 *****************************************************************************/

///* enables debug prints */
//#define ADI_DEBUG  (1)
#define BF609_EZ_BRD 0

#include "BF609_EMAC_Test_Core0.h"

#include <ccblkfn.h>
#include <adi_osal.h>
#include <services/gpio/adi_gpio.h> //should before adi_initialize.h
#include "adi_initialize.h"
#include "cgu_init.h"

#include "post_debug.h"

#include "adi_gemac_int.h"

#include "xl-6004_forward_protocol.h"
#include "arith.h" //handle time

#include "BF609_Update_Protocol.h"
#include "VerUpdate.h"

//////////////////////////////////////////

#define MIN_ETHER_FRAME_LEN 64
#define MAX_ETHER_FRAME_LEN 1600

/*! Enters critical region */
#define ENTER_CRITICAL_REGION()  (adi_osal_EnterCriticalRegion())
/*! Exit critical region */
#define EXIT_CRITICAL_REGION()   (adi_osal_ExitCriticalRegion())


/*! size of the memory block to allocate to the stack.  */
#define ETHER_STACK_SIZE          (1024*1024*10)

//
void HandleLoop(void);
void Init_UART(void);
int EtherSend ( ADI_ETHER_HANDLE  const hDevice, ADI_ETHER_BUFFER *tx_frame );
ADI_ETHER_BUFFER *EtherRecv ( ADI_ETHER_HANDLE  const hDevice );
static int InitBuff ( const unsigned int inMemSize, char *MemArea, ADI_ETHER_HANDLE hDevice, ETH_CFG_INFO *bsInfo );
static int InitFrameBuff ( ADI_ETHER_HANDLE  const hDevice, ETH_CFG_INFO *bsInfo );
static ADI_ETHER_BUFFER *PackEtherFrame ( char *EthFrmHeader, int HeaderLen, char *EthFrmData, int DataLen, ETH_CFG_INFO *bsInfo );


/* ADI_EMAC_FRAME_Q manegment */
void               clear_queue ( ADI_EMAC_FRAME_Q *pQueue );
ADI_ETHER_BUFFER  *pop_queue ( ADI_EMAC_FRAME_Q *pQueue );
int                push_queue ( ADI_EMAC_FRAME_Q *pQueue, ADI_ETHER_BUFFER  *pElem );
/********************************/

extern void Init_PTPAuxin(void);
extern void Enable_Time_Stamp_Auxin_Interrupt(void);
extern void enable_rx ( ADI_ETHER_HANDLE phDevice );
extern void enable_tx ( ADI_ETHER_HANDLE phDevice );
extern void SetFlexiblePPSOutput( ADI_ETHER_HANDLE phDevice, PPS_TYPE ePPSType, TimeInternal tmStart,
									unsigned int uPPSInterval,unsigned int uPPSWidth);
extern void ResetSysTime(ADI_ETHER_HANDLE phDevice);
//enable emac tx,rx
extern void enable_emac_tx_rx ( ADI_ETHER_HANDLE phDevice );

/* configures the soft switches */
void ConfigSoftSwitches ( void );




void External_PPS_Trigger_Callback ( ADI_GPIO_PIN_INTERRUPT   const ePinInt,
									uint32_t                 const Data,
									void*                    pCBParam);

/* ***　　global vars　　****************/
ADI_ETHER_HANDLE 		g_hDev[MAX_NETWORK_IF] = {0};
ADI_ETHER_DRIVER_ENTRY 	*g_pDevEntry[MAX_NETWORK_IF] = {&GEMAC0DriverEntry, &GEMAC1DriverEntry};
ETHERNET_CALLBACK_FUN 	g_pEthCallBack[MAX_NETWORK_IF] = {Ethernet0_Callback, Ethernet1_Callback};
int 					g_AuxiTMIsFirstUpdated = 1;

void main ( void )
{
	ADI_ETHER_HANDLE   hEthernet;
	ADI_ETHER_RESULT   etherResult;
	ADI_ETHER_DEV_INIT EtherInitData[MAX_NETWORK_IF] = { { true, &memtable[0] }, { true, &memtable[1] }}; // data-cache,driver memory
	uint32_t reg_data;
	int i, nEtherDevUsed;
	char *ether_stack_block;

	ADI_GPIO_RESULT gpio_result;
	uint32_t gpioMaxCallbacks;
	int nRet;


	/**
	 * Initialize managed drivers and/or services that have been added to
	 * the project.
	 * @return zero on success
	 */
	adi_initComponents();
	
	/**
	 * The default startup code does not include any functionality to allow
	 * core 0 to enable core 1. A convenient way to enable core 1 is to use the
	 * 'adi_core_1_enable' function.
	 */
	adi_core_1_enable();
	
	/* Begin adding your custom code here */
	g_AuxiTMIsFirstUpdated = 1;
	
	/* init CGU first time */
	CGU_Init ( MULTIPLIER_SEL, CCLK_SEL, DDRCLK_SEL );				/* CCLK=16.384*iMultiplier /1 Mhz, 16.384*iMultiplier/iDDCLKSel Mhz DDR2 CLK */
	
#if defined(__DEBUG_FILE__)
    /* open the debug file */
    pDebugFile = fopen(__DEBUG_FILE_NAME__, "w");
    if (pDebugFile == 0)
    {
    	fclose(pDebugFile);
    	return;
    }
#elif defined(__DEBUG_UART__)
    Init_UART();
#endif

    Init_PTPAuxin();

	/* configures the switches */
#if BF609_EZ_BRD
	
	DEBUG_STATEMENT ( "Configuring switches for the ethernet operation \n\n" );
	ConfigSoftSwitches();
	
#endif
	
	/* open ethernet device */
	nEtherDevUsed = ( user_net_num_ifces > MAX_NETWORK_IF ) ? MAX_NETWORK_IF : user_net_num_ifces;

#if BF609_EZ_BRD
	nEtherDevUsed = 1;
#endif

	DEBUG_STATEMENT ( " init EMAC\n\n" );

	for ( i = 0; i < nEtherDevUsed; i++ )
	{
		etherResult = adi_ether_Open ( g_pDevEntry[i], &EtherInitData[i], g_pEthCallBack[i], &hEthernet );
		if ( etherResult != ADI_ETHER_RESULT_SUCCESS )
		{
			DEBUG_STATEMENT ( "adi_ether_Open: failed to open ethernet driver\n\n" );
			return ;
		}
		
		g_hDev[i] = hEthernet;
		
		/* get the mac address */
		memcpy ( ( ( ADI_EMAC_DEVICE * ) hEthernet )->MacAddress, user_net_config_info[i].hwaddr, 6 );

		/* allocate memory  */
		ether_stack_block = heap_malloc ( i+1, ETHER_STACK_SIZE );
		if ( ether_stack_block == NULL )
		{
			DEBUG_PRINT ( " heap_malloc: in heap %d, failed to allocate memory to the stack \n\n" , i+1);
			return ;
		}
		
		/* init buf mem */
		nRet = InitBuff ( ETHER_STACK_SIZE, ether_stack_block, hEthernet, &user_net_config_info[i] );
		if(nRet<0)
		{
			DEBUG_STATEMENT ( " InitBuff: failed to enable Init Buffs\n\n" );
			return ;
		}

		/* Enable the MAC */
		etherResult = adi_ether_EnableMAC ( hEthernet );
		if ( etherResult != ADI_ETHER_RESULT_SUCCESS )
		{
			DEBUG_STATEMENT ( " adi_ether_EnableMAC: failed to enable EMAC\n\n" );
			return ;
		}
	}
	
	//enable EMAC INT
	adi_int_EnableInt ( ( (ADI_EMAC_DEVICE *)g_hDev[0])->Interrupt, true);
	adi_int_EnableInt ( ( (ADI_EMAC_DEVICE *)g_hDev[1])->Interrupt, true);

	/* activate rx channel DMA */
	enable_rx ( g_hDev[0] );
	enable_rx ( g_hDev[1] );
	/* activate tx channel DMA */
	enable_tx ( g_hDev[0] );
	enable_tx ( g_hDev[1] );

	//enable emac0 tx,rx
	enable_emac_tx_rx (  g_hDev[0] );

	//enable emac1 tx,rx
	enable_emac_tx_rx (  g_hDev[1] );

	//enable
	Enable_Time_Stamp_Auxin_Interrupt();

	//
	HandleLoop();

	return ;

}//main

void HandleLoop(void)
{
	FORWARD_ETHER_FRAME_BUFFER ForwardFrmBuff;
	BF609_ETHE_FRAME* pForwardFrm = NULL;

	ADI_ETHER_BUFFER *pRecv = NULL,*pSend = NULL;

	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;

	ADI_ETHER_FRAME_BUFFER *pPkt = NULL;

	int FrmLen = 0;

	unsigned int nanSeconds = 0;
	BF609_COMM_ACK_CODE commRet;

	while ( 1 )
	{
		// recv a frame from Eth0;
		pRecv = EtherRecv ( g_hDev[0] );
		if ( pRecv )
		{
			//get time stamp
			pTmBuff = ( RX_TX_TIME_STAMP_BUFFER * ) pRecv;
			nanSeconds = ( unsigned int ) ( pTmBuff->RxTimeStamp.TimeStampLo ) ;

			// get frame length
			FrmLen = pRecv->ProcessedElementCount - 6; //adi_gemac.c process_int 中 加了6 bytes，WHY???

			//send a frame by eth1
			PackForwardFrmHeader ( &ForwardFrmBuff , ( char * ) &nanSeconds );
			pPkt = ( ADI_ETHER_FRAME_BUFFER * ) ( pRecv->Data );
			pSend = PackEtherFrame ( ( char * ) &ForwardFrmBuff, 14, ( char * ) pPkt->Dest, FrmLen, &user_net_config_info[1] );

			EtherSend ( g_hDev[1], pSend );

			//reuse recv buff
			pRecv->pNext = NULL;
			pRecv->ProcessedElementCount = 0;
			pRecv->ProcessedFlag = 0;
			adi_ether_Read ( g_hDev[0], pRecv );

		}//if ( pRecv )

		// recv a frame from Eth1;
		pRecv = EtherRecv ( g_hDev[1] );
		if( pRecv )
		{
			//process frm...
			pForwardFrm = ( BF609_ETHE_FRAME * ) ( pRecv->Data );

			if( pForwardFrm->MU_Addr == board_info.DA )
			{
				commRet = VerUpdate((uint8_t*)pForwardFrm);
			}
			else
			{
				commRet = NAK_ERROR_DESTADDR_UNMATCH;
			}


			//reuse recv buff
			pRecv->pNext = NULL;
			pRecv->ProcessedElementCount = 0;
			pRecv->ProcessedFlag = 0;
			adi_ether_Read ( g_hDev[1], pRecv );
		}
	}//while
}

/////////////CALL_BACK
void External_PPS_Trigger_Callback (
    ADI_GPIO_PIN_INTERRUPT   const ePinInt,
    uint32_t                 const Data,
    void*                    pCBParam)

{
	TimeInternal tmStart = {0,0};
	ENTER_CRITICAL_REGION();

	//
	SetFlexiblePPSOutput( (ADI_ETHER_HANDLE) pCBParam, PULSE_SINGLE,tmStart, 0x5F5E0ff, 0x2FAF07f);

	//reset the system time
	ResetSysTime( (ADI_ETHER_HANDLE) pCBParam  );

	EXIT_CRITICAL_REGION();

}

void Ethernet0_Callback ( void *arg1, unsigned int event, void *FrameBuffers )
{
	/*
	ADI_ETHER_EVENT_FRAME_RCVD  One or more frames received.
	ADI_ETHER_EVENT_FRAME_XMIT  One or more frames tramsmitted.
	ADI_ETHER_EVENT_INTERRUPT  Ethernet event has occured.
	ADI_ETHER_EVENT_PHY_INTERRUPT  PHY interrupt has occured
	*/
	
	//ADI_ETHER_FRAME_BUFFER *pPkt;
	
	ADI_ETHER_BUFFER *pack_list = ( ADI_ETHER_BUFFER * ) FrameBuffers;

	ADI_ETHER_BUFFER *pRecv;
	
	

	if ( pack_list != NULL )
	{

		/* packet list may get modified from the TCP thread and all
		 * instances of it were protected using the critical regions
		 * Here we are in a critical region because we do not want to be
		 * preempted with a high priority interrupt.
		 */
		//int int_sts = cli();
		ENTER_CRITICAL_REGION();
		
		switch ( event )
		{
		
			case ADI_ETHER_EVENT_FRAME_RCVD:
			
				while ( pack_list )
				{
					pRecv = pack_list;
					
					pack_list = pack_list->pNext;
					
					pRecv->pNext = NULL;

					//Notes: 过滤其他非SV报文  by wjm@ 204-8-8
					//if ( pUChar[16] == 0x88 && pUChar[17] == 0xba )
					//Notes:只做纯粹的接收、打时标、协议转发工作，报文过滤交由上层应用处理， by wjm@2014-8-16 AM 10:21
					
					EnQueue ( &user_net_config_info[0].rx_completed_q, pRecv );
					
				}
				
				break;
				
			case ADI_ETHER_EVENT_FRAME_XMIT:
			
				if ( pack_list )
				{
					push_queue ( &user_net_config_info[0].xmt_queue, pack_list );
				}
				
				break;
				
			case ADI_ETHER_EVENT_INTERRUPT:
				break;
				
			case ADI_ETHER_EVENT_PHY_INTERRUPT:
				break;
		}
		
		//sti(int_sts);
		EXIT_CRITICAL_REGION();
	}
	
}

void Ethernet1_Callback ( void *arg1, unsigned int event, void *FrameBuffers )
{

	ADI_ETHER_BUFFER *pack_list = ( ADI_ETHER_BUFFER * ) FrameBuffers;
	ADI_ETHER_BUFFER *pRecv;
	
	if ( pack_list != NULL )
	{
	
		/* packet list may get modified from the TCP thread and all
		 * instances of it were protected using the critical regions
		 * Here we are in a critical region because we do not want to be
		 * preempted with a high priority interrupt.
		 */
		//int int_sts = cli();
		ENTER_CRITICAL_REGION();
		
		switch ( event )
		{
		
			case ADI_ETHER_EVENT_FRAME_RCVD:
			
				while ( pack_list )
				{
					pRecv = pack_list;
					
					pack_list = pack_list->pNext;
					
					pRecv->pNext = NULL;
					
					EnQueue ( &user_net_config_info[1].rx_completed_q, pRecv );
				}
				
				break;
				
			case ADI_ETHER_EVENT_FRAME_XMIT:
			
				if ( pack_list )
				{
					push_queue ( &user_net_config_info[1].xmt_queue, pack_list );
				}
				
				break;
				
			case ADI_ETHER_EVENT_INTERRUPT:
				break;
				
			case ADI_ETHER_EVENT_PHY_INTERRUPT:
				break;
		}
		
		//sti(int_sts);
		EXIT_CRITICAL_REGION();
	}
	
}




int EtherSend ( ADI_ETHER_HANDLE  const hDevice, ADI_ETHER_BUFFER *tx_frame )
{
	//	int i , nSelectedDev = 0;
	if ( !hDevice || !tx_frame )
	{
		DEBUG_STATEMENT ( " EtherSend: Input Params IS NULL \n\n" );
		return 0;
	}
	
	ADI_ETHER_RESULT eResult = adi_ether_Write ( hDevice, tx_frame );
	
	if ( eResult != ADI_ETHER_RESULT_SUCCESS )
	{
		DEBUG_STATEMENT ( " EtherSend: adi_ether_Write failed \n\n" );
		return 0;
	}
	
	return 1;
	
}

ADI_ETHER_BUFFER *EtherRecv ( ADI_ETHER_HANDLE  const hDevice )
{
	
	ADI_ETHER_BUFFER *pack = NULL;
	
	int nSelectedDev = 0;

	if ( hDevice == g_hDev[0] )
	{
		nSelectedDev = 0;
	}
	else if( hDevice == g_hDev[1] )
	{
		nSelectedDev = 1;
	}
	else
	{
		DEBUG_STATEMENT ( " EtherRecv: Can not find the  hDev \n\n" );
		return NULL;
	}

	//一次返回一个
	ENTER_CRITICAL_REGION();
	DeQueue ( &user_net_config_info[nSelectedDev].rx_completed_q, ( QElem * ) &pack ) ;
	EXIT_CRITICAL_REGION();
	
	return pack;
	
}


static ADI_ETHER_BUFFER *PackEtherFrame ( char *EthFrmHeader, int HeaderLen, char *EthFrmData, int DataLen, ETH_CFG_INFO *bsInfo )
{

	ADI_ETHER_BUFFER *tx;
	ADI_ETHER_FRAME_BUFFER *frm;
	
	char *head, *data, *Dst;
	
	unsigned short *ps;
	
	int  len;
	
	len = HeaderLen + DataLen;
	
	if ( len > bsInfo->tx_buff_datalen )
	{
		// frame too big for our buffers
		DEBUG_PRINT ( " PackEtherFrame:  frame (Len:%d) too big for our buffers (Len:%d)\n\n", len, bsInfo->tx_buff_datalen );
		return NULL;
	}
	
	// remove first free one from the list
	tx = pop_queue ( &bsInfo->xmt_queue );
	
	if ( tx == NULL )
	{
		DEBUG_STATEMENT ( " PackEtherFrame:  xmt_queue IS no free buff!\n\n " );
		return NULL;
	}
	
	// copy data from pbuf(s) into our buffer
	head = EthFrmHeader;
	data = EthFrmData;
	
	// the first two bytes reserved for length
	Dst = ( char * ) tx->Data + 2;
	
	//
	memcpy ( Dst, head, HeaderLen );
	Dst += HeaderLen;
	
	memcpy ( Dst, data, DataLen );
	
	tx->ElementCount = len + 2; // total element count including 2 byte header
		
	ps = ( unsigned short * ) tx->Data;
	*ps = tx->ElementCount - 2; // only the frame size excluding 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info
	
	return tx;
}



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
		
	clear_queue ( &bsInfo->xmt_queue );
	
	InitQueue ( &bsInfo->rx_completed_q );
	InitQueue ( &bsInfo->tx_completed_q );
	
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
				bsInfo->xmt_queue.pQueueTail = p;
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
	bsInfo->xmt_queue.pQueueHead = tx_head;
	bsInfo->xmt_queue.ElementCount = bsInfo->tx_buffs;

	return 1;
}

static int InitBuff ( const unsigned int inMemSize, char *MemArea, ADI_ETHER_HANDLE hDevice, ETH_CFG_INFO *bsInfo )
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
		/*
		buffs = ( BUFF_AREA_LEN ) / ( rx_len + tx_len );
		if ( buffs < 1 )
		{
			DEBUG_STATEMENT ( "InitBuff: NO enough memory!\n\n" );
			return -1;
		}
		*/

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




/* reverses the given mac_address */
inline void reverse_mac ( char *ptr )
{
	int i, j;
	char c;
	
	for ( i = 0, j = 5; i < 3; i++, j-- )
	{
		c = * ( ptr + i );
		* ( ptr + i ) = * ( ptr + j );
		* ( ptr + j ) = c;
	}
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
	
	return NumInputBuffers;
}
