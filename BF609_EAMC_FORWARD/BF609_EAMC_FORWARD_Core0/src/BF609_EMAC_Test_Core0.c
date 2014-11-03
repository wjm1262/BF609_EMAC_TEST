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

#include "arith.h" //handle time

#include "mem_manager.h"
#include "queue.h"

#include "xl-6004_forward_protocol.h"

#include "core_timer.h"
#include "timer_isr.h"

#define EMAC0_NUM_RECV_DESC    (1200)  /*! Number of receive DMA descriptors  */
#define EMAC0_NUM_XMIT_DESC    (10)  /*! Number of transmit DMA descriptors */
#define EMAC1_NUM_RECV_DESC    (100)  /*! Number of receive DMA descriptors  */
#define EMAC1_NUM_XMIT_DESC    (1200)  /*! Number of transmit DMA descriptors */


//
void HandleLoop(void);
void Init_UART(void);
int EtherSend ( ADI_ETHER_HANDLE  const hDevice, ADI_ETHER_BUFFER *tx_frame );
ADI_ETHER_BUFFER *EtherRecv ( ADI_ETHER_HANDLE  const hDevice );



/* configures the soft switches */
void ConfigSoftSwitches ( void );

void External_PPS_Trigger_Callback ( ADI_GPIO_PIN_INTERRUPT   const ePinInt,
									uint32_t                 const Data,
									void*                    pCBParam);

/* ***　　global vars　　****************/

extern FORWARD_ETHER_FRAME board_info;

/*! size of the memory block to allocate to the stack.  */
const unsigned g_contEthHeapSize[2]={1024*1024*12,1024*1024*12};

ADI_ETHER_HANDLE 		g_hDev[MAX_NETWORK_IF] = {0};
ADI_ETHER_DRIVER_ENTRY 	*g_pDevEntry[MAX_NETWORK_IF] = {&GEMAC0DriverEntry, &GEMAC1DriverEntry};
ETHERNET_CALLBACK_FUN 	g_pEthCallBack[MAX_NETWORK_IF] = {Ethernet0_Callback, Ethernet1_Callback};
int 					g_AuxiTMIsFirstUpdated = 1;

#pragma alignment_region (32)
uint8_t BaseMemSize0[32];
uint8_t MemRcve0[EMAC0_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */
uint8_t MemXmit0[EMAC0_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */

uint8_t BaseMemSize1[32];
uint8_t MemRcve1[EMAC1_NUM_RECV_DESC * 32]; /*! Receive DMA descriptor memory  */
uint8_t MemXmit1[EMAC1_NUM_XMIT_DESC * 32]; /*! Transmit DMA descriptor memory */
#pragma alignment_region_end

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

/* init the LED service
 * used for indicating the update process
  */
static void GPIO_Init(void)
{
	ADI_GPIO_RESULT result;
	static uint8_t gpioMemory[32];
	uint32_t gpioMaxCallbacks;

	result = adi_gpio_Init( (void*)gpioMemory,	32,	&gpioMaxCallbacks);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("%s failed\n\n", result);
	}

	/* set GPIO output LED 1 */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_G, ADI_GPIO_PIN_13, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("%s failed\n\n", result);
	}

	/* LED1 */
	result = adi_gpio_Set(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("%s failed\n\n", result);
	}

	/* set GPIO output WDI */
	result = adi_gpio_SetDirection(	ADI_GPIO_PORT_C, ADI_GPIO_PIN_15, ADI_GPIO_DIRECTION_OUTPUT);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("%s failed\n\n", result);
	}
	/* LED1 */
	result = adi_gpio_Set(ADI_GPIO_PORT_C, ADI_GPIO_PIN_15);
	if (result != ADI_GPIO_SUCCESS)
	{
		DEBUG_PRINT("%s failed\n\n", result);
	}
}

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

    GPIO_Init();

    Init_PTPAuxin();

    CoreTimerInit();

    Init_Timer_Interrupts();


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

	snprintf(VersionString + 15, 100, "Board ID:%d, Built on %s, at %s ", board_info.MUAddr, __DATE__, __TIME__);

	DEBUG_PRINT ( " %s \n\n" , VersionString);
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
		ether_stack_block = heap_malloc ( i+1, g_contEthHeapSize[i] );
		if ( ether_stack_block == NULL )
		{
			DEBUG_PRINT ( " heap_malloc: in heap %d, failed to allocate memory to the stack \n\n" , i+1);
			return ;
		}
		
		/* init buf mem */
		nRet = InitBuff ( g_contEthHeapSize[i],
				ether_stack_block, hEthernet,
				&user_net_config_info[i] );
		if( nRet < 0 )
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
	FORWARD_ETHER_FRAME* pForwardFrm = NULL;

	ADI_ETHER_BUFFER *pRecv = NULL,*pSend = NULL;

	RX_TX_TIME_STAMP_BUFFER *pTmBuff = NULL;

	ADI_ETHER_FRAME_BUFFER *pPkt = NULL;

	uint32_t FrmLen = 0;

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

			//send a frame by emac1
			pSend = PackForwardSMVFrame ( nanSeconds, (char*)pRecv->Data +2,
					FrmLen,  &user_net_config_info[1] );

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
 			//process control messages frm...
			pForwardFrm = ( FORWARD_ETHER_FRAME * ) ( pRecv->Data );

			if( pForwardFrm->MUAddr == board_info.MUAddr )
			{
				HandleControlMessage((uint8_t*)pForwardFrm);
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
	ADI_ETHER_BUFFER *pRecv, *pXmt;
	ADI_ETHER_FRAME_BUFFER * pPkt;
	
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
#if 0
				while ( pack_list )
				{
					pXmt = pack_list;
					pack_list = pack_list->pNext;
					pXmt->pNext = NULL;

					pPkt = ( ADI_ETHER_FRAME_BUFFER * ) ( pXmt->Data );

					// forward SMV frame to PC
					if( (pPkt->LTfield[0] == BF609_FORWARD_SMV_TYPE_LO) &&
							(pPkt->LTfield[1] == BF609_FORWARD_SMV_TYPE_HI ))
					{
						//reuse buff, return the buff to EMAC0' RX Channel
						pXmt->pNext = NULL;
						pXmt->ProcessedElementCount = 0;
						pXmt->ProcessedFlag = 0;
						adi_ether_Read ( g_hDev[0], pXmt );
					}
					// send other frame to PC controller, such as ACK frame for control messages.
					else
					{
						if( (pPkt->LTfield[0] == BF609_UPDATE_VER_ACKOK_TYPE_LO) &&
								(pPkt->LTfield[1] == BF609_UPDATE_VER_ACKOK_TYPE_HI ))
						{
							g_ACKOK_XMT_Completed = 1;
						}

						push_queue ( &user_net_config_info[1].xmt_queue, pXmt );
					}
				}
#endif
				while ( pack_list )
				{
					pXmt = pack_list;
					pack_list = pack_list->pNext;
					pXmt->pNext = NULL;

					pPkt = ( ADI_ETHER_FRAME_BUFFER * ) ( pXmt->Data );

					if( (pPkt->LTfield[0] == BF609_UPDATE_VER_ACKOK_TYPE_LO) &&
							(pPkt->LTfield[1] == BF609_UPDATE_VER_ACKOK_TYPE_HI ))
					{
						g_ACKOK_XMT_Completed = 1;
					}

					push_queue ( &user_net_config_info[1].xmt_queue, pXmt );

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

