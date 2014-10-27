/*
 * xl-6004_forward_protocol.c
 *
 *  Created on: 2014-10-22
 *      Author: Wu JM
 */
#include "BF609_EMAC_Test_Core0.h"

#include <drivers/ethernet/adi_ether.h>
#include "adi_gemac_int.h"
#include <adi_osal.h>

#include "xl-6004_forward_protocol.h"

#include "post_debug.h"

#include "flash_erase.h"

#define LDR_DATA_BUFFER_LOCATE_HEAP 2  //define the ldr data buffer area in heap.

extern int EtherSend ( ADI_ETHER_HANDLE  const hDevice, ADI_ETHER_BUFFER *tx_frame );

static BF609_COMM_ACK_CODE ProcessUpdateVersion(const CONTROL_FRAME *pCtrlDataBuf,
		uint32_t* pLDR_Len );


/////////////////////////////////
volatile int g_ACKOK_XMT_Completed = 0;
static char VersionString[128] = "ver2.0.4@2014-10-25 AM 10:50";

/*
 * 控制域
	0x00：控制信息帧，用于609与PC端进行控制信息的通信，不进行任何转发操作
	0x01：609 toPC端，发送实时以太网接口接收到数据
	0x02：609 to PC端，发送光串口接收数据
	0x03：609 toPC端，发送开关量输入接口数据

	0x81：PC端 to 609：转发数据到实时以太网接口
	0x82：PC端 to 609：转发数据到光串口输出
 *
 * */
FORWARD_ETHER_FRAME board_info =
{
	0,
	{0x06, 0x05, 0x04, 0x03, 0x02, 0x01},
	0,
	0x00,
	BF609_FORWARD_SMV_PC,
	{BF609_FORWARD_SMV_TYPE_LO, BF609_FORWARD_SMV_TYPE_HI_BASE},
	{0}
};

/********************/
//NOTES: if the SMVFrame more than 1500 bytes, then it is been cut off to 1500 bytes.
ADI_ETHER_BUFFER *PackForwardSMVFrame ( uint32_t unNanoSecond, char *SMVFrame,
		uint16_t SmvFrmLen, ETH_CFG_INFO *bsInfo )
{

	ADI_ETHER_BUFFER *tx;

	char *head, *data, *Dst;

	uint16_t  PayLoadLen = SmvFrmLen;
	uint16_t HeaderLen =14;


	if( PayLoadLen > MAX_ETHER_FRAME_LEN )
	{
		//if the SMVFrame more than 1500 bytes, then it is been cut off to 1500 bytes.
		//DEBUG_PRINT ( "PackForwardSMVFrame: frame (Len:%d) more than MAX_ETHER_FRAME_LEN (Len:%d), cut off to 1500bytes.\n\n", SmvFrmLen, MAX_ETHER_FRAME_LEN );
		PayLoadLen = MAX_ETHER_FRAME_LEN;
	}

	// remove first free one from the list
	tx = pop_queue ( &bsInfo->xmt_queue );

	if ( tx == NULL )
	{
		DEBUG_STATEMENT ( "PackForwardSMVFrame: xmt_queue no free buff!\n\n " );
		return NULL;
	}

	// copy data from pbuf(s) into our buffer
	data = SMVFrame;
	head = ( char * ) tx->Data;
	PackForwardSMVFrmHeader ( head, unNanoSecond, PayLoadLen);

	// the first two bytes reserved for length
	Dst = ( char * ) tx->Data + 2 + HeaderLen;
	memcpy ( Dst, data, PayLoadLen );

	tx->ElementCount = HeaderLen + PayLoadLen + 2; // total element count including 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}

void PackForwardSMVFrmHeader ( void *pForwardFrmHeader, uint32_t unNanoSecond,
		uint16_t unPktDataLen )
{
	FORWARD_ETHER_FRAME* pHeader = (FORWARD_ETHER_FRAME*)pForwardFrmHeader;

	pHeader->NoBytes    = 14 + unPktDataLen;//only the frame size excluding 2 byte header
	pHeader->DestMAC[0] = board_info.DestMAC[0];
	pHeader->DestMAC[1] = board_info.DestMAC[1];
	pHeader->DestMAC[2] = board_info.DestMAC[2];
	pHeader->DestMAC[3] = board_info.DestMAC[3];
	pHeader->DestMAC[4] = board_info.DestMAC[4];
	pHeader->DestMAC[5] = board_info.DestMAC[5];

	pHeader->TimeStamp 	= unNanoSecond;

	pHeader->MUAddr 	= board_info.MUAddr; //
	pHeader->CtrlField 	= BF609_FORWARD_SMV_PC;//forward smv

	pHeader->LTfield[0] = BF609_FORWARD_SMV_TYPE_LO;
	pHeader->LTfield[1] = BF609_FORWARD_SMV_TYPE_HI_BASE + board_info.MUAddr;
}

ADI_ETHER_BUFFER *PackACKFrmOfUpdateVerion ( BF609_COMM_ACK_CODE AckCode,
		void *pCtrlInfoFrmBuf, ETH_CFG_INFO *bsInfo )
{

	ADI_ETHER_BUFFER *tx;

	FORWARD_ETHER_FRAME *pRxData, *pTxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrFrm;	//control data ptr

	unsigned short *ps;

	int  len;

	pRxData = (FORWARD_ETHER_FRAME *)pCtrlInfoFrmBuf;

	// remove first free one from the list
	tx = pop_queue ( &bsInfo->xmt_queue );
	if ( tx == NULL )
	{
		DEBUG_STATEMENT ( " PackEtherFrame:  xmt_queue IS no free buff!\n\n " );
		return NULL;
	}

	// init header of ether frm, default it is ACK_OK
	pTxData = ( FORWARD_ETHER_FRAME * ) tx->Data;

	pTxData->DestMAC[0] = board_info.DestMAC[0];
	pTxData->DestMAC[1] = board_info.DestMAC[1];
	pTxData->DestMAC[2] = board_info.DestMAC[2];
	pTxData->DestMAC[3] = board_info.DestMAC[3];
	pTxData->DestMAC[4] = board_info.DestMAC[4];
	pTxData->DestMAC[5] = board_info.DestMAC[5];
	pTxData->MUAddr     = board_info.MUAddr;
	pTxData->CtrlField  = pRxData->CtrlField;//BF609_CTR;//0x00
	pTxData->LTfield[0] = BF609_UPDATE_VER_ACKOK_TYPE_LO;
	pTxData->LTfield[1] = BF609_UPDATE_VER_ACKOK_TYPE_HI;

	//init data of ether frm
	pTxData->PktData[0] =  0x68;
	pTxData->PktData[1] = 0x0a;
	pTxData->PktData[2] = 0x00;
	pTxData->PktData[3] =  0x68;
	pTxData->PktData[4] = 0x00;
	pTxData->PktData[5] = 0x10;// ACK_OK
	pTxData->PktData[6] =  0x01;//VERSION_UPDATE
	pTxData->PktData[7] = 0x00;
	pTxData->PktData[8] = 0x00;
	pTxData->PktData[9] =  0x16;

	if(AckCode != ACK_OK)
	{
		pTxData->LTfield[0] = BF609_UPDATE_VER_NAK_TYPE_LO;
		pTxData->LTfield[1] = BF609_UPDATE_VER_NAK_TYPE_HI;

		pTxData->PktData[5] = 0x80;// NAK
		pTxData->PktData[7] = AckCode;

	}

	tx->ElementCount = 14+10 + 2; // total element count including 2 byte header

	ps = ( unsigned short * ) tx->Data;
	*ps = tx->ElementCount - 2; // only the frame size excluding 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}

ADI_ETHER_BUFFER *PackACKFrmOfReadVersion ( void *pCtrlInfoFrmBuf, ETH_CFG_INFO *bsInfo )
{

	ADI_ETHER_BUFFER *tx;

	FORWARD_ETHER_FRAME *pRxData, *pTxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrFrm;	//control data ptr

	unsigned short *ps;


	pRxData = (FORWARD_ETHER_FRAME *)pCtrlInfoFrmBuf;

	// remove first free one from the list
	tx = pop_queue ( &bsInfo->xmt_queue );
	if ( tx == NULL )
	{
		DEBUG_STATEMENT ( " PackEtherFrame:  xmt_queue IS no free buff!\n\n " );
		return NULL;
	}

	// init header of ether frm, default it is ACK_OK
	pTxData = ( FORWARD_ETHER_FRAME * ) tx->Data;

	pTxData->DestMAC[0] = board_info.DestMAC[0];
	pTxData->DestMAC[1] = board_info.DestMAC[1];
	pTxData->DestMAC[2] = board_info.DestMAC[2];
	pTxData->DestMAC[3] = board_info.DestMAC[3];
	pTxData->DestMAC[4] = board_info.DestMAC[4];
	pTxData->DestMAC[5] = board_info.DestMAC[5];
	pTxData->MUAddr     = board_info.MUAddr;
	pTxData->CtrlField  = pRxData->CtrlField;//BF609_CTR;//0x00

	pTxData->LTfield[0] = BF609_READ_VER_TYPE_LO;
	pTxData->LTfield[1] = BF609_READ_VER_TYPE_HI;

	//init data of ether frm
	size_t len = strlen(VersionString);
	pTxData->PktData[0] =  0x68;
	pTxData->PktData[1] = 8+len;
	pTxData->PktData[2] = 0x00;
	pTxData->PktData[3] =  0x68;
	pTxData->PktData[4] = 0x00;
	pTxData->PktData[5] = VERSION_GET;//

	memcpy(&(pTxData->PktData[6]), VersionString, len );

	pTxData->PktData[6+len] =  0x00;//chksum, no use
	pTxData->PktData[7+len] = 0x16;

	tx->ElementCount = 14+ pTxData->PktData[1]  + 2; // total element count including 2 byte header

	ps = ( unsigned short * ) tx->Data;
	*ps = tx->ElementCount - 2; // only the frame size excluding 2 byte header
	tx->PayLoad =  0; // payload is part of the packet
	tx->StatusWord = 0; // changes from 0 to the status info

	return tx;
}


/*
 * HandleControlMessage
 * @ *pBuf: the pointer of BF609 control frame
 */
void HandleControlMessage(void *pBuf)
{
	ADI_ETHER_BUFFER *pSend;

	BF609_COMM_ACK_CODE ret;

	FORWARD_ETHER_FRAME *pRxData; 	//Ethernet data ptr
	CONTROL_FRAME    *pCtrlData;	//control data ptr
	uint8_t Ctr_Cmd;
	uint32_t LDR_Len;

	pRxData   = (FORWARD_ETHER_FRAME *)pBuf;

	uint8_t  BF609_Cmd = pRxData->CtrlField;

	switch(BF609_Cmd)
	{
		case BF609_CTR://0x00，控制信息帧
		{
			 pCtrlData = (CONTROL_FRAME *)(pRxData->PktData);
			 Ctr_Cmd  = pCtrlData->CtrCmd;

			 if(Ctr_Cmd == VERSION_UPDATE) 		// Update version
			 {
				 ret = ProcessUpdateVersion(pCtrlData, &LDR_Len );

				 if( ret == ACK_OK)
				 {
					 pSend = PackACKFrmOfUpdateVerion ( ret, pBuf, &user_net_config_info[1] );
					 EtherSend ( g_hDev[1], pSend );

					 while(g_ACKOK_XMT_Completed == 0)
					 {
						 asm("nop;");
					 }

					 adi_osal_EnterCriticalRegion();
					 FlashErase(g_pLdrDataBuff, LDR_Len);
					 adi_osal_ExitCriticalRegion();
				 }
				 else if(ret != ACK_FRM_OK)
				 {
					 //NAK, send the ACK of control info
					 restart_transfers(g_hDev[0]);

					 pSend = PackACKFrmOfUpdateVerion ( ret, pBuf, &user_net_config_info[1] );
					 EtherSend ( g_hDev[1], pSend );
				 }
			 }//VERSION_UPDATE
			 else if( Ctr_Cmd == VERSION_GET)
			 {
				 DEBUG_STATEMENT ( " HandleControlMessage: read version!\n\n " );
				 pSend = PackACKFrmOfReadVersion (  pBuf, &user_net_config_info[1] );
				 EtherSend ( g_hDev[1], pSend );
			 }

			break;
		}//BF609_CTR
		default:
			ret = NAK_ERROR_UNKNOWN_COMMAND;
			break;
	}//Ctr_Cmd

	return ;
}



static BF609_COMM_ACK_CODE ProcessUpdateVersion(const CONTROL_FRAME *pCtrlDataBuf,
		uint32_t* pLDR_Len )
{
	 // get the ldr info in frame
	const CONTROL_FRAME    *pCtrData = pCtrlDataBuf;	//control data ptr
	LDR_FRAME        *pLdrFrame;	//LDR
	uint8_t 		 *pLdrPartData;


	uint8_t  CheckSum;

	uint32_t DataLen;
	uint32_t LDR_Index;
	uint32_t LDR_Len;
	uint16_t CRC16 ;


	pLdrFrame    = (LDR_FRAME *)(pCtrData->CtrData);
	DataLen     = pLdrFrame->DataLen;
	LDR_Index   = pLdrFrame->LDR_Index;

	// get the length of ldr file, and malloc memory to save
	*pLDR_Len = pLdrFrame->LDR_Len;
	LDR_Len = pLdrFrame->LDR_Len;

	if( LDR_Index == 0 )
	{
		 //stop EMAC0
		stop_transfers( g_hDev[0] );
		DEBUG_STATEMENT ( " ProcessUpdateVersion: stop EMAC0!\n\n " );
	}

	if( LDR_Index > LDR_Len )
	{
		return NAK_ERROR_INDEX;
	}

	if(g_pLdrDataBuff == NULL)  // malloc only in the first time
	{
		g_pLdrDataBuff = (uint8_t *)heap_malloc(LDR_DATA_BUFFER_LOCATE_HEAP, LDR_Len);

		if(g_pLdrDataBuff == NULL)
		{
			return NAK_ERROR_MEM_ALLOC_FAILED;
		}
	}

	//check the checksum,from MU_addr to last byte of data field.
	pLdrPartData = (uint8_t *)(pLdrFrame->PartData);
	CheckSum   = GetCheckSum8(0, (uint8_t *)pLdrFrame, sizeof(*pLdrFrame)) +\
					   GetCheckSum8(0, pLdrPartData, DataLen);
	CheckSum = CheckSum + pCtrData->MU_Addr + pCtrData->CtrCmd;

	//copy frame data to update buffer if CheckSum is correct.
	if( CheckSum == pLdrPartData[DataLen])
	{
		memcpy( g_pLdrDataBuff + LDR_Index, pLdrPartData, DataLen);
	}
	else
	{
		return NAK_ERROR_FRM_CHKSUM;
	}

	//last frame,completed receive LDR file data,erase the flash
	if(DataLen == (LDR_Len-LDR_Index))
	{
		 CRC16 = GetCrc16(0, g_pLdrDataBuff, LDR_Len);
		 if(CRC16 == (pLdrFrame->LDR_CRC)) //erase flash if CRC correct
		 {
			 return ACK_OK;
		 }
		 else
		 {
			 return NAK_ERROR_FILE_CRC;
		 }
	}

	return ACK_FRM_OK;
}

/*
 * CRC16 calculate (int start, char *p, int n)
 */
uint16_t GetCrc16(int start,unsigned char *p, int n)
{
  int crc = start;
  register int r;
  static uint16_t crc_16_table[16] = {
	  0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
	  0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400
	};

   /* while there is more data to process */
   while (n-- > 0)
   {
     /* compute checksum of lower four bits of *p */
     r = crc_16_table[crc & 0xF];
     crc = (crc >> 4) & 0x0FFF;
     crc = crc ^ r ^ crc_16_table[*p & 0xF];

     /* now compute checksum of upper four bits of *p */
     r = crc_16_table[crc & 0xF];
     crc = (crc >> 4) & 0x0FFF;
     crc = crc ^ r ^ crc_16_table[(*p >> 4) & 0xF];

     /* next... */
     p++;
   }

   return(crc);
}
/*
 * calculate the CheckSum
 */
uint8_t GetCheckSum8(int start, uint8_t *pucData, int len)
{
	uint8_t CheckSum = 0;
	int i = 0;
	for(i = start; i < (start + len); i++)
	{
		CheckSum += pucData[i];
	}

	return CheckSum;
}
