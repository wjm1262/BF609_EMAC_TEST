/*
* xl-6004_forward_protocol.h
*
*  Created on: 2014-7-18
*      Author: Wu JM
*/

#ifndef XL_6004_FORWARD_PROTOCOL_H_
#define XL_6004_FORWARD_PROTOCOL_H_


#include <stdio.h>
#include <string.h>
#include "mem_manager.h"

extern volatile int g_ACKOK_XMT_Completed ;
extern char VersionString[128] ;
/*
 * BF609 Ethernet command Macro
 */
#define BF609_CTR              0X00  //0x00，控制信息帧
#define BF609_FORWARD_SMV_PC   0X01  //0x01，发送实时以太网接口接收到数据
#define BF609_FORWARD_FT3_PC   0X02  //0x02，发送光串口接收数据
#define BF609_FORWARD_GOOSE_PC 0X03  //0x03，发送开关量输入接口数据
#define BF609_FORWARD_PC_ETHE  0x81  //0x81，转发数据到实时以太网接口
#define BF609_FORWARD_PC_GOOSE 0x82  //0x82，转发数据到光串口输出
/*
 * control frame command macro
 */
#define VERSION_UPDATE     0X01
#define VERSION_GET        0X02
#define MODULE_CONFIG_SET  0X03
#define MODULE_CONFIG_GET  0X04

/*
 * Ethernet forward frame type macro define,
 */
/*
Typed Ethernet frames are frames with a length/type field that takes the type interpretation
because it is greater than or equal to 0x600 (1536).
*/
#define BF609_FORWARD_SMV_TYPE_LO     0X06
#define BF609_FORWARD_SMV_TYPE_HI_BASE        0XF0
/* in order to distinguish the FORWARD SMV Messages from different BF609_BOARDs,
 * the forward frame type is based on the BF609_BOARDs' MUAddr (see struct FORWARD_ETHER_FRAME
 *  definition ), such as,
 *  LTfield[0] = BF609_FORWARD_SMV_TYPE_LO;
 *  LTfield[1] = BF609_FORWARD_SMV_TYPE_HI_BASE + MUAddr;
 *  NOTES: the upper limit of forward frame type is 0x06ff
 */

#define BF609_UPDATE_VER_ACKOK_TYPE_LO     0X06
#define BF609_UPDATE_VER_ACKOK_TYPE_HI        0XeD
#define BF609_UPDATE_VER_NAK_TYPE_LO     0X06
#define BF609_UPDATE_VER_NAK_TYPE_HI        0XeC

#define BF609_UPDATE_VER_TYPE_LO     0X06
#define BF609_UPDATE_VER_TYPE_HI        0XeB
#define BF609_READ_VER_TYPE_LO     0X06
#define BF609_READ_VER_TYPE_HI        0XeA
/*
 * the ERROR Code for Negative Acknowledge (NAK)
 */
typedef enum BF609_COMM_ACK_CODE
{
	//positive acknowledgement
	ACK_OK = 0X00,
	ACK_FRM_OK =0X01,

	//Negative Acknowledge (NAK)
	NAK_ERROR_MEM_ALLOC_FAILED =0X10,
	NAK_ERROR_INDEX = 0X11,
	NAK_ERROR_FRM_CHKSUM = 0X12,
	NAK_ERROR_FILE_CRC = 0X13,
	NAK_ERROR_UNKNOWN_COMMAND=0X14,
	NAK_ERROR_DESTADDR_UNMATCH=0X15,
}BF609_COMM_ACK_CODE;



/*
 * 合并单元校验台的以太网数据格式
 */
#pragma pack(1) //协议帧严格按照协议来，禁止内存对齐
/* \struct FORWADRD_ETHER_FRAME
 *
 * Structure map the ethernet forward frame
 */
/*
 * 类型		长度		备注
   MAC标识	6字节
	时标		4字节		单位ns，低位在前
	地址编码	1字节
	控制域	1字节
	保留		2字节	             转发SV时，类型为0X06fe
	网络数据	<1500字节   网络数据格式
 *
 * */
typedef struct FORWARD_ETHER_FRAME
{
	uint16_t NoBytes;
	uint8_t  DestMAC[6];    	 /*  MAC Address */
	uint32_t TimeStamp;  	/*  time stamp */
	uint8_t  MUAddr;    	/*  MU 的地址编码 	*/
	uint8_t  CtrlField; 		/*  控制 域              	*/
	uint8_t  LTfield[2]; 	/*  Reserved for length/type field ,  转发SMV时，类型为0X06fe */
	uint8_t  PktData[1500];   		/*  数据指针          	*/
}FORWARD_ETHER_FRAME;

/*
 * 控制信息帧格式定义
 * MAC: 01,02,03,04,05,06
 * CtrCommand：0x00
 * pData: 结构定义如下
 */
typedef struct CONTROL_FRAME
{
	uint8_t StardCode;   /* 帧头，0x68*/
	uint8_t LenLo;       /* 从帧头到帧尾的长度的高低字节，从0x68-ox16的字节数*/
	uint8_t LenHi;
	uint8_t MarkCode;    /* 0x68   */

	uint8_t  MU_Addr;    /* legacy，0*/
	uint8_t  CtrCmd;    /* 控制域 */
	uint8_t  CtrData[0];      /* 数据，柔性数组*/

	uint8_t CheckSum;    /* 校验和，从 MU_Addr-数据域的最后一个字节 的8位校验和*/
	uint8_t EndCode;     /* 帧尾，0x16 */
}CONTROL_FRAME;


/*
 * CtrCommand ： 0x01 程序更新命令
 * 程序更新的帧格式定义
 */
typedef struct LDR_FRAME
{
	uint32_t LDR_Len;   	/* LDR 文件的总字节数  */
	uint16_t LDR_CRC;  	 	/* 整个LDR文件的CRC校验码 */
	uint32_t LDR_Index; 	/* 当前数据包LDR偏移位置 */

	uint16_t DataLen;       /* 当前数据包的数据长度 */
	uint8_t  PartData[0];   		/* 当前数据包，柔性数组  */
}LDR_FRAME;
#pragma pack() //恢复默认的内存字节对齐

extern FORWARD_ETHER_FRAME board_info;


#ifdef __cplusplus
extern "C"  {
#endif

void HandleControlMessage(void *pBuf);

/* CreateForwardSMVFrame: copy the data from SMVFrame to a new buffer, so uses 'memcpy',
 * 						which is high time consumption.(for 30000ns)
 *
 * PackForwardSMVFrame: there are 14 bytes reserved at the head of original recv
 * 						buffer (see 'set_descriptor' ), so, only to need pack ForwardSMVFrmHeader
 * 						at the reserved room,which is low time consumption.(for 1600ns)
 */

//NOTES: if the SMVFrame more than 1500 bytes, then it is been cut off to 1500 bytes.
ADI_ETHER_BUFFER *CreateForwardSMVFrame ( uint32_t unNanoSecond, char *SMVFrame,
		uint16_t SmvFrmLen, ETH_CFG_INFO *bsInfo );

ADI_ETHER_BUFFER *PackForwardSMVFrame( uint32_t unNanoSecond, ADI_ETHER_BUFFER *pSrcEthBuf,
		uint16_t SmvFrmLen );



ADI_ETHER_BUFFER *PackACKFrmOfUpdateVerion ( BF609_COMM_ACK_CODE AckCode,
													void *pCtrlInfoFrmBuf,
													ETH_CFG_INFO *bsInfo );

uint8_t GetCheckSum8(int start, uint8_t *pucData, int len);

uint16_t GetCrc16(int start,unsigned char *p, int n);

#ifdef __cplusplus
}
#endif

#endif /* XL_6004_FORWARD_PROTOCOL_H_ */
