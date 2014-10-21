/*
 * BF609_Update_Protocol.h
 *
 *  Created on: 2014-8-11
 *      Author: ChiliWang
 */

#ifndef BF609_UPDATE_PROTOCOL_H_
#define BF609_UPDATE_PROTOCOL_H_
#include <stdint.h>
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
 * the ERROR Code for Negative Acknowledge (NAK)
 */
typedef enum BF609_COMM_ACK_CODE
{
	//positive acknowledgement
	ACK_OK = 0X00,
	//Negative Acknowledge (NAK)
	NAK_ERROR_INDEX = 0X01,
	NAK_ERROR_FRM_CHKSUM = 0X02,
	NAK_ERROR_FILE_CRC = 0X03,
	NAK_ERROR_UNKNOWN_COMMAND=0X04,
	NAK_ERROR_DESTADDR_UNMATCH=0X05,
}BF609_COMM_ACK_CODE;



/*
 * 合并单元校验台的以太网数据格式
 */
#pragma pack(1) //协议帧严格按照协议来，禁止内存对齐
typedef struct MU_ETHE_FRAME{
	uint16_t forhead;
	uint8_t  MAC[6];    	 /*  MAC Address */
	uint32_t TimeStamp;  	/*  time stamp */
	uint8_t  MU_Addr;    	/*  MU 的地址编码 	*/
	uint8_t  CtrCmd; 		/*  控制命令 域              	*/
	uint8_t  reserver[2]; 	/*  保留                    	*/
	uint8_t  Data[1500];   		/*  数据指针          	*/

}BF609_ETHE_FRAME;

/*
 * 控制信息帧格式定义
 * MAC: 01,02,03,04,05,06
 * CtrCommand：0x00
 * pData: 结构定义如下
 */
typedef struct CONTROL_FRAME{
	uint8_t StardCode;   /* 帧头，0x86*/
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
typedef struct LDR_FRAME{
	uint32_t LDR_Len;   	/* LDR 文件的总字节数  */
	uint16_t LDR_CRC;  	 	/* 整个LDR文件的CRC校验码 */
	uint32_t LDR_Index; 	/* 当前数据包LDR偏移位置 */

	uint16_t DataLen;       /* 当前数据包的数据长度 */
	uint8_t  PartData[0];   		/* 当前数据包，柔性数组  */
}LDR_FRAME;
#pragma pack() //恢复默认的内存字节对齐
#endif /* BF609_UPDATE_PROTOCOL_H_ */
