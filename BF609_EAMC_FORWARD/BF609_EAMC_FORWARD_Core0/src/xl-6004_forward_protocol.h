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

/*
 * BF609 Ethernet command Macro
 */
#define BF609_CTR              0X00  //0x00��������Ϣ֡
#define BF609_FORWARD_SMV_PC   0X01  //0x01������ʵʱ��̫���ӿڽ��յ�����
#define BF609_FORWARD_FT3_PC   0X02  //0x02�����͹⴮�ڽ�������
#define BF609_FORWARD_GOOSE_PC 0X03  //0x03�����Ϳ���������ӿ�����
#define BF609_FORWARD_PC_ETHE  0x81  //0x81��ת�����ݵ�ʵʱ��̫���ӿ�
#define BF609_FORWARD_PC_GOOSE 0x82  //0x82��ת�����ݵ��⴮�����
/*
 * control frame command macro
 */
#define VERSION_UPDATE     0X01
#define VERSION_GET        0X02
#define MODULE_CONFIG_SET  0X03
#define MODULE_CONFIG_GET  0X04

/*
 * other macro
 */
#define BF609_FORWARD_SMV_TYPE_LO     0X06
#define BF609_FORWARD_SMV_TYPE_HI        0XFE
#define BF609_UPDATE_VER_ACKOK_TYPE_LO     0X06
#define BF609_UPDATE_VER_ACKOK_TYPE_HI        0XFD
#define BF609_UPDATE_VER_NAK_TYPE_LO     0X06
#define BF609_UPDATE_VER_NAK_TYPE_HI        0XFC

#define BF609_UPDATE_VER_TYPE_LO     0X06
#define BF609_UPDATE_VER_TYPE_HI        0XFB
#define BF609_READ_VER_TYPE_LO     0X06
#define BF609_READ_VER_TYPE_HI        0XFA
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
 * �ϲ���ԪУ��̨����̫�����ݸ�ʽ
 */
#pragma pack(1) //Э��֡�ϸ���Э��������ֹ�ڴ����
/* \struct FORWADRD_ETHER_FRAME
 *
 * Structure map the ethernet forward frame
 */
/*
 * ����		����		��ע
   MAC��ʶ	6�ֽ�
	ʱ��		4�ֽ�		��λns����λ��ǰ
	��ַ����	1�ֽ�
	������	1�ֽ�
	����		2�ֽ�	             ת��SVʱ������Ϊ0X06fe
	��������	<1500�ֽ�   �������ݸ�ʽ
 *
 * */
typedef struct FORWARD_ETHER_FRAME
{
	uint16_t NoBytes;
	uint8_t  DestMAC[6];    	 /*  MAC Address */
	uint32_t TimeStamp;  	/*  time stamp */
	uint8_t  MUAddr;    	/*  MU �ĵ�ַ���� 	*/
	uint8_t  CtrlField; 		/*  ���� ��              	*/
	uint8_t  LTfield[2]; 	/*  Reserved for length/type field ,  ת��SMVʱ������Ϊ0X06fe */
	uint8_t  PktData[1500];   		/*  ����ָ��          	*/
}FORWARD_ETHER_FRAME;

/*
 * ������Ϣ֡��ʽ����
 * MAC: 01,02,03,04,05,06
 * CtrCommand��0x00
 * pData: �ṹ��������
 */
typedef struct CONTROL_FRAME
{
	uint8_t StardCode;   /* ֡ͷ��0x68*/
	uint8_t LenLo;       /* ��֡ͷ��֡β�ĳ��ȵĸߵ��ֽڣ���0x68-ox16���ֽ���*/
	uint8_t LenHi;
	uint8_t MarkCode;    /* 0x68   */

	uint8_t  MU_Addr;    /* legacy��0*/
	uint8_t  CtrCmd;    /* ������ */
	uint8_t  CtrData[0];      /* ���ݣ���������*/

	uint8_t CheckSum;    /* У��ͣ��� MU_Addr-����������һ���ֽ� ��8λУ���*/
	uint8_t EndCode;     /* ֡β��0x16 */
}CONTROL_FRAME;


/*
 * CtrCommand �� 0x01 �����������
 * ������µ�֡��ʽ����
 */
typedef struct LDR_FRAME
{
	uint32_t LDR_Len;   	/* LDR �ļ������ֽ���  */
	uint16_t LDR_CRC;  	 	/* ����LDR�ļ���CRCУ���� */
	uint32_t LDR_Index; 	/* ��ǰ���ݰ�LDRƫ��λ�� */

	uint16_t DataLen;       /* ��ǰ���ݰ������ݳ��� */
	uint8_t  PartData[0];   		/* ��ǰ���ݰ�����������  */
}LDR_FRAME;
#pragma pack() //�ָ�Ĭ�ϵ��ڴ��ֽڶ���

extern FORWARD_ETHER_FRAME board_info;


#ifdef __cplusplus
extern "C"  {
#endif

void HandleControlMessage(void *pBuf);

ADI_ETHER_BUFFER *PackForwardSMVFrame ( uint32_t unNanoSecond, char *SMVFrame,
		uint16_t SmvFrmLen, ETH_CFG_INFO *bsInfo );

void PackForwardSMVFrmHeader ( void *pForwardFrmHeader,
		uint32_t unNanoSecond,
		uint16_t unPktDataLen );

ADI_ETHER_BUFFER *PackACKFrmOfUpdateVerion ( BF609_COMM_ACK_CODE AckCode,
													void *pCtrlInfoFrmBuf,
													ETH_CFG_INFO *bsInfo );

uint8_t GetCheckSum8(int start, uint8_t *pucData, int len);

uint16_t GetCrc16(int start,unsigned char *p, int n);

#ifdef __cplusplus
}
#endif

#endif /* XL_6004_FORWARD_PROTOCOL_H_ */
