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
 * �ϲ���ԪУ��̨����̫�����ݸ�ʽ
 */
#pragma pack(1) //Э��֡�ϸ���Э��������ֹ�ڴ����
typedef struct MU_ETHE_FRAME{
	uint16_t forhead;
	uint8_t  MAC[6];    	 /*  MAC Address */
	uint32_t TimeStamp;  	/*  time stamp */
	uint8_t  MU_Addr;    	/*  MU �ĵ�ַ���� 	*/
	uint8_t  CtrCmd; 		/*  �������� ��              	*/
	uint8_t  reserver[2]; 	/*  ����                    	*/
	uint8_t  Data[1500];   		/*  ����ָ��          	*/

}BF609_ETHE_FRAME;

/*
 * ������Ϣ֡��ʽ����
 * MAC: 01,02,03,04,05,06
 * CtrCommand��0x00
 * pData: �ṹ��������
 */
typedef struct CONTROL_FRAME{
	uint8_t StardCode;   /* ֡ͷ��0x86*/
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
typedef struct LDR_FRAME{
	uint32_t LDR_Len;   	/* LDR �ļ������ֽ���  */
	uint16_t LDR_CRC;  	 	/* ����LDR�ļ���CRCУ���� */
	uint32_t LDR_Index; 	/* ��ǰ���ݰ�LDRƫ��λ�� */

	uint16_t DataLen;       /* ��ǰ���ݰ������ݳ��� */
	uint8_t  PartData[0];   		/* ��ǰ���ݰ�����������  */
}LDR_FRAME;
#pragma pack() //�ָ�Ĭ�ϵ��ڴ��ֽڶ���
#endif /* BF609_UPDATE_PROTOCOL_H_ */
