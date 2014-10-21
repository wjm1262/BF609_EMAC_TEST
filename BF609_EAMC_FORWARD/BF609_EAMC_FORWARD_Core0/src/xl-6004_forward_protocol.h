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

#ifdef __cplusplus
extern "C"  {
#endif


	/* \struct FORWADRD_ETHER_FRAME_BUFFER
	 *
	 * Structure map the ethernet MAC frame
	 */
	typedef struct FORWARD_ETHER_FRAME_BUFFER
	{
		uint8_t      Dest[6];               /*!< destination MAC address  */
		uint8_t      tmStamp[4];               /*!< time stamp of the Frame header received  */
		uint8_t		 DA;			/*±íÎ»±àÂë */
		uint8_t		Ctrl;			/* ¿ØÖÆÓò*/
		uint8_t      LTfield[2];            /*Reserved for length/type field    */
	} FORWARD_ETHER_FRAME_BUFFER;
	
	FORWARD_ETHER_FRAME_BUFFER board_info =
	{
		{0x06, 0x05, 0x04, 0x03, 0x02, 0x01},
		{0},
		0x04,
		0x01,
		{0x06, 0xfe}
	};
	#pragma inline
	void PackForwardFrmHeader ( FORWARD_ETHER_FRAME_BUFFER *pHeader, char *tm )
	{
		memcpy ( pHeader->Dest, board_info.Dest , 6 );
		
		memcpy ( pHeader->tmStamp, tm, 4 );
		
		pHeader->DA = board_info.DA;//
		pHeader->Ctrl = board_info.Ctrl;
		pHeader->LTfield[0] = board_info.LTfield[0];
		pHeader->LTfield[1] = board_info.LTfield[1];
	}

	
#ifdef __cplusplus
}
#endif


#endif /* XL_6004_FORWARD_PROTOCOL_H_ */
