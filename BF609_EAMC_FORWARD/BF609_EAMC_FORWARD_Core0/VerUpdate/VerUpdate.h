/*
 * VerUpdate.h
 *
 *  Created on: 2014-8-27
 *      Author: ChiliWang
 */

#ifndef VERUPDATE_H_
#define VERUPDATE_H_

#define LDR_DATA_BUFFER_LOCATE_HEAP 1  //define the ldr data buffer area in heap.
/*
 * update the version
 * @ *pBuf: the pointer of Ethernet frame
 */
BF609_COMM_ACK_CODE VerUpdate(void *pBuf);

#endif /* VERUPDATE_H_ */
