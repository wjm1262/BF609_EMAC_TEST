/*****************************************************************************
* BF609_EMAC_Test_Core0.h
*****************************************************************************/

#ifndef __BF609_EMAC_TEST_CORE0_H__
#define __BF609_EMAC_TEST_CORE0_H__


/* Add your custom header content here */
#include <sys/exception.h>
#include <drivers/ethernet/adi_ether.h>

#include <stdio.h>
#include <string.h>
#include <cplb.h>

#include <stddef.h>

#include "adi_gemac_int.h"

#define MIN_ETHER_FRAME_LEN (64)
#define MAX_ETHER_FRAME_LEN (1500)

// check for four byte alignment
#define MY_CHECK_ALIGNMENT(x) (((x+3)&(~0x3)) == x)

#define COPY_SVFRM 1



/***************************************************************
 *
 * ADSP-BF60x family configuration
 */
#if defined( __ADSPBF60x__)
#include <adi_osal.h>
#include <drivers/ethernet/adi_ether_gemac.h>
#include <services/int/adi_int.h>
#include <services/int/adi_sec.h>

#if defined(__ADSPBF609__)
#include <cdefbf609.h>
#include <defbf609.h>
#elif defined(__ADSPBF608__)
#include <cdefbf608.h>
#include <defbf608.h>
#elif defined(__ADSPBF607__)
#include <cdefbf607.h>
#include <defbf607.h>
#elif defined(__ADSPBF606__)
#include <cdefbf606.h>
#include <defbf606.h>
#endif

//   #define EMAC_DRIVER_ENTRY (GEMAC1DriverEntry) /* EMAC0 Driver Entry point */



#endif /* __ADSPBF60x__ */
/***********************************************************/

extern ADI_ETHER_HANDLE 		g_hDev[MAX_NETWORK_IF];

#ifdef __cplusplus
extern "C"  {
#endif



/*! Enters critical region */
#define ENTER_CRITICAL_REGION()  (adi_osal_EnterCriticalRegion())
/*! Exit critical region */
#define EXIT_CRITICAL_REGION()   (adi_osal_ExitCriticalRegion())


/*********************** user-defined Functions  ******************************************/
typedef  void ( *ETHERNET_CALLBACK_FUN ) ( void *arg1, unsigned int event, void *FrameBuffers );

void Ethernet0_Callback ( void *arg1, unsigned int event, void *FrameBuffers );
void Ethernet1_Callback ( void *arg1, unsigned int event, void *FrameBuffers );



/*******************************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __BF609_EMAC_TEST_CORE0_H__ */
