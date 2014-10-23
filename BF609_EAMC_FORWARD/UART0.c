/*
 * UART_Init.c
 *
 *  Created on: 2014-4-15
 *      Author: Administrator
 */
/*
 * UART initialization.
 * 1, #include <services\pwr\adi_pwr.h>
 * 2, #include <drivers\uart\adi_uart.h>
 * 3, #include <stdio.h>
 * 4, #define BAUD_RATE and #define the CLK value as default.
 * 5, Define ADI_UART_HANDLE and the memory used by the UART.
 * 6, Setting the pin multiplex in system.svc.
 */
#include "UART0.h"
#include <stdio.h>
/* default power settings */
#define MHZTOHZ       (1000000)
#define DF_DEFAULT    (0x0)
#define MSEL_DEFAULT  (0x10)
#define SSEL_DEFAULT  (0x8)
#define CSEL_DEFAULT  (0x4)

#define myCLKIN         (25 * MHZTOHZ)   /* should corresponding to the fact input osc. */
#define myCORE_MAX      (500 * MHZTOHZ)
#define mySYSCLK_MAX    (250 * MHZTOHZ)
#define mySCLK_MAX      (125 * MHZTOHZ)
#define myVCO_MIN       (72 * MHZTOHZ)
#define STDIO_UART_PHYSICAL_DEV_ID 0
#define BAUD_RATE 9600
static ADI_STDIO_DEVICE_HANDLE hSTDIOJTAG = NULL;
static ADI_STDIO_DEVICE_HANDLE hSTDIOUART = NULL;
static ADI_UART_HANDLE hPhysicalUART = NULL;
ADI_UART_HANDLE UART0_Init(void)
{
    /* STDIO return code */
    ADI_STDIO_RESULT       eResult     = ADI_STDIO_RESULT_SUCCESS;
    /* DDSS return code */
    ADI_UART_RESULT               nResult     = ADI_UART_SUCCESS;
    /* Initialize Power service */
    if(adi_pwr_Init(myCLKIN, myCORE_MAX, mySYSCLK_MAX, myVCO_MIN) != ADI_PWR_SUCCESS)
    {
    	printf("0x%08X :Failed to initialize power service \n", eResult);
    }
    /*
     * Initialize UART
     */
    /* Initialize the STDIO service */
    do{
    if((eResult = adi_stdio_Init(                     /* DCB Manager handle */
                                 &hSTDIOJTAG
                                 )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to initialize STDIO service, Error Code: 0x%08X\n",eResult);
        break;
    }

    /* Register UART device type with STDIO service */
    adi_stdio_RegisterUART();

    printf("\n\nOpening UART Device for STDIO ... \n\n");


    /* Open UART Device for STDIO*/
    if((eResult = adi_stdio_OpenDevice (ADI_STDIO_DEVICE_TYPE_UART,  /* Device Type       */
                                        STDIO_UART_PHYSICAL_DEV_ID,  /* Device Number     */
                                        &hSTDIOUART                  /* Pointer to Handle */
                                        )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to Open UART Device, Error Code: 0x%08X\n",eResult);
        break;
    }

    /* Set charecter echo for UART device */
    if((eResult = adi_stdio_ControlDevice (hSTDIOUART,                          /* Device Handle     */
                                           ADI_STDIO_COMMAND_ENABLE_CHAR_ECHO,  /* Command Type      */
                                           (void *) true                        /* Command Value     */
                                           )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to set character echo for UART Device, Error Code: 0x%08X\n",eResult);
        break;
    }

    /* Redirect stream to UART */
    printf("Redirecting streams to UART, see the next message on UART terminal\n");
    if((eResult = adi_stdio_RedirectStream (hSTDIOUART,                         /* Device Handle     */
                                            ADI_STDIO_STREAM_ALL_CONSOLE_IO     /* Stream Type       */
                                            )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to Redirect all streams to UART Device, Error Code: 0x%08X\n",eResult);
        break;
    }

    /* Get physical device handle for UART */
    if((eResult = adi_stdio_ControlDevice (hSTDIOUART,
                                           ADI_STDIO_COMMAND_GET_DEVICE_HANDLE,
                                           &hPhysicalUART
                                           )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to retrieve physical device handle for UART, Error Code: 0x%08X\n",eResult);
        break;
    }


    /* Set baud rate for UART device */
    if((eResult = adi_stdio_ControlDevice (hSTDIOUART,                            /* Device Handle     */
                                           ADI_STDIO_COMMAND_SET_UART_BAUD_RATE,  /* Command Type      */
                                           (void *)BAUD_RATE                             /* Command Value     */
                                           )) != ADI_STDIO_RESULT_SUCCESS)
    {
        printf("Failed to set baud rate for UART Device, Error Code: 0x%08X\n",eResult);
        break;
    }


    if ((nResult = adi_uart_SetFCPolarity (hPhysicalUART,
                                   (void *) true
                                   )) != ADI_UART_SUCCESS)
    {
        printf("Failed to set ADI_UART_CMD_ENABLE_CTS_RTS for UART Device, Error Code: 0x%08X\n",eResult);
        break;
    }
   }while(0);
    printf("redirect UART is OK.\n");
    return hPhysicalUART;
}

