/*
 * (C) Copyright 2012 - Analog Devices, Inc. All Rights Reserved.
 *
 * This software is proprietary and confidential.  By using this software
 * you agree to the terms of the associated Analog Devices License Agreement.
 *
 * Project Name:  	Power_On_Self_Test
 *
 * Hardware:		ADSP-BF609 EZ-Board
 *
 * Description:	This file initializes and tests the UART on the BNF609 EZ-Board.
 */

#include <cdefbf609.h>
#include <ccblkfn.h>
#include "post_debug.h"
//#include "post_common.h"
#include "timer_isr.h"

#if defined(__DEBUG_FILE__)
#include <string.h>
extern FILE *pDebugFile;				/* debug file when directing output to a file */
#endif

#define MAX_TEST_CHARS		5000

/* if we are using the UART to print debug info, define the following */
#ifdef __DEBUG_UART__
#define UART_DEBUG_BUFFER_LINE_SIZE 256
char UART_DEBUG_BUFFER[UART_DEBUG_BUFFER_LINE_SIZE];
#endif

/*******************************************************************
*  function prototypes
*******************************************************************/
void Init_UART(void);
int PutChar(const char c);
int GetChar(char *const c);
int TEST_UART(void);
void ClearReceiver(void);

/* if we are using the UART to print debug info, define the following */
#ifdef __DEBUG_UART__
int UART_DEBUG_PRINT(void);
#endif

//extern volatile bool g_button_1_pushed;

/*
 *   Function:    Init_UART
 *   Description: Initialize UART with the appropriate values
 */
void Init_UART(void)
{
	static bool bInitialized = false;

	*pREG_PORTD_FER_SET |= 0x180;
	ssync();

	/* configure UART0 RX and UART0 TX pins */
	*pREG_PORTD_MUX |= 0x14000;
	ssync();

	/*****************************************************************************
	 *
	 *  First of all, enable UART clock, put in UART mode, word length 8.
	 *
	 ****************************************************************************/
	*pREG_UART0_CTL = 0x301;//data:8bit, stop:1, parity:no


	/* Bit Rate = 		   SCLK
					-------------------				EDBO is a 1
					[16^(1-EDBO)] * Div

			   ##################################

					125 * 1000000
				  -------------------  = 9600.6
					 [16^0] * 13021
	*/

	//*pREG_UART0_CLK = 0x800032DD;
//	*pREG_UART0_CLK = 0x80005161;//SCLK:200M,9600
//	*pREG_UART0_CLK = 0x80008555;//SCLK,163.84m,4800
	*pREG_UART0_CLK = 0x8000058e;//SCLK,163.84m,115200

	/* reroute tx/rx interrupts to status interrupt output */
	*pREG_UART0_IMSK_SET = 0x180;

}


/*
 *   Function:    PutChar
 *   Description: Writes a character to the UART.
 */
int PutChar(const char cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( (*pREG_UART0_STAT & 0x20) )
			{
				*pREG_UART0_THR = cVal;
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}


/*
 *   Function:    ClearReceiver
 *   Description: Clears the receive FIFO
 */
void ClearReceiver(void)
{
	int nStatus = 0;
	char temp;

	while( *pREG_UART0_STAT & 0x1 )
	{
		/* Anomaly 16000030 workaround */
		unsigned int uiTIMASK = cli();
		temp = *pREG_UART0_RBR;
		sti(uiTIMASK);
	}

	*pREG_UART0_STAT = 0xFFFFFFFF;

}


/*
 *   Function:    GetChar
 *   Description: Reads a character from the UART.
 */
int GetChar(char *const cVal)
{
	int nStatus = 0;
	unsigned int nTimer = SetTimeout(1000);
	if( ((unsigned int)-1) != nTimer )
	{
		do{
			if( *pREG_UART0_STAT & 0x1 )
			{
				/* Anomaly 16000030 workaround */
				unsigned int uiTIMASK = cli();
				*cVal = *pREG_UART0_RBR;
				sti(uiTIMASK);
				nStatus = 1;
				break;
			}
		}while( !IsTimedout(nTimer) );
	}

	ClearTimeout(nTimer);

	return nStatus;
}

#if 0
int TEST_UART( void )
{
	int n, i;
	char cTxChar;
	char cRxChar;
	bool bPB2 = false;	/* flags to see if button was pushed */

	DEBUG_HEADER( "UART Test" );

#ifdef __DEBUG_UART__
	DEBUG_STATEMENT( "\nTest settings:" );
	DEBUG_STATEMENT( "\n   Install loopback cable and press PB2\n" );
	DEBUG_STATEMENT( "\n   When LEDs indicate test pass, remove loopback cable\n" );

	while( !bPB2)
	{
		idle();

		/* was pb2 pushed? */
		if( g_button_1_pushed && !bPB2 )
		{
			bPB2 = true;
		}
	}
#endif

	g_button_1_pushed = false;

	/*
	 * When using __DEBUG_UART__ do the init again to get ready
	 * for the loopback
	 *
	 * Otherwise init for the first time
	 */
	Init_UART();

	/*
	 * clear our RX buffer in case any debug info was
	 * looped back
	 */
	ClearReceiver();

	for(n = 0; n < MAX_TEST_CHARS; n++)
	{
		cTxChar = (n & 0xFF);

		/* write a char */
		if( 0 == PutChar(cTxChar) )
		{
			return( test_fail() );
		}

#ifdef __DEBUG_UART__
		/* need a delay between writing and reading the first byte if there's
			already data in the buffer from using UART for debug info */
		if ( 0 == n )
		{
			for (i = 0; i < 300000; i++)
				NOP;
		}
#endif

		/* read a char */
		if( 0 == GetChar( &cRxChar ) )
		{
			return( test_fail() );
		}

		/* it should match */
		if( cTxChar != cRxChar )
		{
			return( test_fail() );
		}
	}

	/* return status */
	return( test_pass() );
}

#endif


#ifdef __DEBUG_UART__
/*
 *   Function:    UART_DEBUG_PRINT
 *   Description: Prints debug info over the UART using a predefined
*				 buffer.
 */
int UART_DEBUG_PRINT(void)
{
	unsigned int i = 0;		/* index */
	char temp;				/* temp char */


	/* loop through the debug buffer until the end, a NULL, or an error */
	for ( i = 0; i < UART_DEBUG_BUFFER_LINE_SIZE; i++)
	{
		temp = UART_DEBUG_BUFFER[i];

		/* if not NULL then print it */
		if (temp)
		{
			if( 0 == PutChar(temp) )
			{
				/* if error was detected then quit */
				return 0;
			}

			/* if it was a newline we need to add a carriage return */
			if ( 0x0a == temp )
			{
				if( 0 == PutChar(0x0d) )
				{
					/* if error was detected then quit */
					return 0;
				}
			}
		}
		else
		{
			/*
			 * clear our RX buffer in case any debug info was
			 * looped back
			 */
			ClearReceiver();

			/* else NULL was found */
			return 1;
		}
	}

	/*
	 * clear our RX buffer in case any debug info was
	 * looped back
	 */
	ClearReceiver();

	return 1;
}
#endif

