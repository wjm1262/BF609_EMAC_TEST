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
 * Description:	This file implements timers on the EZ-Board.
 */

#include <ccblkfn.h>
#include <services/tmr/adi_tmr.h>
#include <services/int/adi_int.h>
#include <services/tmr/adi_tmr_bf6xx.h>
#include <adi_types.h>
//#include "post_common.h"
#include "post_debug.h"
#include "Timer_ISR.h"
#include <services/int/adi_sec.h>
#include <services/gpio/adi_gpio.h>

#if defined(__DEBUG_FILE__)
#include <string.h>
extern FILE *pDebugFile;				/* debug file when directing output to a file */
#endif

#define MAX_NUM_COUNTDOWN_TIMERS 5

typedef struct CountDownTimer_TAG
{
	bool m_IsActive;
	unsigned long m_ulTimeoutCounter;
}countdowntimer;

static volatile unsigned long g_ulTickCount;
static countdowntimer sCountDownTimer[MAX_NUM_COUNTDOWN_TIMERS] = { {0,0},{0,0},{0,0},{0,0},{0,0} };

/* Memory required for opening the timer */
uint8_t TimerMemory[ADI_TMR_MEMORY];

/* Timer callback function */
static void TimerHandler(void *pCBParam, uint32_t Event, void *pArg);

#define TEST_TIMER_NUM      1
#define TIMER_WIDTH         (81920000)
#define TIMER_DELAY          (40960000)
#define TIMER_PERIOD        (163840000)

/* Timer handle */
static ADI_TMR_HANDLE ghTimer = 0;


bool EnableGPTimer(bool bEnable)
{
	ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;
	if(ghTimer)
	{
		/* Enable the timer*/
		if((eTmrResult = adi_tmr_Enable(
				ghTimer,
				bEnable)) != ADI_TMR_SUCCESS)
		{
			DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
			return false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		return false;
	}
}



/*******************************************************************
*   Function:    Init_Timer_Interrupts
*   Description: This function initializes the interrupts for Timer0
*******************************************************************/
void Init_Timer_Interrupts(void)
{
    ADI_TMR_RESULT eTmrResult   =   ADI_TMR_SUCCESS;

    if(!ghTimer)
    {
    /* Open the timer */
    if( (eTmrResult = adi_tmr_Open (
            TEST_TIMER_NUM,
            TimerMemory,
            ADI_TMR_MEMORY,
            TimerHandler,
            NULL,
            &ghTimer)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Timer open failed 0x%08X \n", eTmrResult);
    }

    /* Set the mode to PWM OUT */
    if((eTmrResult = adi_tmr_SetMode(
            ghTimer,
            ADI_TMR_MODE_CONTINUOUS_PWMOUT)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to open timer in PWM out mode 0x%08X \n\n", eTmrResult);
    }

    /* Set the IRQ mode to get interrupt after timer counts to Delay + Width */
    if((eTmrResult = adi_tmr_SetIRQMode(
            ghTimer,
            ADI_TMR_IRQMODE_WIDTH_DELAY)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to set the timer IRQ mode 0x%08X \n\n", eTmrResult);
    }

    /* Set the Period */
    if((eTmrResult = adi_tmr_SetPeriod(
            ghTimer,
            TIMER_PERIOD)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to set the timer Period 0x%08X \n\n", eTmrResult);
    }

    /* Set the timer width */
    if((eTmrResult = adi_tmr_SetWidth(
            ghTimer,
            TIMER_WIDTH)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to set the timer Width 0x%08X \n\n", eTmrResult);
    }

    /* Set the timer Delay */
    if((eTmrResult = adi_tmr_SetDelay(
            ghTimer,
            TIMER_DELAY)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to set the timer Delay 0x%08X \n\n", eTmrResult);
    }

    /* Enable the timer to stop gracefully */
    if((eTmrResult = adi_tmr_EnableGracefulStop(
            ghTimer,
            true)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to enable the timer to stop gracefully 0x%08X \n\n", eTmrResult);
    }
    }
    /* Enable the timer*/
    if((eTmrResult = adi_tmr_Enable(
            ghTimer,
            true)) != ADI_TMR_SUCCESS)
    {
    	DEBUG_PRINT("Failed to enable the timer 0x%08X \n\n", eTmrResult);
    }
}


/*******************************************************************
*   Function:    Delay
*   Description: Delay for a fixed number of Ms, blocks
*******************************************************************/
void Delay(const unsigned long ulMs)
{
	unsigned int uiTIMASK = cli();

	g_ulTickCount = 0;
	unsigned long ulEnd = (g_ulTickCount + ulMs);

	sti(uiTIMASK);

    while( g_ulTickCount < ulEnd )
	{
		asm("nop;");
	}
}


/*******************************************************************
*   Function:    SetTimeout
*   Description: Set a value for a global timeout, return the timer
*******************************************************************/
unsigned int SetTimeout(const unsigned long ulTicks)
{
	unsigned int uiTIMASK = cli();
	unsigned int n;

	/* we don't care which countdown timer is used, so search for a free
		timer structure */
	for( n = 0;  n < MAX_NUM_COUNTDOWN_TIMERS; n++ )
	{
		if( false == sCountDownTimer[n].m_IsActive )
		{
			sCountDownTimer[n].m_IsActive = true;
			sCountDownTimer[n].m_ulTimeoutCounter = ulTicks;

			sti(uiTIMASK);
			return n;
		}
	}

	sti(uiTIMASK);
	return ((unsigned int)-1);
}

/*******************************************************************
*   Function:    ClearTimeout
*   Description: Set a value for a global timeout, return the timer
*******************************************************************/
unsigned long ClearTimeout(const unsigned int nIndex)
{
	unsigned int uiTIMASK = cli();
	unsigned long ulTemp = (unsigned int)(-1);

	if( nIndex < MAX_NUM_COUNTDOWN_TIMERS )
	{
		/* turn off the timer */
		ulTemp = sCountDownTimer[nIndex].m_ulTimeoutCounter;
		sCountDownTimer[nIndex].m_ulTimeoutCounter = 0;
		sCountDownTimer[nIndex].m_IsActive = false;
	}

	sti(uiTIMASK);
	return (ulTemp);
}

/*******************************************************************
*   Function:    IsTimedout
*   Description: Checks to see if the timeout value has expired
*******************************************************************/
bool IsTimedout(const unsigned int nIndex)
{
	unsigned int uiTIMASK = cli();
	if( nIndex < MAX_NUM_COUNTDOWN_TIMERS )
	{
		sti(uiTIMASK);
		return ( 0 == sCountDownTimer[nIndex].m_ulTimeoutCounter );
	}

	sti(uiTIMASK);
	return 0;
}

/*
  *   Function:    void Timer_ISR( void *, uint_32, void *)
 *   Description: Timer ISR
 */
static void TimerHandler(void *pCBParam, uint32_t Event, void *pArg)
{
	unsigned int n;
	
	g_ulTickCount++;

	adi_gpio_Toggle(ADI_GPIO_PORT_C, ADI_GPIO_PIN_15);

	/* decrement each counter if it is non-zero */
	for( n = 0; n < MAX_NUM_COUNTDOWN_TIMERS; n++ )
	{
		if( 0 != sCountDownTimer[n].m_ulTimeoutCounter )
		{
			sCountDownTimer[n].m_ulTimeoutCounter--;
		}
	}
}
