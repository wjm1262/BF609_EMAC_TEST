/*
 * (C) Copyright 2012 - Analog Devices, Inc.  All rights reserved.
 *
 * FILE:     timer_isr.h ( )
 *
 * CHANGES:  1.00.0  - initial release
 */

#ifndef _TIMER_ISR_H_
#define _TIMER_ISR_H_

#include <sys\exception.h>
#include <cdefbf609.h>
#include <ccblkfn.h>

/*******************************************************************
*  global variables and defines
*******************************************************************/
#define TIMEOUT_PERIOD	0x00001000


/*******************************************************************
*  function prototypes
*******************************************************************/
void Init_Timers(void);
void Init_Timer_Interrupts(void);
void Delay(const unsigned long ulMs);
unsigned int SetTimeout(const unsigned long ulTicks);
unsigned long ClearTimeout(const unsigned int nIndex);
bool IsTimedout(const unsigned int nIndex);

bool EnableGPTimer(bool);

void Timer_ISR(void);
#endif /* _TIMER_ISR_H_ */

