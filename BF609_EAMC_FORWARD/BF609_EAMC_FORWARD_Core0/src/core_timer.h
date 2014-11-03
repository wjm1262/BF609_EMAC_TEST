/*
 * core_timer.h
 *
 *  Created on: 2014-10-28
 *      Author: Administrator
 */

#ifndef CORE_TIMER_H_
#define CORE_TIMER_H_

#include <ccblkfn.h>
#include <stdio.h>
#include <services/tmr/adi_ctmr.h>
#include <services/gpio/adi_gpio.h>

/*
 * Core timer initialization
 * 1, #include <services/tmr/adi_ctmr.h>
 * 2, Define the ADI_CTMR_HANDLE of the core timer.
 * 3, Define the period and the prescale to set the timer cycle,the default core clock is 500M.
 * 4, Define the interrupt callback function if used.
 */
/* Period and prescale of the core timer */
#include <services/tmr/adi_ctmr.h>
#define MEGA (1000000u)
#define PERIOD (327680000)
#define PRESCALE (0u)
/* the handle of the core timer */
ADI_CTMR_HANDLE ghCoreTimer;
/*
 * the handler of the core timer.
 */
static void CoreTimerHandler(void *pCBParam, uint32_t Event, void *pArg)
{
	//adi_gpio_Toggle(ADI_GPIO_PORT_C, ADI_GPIO_PIN_15);
	adi_gpio_Toggle(ADI_GPIO_PORT_G, ADI_GPIO_PIN_13);
}

void CoreTimerInit(void)
{
	ADI_CTMR_RESULT result;

	do{

		result = adi_ctmr_Open(ADI_CTMR_DEV0, CoreTimerHandler, NULL, &ghCoreTimer);
		if (result != 0)
			break;
		result = adi_ctmr_EnableAutoReload(ghCoreTimer, true);
		if (result != 0)
			break;
		result = adi_ctmr_SetPeriod(ghCoreTimer, PERIOD);
		if (result != 0)
			break;
		result = adi_ctmr_SetScale(ghCoreTimer, PRESCALE);
		if (result != 0)
			break;
		result = adi_ctmr_Enable(ghCoreTimer, true);

	}while(0);

	if (result != 0)
		printf("initialization failed! error code is %d\n", result);

}



#endif /* CORE_TIMER_H_ */
