/*
 * UART_Init.h
 *
 *  Created on: 2014-4-15
 *      Author: Administrator
 */

#ifndef UART_INIT_H_
#define UART_INIT_H_
#include <drivers\uart\adi_uart.h>
#include <services\pwr\adi_pwr.h>
#include <services\stdio\adi_stdio.h>
#include <stdio.h>
/*
 * inti the UART,user should include the adi_uart.h and this header file. in addition to,
 * user should define the TX and RX buffer by themselves.
 * user should also define the ADI_UART_HANDLE which need to be used to read the buffer.
 */
ADI_UART_HANDLE UART0_Init(void);

#endif /* UART_INIT_H_ */
