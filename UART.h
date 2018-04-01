/*
 * UART.h
 *
 *  Created on: Mar 30, 2018
 *      Author: adria
 */

#ifndef UART_H_
#define UART_H_



#endif /* UART_H_ */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "fsl_port.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"
#include "event_groups.h"



void uart_Init();
void uart_callback(UART_Type *base, uart_handle_t uart_handle, status_t status, void *userData);
void PC_Terminal_Task(void *arg);

