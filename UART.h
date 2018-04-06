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

#define KEY_ENTER 	0xD
#define KEY_ESC		0x1B
#define KEY_1		0x31
#define KEY_2		0x32
#define KEY_3		0x33
#define KEY_4		0x34
#define KEY_5		0x35
#define KEY_6		0x36
#define KEY_7		0x37
#define KEY_8		0x38
#define KEY_9		0x39

/*******************************************************************************
 * PROTOTIPOS DE FUNCIÓN
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void UART_Send(UART_Type *base, uint8_t *data, size_t length, uart_handle_t *uart_handle);

void UART_Receive(UART_Type *base, uint8_t *data, size_t length, uart_handle_t *uart_handle);


void uart_Init();

void PC_Terminal_Task(void *arg);

