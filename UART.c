
/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    UART.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_uart.h"
#include "semphr.h"

#define MEM_SLAVE_DIRECTION

void uart_callback(UART_Type *base, uart_handle);


void uart_callback(UART_Type *base, uart_handle)
{
	userData = userData;
	BaseType_t xHigherPriorityTaskWoken;
	if (kStatus_UART_TxIdle == status)
	{
		xEventGroupGetBitsFromISR(g_uart_event, TX_FINISHED, &xHigherPriorityTaskWoken);

	}
}

void uart_Init(){

	CLOCK_EnableClock(kCLOCK_PortB);
	PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);

	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = true;
	config.enableRx = true;

	UART_Init(UART0, &config, CLOCK_GetFreq(UART0_CLK_SRC));
	UART_TransferCreateHandle(UART0, &g_uartHandle, uart_callback, NULL);
	NVIC_SetPriority(UART0_RX_TX_IRQn,5);

	SendXfer.data = &g_txBuffer;
	sendXfer.dataSize = 1;
	receiveXfer.data = &g_rxBuffer;
	receiveXfer.dataSize = 1;


	UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, &receivedB)
	xEventGroupWaitBits(g_uart_evet, RX_FINISHED, pdTRUE, pdTRUE, port_MAXDELAY)
	g_txBuffer = g_rxBuffer - 'a' + 'A';
	UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
}

/*
 * @brief   Application entry point.
 */
int main(void) {

  	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    xTsakCreate(uart_task, "UART", 200, NULL, 2 , NULL);

    BOARD_InitDebugConsole();



    for(;;) {

    }
    return 0 ;
}
