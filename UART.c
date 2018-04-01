/*
 * UART.c
 *
 *  Created on: March 21th, 2018
 *      Authors: Adrian Ramos Perez and Alex Avila Chavira
 */

#include "UART.h"
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
#include "portmacro.h"

#define TX_FINISHED		1>>0
#define RX_FINISHED		1<<1
#define BUFFER_LENGHT	8

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void UART_Send(UART_Type *base, uint8_t *data);

EventGroupHandle_t g_uart_event;

/*** VARIABLES *********************************************************************/
uint8_t erase[] = "\033[2J";
uint8_t background[] = "\033[41m";
uint8_t sendData[] = " SISTEMA DE COMUNICACION CLIENTE-SERVIDOR\r\n\n"
" 1) Leer Memoria I2C\r\n 2) Escribir Memoria I2C\r\n 3) Establecer Hora \r\n 4) Establecer Fecha\r\n 5) Formato de hora\r\n"
" 6) Leer Hora\r\n 7)Leer Fecha\r\n 8) Comunicacion con terminal\r\n 9) Eco en LCD\r\n";

uart_handle_t g_uartHandle;
uart_handle_t g_uartHandle4;

uart_transfer_t sendXfer;
uart_transfer_t receiveXfer;

volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;
uint8_t receiveData[32];
uint8_t g_txBuffer[BUFFER_LENGHT];
uint8_t g_rxBuffer[BUFFER_LENGHT];


/* ESTRUCTURA DE CONFIGURACIÓN PARA LA UART 0*/
uart_config_t config_uart0 = {
	.baudRate_Bps = 115200,
	.parityMode = kUART_ParityDisabled,
	.stopBitCount = kUART_OneStopBit,
	.enableTx = true,
	.enableRx = true,
};

/* ESTRUCTURA DE CONFIGURACIÓN PARA LA UART 4*/
uart_config_t config_uart4 =
{
	.baudRate_Bps = 9600,
	.parityMode = kUART_ParityDisabled,
	.stopBitCount = kUART_OneStopBit,
	.enableTx = true,
	.enableRx = true,
};
/********************************************************************************/

/* Basado en el ejemplo 'UART Send/receive using an interrupt method' de NXP: https://mcuxpresso.nxp.com/apidoc/2.0/group__uart__driver.html#ga2868b6ea396ab212547f2157380429c5 */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData)
{
	userData = userData;
	BaseType_t xHigherPriorityTaskWoken;
	if (kStatus_UART_TxIdle == status)
	{
		xEventGroupSetBitsFromISR(g_uart_event, TX_FINISHED, &xHigherPriorityTaskWoken);
		txOnGoing = false;
	}
	if (kStatus_UART_RxIdle == status)
	{
		xEventGroupSetBitsFromISR(g_uart_event, RX_FINISHED, &xHigherPriorityTaskWoken);
		rxOnGoing = false;
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART_Send(UART_Type *base, uint8_t *data) /* Con esta función podremos enviar más cómodamente */
{
	sendXfer.data = data;
	sendXfer.dataSize = 200;
	txOnGoing = true;
	UART_TransferSendNonBlocking(base, &g_uartHandle, &sendXfer);
	while (txOnGoing)
	{
	}
}

void uart_Init(void *arg) 	// Función para inicializar(configurar) las UARTs 0 y 4
{
	CLOCK_EnableClock(kCLOCK_PortB);			/* Habilitar reloj del puerto B */
	PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);

	UART_Init(UART0, &config_uart0, CLOCK_GetFreq(UART0_CLK_SRC));	// Inicialización de los parámetros de UART0
	UART_Init(UART4, &config_uart4, CLOCK_GetFreq(UART4_CLK_SRC));  // Inicialización de los parámetros de UART4

	UART_TransferCreateHandle(UART0, &g_uartHandle, UART_UserCallback, NULL);
	UART_TransferCreateHandle(UART4, &g_uartHandle4, UART_UserCallback, NULL);

	NVIC_SetPriority(UART0_RX_TX_IRQn,5);
	NVIC_SetPriority(UART4_RX_TX_IRQn,5);

		//sendXfer.data = sendData;
		//sendXfer.dataSize = sizeof(sendData) - 1;
		receiveXfer.data = g_rxBuffer;
		receiveXfer.dataSize = BUFFER_LENGHT;

		//UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, &receivedBytes);
		//xEventGroupWaitBits(g_uart_event, RX_FINISHED, pdTRUE, pdTRUE, portMAX_DELAY);
		//g_txBuffer = g_rxBuffer - 'a' + 'A';

		/*txOnGoing = true;
		UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);
		while (txOnGoing)
		{
		}*/
		UART_Send(UART0, sendData);

}

void PC_Terminal_Task(void *arg)
{
	sendXfer.data = sendData;
	sendXfer.dataSize = sizeof(sendData)/sizeof(sendData[0]);

	txOnGoing = true;
    UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);

    /* Send data */
    for(;;)
    {
        /* send back the received data */
    	//UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
    	receiveXfer.data = g_txBuffer;
    	receiveXfer.dataSize = BUFFER_LENGHT;
    	txOnGoing = true;
    	UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, NULL);

        //UART_RTOS_Send(&handle, (uint8_t *)recv_buffer, n);
    	sendXfer.data = g_rxBuffer;	/* Aquí se llena el búfer de salida con lo que entró */
    	sendXfer.dataSize = BUFFER_LENGHT;
    	txOnGoing = true;
        UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);

	}

    vTaskDelay(100);
}
