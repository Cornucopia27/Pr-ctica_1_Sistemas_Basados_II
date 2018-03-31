/*
 * UART.c
 *
 *  Created on: March 21th, 2018
 *      Authors: Adrian Ramos Perez and Alex Avila Chavira
 */

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

/*** VARIABLES *********************************************************************/
const char *teratermtxt = "Esta es la terminal del PC\r\n";
uint8_t sendData[] = {"Probando"};
uint8_t background_buffer[32];
uint8_t recv_buffer[1];
uart_handle_t g_uartHandle;
uart_handle_t handle;
uart_handle_t handle_uart4;
uart_transfer_t sendXfer;
uart_transfer_t receiveXfer;
volatile bool txFinished;
volatile bool rxFinished;

uint8_t

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
void uart_callback(UART_Type *base, uart_handle_t uart_handle, status_t status, void *userData)
{
	userData = userData;
	BaseType_t xHigherPriorityTaskWoken;
	if (kStatus_UART_TxIdle == status)
	{
		//xEventGroupGetBitsFromISR(g_uart_event, TX_FINISHED, &xHigherPriorityTaskWoken);
		txFinished = true;
	}
	if (kStatus_UART_RxIdle == status)
	{
		rxFinished = true;
	}
}


void uart_Init() 	// Función para inicializar(configurar) las UARTs 0 y 4
{
	CLOCK_EnableClock(kCLOCK_PortB);			/* Habilitar reloj del puerto B */
	PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);

	/*UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = true;
	config.enableRx = true;*/

	UART_Init(UART0, &config_uart0, CLOCK_GetFreq(UART0_CLK_SRC));	// Inicialización de los parámetros de UART0
	UART_Init(UART4, &config_uart4, CLOCK_GetFreq(UART4_CLK_SRC));  // Inicialización de los parámetros de UART4

	UART_TransferCreateHandle(UART0, &g_uartHandle, uart_callback, NULL);
	NVIC_SetPriority(UART0_RX_TX_IRQn,5);
	NVIC_SetPriority(UART4_RX_TX_IRQn,5);

		sendXfer.data = &g_txBuffer;
		sendXfer.dataSize = 1;
		receiveXfer.data = &g_rxBuffer;
		receiveXfer.dataSize = 1;


		UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, &receivedB);
		xEventGroupWaitBits(g_uart_evet, RX_FINISHED, pdTRUE, pdTRUE, port_MAXDELAY);
		g_txBuffer = g_rxBuffer - 'a' + 'A';
		UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);

}

void PC_Terminal_Task(void *pvParameters)
{

    /* Enviar el menú de opciones a escoger */
    /*if (0 > UART_RTOS_Send(&handle, (uint8_t *)teratermtxt, strlen(teratermtxt)))
    {
        vTaskSuspend(NULL);
    }*/
    UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);

    /* Send data */
    for(;;)
    {
        /* send back the received data */
    	//UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
    	//receiveXfer.data = ;
    	//receiveXfer.dataSize = ;
    	UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, NULL);

        //UART_RTOS_Send(&handle, (uint8_t *)recv_buffer, n);
    	sendXfer.data = sendData;
    	sendXfer.dataSize = sizeof(sendData)/sizeof(sendData[0]);
        UART_TransferSendNonBlocking(UART0, &g_uartHandle, &sendXfer);

        vTaskDelay(100);

	}
    //UART_RTOS_Deinit(&handle);
    UART_Deinit(UART0);

    vTaskDelay(100);
}
