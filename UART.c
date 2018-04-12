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
#include "event_groups.h"
#include "portmacro.h"
#include "UART.h"

#define TX_FINISHED			1>>0
#define RX_FINISHED			1<<1
#define KEY_BUFFER_LENGTH 	1
#define BUFFER_LENGHT		8
#define ADDRESS_BUFFER_LENGTH 4
#define SIZE_BUFFER_LENGTH  2

/******************************************************************************************************************************
 * PROTOTIPOS DE FUNCIÓN
 ******************************************************************************************************************************/

/* UART user callback */
void UART_UserCallback(UART_Type *base, uart_handle_t *handle, status_t status, void *userData);

void UART_Send(UART_Type *base, uint8_t *data, size_t length, uart_handle_t *uart_handle);

void UART_Receive(UART_Type *base, uint8_t *data, size_t length, uart_handle_t *uart_handle);

void reprint();

/********************************************************************************************************************************
* VARIABLES
*******************************************************************************************************************************/
//EventGroupHandle_t g_uart_event;
typedef enum {MAIN_MENU,READ_MEM,WRITE_MEM,SET_HOUR,SET_DATE,HOUR_FORMAT,READ_HOUR,READ_DATE,TERMINAL2,LCD_ECHO} state;
uint8_t erase[] = "\033[2J";
uint8_t background[] = "\033[41m";
uint8_t mainMenu[] = " SISTEMA DE COMUNICACION CLIENTE-SERVIDOR\r\n\n"
" 1) Leer Memoria I2C\r\n 2) Escribir Memoria I2C\r\n 3) Establecer Hora \r\n 4) Establecer Fecha\r\n 5) Formato de hora\r\n"
" 6) Leer Hora\r\n 7)Leer Fecha\r\n 8) Comunicacion con terminal\r\n 9) Eco en LCD\r\n";
uint8_t sendData[] = "PC TASK EN EJECUCION\r\n";
uint8_t enter[] = "\r\n";
uint8_t space[] = " ";
uint8_t escape[] = "escape presionado";

uint8_t read_mem[] = "\r\nLEER MEMORIA I2C\r\n\r\n Direccion de lectura: 0x";
uint8_t write_mem[] = "\r\nESCRIBIR MEMORIA I2C\r\n\r\n ";
uint8_t set_hour[] = "\r\nESTABLECER HORA\r\n\r\n ";
uint8_t set_date[] = "\r\nESTABLECER FECHA\r\n\r\n ";
uint8_t hour_format[] = "\r\nFORMATO DE HORA\r\n\r\n";
uint8_t read_hour[] = "\r\nREAD HOUR\r\n\r\n";
uint8_t read_date[] = "\r\nREAD DATE\r\n\r\n";
uint8_t term2[] = "\r\nCOMUNICACIÓN CON TERMINAL 2\r\n\r\n";
uint8_t lcd_echo[] = "\r\nECO EN LCD\r\n\r\n";
uint8_t contenido[] = "\r\nContenido: \r\n\r\n";
uint8_t texto[] = "\r\nTexto a guardar: \r\n\r\n";
uint8_t longitud[] = "\r\nLongitud en bytes: ";
uint8_t presiona[] = "\r\nPresiona una tecla para continuar: \r\n\r\n";

uart_handle_t g_uartHandle;
uart_handle_t g_uartHandle4;

volatile bool txOnGoing = false;
volatile bool rxOnGoing = false;
volatile bool Key_Flag = false;
volatile bool print_pressedKey_Flag = false;

volatile bool ask_address = false;
volatile bool ask_size = false;
volatile bool send_size;
volatile uint8_t show_Flag = false;

volatile uint8_t number_of_times_pressed = 0;
volatile uint8_t isBufferFull = false;


uint8_t receiveData[32];
uint8_t g_txBuffer[KEY_BUFFER_LENGTH];
uint8_t g_rxBuffer[KEY_BUFFER_LENGTH];
uint8_t anotherbufer[4];
uint8_t addressbuffer[ADDRESS_BUFFER_LENGTH];
uint8_t sizebuffer[SIZE_BUFFER_LENGTH];

extern void UART0_DriverIRQHandler(void);
extern void UART4_DriverIRQHandler(void);

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
	//BaseType_t xHigherPriorityTaskWoken;
	if (kStatus_UART_TxIdle == status)
	{
		txOnGoing = false;
		//xEventGroupSetBitsFromISR(g_uart_event, TX_FINISHED, &xHigherPriorityTaskWoken);

	}
	if (kStatus_UART_RxIdle == status)
	{
		rxOnGoing = false;
		print_pressedKey_Flag = true;
		number_of_times_pressed++;
		if(4 == number_of_times_pressed )
			isBufferFull = true;
		if(number_of_times_pressed > 4)
			number_of_times_pressed = 1;
		//xEventGroupSetBitsFromISR(g_uart_event, RX_FINISHED, &xHigherPriorityTaskWoken);

	}
	//portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void UART_Send(UART_Type *base, uint8_t *data, size_t length, uart_handle_t *uart_handle) /* Con esta función podremos enviar más cómodamente */
{
	uart_transfer_t sendXfer;
	sendXfer.data = data;
	sendXfer.dataSize = length;
	txOnGoing = true;
	UART_TransferSendNonBlocking(base, uart_handle, &sendXfer);

	while (txOnGoing)
	{
	}
}

void UART_Receive(UART_Type *base, uint8_t *recv_buffer, size_t length, uart_handle_t *uart_handle)
{
	uart_transfer_t receiveXfer;
	receiveXfer.data = recv_buffer;
	receiveXfer.dataSize = length;
	rxOnGoing = true;
	UART_TransferReceiveNonBlocking(base, uart_handle, &receiveXfer, NULL);
}


void uart_Init() 	// Función para inicializar(configurar) las UARTs 0 y 4
{
	//g_uart_event = xEventGroupCreate();

	CLOCK_EnableClock(kCLOCK_PortB);			/* Habilitar reloj del puerto B */
	PORT_SetPinMux(PORTB, 16, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTB, 17, kPORT_MuxAlt3);
	CLOCK_EnableClock(kCLOCK_PortC);			/* Habilitar reloj del puerto C */
	PORT_SetPinMux(PORTC, 14, kPORT_MuxAlt3);
	PORT_SetPinMux(PORTC, 15, kPORT_MuxAlt3);

	UART_GetDefaultConfig(&config_uart0);
	config_uart0.baudRate_Bps = 9600;
	config_uart0.enableTx = true;
	config_uart0.enableRx = true;
	UART_GetDefaultConfig(&config_uart4);
	config_uart4.baudRate_Bps = 9600;
	config_uart4.enableTx = true;
	config_uart4.enableRx = true;

	UART_Init(UART0, &config_uart0, CLOCK_GetFreq(UART0_CLK_SRC));	// Inicialización de los parámetros de UART0
	UART_Init(UART4, &config_uart4, CLOCK_GetFreq(UART4_CLK_SRC));  // Inicialización de los parámetros de UART4

	UART_TransferCreateHandle(UART0, &g_uartHandle, UART_UserCallback, NULL);
	UART_TransferCreateHandle(UART4, &g_uartHandle4, UART_UserCallback, NULL);

	NVIC_SetPriority(UART0_RX_TX_IRQn,5);
	NVIC_SetPriority(UART4_RX_TX_IRQn,5);

		//UART_TransferReceiveNonBlocking(UART0, &g_uartHandle, &receiveXfer, &receivedBytes);
		//xEventGroupWaitBits(g_uart_event, RX_FINISHED, pdTRUE, pdTRUE, portMAX_DELAY);
		//g_txBuffer = g_rxBuffer - 'a' + 'A';

		//UART_Send(UART4, erase, sizeof(erase) - 1, &g_uartHandle4);
		UART_Send(UART4, mainMenu, sizeof(mainMenu) - 1,  &g_uartHandle4);
		/* LIMPIAR PANTALLA */
		UART_Send(UART0, erase, sizeof(erase) - 1, &g_uartHandle);
		/* BACKGROUND ROJO */
		UART_Send(UART0, background, sizeof(background) - 1, &g_uartHandle);
		UART_Send(UART4, background, sizeof(background) - 1, &g_uartHandle4);
		/* ENVÍA MENÚ PRINCIPAL*/
		UART_Send(UART0, mainMenu, sizeof(mainMenu) - 1, &g_uartHandle);
}

state changeState(uint8_t pressed_key)
{
	state estado = MAIN_MENU;
	if( KEY_ESC == pressed_key)
		estado = MAIN_MENU;

	if( KEY_1 == pressed_key)
			estado = READ_MEM;
	if( KEY_2 == pressed_key)
			estado = WRITE_MEM;
	if( KEY_3 == pressed_key)
			estado = SET_HOUR;
	if( KEY_4 == pressed_key)
			estado = SET_DATE;
	if( KEY_5 == pressed_key)
			estado = HOUR_FORMAT;
	if( KEY_6 == pressed_key)
			estado = READ_HOUR;
	if( KEY_7 == pressed_key)
			estado = READ_DATE;
	if( KEY_8 == pressed_key)
			estado = TERMINAL2;
	if( KEY_9 == pressed_key)
			estado = LCD_ECHO;
	return estado;
}

void printPressedKey()
{
	if( true == print_pressedKey_Flag)						/* Si se ha  presionado tecla nueva */
	{
		UART_Send(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);	/* Se envía esa misma tecla a pantalla*/
		UART_Send(UART4, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle4);	/* Se envía esa misma tecla a pantalla*/
		print_pressedKey_Flag = false;						/* Se pone la bandera en falso */
	}
}

void printBufer(uint8_t * bufer, size_t size)
{
	if( true == print_pressedKey_Flag)						/* Si se ha  presionado tecla nueva */
	{
		UART_Send(UART0, space, sizeof(space)-1, &g_uartHandle);
		UART_Send(UART0, bufer, size, &g_uartHandle);	/* Se envía esa misma tecla a pantalla*/
		UART_Send(UART4, bufer, size, &g_uartHandle4);	/* Se envía esa misma tecla a pantalla*/

		UART_Send(UART0, enter, sizeof(enter)-1, &g_uartHandle);
		print_pressedKey_Flag = false;						/* Se pone la bandera en falso */
	}
}

void printOptionMenu(uint8_t * data, size_t size)
{
	if( true == Key_Flag )
	{
		UART_Send(UART0, erase, sizeof(erase) - 1, &g_uartHandle);
		UART_Send(UART0, data, size, &g_uartHandle);
		UART_Send(UART4, data, size, &g_uartHandle4);
		Key_Flag = false;
		print_pressedKey_Flag = false;
	}
}
void PC_Terminal_Task(void *arg)
{
	UART_Send(UART0, sendData, sizeof(sendData) - 1, &g_uartHandle); //MENSAJE PARA IDENTIFICAR LA TAREA EN EJECUCIÓN

	state State = MAIN_MENU;
	uint8_t pressed_key;
	*g_rxBuffer = 0;			/* Limpiar búfer */
	*anotherbufer = 0;
	uint8_t index = 0;

    for(;;)
    {

    	switch(State)
    	{
			case MAIN_MENU:
				printOptionMenu(mainMenu, sizeof(mainMenu)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				State = changeState(pressed_key);
				if( State != MAIN_MENU)	/* Si se va a cambiar de estado se habilita impresión una vez de otros menús */
					Key_Flag = true;
				break;

			case READ_MEM:
				printOptionMenu(read_mem, sizeof(read_mem)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else if(KEY_ENTER == pressed_key)
				{

					if( true == ask_address ) // && (sizeof(addressbuffer) == 32) si está lleno el buffer
					{
						printBufer(addressbuffer, ADDRESS_BUFFER_LENGTH);
						// ESCRIBIR POR I2C LA INDICACIÓN LEER MEMORIA EN LA DIRECCIÓN GUARDADA EN 'addressbuffer'
						ask_address = false;
						ask_size = true;
					}
					if( true == send_size)
					{
						printBufer(sizebuffer, SIZE_BUFFER_LENGTH);
						// ESCRIBIR POR I2C EL TAMAÑO GUARDADO EN 'sizebuffer'
						ask_size = false;
						show_Flag = true;
					}
				}
				else
				{
					printPressedKey();
					ask_address = true;
					if( ask_address == true)
					{
						addressbuffer[number_of_times_pressed - 1] = pressed_key;
					}
					if( ask_size == true)
					{
						sizebuffer[number_of_times_pressed - 1] = pressed_key;
					}
				}
				if( ask_size == true)
				{
					printBufer(longitud, sizeof(longitud) - 1);
					if(KEY_ENTER == pressed_key)
						send_size = true;
				}

				if( show_Flag == true )
				{
					printBufer(contenido, sizeof(contenido) - 1);
					//IMPRIMIR LO LEÍDO DE ESA DIRECCIÓN DE MEMORIA , USANDO UART_SEND CON EL BÚFER LEÍDO de I2C
					printBufer(presiona, sizeof(presiona) - 1);
					show_Flag = false;
				}
				break;

			case WRITE_MEM:
				printOptionMenu(write_mem, sizeof(write_mem)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case SET_HOUR:
				printOptionMenu(set_hour, sizeof(set_hour)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case SET_DATE:
				printOptionMenu(set_date, sizeof(set_date)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case HOUR_FORMAT:	/* FORMATO DE HORA */
				printOptionMenu(hour_format, sizeof(hour_format)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case READ_HOUR:	/* LEER HORA*/
				printOptionMenu(read_hour, sizeof(read_hour)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case READ_DATE:	/* LEER FECHA */
				printOptionMenu(read_date, sizeof(read_date)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case TERMINAL2:
				printOptionMenu(term2, sizeof(term2)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			case LCD_ECHO:
				printOptionMenu(lcd_echo, sizeof(lcd_echo)-1);
				UART_Receive(UART0, g_rxBuffer, KEY_BUFFER_LENGTH, &g_uartHandle);
				pressed_key = *g_rxBuffer;
				if(KEY_ESC == pressed_key)
				{
					State = MAIN_MENU;
					Key_Flag = true;
				}
				else
				{
					printPressedKey();
				}
				break;

			default:
				State = MAIN_MENU;
				break;
    	}// END CASE
	}//END FOR

    vTaskDelay(10);
}//END TASK
