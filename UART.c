/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

#include "pin_mux.h"
#include "clock_config.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART UART0
#define DEMO_UART_4 UART4

#define DEMO_UART_CLKSRC UART0_CLK_SRC
#define DEMO_UART4_CLKSRC UART4_CLK_SRC

#define DEMO_UART_RX_TX_IRQn UART0_RX_TX_IRQn
#define DEMO_UART4_RX_TX_IRQn UART4_RX_TX_IRQn

/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void uart4_task(void *pvParameters);

#define DEMO_RING_BUFFER_SIZE 16
#define DEMO_RING_BUFFER_SIZE_UART4 16

uint8_t demoRingBuffer[DEMO_RING_BUFFER_SIZE];
volatile uint16_t txIndex; /* Index of the data to send out. */
volatile uint16_t rxIndex; /* Index of the memory to save new arrived data. */

uint8_t demoRingBuffer_UART4[DEMO_RING_BUFFER_SIZE_UART4];
volatile uint16_t txIndex_UART4; /* Index of the data to send out. */
volatile uint16_t rxIndex_UART4; /* Index of the memory to save new arrived data. */
/*******************************************************************************
 * Variables
 ******************************************************************************/
const char *to_send = "teraterm\r\n";
const char *to_send4 = "blueterm\r\n";
const char *send_ring_overrun = "\r\nRing buffer overrun!\r\n";
const char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";

uint8_t background_buffer_uart4[32];
uint8_t recv_buffer_uart4[4];


uint8_t background_buffer[32];
uint8_t recv_buffer[1];

uart_rtos_handle_t handle;
struct _uart_handle t_handle;

uart_rtos_handle_t handle_uart4;
struct _uart_handle t_handle_uart4;


uart_rtos_config_t uart_config = {

    .baudrate = 115200,
    .parity = kUART_ParityDisabled,
    .stopbits = kUART_OneStopBit,
    .buffer = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

uart_rtos_config_t uart4_config = {
    .baudrate = 9600,
    .parity = kUART_ParityDisabled,
    .stopbits = kUART_OneStopBit,
    .buffer = background_buffer_uart4,
    .buffer_size = sizeof(background_buffer_uart4),
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitPins();                           //we need to configure the uart 4
    BOARD_BootClockRUN();
    NVIC_SetPriority(DEMO_UART_RX_TX_IRQn, 5);
    NVIC_SetPriority(DEMO_UART4_RX_TX_IRQn, 5);


    uart_config.srcclk = CLOCK_GetFreq(DEMO_UART_CLKSRC);
    uart_config.base = DEMO_UART;

    uart4_config.srcclk = CLOCK_GetFreq(DEMO_UART4_CLKSRC);
   	uart4_config.base = DEMO_UART_4;



    xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE, NULL, uart_task_PRIORITY, NULL);
    xTaskCreate(uart4_task, "Uart4_task", configMINIMAL_STACK_SIZE, NULL, uart_task_PRIORITY, NULL);

    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for printing of "Hello world." message.
 */
void DEMO_UART_IRQHandler(void)
{
    uint8_t data;

    /* If new data arrived. */
    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART))
    {
        data = UART_ReadByte(DEMO_UART);

        /* If ring buffer is not full, add data to ring buffer. */
        if (((rxIndex + 1) % DEMO_RING_BUFFER_SIZE) != txIndex)
        {
            demoRingBuffer[rxIndex] = data;
            rxIndex++;
            rxIndex %= DEMO_RING_BUFFER_SIZE;
            UART_WriteByte(DEMO_UART_4, demoRingBuffer[txIndex]);
        }
    }
}

void DEMO_UART4_IQRHandler(void)
{
	uint8_t data_UART4;

	        uart_config.srcclk = CLOCK_GetFreq(DEMO_UART4_CLKSRC);
		    uart_config.base = DEMO_UART;
	    /* If new data arrived. */
	    if ((kUART_RxDataRegFullFlag | kUART_RxOverrunFlag) & UART_GetStatusFlags(DEMO_UART_4))
	    {
	        data_UART4 = UART_ReadByte(DEMO_UART_4);

	        /* If ring buffer is not full, add data to ring buffer. */
	        if (((rxIndex_UART4 + 1) % DEMO_RING_BUFFER_SIZE_UART4) != txIndex_UART4)
	        {
	            demoRingBuffer_UART4[rxIndex_UART4 ] = data_UART4 ;
	            rxIndex_UART4++;
	            rxIndex_UART4 %= DEMO_RING_BUFFER_SIZE_UART4;
	            UART_WriteByte(DEMO_UART, demoRingBuffer[txIndex_UART4]);
	        }
	    }
}

static void uart4_task(void *pvParameter)
{

	    int error_uart4;
	    size_t n_uart4;
	   // I define the base and srcclk on the top (inside uart rtos config)


	    if (0 > UART_RTOS_Init(&handle_uart4, &t_handle_uart4, &uart4_config))
	    {
	        vTaskSuspend(NULL);
	    }

	    /* Send some data */
	    if (0 > UART_RTOS_Send(&handle_uart4, (uint8_t *)to_send4, strlen(to_send)))
	    {
	        vTaskSuspend(NULL);
	    }

	    /* Send data */
	    do
	    {
	        error_uart4 = UART_RTOS_Receive(&handle_uart4, recv_buffer_uart4, sizeof(recv_buffer_uart4), &n_uart4);
	        if (error_uart4 == kStatus_UART_RxHardwareOverrun)
	        {
	            /* Notify about hardware buffer overrun */
	            if (kStatus_Success !=
	                UART_RTOS_Send(&handle_uart4, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
	            {
	                vTaskSuspend(NULL);
	            }
	        }
	        if (error_uart4 == kStatus_UART_RxRingBufferOverrun)
	        {
	            /* Notify about ring buffer overrun */
	            if (kStatus_Success != UART_RTOS_Send(&handle_uart4, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
	            {
	                vTaskSuspend(NULL);
	            }
	        }
	        if (n_uart4 > 0)
	        {
	            /* send back the received data */
	            UART_RTOS_Send(&handle_uart4, (uint8_t *)recv_buffer_uart4, n_uart4);
	        }
	        vTaskDelay(1000);
	    } while (kStatus_Success == error_uart4);

	    UART_RTOS_Deinit(&handle_uart4);

	    vTaskDelay(1000);
}

static void uart_task(void *pvParameters)
{
    int error;
    size_t n;
   // I define the base and srcclk on the top (inside uart rtos config)

    if (0 > UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Send some data */
    if (0 > UART_RTOS_Send(&handle, (uint8_t *)to_send, strlen(to_send)))
    {
        vTaskSuspend(NULL);
    }

    /* Send data */
    do
    {
        error = UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
        if (error == kStatus_UART_RxHardwareOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                UART_RTOS_Send(&handle, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (error == kStatus_UART_RxRingBufferOverrun)
        {
            /* Notify about ring buffer overrun */
            if (kStatus_Success != UART_RTOS_Send(&handle, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
            /* send back the received data */
            UART_RTOS_Send(&handle, (uint8_t *)recv_buffer, n);
        }
        vTaskDelay(1000);
    } while (kStatus_Success == error);

    UART_RTOS_Deinit(&handle);

    vTaskDelay(1000);
}
