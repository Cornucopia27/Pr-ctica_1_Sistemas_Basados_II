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
 * @file    I2C.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "I2C_driver.h"
#include "MEM24LC256.h"
#include "PCF8583.h"
#include "LCDNokia5110.h"
#include "UART.h"
#include "init.h"
/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */
Time Current_Time = {0,0,0};
Time Other_Time = {0,0,0};
SemaphoreHandle_t lcd_mutex;
bool format;

void task_lcd()
{
	static uint8_t Time_string[TIME_STRING_FULL_SIZE]; // hh:mm:ss ff
	static uint8_t horas = 0;
	for(;;)
	{
	Current_Time = PCF_request();
	vTaskDelay(pdMS_TO_TICKS(100));
	xSemaphoreTake(lcd_mutex, portMAX_DELAY);
	if(false == Get_Format())
	{
		Time_string[0] = ((Current_Time.Hours & HOURS_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
		Time_string[1] = (Current_Time.Hours & UNITS_MASK) + ASCII_ADD_VALUE;
		Time_string[2] = ':';
		Time_string[3] = ((Current_Time.Minutes & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
		Time_string[4] = (Current_Time.Minutes & UNITS_MASK) + ASCII_ADD_VALUE;
		Time_string[5] = ':';
		Time_string[6] = ((Current_Time.Seconds & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
		Time_string[7] = (Current_Time.Seconds & UNITS_MASK) + ASCII_ADD_VALUE;
		Time_string[8] = ' ';
		Time_string[9] = ' ';
		Time_string[10] = ' ';
		Time_string[11] = '\0';
	}
	else if(true == Get_Format())
	{
		horas = (((Current_Time.Hours & HOURS_TENS_MASK) >> SHIFT_FOUR) * DECIMAL_SPREADER);
		horas = horas + ((Current_Time.Hours) & UNITS_MASK);
		if(HALF_DAY <= horas)
		{
			horas = horas - HALF_DAY;
			Time_string[0] = (horas / DECIMAL_SPREADER) + ASCII_ADD_VALUE;
			Time_string[1] = (horas % DECIMAL_SPREADER) + ASCII_ADD_VALUE;
			Time_string[2] = ':';
			Time_string[3] = ((Current_Time.Minutes & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
			Time_string[4] = (Current_Time.Minutes & UNITS_MASK) + ASCII_ADD_VALUE;
			Time_string[5] = ':';
			Time_string[6] = ((Current_Time.Seconds & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
			Time_string[7] = (Current_Time.Seconds & UNITS_MASK) + ASCII_ADD_VALUE;
			Time_string[8] = ' ';
			Time_string[9] = 'P';
			Time_string[10] = 'M';
			Time_string[11] = '\0';
		}
		else
		{
			Time_string[0] = (((Current_Time.Hours) & HOURS_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
			Time_string[1] = ((Current_Time.Hours) & UNITS_MASK) + ASCII_ADD_VALUE;
			Time_string[2] = ':';
			Time_string[3] = ((Current_Time.Minutes & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
			Time_string[4] = (Current_Time.Minutes & UNITS_MASK) + ASCII_ADD_VALUE;
			Time_string[5] = ':';
			Time_string[6] = ((Current_Time.Seconds & MINSEC_TENS_MASK) >> SHIFT_FOUR) + ASCII_ADD_VALUE;
			Time_string[7] = (Current_Time.Seconds & UNITS_MASK) + ASCII_ADD_VALUE;
			Time_string[8] = ' ';
			Time_string[9] = 'A';
			Time_string[10] = 'M';
			Time_string[11] = '\0';
		}
	}
	LCDNokia_clear();/*! It clears the information printed in the LCD*/
	LCDNokia_gotoXY(5,0); /*! It establishes the position to print the messages in the LCD*/
	LCDNokia_sendString(Time_string); /*! It print a string stored in an array*/
	vTaskDelay(pdMS_TO_TICKS(500));
	LCDNokia_clear();/*! It clears the information printed in the LCD*/
	xSemaphoreGive(lcd_mutex);
	}
}

int main(void) {

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();
    Other_Time.Hours = 0x23;
    Other_Time.Minutes = 0x59;
    Other_Time.Seconds = 0x52;
    SPI_common_init();
    LCDNokia_init(); /*! Configuration function for the LCD */
    LCDNokia_clear();
    I2C_common_init();
    uart_Init();
    PCF8583_setHours(&Other_Time.Hours);
    PCF8583_setMinutes(&Other_Time.Minutes);
    PCF8583_setSeconds(&Other_Time.Seconds);
    Set_Format(false);
    format = Get_Format();
    lcd_mutex = xSemaphoreCreateMutex();
    Create_PcfHandles();
//	common_init();
    uint8_t test[] = "Esto es una prueba";
    uint8_t test_buffer[String_size(test)];
    MEM24LC256_write_Data(0x20, String_size(test), &test[0]);
    xTaskCreate(PCF_task, "taskPcf", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(task_lcd, "taskLcd", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(PC_Terminal_Task, "PC_Terminal_Task", 200, (void*)UART0, configMAX_PRIORITIES-1, NULL);
    vTaskStartScheduler();

    return 0 ;
}
