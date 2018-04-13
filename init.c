/*
 * init.c
 *
 *  Created on: Apr 13, 2018
 *      Author: ALEX
 */

#include "init.h"

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

void common_init()
{
    Other_Time.Hours = 0x23;
    Other_Time.Minutes = 0x59;
    Other_Time.Seconds = 0x52;
    SPI_common_init();
    LCDNokia_init(); /*! Configuration function for the LCD */
    LCDNokia_clear();
    I2C_common_init();
    PCF8583_setHours(&Other_Time.Hours);
    PCF8583_setMinutes(&Other_Time.Minutes);
    PCF8583_setSeconds(&Other_Time.Seconds);
    Set_Format(false);
    format = Get_Format();
    lcd_mutex = xSemaphoreCreateMutex();
    Create_PcfHandles();
}
