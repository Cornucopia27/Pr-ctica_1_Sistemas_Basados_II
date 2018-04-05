/*
 * PCF8583.c
 *
 *  Created on: Mar 17, 2018
 *      Author: ALEX
 */
#include "PCF8583.h"

SemaphoreHandle_t pcf_semaphore;
SemaphoreHandle_t pcf_mutex;

void PCF8583_setSeconds(uint8_t* buffer)
{
    I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, SECONDS_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getSeconds(uint8_t* buffer)
{
    return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, SECONDS_ADDRESS, 1, buffer);
}

void PCF8583_setMinutes(uint8_t* buffer)
{
	I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, MINUTES_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getMinutes(uint8_t* buffer)
{
	return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, MINUTES_ADDRESS, 1, buffer);
}

void PCF8583_setHours(uint8_t* buffer)
{
    I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, HOURS_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getHours(uint8_t* buffer)
{
    return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, HOURS_ADDRESS, 1, buffer);
}

void PCF8583_setDays(uint8_t* buffer)
{
    I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, DAYS_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getDays(uint8_t* buffer)
{
    return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, DAYS_ADDRESS, 1, buffer);
}

void PCF8583_setMonths(uint8_t* buffer)
{
    I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, MONTHS_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getMonths(uint8_t* buffer)
{
    return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, MONTHS_ADDRESS, 1, buffer);
}

void PCF8583_setYears(uint8_t* buffer)
{
    I2C_write_Data(PCF8563_GENERAL_ADDRESS, 1, YEARS_ADDRESS, 1, buffer);
}

uint8_t* PCF8583_getYears(uint8_t* buffer)
{
    return I2C_read_Data(PCF8563_GENERAL_ADDRESS, 1, YEARS_ADDRESS, 1, buffer);
}

//PCF_task()
//{
//	for(;;)
//	{
//		xSemaphoreTake(pcf_semaphore, portMAX_DELAY);
//		xSemaphoreTake(pcf_mutex, portMAX_DELAY);
//
//	}
//}
//Full_Date current_Date = {{0,0,0},{0,0,0}};

//void updateSystemTimeDate(){//update the structure values
//	current_Date.currentTime.Seconds = (PCF8583_getSeconds());
//	current_Date.currentTime.Minutes =	(PCF8583_getMinutes());
//	current_Date.currentTime.Hours =	(PCF8583_getHours());
//
//	current_Date.currentDate.Years = (PCF8583_getYears());
//	current_Date.currentDate.Months = (PCF8583_getMonths());
//	current_Date.currentDate.Days = (PCF8583_getDays());
//}
