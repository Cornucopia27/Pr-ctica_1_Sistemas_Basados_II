/*
 * PCF8583.c
 *
 *  Created on: Mar 17, 2018
 *      Author: ALEX
 */
#include "PCF8583.h"

SemaphoreHandle_t pcf_semaphore;
SemaphoreHandle_t pcf_mutex;
QueueHandle_t pcf_queue;
static bool format;

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

void PCF_task()
{
	static uint8_t buffer[3];
	static Time time_passer;
	for(;;)
	{
		xSemaphoreTake(pcf_semaphore, portMAX_DELAY);
		xSemaphoreTake(pcf_mutex, portMAX_DELAY);
		PCF8583_getHours(&buffer[0]);
		PCF8583_getMinutes(&buffer[1]);
		PCF8583_getSeconds(&buffer[2]);
		time_passer.Hours = buffer[0];
		time_passer.Minutes = buffer[1];
		time_passer.Seconds = buffer[2];
		xSemaphoreGive(pcf_mutex);
		xQueueSend(pcf_queue, &time_passer, portMAX_DELAY);
	}
}

Time PCF_request()
{
	Time time_passer;
	xSemaphoreGive(pcf_semaphore);
	xQueueReceive(pcf_queue, &time_passer, portMAX_DELAY);
	return time_passer;
}

void Create_PcfHandles()
{
    pcf_semaphore = xSemaphoreCreateBinary();
    pcf_mutex = xSemaphoreCreateMutex();
    pcf_queue = xQueueCreate(1, sizeof(Time*));
}

void Set_Format(bool val)
{
	format = val;
}

bool Get_Format()
{
	return format;
}
