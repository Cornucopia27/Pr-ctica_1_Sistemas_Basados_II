/*
 * PCF8583.h
 *
 *  Created on: Mar 17, 2018
 *      Author: ALEX
 */

#ifndef PCF8583_H_
#define PCF8583_H_

#include "I2C_driver.h"
#define PCF8563_GENERAL_ADDRESS 0X51
#define SECONDS_ADDRESS 0x02 // (HEX) address where seconds are.
#define MINUTES_ADDRESS 0x03 // (HEX) address where minutes are.
#define HOURS_ADDRESS 0x04 // (HEX) address where hours are.
#define DAYS_ADDRESS 0x05 // (HEX) address where days are.
#define MONTHS_ADDRESS 0x07 // (HEX) address where months are.
#define YEARS_ADDRESS 0x08 // (HEX) address where years are.

/*! This data type represents the Time values*/
typedef struct{
	uint8_t Seconds;
	uint8_t Minutes;
	uint8_t Hours;
}Time;
/*! This data type represents the Date values*/
typedef struct{
	uint8_t Years;
	uint8_t Months;
	uint8_t Days;
}Date;

typedef struct{
	Time currentTime;
	Date currentDate;
}Full_Date;

void PCF8583_setSeconds(uint8_t* buffer);
void PCF8583_setMinutes(uint8_t* buffer);
void PCF8583_setHours(uint8_t* buffer);
void PCF8583_setYears(uint8_t* buffer);
void PCF8583_setMonths(uint8_t* buffer);
void PCF8583_setDays(uint8_t* buffer);

uint8_t* PCF8583_getSeconds(uint8_t* buffer);
uint8_t* PCF8583_getMinutes(uint8_t* buffer);
uint8_t* PCF8583_getHours(uint8_t* buffer);
uint8_t* PCF8583_getYears(uint8_t* buffer);
uint8_t* PCF8583_getMonths(uint8_t* buffer);
uint8_t* PCF8583_getDays(uint8_t* buffer);


void updateSystemTimeDate();
#endif /* PCF8583_H_ */
