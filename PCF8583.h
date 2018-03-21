/*
 * PCF8583.h
 *
 *  Created on: Mar 17, 2018
 *      Author: ALEX
 */

#ifndef PCF8583_H_
#define PCF8583_H_

void PCF8563_setSeconds(uint8_t data);
void PCF8563_setMinutes(uint8_t data);
void PCF8563_setHours(uint8_t data);
void PCF8563_setYears(uint8_t data);
void PCF8563_setMonths(uint8_t data);
void PCF8563_setDays(uint8_t data);

uint8_t PCF8563_getSeconds();
uint8_t PCF8563_getMinutes();
uint8_t PCF8563_getHours();
uint8_t PCF8563_getYears();
uint8_t PCF8563_getMonths();
uint8_t PCF8563_getDays();

#endif /* PCF8583_H_ */
