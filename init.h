/*
 * init.h
 *
 *  Created on: Apr 13, 2018
 *      Author: ALEX
 */

#ifndef INIT_H_
#define INIT_H_

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "FreeRTOS.h"
#include "I2C_driver.h"
#include "MEM24LC256.h"
#include "PCF8583.h"
#include "LCDNokia5110.h"
#include "UART.h"

#define HOURS_TENS_MASK 0x30
#define MINSEC_TENS_MASK 0XF0
#define UNITS_MASK 0X0F
#define TIME_STRING_FULL_SIZE 12
#define ASCII_ADD_VALUE 48
#define SHIFT_FOUR 4
#define DECIMAL_SPREADER 10
#define HALF_DAY 12



void task_lcd();
void common_init();

#endif /* INIT_H_ */
