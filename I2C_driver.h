/*
 * I2C_driver.h
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"
#include "fsl_port.h"
#include "FreeRTOS.h"

void I2C_common_init();
uint8_t I2C_write_Data(uint16_t slave_address, uint16_t address, uint16_t data_size, uint8_t* buffer);
uint8_t I2C_read_Data(uint16_t slave_address, uint16_t address, uint16_t data_size, uint8_t* buffer);
#endif /* I2C_DRIVER_H_ */
