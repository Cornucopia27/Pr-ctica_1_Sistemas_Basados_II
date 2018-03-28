/*
 * SPI_driver.h
 *
 *  Created on: Mar 28, 2018
 *      Author: ALEX
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_dspi.h"
#include "fsl_port.h"
#include "FreeRTOS.h"

void SPI_common_init();

#endif /* SPI_DRIVER_H_ */
