/*
 * 24LC256.h
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#ifndef MEM24LC256_H_
#define MEM24LC256_H_

#include "I2C_driver.h"


/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  This function uses the I2C to send the write instruction to be able to write the address
            high and low of the memory where we are storing data and then writes the data into the address.
    \param[in]  uint16 (HEX) address where we get the data from (gets separated into high and low).
    \param[in]  uint8 (HEX) data that is gonna be stored in the data address we get from the param.
    \return void
 */
uint8_t MEM24LC256_write_Data(uint16_t address, uint8_t* buffer);
/********************************************************************************************/
/********************************************************************************************/
/********************************************************************************************/
/*!
    \brief  This function uses the I2C to send the write instruction to be able to write the address
            high and low of the memory where we are storing data and then switches into read mode and receive
            to be able to get the data from the address we are sending.
    \param[in]  uint16 address where we get the data from (gets separated into high and low).
    \return the data from the memory address we give as a param.
 */
uint8_t* MEM24LC256_Read_Data(uint16_t address, uint8_t bytes, uint8_t* data);
uint32_t String_size(uint8_t* data);

#endif /* 24LC256_H_ */
