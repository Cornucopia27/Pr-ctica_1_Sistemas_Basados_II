/*
 * 24LC256.c
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#include "MEM24LC256.h"
#include "FreeRTOS.h"


uint8_t MEM24LC256_write_Data(uint16_t address, uint16_t data_size, uint8_t* buffer)
{
    I2C_write_Data(MEM24LC256_GENERAL_ADDRESS,address,data_size, buffer);
    return 0;
}

uint8_t* MEM24LC256_Read_Data(uint16_t address, uint16_t data_size, uint8_t* buffer)
{

   return I2C_read_Data(MEM24LC256_GENERAL_ADDRESS,address,data_size, buffer);
}

uint16_t String_size(uint8_t* data)
{
    uint16_t letter_counter = 0;
    while(data[letter_counter] != '\0')
    {
        letter_counter ++;
    }
    return letter_counter;
}
