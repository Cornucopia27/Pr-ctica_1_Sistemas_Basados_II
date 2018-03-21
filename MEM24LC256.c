/*
 * 24LC256.c
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#include "MEM24LC256.h"
#include "FreeRTOS.h"


uint8_t MEM24LC256_write_Data(uint16_t address, uint8_t* buffer)
{
    I2C_write_Data(address, buffer);
    return 0;
}

uint8_t* MEM24LC256_Read_Data(uint16_t address,uint8_t bytes, uint8_t* data)
{
    ;
//    return I2C_read_Data(address, data);
}

uint32_t String_size(uint8_t* data)
{
    uint32_t letter_counter = 0;
    while(data[letter_counter] != '\0')
    {
        letter_counter ++;
    }
    return letter_counter;
}
