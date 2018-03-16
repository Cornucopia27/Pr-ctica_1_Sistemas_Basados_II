/*
 * 24LC256.c
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#include "MEM24LC256.h"
#include "FreeRTOS.h"


i2c_master_handle_t mem_transfer_handle; //handle created for the callback

uint8_t MEM24LC256_write_Data(uint16_t address, uint8_t* buffer)
{
    masterXfer.slaveAddress = MEM24LC256_GENERAL_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddressSize = 2;
    masterXfer.subaddress = address;
    masterXfer.data = &buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_i2cHandle, &masterXfer);
//    while((MASTER_XFER_SUCCEDED))
}
