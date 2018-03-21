/*
 * I2C_driver.c
 *
 *  Created on: Mar 16, 2018
 *      Author: ALEX
 */

#include "I2C_driver.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "FreeRTOS.h"

bool g_MasterCompletionFlag = false;
i2c_master_transfer_t masterXfer;
i2c_master_handle_t g_i2cHandle; //handle created for the callback

//TODO cambiar a I2C1, usar semaforo en el callback para que no sea bloqueante, y revisar en el callback la variable del semaphoro binario, poner a dormir la tarea hasta ese punto
//TODO añadir función para saber cuanto mide el string y añadir parámetro a write data y read data de cuanto desea leer, escribir.
static void i2c_master_callback( I2C_Type *base, i2c_master_handle_t *handle,
        status_t status, void * userData )
{
    if ( status == kStatus_Success )
    {
        g_MasterCompletionFlag = true;
    }
}

void I2C_common_init()
{
    CLOCK_EnableClock( kCLOCK_PortE ); //for the I2C0 SCL and SDA enable
    CLOCK_EnableClock( kCLOCK_I2c0 );   //for the I2C0 clock enable

    PORT_SetPinMux( PORTE , 24, kPORT_MuxAlt5);//I2C0 SCL pin config
    PORT_SetPinMux( PORTE , 25, kPORT_MuxAlt5);//I2C0 SDA pin config

    static i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );
    masterConfig.baudRate_Bps = 115200;
//    masterConfig.enableMaster = true;
    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );

    I2C_MasterTransferCreateHandle( I2C0, &g_i2cHandle, i2c_master_callback,
            NULL );
    NVIC_SetPriority(I2C0_IRQn, 5);
}

uint8_t I2C_write_Data(uint16_t address, uint8_t* buffer)
{
    static i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );

    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );

    masterXfer.slaveAddress = MEM24LC256_GENERAL_ADDRESS;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddressSize = 2;
    masterXfer.subaddress = address;
    masterXfer.data = buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_i2cHandle, &masterXfer);
//    while((MASTER_XFER_SUCCEDED))
    return 0;
}

uint8_t I2C_read_Data(uint16_t address, uint8_t* buffer)
{
    static i2c_slave_config_t slaveConfig;
    I2C_SlaveGetDefaultConfig(&slaveConfig);

    I2C_SlaveInit(I2C0, &slaveConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );
    masterXfer.slaveAddress = MEM24LC256_GENERAL_ADDRESS;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddressSize = 2;
    masterXfer.subaddress = address;
    masterXfer.data = buffer;
    masterXfer.dataSize = 1;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_i2cHandle, &masterXfer);
//    while((MASTER_XFER_SUCCEDED))
    return masterXfer.data;
}
