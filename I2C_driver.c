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

#define EVENT_I2C (1<<0);

bool g_MasterCompletionFlag = false;
i2c_master_transfer_t masterXfer;
i2c_master_handle_t g_i2cHandle; //handle created for the callback
//SemaphoreHandle_t i2c_sempahore;
//EventGroupHandle_t i2c_event;

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
    CLOCK_EnableClock( kCLOCK_PortB ); //for the I2C0 SCL and SDA enable
    CLOCK_EnableClock( kCLOCK_I2c0 );   //for the I2C0 clock enable

    //I2C_0 pins configuration
    port_pin_config_t config_i2c = { kPORT_PullUp, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTB, 2, &config_i2c);  //I2C0 SCL pin configuration #6
    PORT_SetPinConfig(PORTB, 3, &config_i2c);  //I2C0 SDA pin configuration #5

    static i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );
    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );

    I2C_MasterTransferCreateHandle( I2C0, &g_i2cHandle, i2c_master_callback,
            NULL );

    NVIC_EnableIRQ(I2C0_IRQn);
    NVIC_SetPriority(I2C0_IRQn, 5);
}

//TODO poner un param de slave address
uint8_t I2C_write_Data(uint16_t slave_address, uint8_t subaddress_size, uint16_t address, uint16_t data_size, uint8_t* buffer)
{
    static i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );

    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );

    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Write;
    masterXfer.subaddressSize = subaddress_size;
    masterXfer.subaddress = address;
    masterXfer.data = buffer;
    masterXfer.dataSize = data_size;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_i2cHandle, &masterXfer);
    while (!g_MasterCompletionFlag)
        {
        }
    g_MasterCompletionFlag = false;
    return 0;
}

uint8_t I2C_read_Data(uint16_t slave_address, uint8_t subaddress_size, uint16_t address, uint16_t data_size, uint8_t* buffer)
{
    masterXfer.slaveAddress = slave_address;
    masterXfer.direction = kI2C_Read;
    masterXfer.subaddressSize = subaddress_size;
    masterXfer.subaddress = address;
    masterXfer.data = buffer;
    masterXfer.dataSize = data_size;
    masterXfer.flags = kI2C_TransferDefaultFlag;

    I2C_MasterTransferNonBlocking(I2C0, &g_i2cHandle, &masterXfer);
    while (!g_MasterCompletionFlag)
        {
        }
    g_MasterCompletionFlag = false;
    return 0;
}
