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

    i2c_master_config_t masterConfig;
    I2C_MasterGetDefaultConfig( &masterConfig );
    masterConfig.baudRate_Bps = 115200;
//    masterConfig.enableMaster = true;
    I2C_MasterInit( I2C0, &masterConfig, CLOCK_GetFreq( I2C0_CLK_SRC ) );

    I2C_MasterTransferCreateHandle( I2C0, &g_i2cHandle, i2c_master_callback,
            NULL );
}
