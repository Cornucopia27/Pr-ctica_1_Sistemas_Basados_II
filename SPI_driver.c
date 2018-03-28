/*
 * SPI_driver.c
 *
 *  Created on: Mar 28, 2018
 *      Author: ALEX
 */

#include "SPI_driver.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "FreeRTOS.h"

bool g_MasterCompletionFlag = false;
dspi_transfer_t masterXfer;
dspi_master_handle_t g_spiHandle; //handle created for the callback

static void spi_master_callback( SPI_Type *base, dspi_master_handle_t *handle,
        status_t status, void * userData )
{
    if ( status == kStatus_Success )
    {
        g_MasterCompletionFlag = true;
    }
}

void SPI_common_init()
{
    CLOCK_EnableClock( kCLOCK_PortD ); //for the SPI0 sck and sout
    CLOCK_EnableClock( kCLOCK_Spi0 );   //for the SPI0 clock enable

    //spi_0 pins configuration
    port_pin_config_t config_spi = { kPORT_PullDisable, kPORT_SlowSlewRate,
        kPORT_PassiveFilterDisable, kPORT_OpenDrainDisable,
        kPORT_LowDriveStrength, kPORT_MuxAlt2, kPORT_UnlockRegister, };
    PORT_SetPinConfig(PORTD, 1, &config_spi);  //SPI0 SCK pin configuration
    PORT_SetPinConfig(PORTD, 2, &config_spi);  //SPI0 SOUT pin configuration

    static dspi_master_config_t masterConfig;
    DSPI_MasterGetDefaultConfig( &masterConfig );

    DSPI_MasterInit( SPI0, &masterConfig, CLOCK_GetFreq(DSPI0_CLK_SRC));
    DSPI_MasterTransferCreateHandle( SPI0 , &g_spiHandle, spi_master_callback, NULL);

    NVIC_EnableIRQ(SPI0_IRQn);
    NVIC_SetPriority(SPI0_IRQn, 5);
}
