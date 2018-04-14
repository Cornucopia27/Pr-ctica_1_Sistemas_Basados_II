/*
 * init.c
 *
 *  Created on: Apr 13, 2018
 *      Author: ALEX
 */

#include "init.h"

//Time Current_Time = {0,0,0};
//SemaphoreHandle_t lcd_mutex;
//bool format;


//void common_init()
//{
//    Other_Time.Hours = 0x23;
//    Other_Time.Minutes = 0x59;
//    Other_Time.Seconds = 0x52;
//    SPI_common_init();
//    LCDNokia_init(); /*! Configuration function for the LCD */
//    LCDNokia_clear();
//    I2C_common_init();
////    uart_Init();
//    PCF8583_setHours(&Other_Time.Hours);
//    PCF8583_setMinutes(&Other_Time.Minutes);
//    PCF8583_setSeconds(&Other_Time.Seconds);
//    Set_Format(false);
//    format = Get_Format();
//    lcd_mutex = xSemaphoreCreateMutex();
//    Create_PcfHandles();
//}
