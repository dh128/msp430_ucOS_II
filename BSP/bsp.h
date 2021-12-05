/*
*********************************************************************************************************
*                                         BOARD SUPPORT PACKAGE
*
*                            (c) Copyright 2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved. Protected by international copyright laws.
*
*               BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                 Texas Instruments MSP-EXP430F5529LP
*                                               LaunchPad
*
* Filename      : bsp.h
* Version       : V1.00
* Programmer(s) : HS
*********************************************************************************************************
*/

#ifndef  __BSP_H__
#define  __BSP_H__

#include <string.h>
#include <stdio.h>
#include <intrinsics.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <math.h>
#include <time.h>

#include <hal_layer_api.h>
#include <hal_device.h>
#include <hal_macros.h>
#include <hal_pmm.h>
#include <hal_tlv.h>
#include <hal_types.h>
#include <hal_ucs.h>

#include <cjson.h>
#include <malloc.h>
#include <utils_hmac.h>
#include <utils_md5.h>
#include <utils_sha1.h>

#include <g_DeviceIO.h>
#include <g_DeviceUart.h>
#include <g_DeviceRTC.h>
#include <g_DeviceADC.h>
#include <g_DeviceSpi.h>
#include <g_DeviceSD.h>
#include <g_DeviceSDMMC.h>
#include <g_DeviceFlash.h>
#include <g_DevicePrintf.h>
#include <g_DeviceConfig.h>
//#include <g_DeviceDisplay.h>
#include <g_DeviceSPIFlash.h>

#include <g_DeviceGprs.h>
#include <g_DeviceNB.h>
#include <g_DeviceLoRa.h>
#include <g_DeviceNB_MQTT.h>
#include <g_DeviceNB_AEP.h>
#include <g_Platform.h>


#include <Diskio.h>
#include <FatFS.h>
#include <Ffconf.h>
#include <FileTxt.h>
/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

#define  BSP_CPU_CLK_FREQ       16000000u                       /* Frequency in Hz                                      */

extern OS_FLAG_GRP *EventWtFlag;        //喂狗时间标志组
#define     WTD_BIT_NONE            0
#define     WTD_BIT_SCADA           (1<<0)      //Scada任务喂狗位
#define     WTD_BIT_TRANSMIT        (1<<1)      //Transmit任务喂狗位
#define     WTD_BIT_MANAGER         (1<<2)      //Manager任务喂狗位
#define     WTD_BIT_UARTREC         (1<<3)      //UartRec任务喂狗位
#define     WTD_BIT_ALL             (WTD_BIT_SCADA|WTD_BIT_TRANSMIT|WTD_BIT_MANAGER|WTD_BIT_UARTREC)      //喂狗所有标志位

extern uint16_t REGRST;
/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
typedef struct
{
    void (*Init)(void);

    struct Device
    {
        struct Usart0
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint8_t len);
            void (*WriteString)(char *s);
        }Usart0;

        struct Usart1
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint8_t len);
            void (*WriteString)(char *s);
        }Usart1;

        struct Usart2
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint8_t len);
            void (*WriteString)(char *s);
        }Usart2;

        struct Usart3
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint8_t len);
            void (*WriteString)(char  *s);
        }Usart3;

        struct Spi2
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint16_t len);
            uint8_t (*WriteReadData)(uint8_t data);
        }Spi2;

        struct Spi3
        {
            void (*WriteData)(uint8_t data);
            void (*WriteNData)(uint8_t *data,uint8_t len);
            uint8_t (*WriteReadData)(uint8_t data);
        }Spi3;

//        struct Timer
//        {
//            void (*Start)(uint8_t id, TimerModeEnum mode, uint64_t times, function functionPointer);
//            void (*Stop)(uint8_t id);
//        }Timer;

        struct InnerFlash
        {
        	uint8_t (*innerFLASHRead)(uint8_t index,uint32_t add);
        	void (*innerFLASHWrite)(uint8_t *data_ptr,uint8_t *flashAddr,uint16_t count);
        	void (*FlashRsvWrite)(uint8_t *data_ptr,uint8_t index,uint32_t infostartaddr,uint8_t num);
        }InnerFlash;

        struct IOControl
        {
            void (*PowerSet)(ControlPower src);
            void (*ResetWirelesModule)(void);

        }IOControl;

        struct RTC
        {
            void (*ReadExtTime)(uint8_t *data,uint8_t dataType);
            void (*ConfigExtTime)(uint8_t *data,uint8_t dataType);
        }RTC;
    }Device;


}SystemStruct;

extern SystemStruct OSBsp;
void InitWatchDog(void);
void TaskRefreshWTD(OS_FLAG_GRP *GrpouFlag, OS_FLAGS flag);
/*
*********************************************************************************************************
*                                          FUNCTION PROTOTYPES
*********************************************************************************************************
*/


#endif
