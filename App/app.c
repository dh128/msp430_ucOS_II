/*
*********************************************************************************************************
*                                              EXAMPLE CODE
*
*                          (c) Copyright 2003-2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved.  Protected by international copyright laws.
*               Knowledge of the source code may NOT be used to develop a similar product.
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                      Texas Instruments MSP430
*                                               on the
*                                          MSP-EXP430F5259LP
*                                          Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/

#include  <ucos_ii.h>
#include  <cpu.h>
#include  <bsp.h>
#include  <lib_def.h>
#include  <app_cfg.h>
#include  <msp430.h>
#include  <hal_layer_api.h>


/*
*********************************************************************************************************
*                                                 DEFINES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                VARIABLES
*********************************************************************************************************
*/

static OS_STK ScadaTaskStartStk[DEFAULT_TASK_STK_SIZE];
static OS_STK TransmitTaskStartStk[TRANSMIT_TASK_STK_SIZE];
static OS_STK ManagerTaskStartStk[DEFAULT_TASK_STK_SIZE];

/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void  ScadaTaskStart(void *p_arg);


/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary CPU and C initialization.
*
* Arguments   : none
*********************************************************************************************************
*/
void  main (void)
{
    OSInit();                                /* Initialize "uC/OS-II, The Real-Time Kernel"          */
    OSBsp.Init();                            /* Initialize BSP functions                             */
    if(Hal_Platform_Init() == 0){            
        g_Printf_info("Hal_Platform_Init Success\r\n");
        LED_OFF;
        hal_Delay_ms(100);
    }

    Hal_ThreadCreate(ScadaTaskStart,
                    (void *)"ScadaTaskStart",
                    &ScadaTaskStartStk[DEFAULT_TASK_STK_SIZE-1u],
                    SCADA_TASK_TASK_PRIO);
                    
    Hal_ThreadCreate(TransmitTaskStart,
                    (void *)"TransmitTaskStart",
                    &TransmitTaskStartStk[TRANSMIT_TASK_STK_SIZE-1u],
                    TRANSMIT_TASK_TASK_PRIO);

    Hal_ThreadCreate(ManagerTaskStart,
                    (void *)"ManagerTaskStart",
                    &ManagerTaskStartStk[DEFAULT_TASK_STK_SIZE-1u],
                    MANAGER_TASK_TASK_PRIO);


    OSStart();                               /* Start multitasking (i.e. give control to uC/OS-II)   */
}



static  void  ScadaTaskStart (void *p_arg)
{
    
    (void)p_arg;
#if (OS_TASK_STAT_EN > 0)
    OSStatInit();                            /* Determine CPU capacity                       */
#endif
	struct hal_timeval before_Scada;		
	struct hal_timeval after_Scada;
	int32_t	Scada_timeout_sec;
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.       */

    
        if(Hal_getCurrent_work_Mode() == 0){
            if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_OFF){
                // g_Printf_info("SenSor_Power_On\r\n");
                OSBsp.Device.IOControl.PowerSet(BaseBoard_Power_On);
                OSBsp.Device.IOControl.PowerSet(Sensor_Power_On);
                OSBsp.Device.IOControl.PowerSet(Max485_Power_On);
                //个别传感器需预热，任务挂起时间视情况而定，默认10s
                OSTimeDly(5000);  //节拍2ms
                if(App.Data.TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_NOTYET) {
                    InqureSensor();  
                }
                AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCANNING;
                // g_Printf_info("%s ... ...\n",__func__);
                Hal_GetTimeOfDay(&before_Scada);
            }else if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCANNING){
                InqureSensor();
                Hal_GetTimeOfDay(&after_Scada);
                Scada_timeout_sec = after_Scada.tv_sec - before_Scada.tv_sec;
                // g_Printf_info("Scada_timeout_sec = %d\r\n",Scada_timeout_sec);
                // if(Scada_timeout_sec >= SCADATIME){
                if(Scada_timeout_sec >= 30){
                // if(Scada_timeout_sec >= 10){
                AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCAN_OVER;
                // g_Printf_info("ScadaTask is over\n");
                OSTimeDly(500);
                // OSBsp.Device.IOControl.PowerSet(Max485_Power_Off);
                // // OSBsp.Device.IOControl.PowerSet(BaseBoard_Power_Off);
                // OSBsp.Device.IOControl.PowerSet(Sensor_Power_Off);
                }
            }
            
            OSTimeDlyHMSM(0u, 0u, 1u, 0u);  
        }
    }
}



