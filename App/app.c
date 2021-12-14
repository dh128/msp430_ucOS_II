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
static OS_STK WtdTaskStartStk[DEFAULT_TASK_STK_SIZE];
static OS_STK ScadaTaskStartStk[DEFAULT_TASK_STK_SIZE];
static OS_STK TransmitTaskStartStk[TRANSMIT_TASK_STK_SIZE];
static OS_STK ManagerTaskStartStk[DEFAULT_TASK_STK_SIZE];


char rebootBuffer[20]={0};
/*
*********************************************************************************************************
*                                            FUNCTION PROTOTYPES
*********************************************************************************************************
*/
static  void  ScadaTaskStart(void *p_arg);
static  void  WtdTaskStart(void *p_arg);
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
   REGRST = SYSRSTIV;
    OSInit();                                /* Initialize "uC/OS-II, The Real-Time Kernel"          */
    OSBsp.Init();                            /* Initialize BSP functions                             */
    sprintf(rebootBuffer, "\r\nReboot cause %d\r\n",REGRST);
//            g_Printf_dbg("EventWtFlag = %x\r\n",res);
    g_Printf_dbg(rebootBuffer);
//    g_Printf_dbg("Reboot cause %d",REGRST);
    if(Hal_Platform_Init() == 0){            
        g_Printf_info("Hal_Platform_Init Success\r\n");
        LED_OFF;
        hal_Delay_ms(1000);//延时1s

    }
    EventWtFlag = Hal_FlagCreate("wtd_event", WTD_BIT_NONE);
    if(EventWtFlag == NULL){
        g_Printf_info("Create watchdog evnet flag error\r\n");
    }
    Hal_ThreadCreate(WtdTaskStart,
                    (void *)"WtdTaskStart",
                    &WtdTaskStartStk[DEFAULT_TASK_STK_SIZE-1u],
                    WTD_TASK_PRIO);

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

void  WtdTaskStart (void *p_arg)
{
//	char buffer[50]={0};
    CPU_INT08U err;
    uint16_t res = 0;
    while(1)
    {
        res = OSFlagPend(EventWtFlag, WTD_BIT_ALL, OS_FLAG_WAIT_SET_ALL|OS_FLAG_CONSUME, 1000, &err);
        if(err == OS_ERR_NONE){     //获取到全部标志组
            //喂狗
        	InitWatchDog();
            // g_Printf_dbg("EventWtFlag = %x\r\n",res);
            // g_Printf_dbg("feed dog\r\n");
        }else{
//        	sprintf(buffer, "EventWtFlag = %x\r\n",res);
//            g_Printf_dbg("EventWtFlag = %x\r\n",res);
        	// g_Printf_dbg("Event wait timeout\r\n");
        }
        OSTimeDly(1000);
    }
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

	static uint8_t scada_idle_times = 0;
    static uint8_t scada_over_times = 0;
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.       */
        if(Hal_getCurrent_work_Mode() == 0){     //如果不在休眠期
          TaskRefreshWTD(EventWtFlag , WTD_BIT_SCADA);
            if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_OFF){  //如果设备没上电10s预热，第一次还会读传感器数据
                OSTimeDly(30);
                scada_over_times = 0;
                scada_idle_times = 0;
                //  OSTimeDly(6000);  //节拍2ms  //TEST
                g_Printf_info("SenSor_Power_On\r\n");
                OSBsp.Device.IOControl.PowerSet(BaseBoard_Power_On);
                OSBsp.Device.IOControl.PowerSet(Sensor_Power_On);
                OSBsp.Device.IOControl.PowerSet(Max485_Power_On);

                if(App.Data.TerminalInfoData.SendPeriod > NO_LOWPER_PERIOD) //1~3min不进入低功耗，五预热
                {//wj20200215 上传频率大于NO_LOWPER_PERIOD min不让系统进低功耗不关传感器电源，也就不需要预热，这边对应取消TransmitTaskStart、RTC_ISR里面进低功耗
                    //个别传感器需预热，任务挂起时间视情况而定，默认10s
                        OSTimeDly(5000);  //节拍2ms=10s
                }

                if(App.Data.TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_NOTYET) {//如果未读取flash里的传感器置位
                    InqureSensor();  //这里面只会执行检查哪些传感器在线，并记录
                }
                AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCANNING;
                g_Printf_info("%s ... ...\n",__func__);
                Hal_GetTimeOfDay(&before_Scada);
            }
            else if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCANNING){ //设备上电后执行20s的传感器读取
                g_Printf_info("Sensor Scanning... ...\r\n");
                InqureSensor();
                Hal_GetTimeOfDay(&after_Scada);
                Scada_timeout_sec = after_Scada.tv_sec - before_Scada.tv_sec;
                g_Printf_info("Scada_timeout_sec = %d\r\n",Scada_timeout_sec);
                if(Scada_timeout_sec >= SCADATIME){//如果数据上传函数也在激活状态，很有可能上传时间把SCADATIME占用，导致提前结束
                    //if(Scada_timeout_sec >= 20){
                    #if (PRODUCT_TYPE == Flowmeter_Station)
                        CalcData();
                        OSTimeDly(1000);OSTimeDly(1000);OSTimeDly(500);
                    #endif
                    AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCAN_OVER;
                    g_Printf_info("ScadaTask is over\n");
                }
            }
            else if (AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER) //空跑2min
            {
                scada_over_times ++;
                OSTimeDlyHMSM(0u, 0u, 2u, 0u);
                if (scada_over_times == 60)  //120+120s 空跑4min
                {
                    scada_over_times = 0;
#if(TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)                    
                    if(NB_Fota == 0){
                        g_Printf_dbg("DeviceStatus always scan_over, enter low_power!\r\n");
                        Hal_EnterLowPower_Mode();
                    }
#else
                    g_Printf_dbg("DeviceStatus always scan_over, enter low_power!\r\n");
                    Hal_EnterLowPower_Mode();
#endif
                }
                if(App.Data.TerminalInfoData.SendPeriod <= NO_LOWPER_PERIOD)
                {
#if(TRANSMIT_TYPE == GPRS_Mode)
                    //注意，如果httpinit已经完成，说明要发送数据了，就不置位
                    if(AppDataPointer->TransMethodData.GPRSStatus < GPRS_Http_Init_Done)
                    {
                            //使scada函数能够一直运行1s/次(0u, 0u, 0u, 500u)时，其他值类比
                            AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;//让scada函数再次启动
                    }
#endif

                }
        
            }
            else  //DEVICE_STATUS_POWER_IDLE
            {
                g_Printf_info("ScadaTask idle!\n");
                scada_idle_times++;
                OSTimeDlyHMSM(0u, 0u, 2u, 0u);
                if (scada_idle_times == 60)  //120+120s 空跑4min
                {
                    g_Printf_dbg("DeviceStatus always idle, low power!\r\n");
                    scada_idle_times = 0;
                    Hal_EnterLowPower_Mode();
                    // hal_Reboot();  //复位 是否需要主机都复位？或者只需要
                }
            }

            OSTimeDlyHMSM(0u, 0u, 2u, 0u);    //放弃控制权，让上传函数运行

        }
        else
        {
        	g_Printf_dbg("ScadaTaskStart ERR!\r\n");
        	OSTimeDlyHMSM(0u, 0u, 0u, 100u);    //放弃控制权，让上传函数运行
        }
    }
}

