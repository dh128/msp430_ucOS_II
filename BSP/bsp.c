/*
*********************************************************************************************************
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
*
*                                      Texas Instruments MSP430
*                                               on the
*                                          MSP-EXP430F5259LP
*                                          Evaluation Board
*
* Filename      : bsp.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/

#include  <bsp.h>
#include  <msp430.h>


/*
*********************************************************************************************************
*                                               CONSTANTS
*********************************************************************************************************
*/

#define  BSP_DLY_CONST      (BSP_CPU_CLK_FREQ / 1000000u)
#define  BSP_DLY_MAX        (0xFFFFFFFF / BSP_DLY_CONST)


/*
*********************************************************************************************************
*                                               DATA TYPES
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
OS_FLAG_GRP *EventWtFlag;        //喂狗时间标志组
uint16_t REGRST;
/*
*********************************************************************************************************
*                                               PROTOTYPES
*********************************************************************************************************
*/
void TimerBInit(void);
static void BSP_OSTickInit(void);
static void BSP_OSClockInit(void);
static void BSP_OSCloseWatchDog(void);
/*
******************************************************************************************************************************
******************************************************************************************************************************
**                                         Global Functions
******************************************************************************************************************************
******************************************************************************************************************************
*/

/*
*********************************************************************************************************
*                                                BSP_Init()
*
* Description : This function should be called by your application code before you make use of any of the
*               functions found in this module.
*
* Arguments   : None
*
* Returns     : None
*********************************************************************************************************
*/

void  BSP_Init(void)
{
    __disable_interrupt();                                      /* Disable all int. until we are ready to accept them   */
    
    // BSP_OSCloseWatchDog();
    InitWatchDog();
    BSP_OSClockInit();
    BSP_OSTickInit();                                           /* Initialize the OS tick timer   */
    // TimerBInit();
    g_Device_InnerRTC_Init();   
    g_Device_IO_Init();
    g_Device_Usart0_Init(9600);                                //通信模块串口
    g_Device_Usart1_Init(9600);                                //Socket串口
    g_Device_Usart2_Init(115200);                              //Debug串口
    g_Device_Usart3_Init(9600);                                //485串口
    g_Device_ADC_Init(); 
    g_Device_SD_Init();
    // g_Device_SPI3_Init();  //++++
    g_Device_InnerFlash_Init();
    hal_Delay_ms(100);
    g_Printf_info("BSP init over\r\n");
    g_Device_ExtRTC_Init();
    hal_Delay_ms(100);
    g_Printf_info("RTC init over\r\n");
#if HAVE_SDCARD_SERVICE
    SD_Status = g_Device_SDCard_Check();
    hal_Delay_ms(100);
    g_Printf_info("SD init over\r\n");
    OSBsp.Device.IOControl.PowerSet(SDCard_Power_Off);
#endif  
    Recive_485_Enable;
   // ScadaData_base_Init();   //后面已经执行了一�
    hal_Delay_ms(100);
}

/*******************************************************************************
* 函数名	: TimerBInit
* 描述	    : timerB0 初始化，用作OSTick,选择ASCLK作为时钟
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void TimerBInit(void)
{
    TBCCTL0 = CCIE;                           // CCR0 interrupt enabled
	TBCCR0 = 65;                             // 32--1ms,    65 -2ms
	TBCTL = TASSEL_1  + TACLR + MC_1;         // ACLK, upmode, clear TAR,START TIMERA
}
/*******************************************************************************
* 函数名	: InitWatchDog
* 描述	: 初始化看门狗定时器，喂狗也用此函数
* 输入参数  : 无
* 返回参数  : 无
* 备注	 :WDTCTL	寄存器只能写入，不能进行或与操作，因此喂狗操作也用此函数，需要同时设置时钟及间隔时间
*******************************************************************************/
void InitWatchDog(void)
{
//	WDTCTL = WDTPW + WDTCNTCL + WDTSSEL0 + WDTIS1;			//看门狗设置ACLK时钟，定时4分16秒
	WDTCTL = WDTPW + WDTCNTCL + WDTSSEL0 + WDTIS0;          // Set Watchdog Timer timeout 1小时8分16秒
        	//口令+清零+ACLK+时间间隔
}
/*
*********************************************************************************************************
*                                            BSP_OSTickInit()
*
* Description   : This function initializes the uC/OS-II's tick source.
*
* Argument    	: None
*
* Returns       : None
*********************************************************************************************************
*/

static void  BSP_OSTickInit(void)
{
//     WDTCTL  = WDT_MDLY_32;                                      /* This configuration produces interrupts every 32  ... */
//                                                                 /* ... ms for SMCLK = 1 MHz.                            */
//                                                                 /* BE SURE TO SET TICK RATE IN OS_CFG.H ...             */
//                                                                 /* ... TO CORRESPOND WITH THIS FREQUENCY!               */
//    //wj02200218 将看门狗做成了系统时钟源 实际2ms
//     SFRIE1 |= 1;                                                /* Enable WDT interrupts.                               */
    TimerBInit();
}

static void BSP_OSClockInit(void){
    // Use the REFO oscillator as the FLL reference, and also for ACLK
    UCSCTL3 = (UCSCTL3 & ~(SELREF_7)) | (SELREF__REFOCLK);   //设置FLL参考时钟为REFOCLK(内部32.768kHz晶振)
    UCSCTL4 = (UCSCTL4 & ~(SELA_7)) | (SELA__REFOCLK);       //ACLK参考时钟源REFOCLK,SMCLk--DOCCLKDIV,MCLK--DOCCLKDIV
    // Start the FLL, which will drive MCLK (not the crystal)
    Init_FLL(BSP_CPU_CLK_FREQ/1000, BSP_CPU_CLK_FREQ/32768);
}

static void BSP_OSCloseWatchDog(void)
{
	WDTCTL = WDTPW + WDTHOLD;       //CloseWatchDog
}



static void Init(void)
{
    BSP_Init();
}

SystemStruct OSBsp =
{
    Init,
};

void TaskRefreshWTD(OS_FLAG_GRP *GrpouFlag, OS_FLAGS flag)
{
    CPU_INT08U err;
    uint16_t res;
    res = OSFlagPost(GrpouFlag , flag, OS_FLAG_SET, &err);
    if(err != OS_ERR_NONE){
        g_Printf_dbg("%s %x failed\n",__func__,flag);
    }else{
        // g_Printf_dbg("%s event Flags = %x\n",__func__,res);
    }

}
// #pragma vector=TIMERB0_VECTOR
// __interrupt void TIMERB0_ISR(void)
// {
//     P2OUT ^= BIT1;
// }
