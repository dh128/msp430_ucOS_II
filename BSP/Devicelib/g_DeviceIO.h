/*******************************************************************************
*
*    File Name:  g_DeviceIO.h
*    Revision:  1.0
*
*    Description:  IO初始化
*
*
* *****************************************************************************/
#ifndef G_DEVICEIO_H
#define G_DEVICEIO_H

typedef enum {
    // BaseBoard_Power_On = 0,
    // BaseBoard_Power_Off = 1,
    // LPModule_Power_On,
    // LPModule_Power_Off,
    // GPRS_Power_On,
    // GPRS_Power_Off,
    // SDCARD_Power_On,
    // SDCARD_Power_Off,
    // GPS_Power_On,
    // GPS_Power_Off,
    // SenSor_Power_On,
    // SenSor_Power_Off,
    // Motor_Power_On,
    // Motor_Power_Off,
    // AIR202_Power_On,
    // AIR202_Power_Off

    BaseBoard_Power_On = 0,
    BaseBoard_Power_Off = 1,
    BaseBoard_12V_Power_On,
    BaseBoard_12V_Power_Off,
    BaseBoard_5V_Power_On,
    BaseBoard_5V_Power_Off,
    Sensor_Power_On,
    Sensor_Power1_On,
    Sensor_Power2_On,   
    Sensor_Power_Off,
    Sensor_Power1_Off,
    Sensor_Power2_Off,   
    AIR202_Power_On,
    AIR202_Power_Off,
    SIM800C_Power_On,
    SIM800C_Power_Off,
    LPModule_Power_On,
    LPModule_Power_Off,
    Base3V3_Power_On,
    Base3V3_Power_Off,
    GPS_Power_On,
    GPS_Power_Off,
    SDCard_Power_On,
    SDCard_Power_Off,
    Max485_Power_On,
    Max485_Power_Off,
    Motor_Power_On,
    Motor_Power_Off
}ControlPower;


//****************LED控制预定义******************//
#define LED_ON	              P2OUT |= BIT1;    //工作状态指示LED 打开
#define LED_OFF               P2OUT &=~ BIT1    //工作状态指示LED 关闭
//****************485控制预定义******************//
#define Send_485_Enable		  P2OUT |= BIT5	    //485使能  发送
#define Recive_485_Enable	  P2OUT &=~BIT5	    //485使能  接收

//******************总电源***********************//
#define Base_12V_ON           P6OUT |= BIT2	    //12V总电源  上电
#define Base_12V_OFF	      P6OUT &=~BIT2 	//12V总电源  掉电
#define Base_5V_ON            P6OUT |= BIT7	    //5V总电源   上电
#define Base_5V_OFF	          P6OUT &=~BIT7 	//5V总电源   掉电
//*****************3.3V电源*********************////为TF卡、SPI Flash、MAX3485、Socket_3V3供电
#define Base_3V3_ON		      P4OUT |= BIT0     //3.3V电源   上电
#define Base_3V3_OFF          P4OUT &=~BIT0     //3.3V电源   掉电
#define W25Q16_ON             P6OUT &= ~BIT1    //W25Q16 3.3V电源   上电
#define W25Q16_OFF            P6OUT |= BIT1     //W25Q16 3.3V电源   掉点
//*****************传感器电源********************//
#define Sensor_12V_1_ON		  P1OUT |= BIT4	    //传感器电源接口1_12V 上电
#define Sensor_12V_1_OFF      P1OUT &=~BIT4 	//传感器电源接口1_12V 掉电
#define Sensor_12V_2_ON		  P1OUT |= BIT5	    //传感器电源接口2_12V 上电
#define Sensor_12V_2_OFF	  P1OUT &=~BIT5 	//传感器电源接口2_12V 掉电
#define Sensor_5V_1_ON		  P1OUT |= BIT3	    //传感器电源接口1_5V  上电
#define Sensor_5V_1_OFF		  P1OUT &=~BIT3 	//传感器电源接口1_5V  掉电
#define Sensor_5V_2_ON		  P6OUT |= BIT4	    //传感器电源接口2_5V  上电
#define Sensor_5V_2_OFF		  P6OUT &=~BIT4 	//传感器电源接口2_5V  掉电
//****************通信模块电源*******************//
#define Transmit_5V_ON		  P4OUT |= BIT2     //通信模块5V电源      上电
#define Transmit_5V_OFF       P4OUT &=~BIT2     //通信模块5V电源      掉电
#define Transmit_HighPower	  P2OUT |= BIT4     //通信模块5V模块      高电平
#define Transmit_LowPower	  P2OUT &=~BIT4     //通信模块5V模块      低电平
#define Transmit_3V3_ON		  P4OUT &=~BIT3     //通信模块3.3V电源    上电   由MCU_3V3 供电
#define Transmit_3V3_OFF      P4OUT |= BIT3     //通信模块3.3V电源    掉电
//*****************插件电源**********************//
#define Socket_5V_ON		  P6OUT |= BIT5     //插口5V电源         上电
#define Socket_5V_OFF         P6OUT &=~BIT5     //插口5V电源         掉电
#define Socket_3V3_ON		  P4OUT &=~BIT1     //插口3.3V电源       上电
#define Socket_3V3_OFF        P4OUT |= BIT1     //插口3.3V电源       掉电
#define Socket_485_SendEnable   P1OUT |= BIT6     //插口485芯片        发送使能 
#define Socket_485_RevEnable    P1OUT &=~BIT6     //插口485芯片        接收使能 
//*****************电池电量采集******************//
#define ScadaBAT_ON		      P2OUT |= BIT0     //电池电量采集       允许
#define ScadaBAT_OFF          P2OUT &=~BIT0     //电池电量采集       禁止

#define CS_1		          P2OUT |= BIT2     
#define CS_0                  P2OUT &=~BIT2     

void g_Device_IO_Init(void);



#endif
