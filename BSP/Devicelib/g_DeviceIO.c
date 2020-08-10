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
* Filename      : g_DeviceIO.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <hal_layer_api.h>
#include  <bsp.h>


/*******************************************************************************
* 函数名		: SetAllSensorPowerOn
* 描述	    	: 打开/关闭传感器板所有传感器电源
* 输入参数  	: onoff
* 返回参数      : 无
*******************************************************************************/
static void SetAllSensorPowerOn(bool onoff)
{
	if(onoff == true){
//******************总电源***********************//
	Base_12V_ON;                  //12V总电源  上电
	Base_5V_ON;                   //5V总电源   上电
//*****************3.3V电源*********************////为TF卡、SPI Flash、MAX3485、Socket_3V3供电
	Base_3V3_ON;                  //3.3V电源   上电
//*****************传感器电源********************//
	Sensor_12V_1_ON;              //传感器电源接口1_12V 上电
	Sensor_12V_2_ON;              //传感器电源接口2_12V 上电
	Sensor_5V_1_ON;               //传感器电源接口1_5V  上电
	Sensor_5V_2_ON;               //传感器电源接口2_5V  上电
		
#if (PRODUCT_TYPE == AliveNest_Station) 
		Send_485_Enable;
		hal_Delay_us(5);
		OSBsp.Device.Usart3.WriteNData(SwitchPMS5003T_Passive,PTCMDLength); //将状态切换为被动式
		hal_Delay_us(1);
		Recive_485_Enable;
		OSBsp.Device.Usart2.WriteString("PMS5003T Passive State Switch OK!\r\n");
#endif
	}else{
//******************总电源***********************//
	Base_12V_OFF;                 //12V总电源  掉电
	Base_5V_OFF;                  //5V总电源   掉电
//*****************3.3V电源*********************////为TF卡、SPI Flash、MAX3485、Socket_3V3供电
	Base_3V3_OFF;                 //3.3V电源   掉电
//*****************传感器电源********************//
	Sensor_12V_1_OFF;             //传感器电源接口1_12V 掉电
	Sensor_12V_2_OFF;             //传感器电源接口2_12V 掉电
	Sensor_5V_1_OFF;              //传感器电源接口1_5V  掉电
	Sensor_5V_2_OFF;              //传感器电源接口2_5V  掉电
	}
}


static void Power(ControlPower type)
{
	switch(type){
		case BaseBoard_Power_On:
			Base_12V_ON;            //12V总电源  上电
			Base_5V_ON;             //5V总电源   上电
			break;
		case BaseBoard_Power_Off:
			Base_12V_OFF;           //12V总电源  掉电
			Base_5V_OFF;            //5V总电源   掉电
			break;
		case BaseBoard_12V_Power_On:
			Base_12V_ON;            //12V总电源  上电
			break;
		case BaseBoard_12V_Power_Off:
			Base_12V_OFF;           //12V总电源  掉电
			break;
		case BaseBoard_5V_Power_On:
			Base_5V_ON;             //5V总电源   上电
			break;
		case BaseBoard_5V_Power_Off:
			Base_5V_OFF;            //5V总电源   掉电
			break;
		case Sensor_Power_On:
            Sensor_12V_1_ON;	    //传感器电源接口1_12V 上电
			Sensor_12V_2_ON;        //传感器电源接口2_12V 上电
			Sensor_5V_1_ON;         //传感器电源接口1_5V  上电
			Sensor_5V_2_ON;         //传感器电源接口2_5V  上电
			break;
		case Sensor_Power1_On:
            Sensor_12V_1_ON;	    //传感器电源接口1_12V 上电
			Sensor_5V_1_ON;         //传感器电源接口1_5V  上电
			break;
		case Sensor_Power2_On:
			Sensor_12V_2_ON;        //传感器电源接口2_12V 上电
			Sensor_5V_2_ON;         //传感器电源接口2_5V  上电
			break;			
		case Sensor_Power_Off:
			Sensor_12V_1_OFF;       //传感器电源接口1_12V 掉电
			Sensor_12V_2_OFF;       //传感器电源接口2_12V 掉电
			Sensor_5V_1_OFF;        //传感器电源接口1_5V  掉电
			Sensor_5V_2_OFF;        //传感器电源接口2_5V  掉电
			break;
		case Sensor_Power1_Off:
			Sensor_12V_1_OFF;       //传感器电源接口1_12V 掉电
			Sensor_5V_1_OFF;        //传感器电源接口1_5V  掉电
			break;
		case Sensor_Power2_Off:
			Sensor_12V_2_OFF;       //传感器电源接口2_12V 掉电
			Sensor_5V_2_OFF;        //传感器电源接口2_5V  掉电
			break;					
		case AIR202_Power_On:
			Transmit_5V_OFF;	    //通信模块5V电源    掉电
			Transmit_HighPower;
			hal_Delay_sec(2);
			g_Printf_info("%s AIR202 On\r\n",__func__);
			Transmit_5V_ON;         //通信模块5V电源    上电
			Transmit_LowPower;      //模块保持2s以上低电平开机，再保持1.5s以上低电平关机
			OSTimeDly(1500);        //节拍2ms
			Transmit_HighPower;
			break;
		case AIR202_Power_Off:
			Transmit_5V_OFF;	    //通信模块5V电源    掉电
			break;
		case SIM800C_Power_On:
			Transmit_5V_ON;	        //通信模块5V电源    上电
			break;
		case SIM800C_Power_Off:
			Transmit_5V_OFF;	    //通信模块5V电源    掉电
			break;
		case LPModule_Power_On:
			P2SEL &=~ (BIT3 + BIT4);//1:复用功能  0：普通功能     
		    P2DIR &=~ BIT3;         //1:输出模式  0：输入模式     NB震铃端口
			P2DIR |= BIT4;          //1:输出模式  0：输入模式     NB复位端口
			P2OUT |= BIT4;          //1:输出为高  0：输出为低     NB复位端口输出为高
			Transmit_3V3_ON;	    //通信模块3V3电源   上电
			break;
		case LPModule_Power_Off:
			Transmit_3V3_OFF;	    //通信模块3V3电源   掉电
			break;
		case GPS_Power_On:
			Socket_3V3_ON;	        //GPS模块3V3电源    上电
			break;
		case GPS_Power_Off:
			Socket_3V3_OFF;	        //GPS模块3V3电源    掉电
			break;
		case Base3V3_Power_On:
			Base_3V3_ON;	        //Base 3V3电源      上电
			break;
		case Base3V3_Power_Off:
			Base_3V3_OFF;	        //Base 3V3电源      掉电
			break;
		case SDCard_Power_On:
			Base_3V3_ON;	        //SD卡3V3电源       上电
			// OSTimeDly(500);
			break;
		case SDCard_Power_Off:
			Base_3V3_OFF;	        //SD卡3V3电源       掉电
			break;
		case Max485_Power_On:
			Base_3V3_ON;	        //485芯片3V3电源    上电
			break;
		case Max485_Power_Off:
			Base_3V3_OFF;	        //485芯片3V3电源     掉电
			break;
		case Motor_Power_On:
			break;
		case Motor_Power_Off:
			break;
	}
}

static void ResetWirelesModule(void)
{
	/***********reset模块复位脚*******************/
	P2OUT &=~BIT4;  //复位管脚  IO_0管脚，接模块的NRST管脚
	hal_Delay_ms(100);hal_Delay_ms(100);hal_Delay_ms(100);
	P2OUT |= BIT4;
	hal_Delay_ms(100);hal_Delay_ms(100);hal_Delay_ms(100);
}

/*******************************************************************************
* 函数名      	: g_Device_IO_Init
* 描述	  		: GPIO口初始化
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_IO_Init(void)
{
	//Initialization of ports (all unused pins as outputs with low-level)
	P1OUT = 0x00;
	P1DIR = 0xFF;
	P2OUT = 0x00;
	P2DIR = 0xF7;	//P2.3设置为输入，连接NB模块的RI
	P3OUT = 0x00;
	P3DIR = 0xFF;
	P4OUT = 0x00;
	P4DIR = 0xFF;        //1:输出模式  0：输入模式
	P5OUT = 0x00;
	P5DIR = 0xFF;
	P6OUT = 0x00;
	P6DIR = 0xFF;

	/******************打开指示灯************************/
	LED_ON;//打开指示灯
	/******************485控制预定义********************/
	Recive_485_Enable;	  	    //485传感器接收使能
	// Out485_Recive_Enable;  	    //485配件接收使能
	/******************输入端口IO配置*******************/
	P6DIR &=~ (BIT0+BIT1); //IO口方式选择，1:输出模式  0：输入模式          AD0-BAT   AD1-TEMP
	/******************禁止传感器电量采集******************/
	ScadaBAT_OFF;
    /*********************关掉无用的电源*******************/
	Power(BaseBoard_Power_Off);	        
	Power(Sensor_Power_Off);      		
	Power(AIR202_Power_Off);	 
	Power(LPModule_Power_Off);	    
	Power(SDCard_Power_Off);      
	Power(Motor_Power_Off);    	
	/*********************打开需要用到的电源****************/
	Power(SDCard_Power_On);         


	OSBsp.Device.IOControl.PowerSet = Power;

	// P2REN |= BIT2;
	// OSBsp.Device.IOControl.ResetWirelesModule = ResetWirelesModule;
}


