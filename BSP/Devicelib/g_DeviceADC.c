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
* Filename      : g_DeviceADC.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <bsp.h>



/*******************************************************************************
 * ADC检测全局变量
*******************************************************************************/
uint32_t TempADC_Value = 0;
uint32_t BatADC_Value = 0;
uint8_t  Flag_FanOPEN = 0;
uint8_t  PowerQuantity[4] = {50};
//uint16_t LastPowerQuantity = 0;
uint16_t SUMPowerQuantity = 0;
//uint8_t  AVGPowerQuantity = 0;

/*******************************************************************************
* Function Name  : g_Device_ADC_Init，通过NTC检测环境温度控制风扇
* Description    : Temp->P6.1    BAT->P6.0
* Input para     : None
* Output para    : None
*******************************************************************************/
void g_Device_ADC_Init(void)
{
	P6SEL = BIT0 + BIT1;                      // P6.0/A0 P6.1/A1, ADC input select

	while(REFCTL0 & REFGENBUSY);              // If ref generator busy, WAIT
	REFCTL0 |= REFVSEL_2+REFON;               // Select internal ref = 2.5V
	ADC10CTL0 = ADC10SHT_2 + ADC10ON;
	ADC10CTL1 |= ADC10SHP;                    // ADCCLK = MODOSC; sampling timer
	ADC10CTL2 |= ADC10RES;                    // 10-bit conversion results
	//  ADC10IE |= ADC10IE0;                    // Enable ADC conv complete interrupt
	ADC10MCTL0 |= ADC10INCH_0 + ADC10SREF_1;  // ADC input channel select; Vref=2.5V
}

/*******************************************************************************
* Function Name  : ADCValueHandle
* Description    : ADC处理函数
* Input para     : None
* Output para    : None
*******************************************************************************/
void ADCValueHandle(void)
{
	static uint16_t BATVoltage_Cache;   //BAT 电压值缓存
	static uint8_t i=0;

//	if((TempADC_Value >= 950)&&(Flag_FanOPEN == 0))
//	{Power
//		Ctr_Fan_ON;	//打开风扇
//		Flag_FanOPEN = 1;
//	}

//	if(BatADC_Value >= 840)  //2^10=1024
//	{
//		PowerQuantity = 100;  //*****4.2v*****//
//	}
//	else if((BatADC_Value >= 800) && (BatADC_Value < 840))
//		PowerQuantity = 90;   //*****4.0v*****//
//	else if((BatADC_Value >= 750) && (BatADC_Value < 800))
//		PowerQuantity = 80;   //*****3.9v*****//
//	else if((BatADC_Value >= 700) && (BatADC_Value < 750))
//		PowerQuantity = 70;   //*****3.8v*****//
//	else if((BatADC_Value >= 660) && (BatADC_Value < 700))
//		PowerQuantity = 60;   //*****3.7v*****//
//	else if((BatADC_Value >= 650) && (BatADC_Value < 660))
//		PowerQuantity = 50;   //*****3.64v*****//
//	else if((BatADC_Value >= 640) && (BatADC_Value < 650))
//		PowerQuantity = 40;   //*****3.57v*****//
//	else if((BatADC_Value >= 635) && (BatADC_Value < 640))
//		PowerQuantity = 30;   //*****3.5v*****//
//	else if((BatADC_Value >= 630) && (BatADC_Value < 635))
//		PowerQuantity = 20;   //*****3.45v*****//
//	else if( BatADC_Value < 630 )
//		PowerQuantity = 10;   //*****3.4v*****//
//	else
//		PowerQuantity = 10;

	if(BatADC_Value > Max_BatADC_Value) {
		BatADC_Value = Max_BatADC_Value;
	} else if(BatADC_Value < Min_BatADC_Value) {
		BatADC_Value = Min_BatADC_Value;
	} else {
		BatADC_Value = BatADC_Value;
	}


	for(i=0; i<3; i++)
	{
		PowerQuantity[i] = PowerQuantity[i+1]; //更替
	}
	PowerQuantity[3] = (uint8_t)((float)(BatADC_Value-Min_BatADC_Value)/(Max_BatADC_Value-Min_BatADC_Value)*100);  //电池电量百分比，精度1%
	BATVoltage_Cache = (uint16_t)( (float)BatADC_Value/1024*2.5*2*100 );          //电池电压
	App.Data.TerminalInfoData.BATVoltage = (float)BATVoltage_Cache/100;      //电池电压
}

/***************************************************************
函数名称：ADCRead1000Routine
函数功能：每1s钟采集一次ADC的数值
输入参数：无
返回值：无
****************************************************************/
void ADCRead1000Routine(void)
{
	static uint8_t ADIndex = 0;

	ADIndex ++;

	switch(ADIndex)
	{
		case 0x01:
			ADC10MCTL0 &=~ ADC10INCH_1;     //关闭AD1,Temp
			ADC10MCTL0 |= ADC10INCH_0;      //打开AD0,BAT
			ADC10CTL0 |= ADC10ENC + ADC10SC;
			break;
		case 0x02:
			BatADC_Value = ADC10MEM0;       //获取当前电压的AD值
			ADC10CTL0 &=~ ADC10ENC;         //一定要加这句话
			break;
		case 0x03:
			ADC10MCTL0 &=~ ADC10INCH_0;     //关闭AD0,BAT
			ADC10MCTL0 |= ADC10INCH_1;      //打开AD1,Temp
			ADC10CTL0 |= ADC10ENC + ADC10SC;
			break;
		case 0x04:
			TempADC_Value = ADC10MEM0;
			ADC10CTL0 &=~ ADC10ENC;
			break;
		case 0x05:
			ADIndex = 0;
			ADCValueHandle();
			break;
	}
}

void GetADCValue(void)
{
	static uint8_t i=0,j=0;

	ScadaBAT_ON;
	OSTimeDly(500); 
	//************电量处理Begin************//
	for(j=0;j<5;j++)
	{
		for(i=0;i<5;i++)
		{
			ADCRead1000Routine();  //查询ADC
			OSTimeDlyHMSM(0u, 0u, 0u, 40);  
		}
	}

	SUMPowerQuantity = 0;
	for (i=0; i<4; i++)
	{
		SUMPowerQuantity += PowerQuantity[i];
	}

	App.Data.TerminalInfoData.PowerQuantity = (uint8_t)(SUMPowerQuantity >> 2); ///4
	//App.Data.TerminalInfoData.PowerQuantity = (uint8_t)((PowerQuantity[0]+PowerQuantity[1])/2);     //电量    3.4 10% 3.5 20% ———— 4.0 70%  4.2 90%
	infor_ChargeAddrBuff[13] = App.Data.TerminalInfoData.PowerQuantity ;                    //电池电量
	Send_Buffer[37] = App.Data.TerminalInfoData.PowerQuantity;        
}




