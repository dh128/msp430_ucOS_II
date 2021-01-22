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
* Filename      : MagicSTICK_Station.c
* Version       : V1.00
* Programmer(s) : Dingh
*********************************************************************************************************
*/
#include <hal_layer_api.h>
#include <bsp.h>

#if (PRODUCT_TYPE == MagicSTICK_Station)

#define SensorNum 3		//三次轮询采集所有参数	1、2、3
#define CMDLength 8
#define SensorKind 0b111111111111

#define WQ_Q_Num 3
// #define WQ_Temp_Q_Num 5


AppStruct App;
DataStruct *AppDataPointer;

uint32_t Send_Buffer[60] = {0xaa, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
							0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
//                          /-----------------/ /-----------------/ /-----------------/ /-------/ /--/ /-------/ /-------/ /-------/ /-------/ /--/
//                               timestamp            Lng经度             lat纬度          海拔     PCI    RSRP      SINR       修正      模拟   保留

char scadaIndex;   	//传感器地址相同 使用每一轮传感器采集次数判断返回数据

const uint8_t ScadaFirst[CMDLength] = {0x01, 0x03, 0x00, 0x30, 0x00, 0x0A, 0xc5, 0xC2};		//温度 溶解氧 溶解氧% 浊度 电导率
const uint8_t ScadaSecond[CMDLength] = {0x01, 0x03, 0x00, 0x42, 0x00, 0x02, 0x64, 0x1F};	//COD
const uint8_t ScadaThird[CMDLength] = {0x01, 0x03, 0x00, 0x4A, 0x00, 0x06, 0xE4, 0x1E};		//PH ORP NH4+

//filtering use
static MagicPlatform WQ_Value[WQ_Q_Num];  //申请存储用于滤波的数据空间
static MagicPlatform WQ_ValueTemp;		  //临时值
static MagicPlatform WQ_ValueTempSum;	 //临时值

uint32_t sensorCahe = 0;  //临时存储各水质指标的值（除水温外）
uint32_t ssensorCahe = 0; //临时存储水温值

float SimulationSensorFloatCahe = 0.0;
// int32_t SimulationSensorIntCahe = 0;
static uint8_t SensorStatus_H;
static uint8_t SensorStatus_L;
static uint8_t SensorReviseStatus_H; //修正
static uint8_t SensorReviseStatus_L;
static uint8_t SensorSimulationStatus_H; //模拟
static uint8_t SensorSimulationStatus_L;

Hex2Float SensorData;

/*******************************************************************************
* 函数名		: AnalyzeComand
* 描述	    	: 解析传感器指令
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
static int AnalyzeComand(uint8_t *data, uint8_t Len)
{
	if (Len > 2)
	{
		uint16_t CalcuResult = 0;
		uint8_t CRC_Result[2];
		hal_Delay_ms(50);

		CalcuResult = Crc16(data, Len - 2);
		CRC_Result[0] = (uint8_t)((CalcuResult & 0xFF00) >> 8);
		CRC_Result[1] = (uint8_t)(CalcuResult & 0xFF);
		if ((data[Len - 2] == CRC_Result[0]) && (data[Len - 1] == CRC_Result[1])) //判断数据接收是否存在异常
		{
			LED_ON;
			if (data[1] == 0x03)
			{
				switch (scadaIndex)
				{
				case 0x01: //温度 溶解氧 溶解氧%（跳过） 浊度 电导率 Float数据小端模式
					
					if (data[2] == 0x14)		//总计20个字节有效数据
					{
						//温度
						SensorData.Hex[0] = data[3]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[4];
						SensorData.Hex[2] = data[5];
						SensorData.Hex[3] = data[6];						
						WQ_ValueTemp.WaterTemp = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 0);			 //传感器状态位置0

						//溶解氧
						SensorData.Hex[0] = data[7]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[8];
						SensorData.Hex[2] = data[9];
						SensorData.Hex[3] = data[10];						
						WQ_ValueTemp.DOValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 1);			 //传感器状态位置1

						//溶解氧
						SensorData.Hex[0] = data[15]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[16];
						SensorData.Hex[2] = data[17];
						SensorData.Hex[3] = data[18];						
						WQ_ValueTemp.ZSValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 2);			 //传感器状态位置2

						//溶解氧
						SensorData.Hex[0] = data[19]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[20];
						SensorData.Hex[2] = data[21];
						SensorData.Hex[3] = data[22];						
						WQ_ValueTemp.ECValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 3);			 //传感器状态位置3

						// if ((WQ_ValueTemp.NH4Value <= 0.0) || (WQ_ValueTemp.NH4Value >= 50.0)) //输值根据实际水情而定，作为参考
						// {
						// 	WQ_ValueTemp.NH4Value = AppDataPointer->MagicData.NH4Value;
						// }
					}
					break;
				case 0x02:	//COD
					hal_SetBit(SensorStatus_H, 1); //传感器状态位置1

					SensorData.Hex[0] = data[3]; //小端模式，高位字节存放在前面
					SensorData.Hex[1] = data[4];
					SensorData.Hex[2] = data[5];
					SensorData.Hex[3] = data[6];
					WQ_ValueTemp.CODValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 4);			 //传感器状态位置4

					if ((WQ_ValueTemp.CODValue <= 5.0) || (WQ_ValueTemp.CODValue >= 60.0)) //输值根据实际水情而定，作为参考
					{
						WQ_ValueTemp.CODValue = AppDataPointer->MagicData.CODValue;
					}
					
					break;
				case 0x03:	//PH  ORP  NH4+ 
					if (data[2] == 0x0C) //数据长度是0C
					{
						hal_SetBit(SensorStatus_L, 2); //传感器状态位置1
						//PH
						SensorData.Hex[0] = data[3]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[4];
						SensorData.Hex[2] = data[5];
						SensorData.Hex[3] = data[6];
						WQ_ValueTemp.PHValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 5);			 //传感器状态位置5
						//ORP
						SensorData.Hex[0] = data[7]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[8];
						SensorData.Hex[2] = data[9];
						SensorData.Hex[3] = data[10];
						WQ_ValueTemp.ORPValue = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 6);			 //传感器状态位置6
						
						//NH4+
						SensorData.Hex[0] = data[11]; //小端模式，高位字节存放在前面
						SensorData.Hex[1] = data[12];
						SensorData.Hex[2] = data[13];
						SensorData.Hex[3] = data[14];
						WQ_ValueTemp.NH4Value = SensorData.Data; //更新WQ数据的最后一个，后续用先进先出的模式进行数组更替
						hal_SetBit(SensorStatus_H, 7);			 //传感器状态位置7

						// if ((WQ_ValueTemp.ORPValue <= -200) || (WQ_ValueTemp.ORPValue >= 300)) //输值根据实际水情而定，作为参考
						// {
						// 	WQ_ValueTemp.ORPValue = AppDataPointer->MagicData.ORPValue;
						// }
						
					}
					break;
				
				default:
					break;
				} //switch scadaIndex END
			}	 //(data[1]==0x03)  END
			Send_Buffer[55] = SensorReviseStatus_H;
			Send_Buffer[56] = SensorReviseStatus_L;
			Clear_CMD_Buffer(dRxBuff, dRxNum);
			dRxNum = 0;
			Len = 0;
			return 1;
		}
		else
		{
			Clear_CMD_Buffer(dRxBuff, dRxNum);
			dRxNum = 0;
			Len = 0;
			return -2;
		}

		// SensorStatusBuff[0] = SensorStatus_H;
		// SensorStatusBuff[1] = SensorStatus_L;
		// AppDataPointer->TerminalInfoData.SensorStatus = (uint16_t)SensorStatus_H*256 + (uint16_t)SensorStatus_L;

		// Clear_CMD_Buffer(dRxBuff,dRxNum);
		// dRxNum=0;
		// Len = 0;
	}
	else
	{
		Clear_CMD_Buffer(dRxBuff, dRxNum);
		dRxNum = 0;
		Len = 0;
		return -1;
	}
}

/*******************************************************************************
* 函数名		: SimulationSensorData
* 描述	    	: 模拟传感器数据
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
static int SimulationSensorData(void)
{
	volatile char simulationIndex;
	volatile uint16_t sensorNOTExistStatus = 0;
	uint8_t str[12];
	uint32_t temp = 0;

	temp = AppDataPointer->TerminalInfoData.SensorStatusSimulation;
	Itoa(temp, str, 2); //2进制输出
	g_Printf_dbg("%s binary format: %s\n", __func__, str);

	if (AppDataPointer->TerminalInfoData.SensorStatusSimulation != AppDataPointer->TerminalInfoData.SensorStatusSimulation_Old)
	{
		infor_ChargeAddrBuff[24] = (uint8_t)(temp >> 8);
		infor_ChargeAddrBuff[25] = (uint8_t)temp;
		OSBsp.Device.InnerFlash.innerFLASHWrite(infor_ChargeAddrBuff, (uint8_t *)(infor_ChargeAddr + 0), 32);
	} //为什么要写入？flash里的24、25代表什么意思？代表哪些传感器是模拟的？wj
	AppDataPointer->TerminalInfoData.SensorStatusSimulation_Old = AppDataPointer->TerminalInfoData.SensorStatusSimulation;

	for (simulationIndex = 1; simulationIndex <= SensorNum; simulationIndex++) //SensorNum = 12
	{
		sensorNOTExistStatus = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) & 0x0800;
		AppDataPointer->TerminalInfoData.SensorStatusSimulation = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) << 1;
		if (sensorNOTExistStatus == 0x0800)
		{
			switch (simulationIndex)
			{
			case 1:
				/**************Temp**************/ //16.1~18.1
				SimulationSensorFloatCahe = 23.01 + (float)(rand() % 2) - (float)(rand() % 2);
				AppDataPointer->MagicData.WaterTemp = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 0);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 0); //传感器模拟状态位置1
				Send_Buffer[15] = (uint32_t)(SimulationSensorFloatCahe * 10) / 256;
				Send_Buffer[16] = (uint32_t)(SimulationSensorFloatCahe * 10) % 256;
				/**************DO****************/ //3.31~4.51
				SimulationSensorFloatCahe = 5.41 + (float)(rand() % 12) / 10 - (float)(rand() % 12) / 10;
				AppDataPointer->MagicData.DOValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 1);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 1); //传感器模拟状态位置1
				Send_Buffer[11] = (uint32_t)(SimulationSensorFloatCahe * 100) / 256;
				Send_Buffer[12] = (uint32_t)(SimulationSensorFloatCahe * 100) % 256;
				/**************ZS**************/ //18.01~23.91
				SimulationSensorFloatCahe = 18.01 + (float)(rand() % 59) / 10 - (float)(rand() % 59) / 10;
				if ((SimulationSensorFloatCahe < 0.0) || (SimulationSensorFloatCahe > 800))
				{
					SimulationSensorFloatCahe = 20.01;
				}
				AppDataPointer->MagicData.ZSValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 2);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 2); //传感器模拟状态位置1
				Send_Buffer[19] = (uint32_t)(SimulationSensorFloatCahe * 100) / 256;
				Send_Buffer[20] = (uint32_t)(SimulationSensorFloatCahe * 100) % 256;
				/**************EC****************/ //321~341
				SimulationSensorFloatCahe = (float)(541 + rand() % 20 - rand() % 20);
				AppDataPointer->MagicData.ECValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 3);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 3); //传感器模拟状态位置1
				Send_Buffer[9] = (uint32_t)(SimulationSensorFloatCahe) / 256;
				Send_Buffer[10] = (uint32_t)(SimulationSensorFloatCahe) % 256;
				break;
			case 2:
				/****************COD*************/ //16.1~25.1
				SimulationSensorFloatCahe = 16.1 + (float)(rand() % 9) - (float)(rand() % 9);
				AppDataPointer->MagicData.CODValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 4);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 4); //传感器模拟状态位置1
				Send_Buffer[7] = (uint32_t)(SimulationSensorFloatCahe * 10) / 256;
				Send_Buffer[8] = (uint32_t)(SimulationSensorFloatCahe * 10) % 256;
				break;
			case 3:
				/**************PH**************/ //7.21~7.61
				SimulationSensorFloatCahe = 7.01 + (float)(rand() % 4) / 10 - (float)(rand() % 4) / 10;
				AppDataPointer->MagicData.PHValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 5);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 5); //传感器模拟状态位置1
				Send_Buffer[21] = (uint32_t)(SimulationSensorFloatCahe * 100) / 256;
				Send_Buffer[22] = (uint32_t)(SimulationSensorFloatCahe * 100) % 256;
				/**************ORP**************/ //71~101
				SimulationSensorFloatCahe = (float)(91 + rand() % 30 - rand() % 30);
				AppDataPointer->MagicData.ORPValue = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 6);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 6); //传感器模拟状态位置1
				if (AppDataPointer->MagicData.ORPValue >= 0)
				{ //ORP为正数
					Send_Buffer[17] = (uint32_t)(SimulationSensorFloatCahe) / 256;
					Send_Buffer[18] = (uint32_t)(SimulationSensorFloatCahe) % 256;
				}
				else
				{ //ORP为负数
					Send_Buffer[17] = (uint32_t)(0xFFFF - ~(int16_t)SimulationSensorFloatCahe) / 256;
					Send_Buffer[18] = (uint32_t)(0xFFFF - ~(int16_t)SimulationSensorFloatCahe) % 256;
				}
				/**************NH4***************/ //1.1~3.1
				SimulationSensorFloatCahe = 0.10 + (float)(rand() % 200) / 100 - (float)(rand() % 200) / 100;
				AppDataPointer->MagicData.NH4Value = SimulationSensorFloatCahe;
				hal_SetBit(SensorStatus_H, 7);			 //传感器状态位置1
				hal_SetBit(SensorSimulationStatus_H, 7); //传感器模拟状态位置1
				Send_Buffer[13] = (uint32_t)(SimulationSensorFloatCahe * 100) / 256;
				Send_Buffer[14] = (uint32_t)(SimulationSensorFloatCahe * 100) % 256;
				break;
			default:
				break;
			}
		}
	}
	Send_Buffer[57] = SensorSimulationStatus_H;
	Send_Buffer[58] = SensorSimulationStatus_L;
	return 1;
}

/*******************************************************************************
* 函数名		: FilteringSensor
* 描述	    	: 传感器数据滤波
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void FilteringSensor(void) 
{

	uint8_t i = 0;
	static uint8_t FilteringNum = 0;
	//各项参数，先进先出
	for (i = 0; i < (WQ_Q_Num - 1); i++)
	{
		WQ_Value[i] = WQ_Value[i + 1];
	}
	WQ_Value[WQ_Q_Num - 1] = WQ_ValueTemp;
	//由于我没有把WQ_ValueTemp清零，如果存在某个传感器以前是好的，突然读取不到数据了，其实在这个滤波里，他会永远保持掉线前的数据，但是上不上传是由hal_GetBit在组包函数里控制

	//均值滤波，与前两轮一起循环滤波
	WQ_ValueTempSum.CODValue = 0.0;
	WQ_ValueTempSum.ECValue = 0;
	WQ_ValueTempSum.DOValue = 0.0;
	WQ_ValueTempSum.NH4Value = 0.0;
	WQ_ValueTempSum.WaterTemp = 0.0;
	WQ_ValueTempSum.ORPValue = 0;
	WQ_ValueTempSum.ZSValue = 0.0;
	WQ_ValueTempSum.PHValue = 0.0;

	//在执行这个函数之前，应该再执行一步，即去除每个水质的异常值（量程或实际范围）之外的
	for (i = 0; i < WQ_Q_Num; i++)
	{
		WQ_ValueTempSum.CODValue += WQ_Value[i].CODValue;
		WQ_ValueTempSum.ECValue += WQ_Value[i].ECValue;
		WQ_ValueTempSum.DOValue += WQ_Value[i].DOValue;
		WQ_ValueTempSum.NH4Value += WQ_Value[i].NH4Value;
		WQ_ValueTempSum.WaterTemp += WQ_Value[i].WaterTemp;
		WQ_ValueTempSum.ORPValue += WQ_Value[i].ORPValue;
		WQ_ValueTempSum.ZSValue += WQ_Value[i].ZSValue;
		WQ_ValueTempSum.PHValue += WQ_Value[i].PHValue;
	}

	AppDataPointer->MagicData.CODValue = WQ_ValueTempSum.CODValue / WQ_Q_Num;
	AppDataPointer->MagicData.ECValue = WQ_ValueTempSum.ECValue / WQ_Q_Num;
	AppDataPointer->MagicData.DOValue = WQ_ValueTempSum.DOValue / WQ_Q_Num;
	AppDataPointer->MagicData.NH4Value = WQ_ValueTempSum.NH4Value / WQ_Q_Num;
	AppDataPointer->MagicData.WaterTemp = WQ_ValueTempSum.WaterTemp / WQ_Q_Num;
	AppDataPointer->MagicData.ORPValue = WQ_ValueTempSum.ORPValue / WQ_Q_Num;
	AppDataPointer->MagicData.ZSValue = WQ_ValueTempSum.ZSValue / WQ_Q_Num;
	AppDataPointer->MagicData.PHValue = WQ_ValueTempSum.PHValue / WQ_Q_Num;

	//这里还需要写代码解决：如果传感器读取到了数值，但一直为0的情况；

	if (FilteringNum > WQ_Q_Num) //前3次不执行
	{
		
		if (AppDataPointer->MagicData.WaterTemp == 0.0)
		{
			AppDataPointer->MagicData.WaterTemp = 15.1;
			hal_ResetBit(SensorStatus_H, 0);
		}
		if (AppDataPointer->MagicData.DOValue == 0.0)
		{
			AppDataPointer->MagicData.DOValue = 3.31;
			hal_ResetBit(SensorStatus_H, 1);
		}
		if (AppDataPointer->MagicData.ZSValue == 0.0)
		{
			AppDataPointer->MagicData.ZSValue = 18.01;
			hal_ResetBit(SensorStatus_H, 2);
		}
		if (AppDataPointer->MagicData.ECValue == 0)
		{
			AppDataPointer->MagicData.ECValue = 321.0; //电导率在空气中可能出现0；
			hal_ResetBit(SensorStatus_H, 3);
		}
		if (AppDataPointer->MagicData.CODValue == 0.0)
		{
			AppDataPointer->MagicData.CODValue = 16.1;
			//但是！！！第一次不应该置位，因为第一次的置位使得flash里也表现得没有这个传感器！！！
			hal_ResetBit(SensorStatus_H, 4); //如果某个传感器值长期为0，我就当他掉线了处理，让他进入模拟数据的函数去；其他水质参数类似；当然这里也可以选择其他处理方法
		}
		if (AppDataPointer->MagicData.PHValue == 0.0)
		{
			AppDataPointer->MagicData.PHValue = 7.21;
			hal_ResetBit(SensorStatus_H, 5);
		}
		if (AppDataPointer->MagicData.ORPValue == 0)
		{
			AppDataPointer->MagicData.ORPValue = 71;
			hal_ResetBit(SensorStatus_H, 6);
		}
		if (AppDataPointer->MagicData.NH4Value == 0.0)
		{
			AppDataPointer->MagicData.NH4Value = 1.1;
			hal_ResetBit(SensorStatus_H, 7);
		}

		FilteringNum++;
	}
	if (FilteringNum > 5) //让上面的程序一直执行；
	{
		FilteringNum = 5;
	}

	//！！！经过分析，这里的filtering函数不能解决：传感器根本就没回复或者回复错误，导致传感根本就不置位的问题。！！！！这才是主要的问题！
	//这个可以通过模拟函数实现，如果传感器置位与flash记录中的不一样，模拟成在原来的输值上下浮动。前提要在上位机上出厂默认设置flash里的模拟开关位=01
}
void InqureSensor(void)
{
	//COD EC DO NH4 | Temp ORP ZS PH | CHL WL WS XX
	
	volatile uint16_t sensorExistStatus = 0;
	volatile uint8_t sensorSN = 0;  //传感器编号，按照协议顺序排列
	volatile uint16_t sensorStatus; //0000 0011 1100 0000     Do,氨氮，温度，ORP

	if (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_NOTYET)
	{
		AppDataPointer->TerminalInfoData.SensorFlashReadStatus = SENSOR_STATUS_READFLASH_ALREADY;
		AppDataPointer->TerminalInfoData.SensorStatus = SensorKind;
	    Teminal_Data_Init();   //数据初始化
	}
	else if ((AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_ALREADY) || (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_OK))
	{
		AppDataPointer->TerminalInfoData.SensorFlashReadStatus = SENSOR_STATUS_READFLASH_OK;
		AppDataPointer->TerminalInfoData.SensorFlashStatus = Hal_getSensorFlashStatus();					//wj20200217把上面一行改成了这一行
		AppDataPointer->TerminalInfoData.SensorStatus = AppDataPointer->TerminalInfoData.SensorFlashStatus; //这里是不是写反了或者上面的应该是SensorFlashStatus？？
	}
	if (AppDataPointer->TerminalInfoData.SensorStatus != 0)
	{
		SensorStatus_H = 0;
		SensorStatus_L = 0;
		for (scadaIndex = 1; scadaIndex <= SensorNum; scadaIndex++) //SensorNum = 12
		{
			// sensorExistStatus = (AppDataPointer->TerminalInfoData.SensorStatus) & 0x0001;
			// AppDataPointer->TerminalInfoData.SensorStatus = (AppDataPointer->TerminalInfoData.SensorStatus) >> 1;
			// if(sensorExistStatus == 1)

			memset(dRxBuff, 0x0, dRxLength);
			dRxNum = 0;
			sensorExistStatus = (AppDataPointer->TerminalInfoData.SensorStatus) & 0x0800;
			AppDataPointer->TerminalInfoData.SensorStatus = (AppDataPointer->TerminalInfoData.SensorStatus) << 1;
			if (sensorExistStatus == 0x0800)
			{
				Send_485_Enable;
				hal_Delay_ms(5);
				switch (scadaIndex)
				{
				case 1:
					sensorSN = 1;
					OSBsp.Device.Usart3.WriteNData(ScadaFirst, CMDLength);
					break;
				case 2:
					sensorSN = 2;
					OSBsp.Device.Usart3.WriteNData(ScadaSecond, CMDLength);
					break;
				case 3:
					sensorSN = 3;
					OSBsp.Device.Usart3.WriteNData(ScadaThird, CMDLength);
					break;
				default:
					break;
				}
				hal_Delay_ms(2); //高波特率降低延时为1-2ms，否则容易丢包；低波特率增加延时，如4800延时10ms，否则容易丢包
				Recive_485_Enable;

				LED_ON;
				OSTimeDly(400); //任务挂起 800ms,等待传感器回复，接收完毕
				int ret = AnalyzeComand(dRxBuff, dRxNum);
				OSTimeDly(50); //挂起 100ms
				uint32_t times = sensorSN;
				if (ret == 1)
				{
					g_Printf_dbg("%s.AnalyzeComand.Sensor answer ok - Sensor SN %d\r\n", __func__, times);
				}
				else if (ret == -2)
				{
					g_Printf_dbg("%s.AnalyzeComand.CRC check failed\r\n", __func__);
				}
				else if (ret == -1)
				{
					g_Printf_dbg("%s.AnalyzeComand.Sensor has no answer - Sensor SN %d\r\n", __func__, times);
					sensorSN = 0;
				}
				LED_OFF;

				Clear_CMD_Buffer(dRxBuff, dRxNum); //发送之前buff清0
				dRxNum = 0;
			}
		}

		//一轮传感器读取完毕后期构建一个滤波函数
		FilteringSensor();
		//其他参数的滤波也应该写在上面；

		AppDataPointer->TerminalInfoData.SensorStatus = (uint16_t)SensorStatus_H * 256 + (uint16_t)SensorStatus_L; //本次读取到的传感器置位

		if (AppDataPointer->TerminalInfoData.SensorFlashWriteStatus == SENSOR_STATUS_WRITEFLASH_NOTYET) //上电后第一次检查哪些传感器在线，并写flash里
		{
			AppDataPointer->TerminalInfoData.SensorFlashWriteStatus = SENSOR_STATUS_WRITEFLASH_ALREADY;
			if (OSBsp.Device.InnerFlash.innerFLASHRead(20, infor_ChargeAddr) == 0x01) //0x01才允许修改Flash
			{
				infor_ChargeAddrBuff[21] = SensorStatus_H;
				infor_ChargeAddrBuff[22] = SensorStatus_L;
				OSBsp.Device.InnerFlash.innerFLASHWrite(infor_ChargeAddrBuff, (uint8_t *)(infor_ChargeAddr + 0), 32);
			}
			AppDataPointer->TerminalInfoData.SensorFlashWriteStatusPrintf = SENSOR_STATUS_WRITEFLASH_PRINTF_ENABLE;
		}

		//根据标志位判断是否需要模拟数据
		if (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_OK)
		{
			if (OSBsp.Device.InnerFlash.innerFLASHRead(23, infor_ChargeAddr) == 0x01) //打开后可长期支持模拟
			{
				AppDataPointer->TerminalInfoData.SensorStatusSimulation = (AppDataPointer->TerminalInfoData.SensorStatus) ^ (AppDataPointer->TerminalInfoData.SensorFlashStatus);
				//传感器损坏，无数据才支持补发
				if (AppDataPointer->TerminalInfoData.SensorStatusSimulation != 0)
				{
					SimulationSensorData();
					// dRxNum=0;
				}
			}
		}

		AppDataPointer->TerminalInfoData.ReviseSimulationCode = ((uint32_t)SensorReviseStatus_H * 256 + (uint32_t)SensorReviseStatus_L) << 16;
		AppDataPointer->TerminalInfoData.ReviseSimulationCode = AppDataPointer->TerminalInfoData.ReviseSimulationCode + (uint32_t)SensorSimulationStatus_H * 256 + (uint32_t)SensorSimulationStatus_L;
	}
	else
	{
		g_Printf_dbg("%s.AnalyzeSensor.No sensor to scan\r\n", __func__);
		AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCAN_OVER;
		g_Printf_info("ScadaTask is over because of No sensor to scan!\n");
		OSTimeDly(10);
	}
}

/*******************************************************************************
* 函数名		: *MakeJsonBodyData
* 描述	    	: 组建json包
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
char *MakeJsonBodyData(DataStruct *DataPointer)
{
	uint32_t TempCahe = 0;
	int32_t TempIntCahe = 0;
#if (ACCESSORY_TYPR == GPS_Mode)
	uint8_t gpsBuffer[15];
#endif
	mallco_dev.init();

	cJSON *pJsonRoot = mymalloc(512 * sizeof(cJSON *));
	cJSON *pSubJson = mymalloc(128 * sizeof(cJSON *));
	;
	char *p;

	pJsonRoot = cJSON_CreateObject();
	if (NULL == pJsonRoot)
	{
		cJSON_Delete(pJsonRoot);
		return NULL;
	}

	cJSON_AddNumberToObject(pJsonRoot, "SN", DataPointer->TerminalInfoData.SerialNumber);
	cJSON_AddNumberToObject(pJsonRoot, "DeviceID", DataPointer->TerminalInfoData.DeviceID);
	cJSON_AddNumberToObject(pJsonRoot, "SeqNum", DataPointer->TransMethodData.SeqNumber);
	if (REGRST != 0)
	{
		cJSON_AddNumberToObject(pJsonRoot, "reboot", REGRST);
		REGRST = 0;
	}

	pSubJson = NULL;
	pSubJson = cJSON_CreateObject();
	if (NULL == pSubJson)
	{
		//create object faild, exit
		cJSON_Delete(pSubJson);
		return NULL;
	}


	if (hal_GetBit(SensorStatus_H, 0))
	{
		cJSON_AddNumberToObject(pSubJson, "Temp", DataPointer->MagicData.WaterTemp);
		//需要组hex包
		TempCahe = (uint32_t)(DataPointer->MagicData.WaterTemp * 10);
		if (TempIntCahe >= 0)
		{														   //temp为正数
			Send_Buffer[15] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
			Send_Buffer[16] = (uint8_t)(TempCahe & 0xFF);		   //=%256
		}
		else
		{																	   //temp为负数
			Send_Buffer[15] = (uint8_t)(0xFFFF - ~(int16_t)TempIntCahe) / 256; //负数先不用
			Send_Buffer[16] = (uint8_t)(0xFFFF - ~(int16_t)TempIntCahe) % 256;
		}
	}
	if (hal_GetBit(SensorStatus_H, 1))
	{
		cJSON_AddNumberToObject(pSubJson, "DoVal", DataPointer->MagicData.DOValue);

		TempCahe = (uint32_t)(DataPointer->MagicData.DOValue * 100);
		Send_Buffer[11] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
		Send_Buffer[12] = (uint8_t)(TempCahe & 0xFF);		   //=%256
	}
	if (hal_GetBit(SensorStatus_H, 2))
	{
		cJSON_AddNumberToObject(pSubJson, "ZS", DataPointer->MagicData.ZSValue);

		TempCahe = (uint32_t)(DataPointer->MagicData.ZSValue * 100);
		Send_Buffer[19] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
		Send_Buffer[20] = (uint8_t)(TempCahe & 0xFF);		   //=%256
	}
	if (hal_GetBit(SensorStatus_H, 3))
	{
		cJSON_AddNumberToObject(pSubJson, "Cond", DataPointer->MagicData.ECValue);

		TempCahe = (uint32_t)(DataPointer->MagicData.ECValue);
		Send_Buffer[9] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
		Send_Buffer[10] = (uint8_t)(TempCahe & 0xFF);		  //=%256
	}
	if (hal_GetBit(SensorStatus_H, 4))
	{
		cJSON_AddNumberToObject(pSubJson, "COD", DataPointer->MagicData.CODValue);

		TempCahe = (uint32_t)(DataPointer->MagicData.CODValue * 10);
		Send_Buffer[7] = (uint8_t)((TempCahe & 0xFF00) >> 8);
		Send_Buffer[8] = (uint8_t)(TempCahe & 0xFF);
	}
	if (hal_GetBit(SensorStatus_H, 5))
	{
		cJSON_AddNumberToObject(pSubJson, "PH", DataPointer->MagicData.PHValue);

		TempCahe = (uint32_t)(DataPointer->MagicData.PHValue * 100);
		Send_Buffer[21] = (uint8_t)((TempCahe & 0xFF00) >> 8);
		Send_Buffer[22] = (uint8_t)(TempCahe & 0xFF);
	}
	if (hal_GetBit(SensorStatus_H, 6))
	{
		cJSON_AddNumberToObject(pSubJson, "ORP", DataPointer->MagicData.ORPValue);

		TempIntCahe = (uint32_t)(DataPointer->MagicData.ORPValue);
		if (TempIntCahe >= 0)
		{														   //ORP为正数
			Send_Buffer[17] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
			Send_Buffer[18] = (uint8_t)(TempCahe & 0xFF);		   //=%256
		}
		else
		{																	   //ORP为负数
			Send_Buffer[17] = (uint8_t)(0xFFFF - ~(int16_t)TempIntCahe) / 256; //负数先不用
			Send_Buffer[18] = (uint8_t)(0xFFFF - ~(int16_t)TempIntCahe) % 256;
		}
	}
	if (hal_GetBit(SensorStatus_H, 7))
	{
		cJSON_AddNumberToObject(pSubJson, "NH4", DataPointer->MagicData.NH4Value);

		TempCahe = (uint32_t)(DataPointer->MagicData.NH4Value * 100);
		Send_Buffer[13] = (uint8_t)((TempCahe & 0xFF00) >> 8); //=/256
		Send_Buffer[14] = (uint8_t)(TempCahe & 0xFF);		   //=%256
	}
	cJSON_AddItemToObject(pJsonRoot, "MagicData", pSubJson);
#if (TRANSMIT_TYPE == GPRS_Mode)
	cJSON_AddStringToObject(pJsonRoot, "CSQ", CSQBuffer);
#endif
#if (TRANSMIT_TYPE == NBIoT_BC95_Mode)
	cJSON_AddNumberToObject(pJsonRoot, "RSRP", DataPointer->TransMethodData.RSRP);
	cJSON_AddNumberToObject(pJsonRoot, "SINR", DataPointer->TransMethodData.SINR);
	cJSON_AddNumberToObject(pJsonRoot, "PCI", DataPointer->TransMethodData.PCI);
#endif

#if (ACCESSORY_TYPR == GPS_Mode)
	// sprintf((char *)gpsBuffer, (const char *)"%lf", DataPointer->TransMethodData.GPSLat_Point);
	// cJSON_AddStringToObject(pJsonRoot, (const char *)"latitude", (const char *)gpsBuffer);
	// sprintf((char *)gpsBuffer, (const char *)"%lf", DataPointer->TransMethodData.GPSLng_Point);
	// cJSON_AddStringToObject(pJsonRoot, (const char *)"longitude", (const char *)gpsBuffer);
#endif

	cJSON_AddNumberToObject(pJsonRoot, "SendPeriod", DataPointer->TerminalInfoData.SendPeriod);
	cJSON_AddNumberToObject(pJsonRoot, "Quanity", DataPointer->TerminalInfoData.PowerQuantity);
	cJSON_AddNumberToObject(pJsonRoot, "Version", DataPointer->TerminalInfoData.Version);

	uint8_t date[8];
	char Uptime[19] = "2019-09-01 00:00:00";
	char filestore[19];
	memset(date, 0x0, 8);
	memset(Uptime, 0x0, 19);
	memset(filestore, 0x0, 19);
	Read_info_RTC(date);
	g_Device_RTCstring_Creat(date, Uptime);
	g_Printf_info("Uptime:%s\r\n", Uptime);
	cJSON_AddStringToObject(pJsonRoot, "Uptime", Uptime);
	cJSON_AddNumberToObject(pJsonRoot, "UnixTimeStamp", UnixTimeStamp);
	cJSON_AddNumberToObject(pJsonRoot, "ReSiC", DataPointer->TerminalInfoData.ReviseSimulationCode);

	p = cJSON_Print(pJsonRoot);
	if (NULL == p)
	{
		//convert json list to string faild, exit
		//because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
		cJSON_Delete(pJsonRoot);
		return NULL;
	}

	cJSON_Delete(pJsonRoot);
	cJSON_Delete(pSubJson);

#if HAVE_SDCARD_SERVICE
	if (SD_Status == 0) //只是一个示例
	{
		OSBsp.Device.IOControl.PowerSet(SDCard_Power_On);
		OSTimeDly(500);
		g_SD_FileName_Creat("0:/", date, filestore);
		g_SD_File_Write(filestore, p);
		g_SD_File_Write(filestore, "\r\n"); //数据换行
	}
#endif
	// OSBsp.Device.IOControl.PowerSet(SDCard_Power_Off);  //++++++++++++++++++++++++

	return p;
}

/*******************************************************************************
* 函数名		: ScadaData_base_Init
* 描述	    	: 初始化采集的数据大小
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void ScadaData_base_Init(void)
{
	//	static uint32_t dataH = 0,dataM = 0,dataL = 0;

	//首先执行指针地址赋值
	AppDataPointer = &(App.Data); //DataStruct * AppDataPointer; 定义结构体类型的指针 ,只是一个名字，具体要定义指针的地址

	//只初始化要用到的数据
#if (TRANSMIT_TYPE == GPRS_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_off;
	AppDataPointer->TransMethodData.GPRSNet = 0;
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.NBStatus = NB_Power_off;
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->LoRa_F8L10D_Mode.LoRaStatus = LoRa_Power_off;
#elif (TRANSMIT_TYPE == LoRa_M100C_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.LoRaStatus = LoRa_Power_off;
#endif
}

/*******************************************************************************
* 函数名		: Terminal_Para_Init
* 描述	    	: 根据Flash储存信息初始化终端参数
* 输入参数  	: 无
* 返回参数 		: 无
*******************************************************************************/
void Terminal_Para_Init(void)
{
	int i = 0;
	Hex2Double TransferData;

	/*********************设备当前运行状态****************************************/
	App.Data.TerminalInfoData.DeviceFirstRunStatus = DEVICE_STATUS_FIRSTRUN_BEGIN;
	/*********************读取Flash数据，并存放在数组中***************************/
	for (i = 0; i < 32; i++)
	{
		infor_ChargeAddrBuff[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i, infor_ChargeAddr);
	}

	/************************地理信息*******************************************/
	for (i = 0; i < 8; i++)
	{
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i + 32, infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Longitude = TransferData.Data;
	for (i = 0; i < 8; i++)
	{
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i + 40, infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Latitude = TransferData.Data;
	for (i = 0; i < 8; i++)
	{
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i + 48, infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Altitude = TransferData.Data;

	/************************DeviceID******************************************/
	App.Data.TerminalInfoData.DeviceID = Hal_getDeviceID();
	Send_Buffer[1] = (App.Data.TerminalInfoData.DeviceID >> 16) & 0xFF;
	Send_Buffer[2] = (App.Data.TerminalInfoData.DeviceID >> 8) & 0xFF;
	Send_Buffer[3] = App.Data.TerminalInfoData.DeviceID & 0xFF;
	/**************************出厂编号******************************************/
	App.Data.TerminalInfoData.SerialNumber = Hal_getSerialNumber();
	/**************************生产日期******************************************/
	App.Data.TerminalInfoData.PD = Hal_getManufactureDate();
	/**************************TerminalIndex 终端类型****************************/
	App.Data.TerminalInfoData.DeviceType = PRODUCT_TYPE;
	Send_Buffer[4] = Water_Station;		//MagicSTICK 与 Water_Station NB位置一致
	/**************************SendPeriod***************************************/
	App.Data.TerminalInfoData.SendPeriod = Hal_getTransmitPeriod(); //发送周期
	Send_Buffer[31] = (App.Data.TerminalInfoData.SendPeriod >> 8) & 0x00FF;
	Send_Buffer[32] = App.Data.TerminalInfoData.SendPeriod & 0x00FF;
	if (App.Data.TerminalInfoData.SendPeriod > 60)
	{
		App.Data.TerminalInfoData.SendPeriod = 5;
		g_Printf_info("Period change as 5 min\r\n");
	}
	/**************************Version******************************************/
	App.Data.TerminalInfoData.Version = Hal_getFirmwareVersion(); //软件版本
	Send_Buffer[34] = App.Data.TerminalInfoData.Version;
	/**************************未读取Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorFlashReadStatus = SENSOR_STATUS_READFLASH_NOTYET;
	/**************************未写入Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorFlashWriteStatus = SENSOR_STATUS_WRITEFLASH_NOTYET;
	/**************************允许同步时间状态***********************/
	App.Data.TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_ENABLE;

#if (TRANSMIT_TYPE == GPRS_Mode)
#ifdef SIM800C
	// Socket_5V_ON;            //GPRS  PowerON-P5.0 //传输板上插GPRS模块时供电
	// delay_sec(5); 			 //wj20180511  为了稳定5V电源一段时间
	// ResetCommunication();    //模块复位管脚复位,对于GPRS模块电压需要达到5V,单片输出管脚电压只有3.3V
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率  GPRS
// InitSim800C();           //初始化GPRS
#endif
#ifdef AIR202
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率
	// Socket_5V_ON;            //GPRS  PowerON-P5.0 //传输板上插GPRS模块时供电
	// Socket_3V_ON;
	// delay_sec(2);
	// AIR202_Power_ON;
	// delay_sec(2);
	// AIR202_Power_OFF;
	if (Hal_getProductKey(App.Data.TerminalInfoData.ProductKey) != 0)
	{
		return;
	}
	g_Printf_info("ProductKey:%s\r\n", App.Data.TerminalInfoData.ProductKey);

	if (Hal_getDeviceName(App.Data.TerminalInfoData.DeviceName) != 0)
	{
		return;
	}
	g_Printf_info("DeviceName:%s\r\n", App.Data.TerminalInfoData.DeviceName);

	if (Hal_getDeviceSecret(App.Data.TerminalInfoData.DeviceSecret) != 0)
	{
		return;
	}
	g_Printf_info("DeviceSecret:%s\r\n", App.Data.TerminalInfoData.DeviceSecret);

	// HashValueSet();
	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Waitfor_SMSReady;
#endif
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode)
	// OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);	  // PowerON-P4.3 //传输板上插LoRa模块时供电
	// OSTimeDly(500);  //节拍2ms
	// ResetCommunication();    		      	//模块复位管脚复位
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率   NBIOT
	// g_Device_NB_Init();
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)

	g_Device_Usart0_Init(115200); //根据所选通信方式选择初始化波特率   LoRa
#if LoRa_QunDeng
	g_Device_Usart0_Init(115200); //根据所选通信方式选择初始化波特率   LoRa
	LoRaDevEui = OSBsp.Device.InnerFlash.innerFLASHRead(9, infor_ChargeAddr);
	LoRaDevEui = LoRaDevEui << 8;
	LoRaDevEui += OSBsp.Device.InnerFlash.innerFLASHRead(10, infor_ChargeAddr);
	LoRa_Deveui[27] = LoRaDevEui / 1000 + 0x30;
	LoRa_Deveui[28] = LoRaDevEui % 1000 / 100 + 0x30;
	LoRa_Deveui[29] = LoRaDevEui % 100 / 10 + 0x30;
	LoRa_Deveui[30] = LoRaDevEui % 10 + 0x30;
#endif
	// InitLoRa_F8L10D();        //初始化LoRa
#elif (TRANSMIT_TYPE == LoRa_OM402_Mode)
	OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);
	;											 //LoRa  PowerON-P4.3 //传输板上插LoRa模块时供电
	hal_Delay_ms(100);							 //wj20180511
	OSBsp.Device.IOControl.ResetWirelesModule(); //模块复位管脚复位
	g_Device_Usart0_Init(9600);					 //根据所选通信方式选择初始化波特率   LoRa
	InitLoRa_OM402();							 //初始化门思LoRa模块
#elif (TRANSMIT_TYPE == LoRa_M100C_Mode)
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率   LoRa
#endif

#if (ACCESSORY_TYPR == None_Mode)
	OSBsp.Device.IOControl.PowerSet(GPS_Power_Off);
#elif (ACCESSORY_TYPR == ELCD_Mode)
	OSBsp.Device.IOControl.PowerSet(GPS_Power_On);
	g_Device_Usart1_Init(115200);
#elif (ACCESSORY_TYPR == GPS_Mode)
	OSBsp.Device.IOControl.PowerSet(GPS_Power_On);
	g_Device_Usart1_Init(9600);
#endif
}

/*******************************************************************************
* 函数名		: Terminal_Data_Init
* 描述	    	: 初始化终端数据
* 输入参数  	: 无
* 返回参数 		: 无
*******************************************************************************/
void Teminal_Data_Init(void)
{
	SensorStatus_H = 0x00;
	SensorStatus_L = 0x00;
	SensorReviseStatus_H = 0x00; //修正
	SensorReviseStatus_L = 0x00;
	Send_Buffer[55] = 0x00;
	Send_Buffer[56] = 0x00;
	SensorSimulationStatus_H = 0x00; //模拟
	SensorSimulationStatus_L = 0x00;
	Send_Buffer[57] = 0x00;
	Send_Buffer[58] = 0x00;
	App.Data.TerminalInfoData.ReviseSimulationCode = 0;

	App.Data.MagicData.CODValue = 0.0;
	App.Data.MagicData.ECValue = 0.0;
	App.Data.MagicData.DOValue = 0.0;
	App.Data.MagicData.NH4Value = 0.0;
	App.Data.MagicData.WaterTemp = 0.0;
	App.Data.MagicData.ORPValue = 0.0;
	App.Data.MagicData.ZSValue = 0.0;
	App.Data.MagicData.PHValue = 0.0;

	WQ_ValueTemp.CODValue = 0.0;
	WQ_ValueTemp.ECValue = 0.0;
	WQ_ValueTemp.DOValue = 0.0;
	WQ_ValueTemp.NH4Value = 0.0;
	WQ_ValueTemp.WaterTemp = 0.0;
	WQ_ValueTemp.ORPValue = 0.0;
	WQ_ValueTemp.ZSValue = 0.0;
	WQ_ValueTemp.PHValue = 0.0;
}

#endif //(PRODUCT_TYPE == Water_Station)
