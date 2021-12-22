﻿/*
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
* Filename      : g_Air_Station.c
* Version       : V1.00
* Programmer(s) : Dingh
*********************************************************************************************************
*/
#include <hal_layer_api.h>
#include <bsp.h>

#if (PRODUCT_TYPE == Air_Station)

#define SensorNum 2
#define CMDLength 8
#define SensorKind 0b111111111111


AppStruct App;
DataStruct *AppDataPointer;
uint32_t Send_Buffer[60] = {0xaa, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00,
							0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF, 0x7F, 0xFF,
							0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff};
//                          /-----------------/ /-----------------/ /-----------------/ /-------/ /--/ /-------/ /-------/ /-------/ /-------/ /--/
//                               timestamp            Lng经度             lat纬度          海拔     PCI    RSRP      SINR       修正      模拟   保留

const uint8_t Inqure_THLP[CMDLength] = {0x03,0x03,0x01,0xF4,0x00,0x08,0x05,0xE0};	 //温湿压光照，PM，噪声
const uint8_t Inqure_TVOC[CMDLength] = {0x09,0x03,0x00,0x07,0x00,0x01,0x34,0x83};	 //TVOC

uint32_t sensorCahe = 0;  //临时存储各指标的值
uint32_t ssensorCahe = 0; //临时存储水温值

float SimulationSensorFloatCahe = 0.0;
int32_t SimulationSensorIntCahe = 0;
static uint8_t SensorStatus_H;
//static uint8_t SensorStatus_L;
static uint8_t SensorReviseStatus_H; //修正
static uint8_t SensorReviseStatus_L;


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
				switch (data[0])
				{
					case 0x03:						   //温湿压PM噪声
						hal_SetBit(SensorStatus_H, 1); //传感器状态位置1						
						//空气湿度
						sensorCahe = (uint32_t)data[3]*256 + data[4];
						AppDataPointer->AirData.AirHumidity = (float)sensorCahe/10;

						//空气温度
						sensorCahe = (uint32_t)data[5]*256 + data[6];
						if(data[5]>=0xF0) //温度为负数，取补码
						{
							sensorCahe = 0XFFFF-((uint32_t)data[5]*256 + data[6])+0X01;
							AppDataPointer->AirData.AirTemperature = (float)sensorCahe/10*(-1);
						}
						else              //温度为正数
						{
							sensorCahe = (uint32_t)data[5]*256 + data[6];
							AppDataPointer->AirData.AirTemperature = (float)sensorCahe/10;
						}
						//噪声
						sensorCahe = (uint32_t)data[7]*256 + data[8];
						AppDataPointer->AirData.Noise = (float)sensorCahe/10;

						//PM2.5
						sensorCahe = (uint32_t)data[9]*256 + data[10];
						AppDataPointer->AirData.PM25 = (float)sensorCahe;

						//PM10
						sensorCahe = (uint32_t)data[11]*256 + data[12];
						AppDataPointer->AirData.PM10 = (float)sensorCahe;
												
						//气压
						sensorCahe = (uint32_t)data[13]*256 + data[14];
						AppDataPointer->AirData.AirPressure = (float)sensorCahe;
						//光照
						// sensorCahe = (uint32_t)data[15]*16777216 + (uint32_t)data[16]*65536 + (uint32_t)data[17]*256 + data[18];
						// AppDataPointer->MeteorologyData.Illumination = (float)sensorCahe/10000;
						// Send_Buffer[39] = data[15];
						// Send_Buffer[40] = data[16];
						// Send_Buffer[41] = data[17];
						// Send_Buffer[42] = data[18];
						break;
					case 0x09:
						hal_SetBit(SensorStatus_H, 2); //传感器状态位置2						
						//空气湿度
						sensorCahe = (uint32_t)data[3]*256 + data[4];
						AppDataPointer->AirData.TVOC = (float)sensorCahe/1000;
						break;
					default:
						break;
				} //switch(data[0]) END
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
// static int SimulationSensorData(void)
// {
	
// 	return 1;
// }

/*******************************************************************************
* 函数名		: InqureSensor
* 描述	    	: 采集传感器数据；分析数据值
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void FilteringSensor(void) 
{

}
void InqureSensor(void)
{
	//COD EC DO NH4 | Temp ORP ZS PH | CHL WL WS XX
	volatile char scadaIndex;
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
//		SensorStatus_L = 0;
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
					OSBsp.Device.Usart3.WriteNData(Inqure_THLP, CMDLength);
					break;
				case 2:
					sensorSN = 2;
					OSBsp.Device.Usart3.WriteNData(Inqure_TVOC, CMDLength);
					break;
				default:
					break;
				}
				hal_Delay_ms(2); //高波特率降低延时为1-2ms，否则容易丢包；低波特率增加延时，如4800延时10ms，否则容易丢包
				Recive_485_Enable;

				LED_ON;
				OSTimeDly(1500); //任务挂起 3000ms,等待传感器回复，接收完毕
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
	uint8_t gpsBuffer[15];

	mallco_dev.init();

	cJSON *pJsonRoot = mymalloc(512 * sizeof(cJSON *));
	cJSON *pSubJson = mymalloc(128 * sizeof(cJSON *));
	char *p;

	pJsonRoot = cJSON_CreateObject();
	if (NULL == pJsonRoot)
	{
		cJSON_Delete(pJsonRoot);
		return NULL;
	}

	cJSON_AddNumberToObject(pJsonRoot, "DeviceID", DataPointer->TerminalInfoData.SerialNumber);
	// cJSON_AddNumberToObject(pJsonRoot, "DeviceID", DataPointer->TerminalInfoData.DeviceID);
	cJSON_AddNumberToObject(pJsonRoot, "SeqNum", DataPointer->TransMethodData.SeqNumber);
	// if (REGRST != 0)
	// {
	// 	cJSON_AddNumberToObject(pJsonRoot, "reboot", REGRST);
	// 	REGRST = 0;
	// }

	pSubJson = NULL;
	pSubJson = cJSON_CreateObject();
	if (NULL == pSubJson)
	{
		//create object faild, exit
		cJSON_Delete(pSubJson);
		return NULL;
	}

	if (hal_GetBit(SensorStatus_H, 1))	//温湿压、PM、噪声
	{
		cJSON_AddNumberToObject(pJsonRoot, "Temp", DataPointer->AirData.AirTemperature);
		cJSON_AddNumberToObject(pJsonRoot, "Humi", DataPointer->AirData.AirHumidity);
		cJSON_AddNumberToObject(pJsonRoot, "Press", DataPointer->AirData.AirPressure);
		cJSON_AddNumberToObject(pJsonRoot, "NS", DataPointer->AirData.Noise);
		cJSON_AddNumberToObject(pJsonRoot, "PM2_5", DataPointer->AirData.PM25);
		cJSON_AddNumberToObject(pJsonRoot, "PM10", DataPointer->AirData.PM10);
	}
	if (hal_GetBit(SensorStatus_H, 2))	//TVOC
	{
		cJSON_AddNumberToObject(pJsonRoot, "TVOC", DataPointer->AirData.TVOC);
	}
	cJSON_AddNumberToObject(pJsonRoot, "serviceId", 12);
	// cJSON_AddItemToObject(pJsonRoot, "WaterData", pSubJson);
#if (TRANSMIT_TYPE == GPRS_Mode)
	cJSON_AddStringToObject(pJsonRoot, "CSQ", CSQBuffer);
#endif
// #if (TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)
// 	cJSON_AddNumberToObject(pJsonRoot, "RSRP", DataPointer->TransMethodData.RSRP);
// 	cJSON_AddNumberToObject(pJsonRoot, "SINR", DataPointer->TransMethodData.SINR);
	cJSON_AddNumberToObject(pJsonRoot, "CSQ", DataPointer->TransMethodData.CSQ);
// #endif



	cJSON_AddNumberToObject(pJsonRoot, "Period", DataPointer->TerminalInfoData.SendPeriod);
	cJSON_AddNumberToObject(pJsonRoot, "BatteryPercentage", DataPointer->TerminalInfoData.PowerQuantity);
	cJSON_AddNumberToObject(pJsonRoot, "Version", DataPointer->TerminalInfoData.Version);

	
	uint8_t date[8];
	char Uptime[19] = "2019-09-01 00:00:00";
	char filestore[19];
	memset(date, 0x0, 8);
	memset(Uptime, 0x0, 19);
	memset(filestore, 0x0, 19);
	// OSBsp.Device.RTC.ReadExtTime(date, RealTime);
	// if ((date[4] > 0x59) || (date[5] > 0x59) || (date[6] > 0x59))
	// {
		Read_info_RTC(date);
	// }
	g_Device_RTCstring_Creat(date, Uptime);
	g_Printf_info("Uptime:%s\r\n", Uptime);
	cJSON_AddStringToObject(pJsonRoot, "Uptime", Uptime);
	// cJSON_AddNumberToObject(pJsonRoot, "UnixTimeStamp", UnixTimeStamp);
	//cJSON_AddNumberToObject(pJsonRoot, "ReSiC", DataPointer->TerminalInfoData.ReviseSimulationCode);

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
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.NBStatus = NB_Power_off;
#elif (TRANSMIT_TYPE == NBIoT_MQTT_Ali)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	// AppDataPointer->TransMethodData.NBStatus = NB_Power_off;	//读取三元组时赋值	
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
	Send_Buffer[4] = App.Data.TerminalInfoData.DeviceType;
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
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)
	// OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);	  // PowerON-P4.3 //传输板上插LoRa模块时供电
	// OSTimeDly(500);  //节拍2ms
	// ResetCommunication();    		      	//模块复位管脚复位
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率   NBIOT
	// g_Device_NB_Init();
#elif (TRANSMIT_TYPE == NBIoT_MQTT_Ali)
	memset(App.Data.TerminalInfoData.ProductKey, 0, 32);
	memset(App.Data.TerminalInfoData.DeviceName, 0, 32);
	memset(App.Data.TerminalInfoData.DeviceSecret, 0, 64);
	g_Device_Usart0_Init(9600); //根据所选通信方式选择初始化波特率   NBIOT
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
	// strcpy(App.Data.TerminalInfoData.ProductKey,"a1N1WQVAcqN");
	// g_Printf_info("ProductKey:%s\r\n", App.Data.TerminalInfoData.ProductKey);
	// strcpy(App.Data.TerminalInfoData.DeviceName,"Device01");
	//  g_Printf_info("DeviceName:%s\r\n", App.Data.TerminalInfoData.DeviceName);
	// strcpy(App.Data.TerminalInfoData.DeviceSecret,"af67f8034088eee4b65ea1b5adf33450");
	// g_Printf_info("DeviceSecret:%s\r\n", App.Data.TerminalInfoData.DeviceSecret);
	// App.Data.TerminalInfoData.DeviceName = "Device01";
	// App.Data.TerminalInfoData.DeviceSecret = "af67f8034088eee4b65ea1b5adf33450";
	AppDataPointer->TransMethodData.NBStatus = NB_Power_off;
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
//	SensorStatus_L = 0x00;
	SensorReviseStatus_H = 0x00; //修正
	SensorReviseStatus_L = 0x00;
	Send_Buffer[55] = 0x00;
	Send_Buffer[56] = 0x00;

	Send_Buffer[57] = 0x00;
	Send_Buffer[58] = 0x00;
	App.Data.TerminalInfoData.ReviseSimulationCode = 0;
	App.Data.AirData.AirTemperature = 0;
	App.Data.AirData.AirHumidity = 0;
	App.Data.AirData.AirPressure = 0;
	App.Data.AirData.PM25 = 0;
	App.Data.AirData.PM10 = 0;
	App.Data.AirData.Noise = 0;
	App.Data.AirData.TVOC = 0;
}

#endif //(PRODUCT_TYPE == Air_Station)
