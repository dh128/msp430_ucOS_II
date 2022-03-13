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
* Filename   : g_WRain_Station.c
* Change Logs:
* Date			Author		Notes
* 2020-07-08	dingh   	the first version
* 2022-02-19	dingh		access to China Telecom AEP
* 2022-03-12	dingh		update sensor record
*********************************************************************************************************
*/
#include  <hal_layer_api.h>
#include  <bsp.h>

#if (PRODUCT_TYPE == WRain_Station)

#define SensorNum			2
#define CMDLength        	8
#define SensorKind          0x03
// uint16_t RainStatus;              //雨量传感器状态位

/* Sensor Exist flag */
#define RAIN			0
#define USLV			1
uint8_t SensorRecord = 0;	/* 传感器记录标志，1--记录，0--不记录 */
uint16_t SensorExist = SensorKind;
AppStruct  App;
DataStruct *AppDataPointer;


uint32_t Send_Buffer[60] = {0xaa,0x00,0x00,0x01,0x01,0x00,0x00,
                            0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,
							0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff};
//                          /-------/ /--/ /-----------------/ /-----------------/ /-----------------/ /-------/ /-------/ /-------/ /-------/ /-------/ /-------/ /--/
//                            Period   ver		timestamp            Lng经度             lat纬度          海拔      RSRP      SINR       PCI      保留		模拟  	 保留


const uint8_t ScadaUSLV[CMDLength]  = {0x01,0x03,0x00,0x01,0x00,0x01,0xD5,0xCA};		      //超声波液位
const uint8_t ScadaRainGauge_XPH[CMDLength]={0x08,0x03,0x00,0x00,0x00,0x01,0x84,0x93};            //雨量-XPH



uint32_t sensorCahe = 0;
static uint8_t SensorStatus_H;
static uint8_t SensorStatus_L;

/*******************************************************************************
* 函数名		: AnalyzeComand
* 描述	    	: 解析传感器指令
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
static int AnalyzeComand(uint8_t *data,uint8_t Len)
{
	if(Len > 2)
	{
		uint16_t CalcuResult = 0;
		uint8_t CRC_Result[2];
		hal_Delay_ms(50);

		CalcuResult = Crc16(data,Len-2);
		CRC_Result[0] = (uint8_t)((CalcuResult & 0xFF00) >> 8);
		CRC_Result[1] = (uint8_t)(CalcuResult & 0xFF);
		if((data[Len-2] == CRC_Result[0]) && (data[Len-1] == CRC_Result[1]))   //判断数据接收是否存在异常
		{
	        LED_ON;
			if(data[1]==0x03)
			{
				switch(data[0])
				{
					case 0x01:		//超声波液位传感器
					    sensorCahe = (uint32_t)data[3]*256 + data[4];
					    AppDataPointer->WRainData.Ultrasonic = (float)sensorCahe/1000;    //mm单位转化为m
						AppDataPointer->WRainData.Real = AppDataPointer->WRainData.Height - AppDataPointer->WRainData.Ultrasonic;
					    hal_SetBit(SensorStatus_L, USLV);   //传感器状态位置1
					    Send_Buffer[7] = (uint32_t)(sensorCahe/10) / 256;
					    Send_Buffer[8] = (uint32_t)(sensorCahe/10) % 256;
					break;
					case 0x08:	  //XPH--雨量
						hal_SetBit(SensorStatus_L, RAIN);     //传感器状态位置1
						// RainStatus = 0B100000000000;
						sensorCahe = (uint32_t)data[3]*256 + data[4];
						AppDataPointer->WRainData.RainGauge = (float)sensorCahe/10;
						AppDataPointer->WRainData.RainGaugeH += AppDataPointer->WRainData.RainGauge;
						AppDataPointer->WRainData.RainGaugeD += AppDataPointer->WRainData.RainGauge;

						Send_Buffer[9] = sensorCahe / 256;
						Send_Buffer[10] = sensorCahe % 256;
						if(AppDataPointer->WRainData.RainGaugeScadaStatus & RAINGAUGE_REPORT_HOUR)//判断发送小时数据
						{
							g_Printf_dbg("RAINGAUGE_REPORT_HOUR\r\n");
							//AppDataPointer->WRainData.RainGaugeScadaStatus &=  ~RAINGAUGE_REPORT_HOUR;
							Send_Buffer[11] = (uint32_t)(AppDataPointer->WRainData.RainGaugeH*10) / 256;
							Send_Buffer[12] = (uint32_t)(AppDataPointer->WRainData.RainGaugeH*10) % 256;
						}
						if(AppDataPointer->WRainData.RainGaugeScadaStatus & RAINGAUGE_REPORT_DAY)//判断发送24小时数据
						{
							g_Printf_dbg("RAINGAUGE_REPORT_HOUR\r\n");
							//AppDataPointer->WRainData.RainGaugeScadaStatus &= ~RAINGAUGE_REPORT_DAY;
							Send_Buffer[13] = (uint32_t)(AppDataPointer->WRainData.RainGaugeD*10) / 256;
							Send_Buffer[14] = (uint32_t)(AppDataPointer->WRainData.RainGaugeD*10) % 256;
						}
						break;
					default:
						break;
				}//switch(data[0]) END
			} //(data[1]==0x03)  END
//			Send_Buffer[55] = SensorReviseStatus_H;
//			Send_Buffer[56] = SensorReviseStatus_L;
			Clear_CMD_Buffer(dRxBuff,dRxNum);
			dRxNum=0;
			Len = 0;
			return 1;
 		}else{
			Clear_CMD_Buffer(dRxBuff,dRxNum);
			dRxNum=0;
			Len = 0;
			return -2;
		}
	}else{
		Clear_CMD_Buffer(dRxBuff,dRxNum);
		dRxNum=0;
		Len = 0;
		return -1;
	}
}

/*******************************************************************************
* 函数名		: InqureSensor
* 描述	    	: 采集传感器数据；分析数据值
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void InqureSensor(void)
{
	//WS WD Temp Humi |Presure PM2.5 PM10  Rain | Radi XX XX XX
	volatile char scadaIndex;
	volatile uint16_t sensorExistStatus = 0;
	volatile uint8_t sensorSN = 0;    //传感器编号，按照协议顺序排列

	if (AppDataPointer->TerminalInfoData.SensorReadStatus == SENSOR_STATUS_READ_NOTYET)
	{
		AppDataPointer->TerminalInfoData.SensorReadStatus = SENSOR_STATUS_READ_OK;
		AppDataPointer->TerminalInfoData.SensorStatus = SensorKind;
	    Teminal_Data_Init();   //数据初始化
	}
	else if ((AppDataPointer->TerminalInfoData.SensorReadStatus == SENSOR_STATUS_READ_OK))
	{	/* 使能传感器标记并且读取一遍后，后续轮询前更新传感器标志 */
		AppDataPointer->TerminalInfoData.SensorStatus = SensorExist;	/* 第一轮保存的标志位 */
	}
	if(AppDataPointer->TerminalInfoData.SensorStatus != 0) {
		for(scadaIndex=1;scadaIndex<=SensorNum;scadaIndex++)
		{
			memset(dRxBuff,0x0,dRxLength); //dRxLength=50，清空，接收Usart3即传感器数据
			dRxNum=0;
			sensorExistStatus = (AppDataPointer->TerminalInfoData.SensorStatus) & 0x0001;
			AppDataPointer->TerminalInfoData.SensorStatus = (AppDataPointer->TerminalInfoData.SensorStatus) >> 1;
			if(sensorExistStatus == 1)
			{
				Send_485_Enable;
				hal_Delay_ms(5);
				switch(scadaIndex)
				{
					case 1:
						sensorSN = 1;
						if(AppDataPointer->WRainData.RainGaugeScadaStatus & RAINGAUGE_SCADA_ENABLE)
						{
							hal_ResetBit(SensorStatus_L, RAIN);
							AppDataPointer->WRainData.RainGaugeScadaStatus &= ~RAINGAUGE_SCADA_ENABLE;
							OSBsp.Device.Usart3.WriteNData((uint8_t *)ScadaRainGauge_XPH,CMDLength);
						}
						//雨量一个周期内只读取一次
						break;
					case 2:
						sensorSN = 2;
						hal_ResetBit(SensorStatus_L, USLV);
						OSBsp.Device.Usart3.WriteNData((uint8_t *)ScadaUSLV,CMDLength);
						break;

					default:
						break;
				}
				hal_Delay_ms(2);//高波特率降低延时为1-2ms，否则容易丢包；低波特率增加延时，如4800延时10ms，否则容易丢包
				// OSTimeDly(2);//高波特率降低延时为1-2ms，否则容易丢包；低波特率增加延时，如4800延时10ms，否则容易丢包
				Recive_485_Enable;
				// LED_ON;
				// OSTimeDly(500);  //任务挂起
				OSTimeDly(400);     //任务挂起
				int ret = AnalyzeComand(dRxBuff,dRxNum);//解包
				OSTimeDly(100);     //LED指示灯延时
				uint32_t times = sensorSN;
                if(ret == 1){
					g_Printf_dbg("%s.AnalyzeComand.Sensor answer ok - Sensor SN %d\r\n",__func__,times);
				}else if(ret == -2){
					g_Printf_dbg("%s.AnalyzeComand.CRC check failed\r\n",__func__);
				}else if(ret == -1){
					g_Printf_dbg("%s.AnalyzeComand.Sensor has no answer - Sensor SN %d\r\n",__func__,times);
					sensorSN = 0;
				}
				LED_OFF;
				Clear_CMD_Buffer(dRxBuff,dRxNum);  //发送之前buff清0
				dRxNum=0;

			}
		}

		/* 上电后第一次检查哪些传感器在线 */
		if (AppDataPointer->TerminalInfoData.SensorWriteStatus == SENSOR_STATUS_WRITE_NOTYET)
		{
			AppDataPointer->TerminalInfoData.SensorWriteStatus = SENSOR_STATUS_WRITE_ALREADY;
			if(SensorRecord){
				SensorExist = (uint16_t)SensorStatus_H * 256 + (uint16_t)SensorStatus_L; //本次读取到的传感器置位
			}
		}
	} else {
		OSTimeDly(10);
		g_Printf_dbg("%s.AnalyzeSensor.No sensor to scan\r\n",__func__);
		AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_SCAN_OVER;
		OSTimeDly(10);
		g_Printf_info("ScadaTask is over\n");
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
    mallco_dev.init();
//    RainStatus = 0;
    cJSON *pJsonRoot = mymalloc(512*sizeof(cJSON *));
	char *p;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        cJSON_Delete(pJsonRoot);
        return NULL;
    }

    cJSON_AddNumberToObject(pJsonRoot, "DeviceID",DataPointer->TerminalInfoData.DeviceID);
    cJSON_AddNumberToObject(pJsonRoot, "SeqNum",DataPointer->TransMethodData.SeqNumber);
	cJSON_AddNumberToObject(pJsonRoot, "serviceId", 12);

	if(hal_GetBit(SensorStatus_L, RAIN)) {
		cJSON_AddNumberToObject(pJsonRoot,"RainR",DataPointer->WRainData.RainGauge);
	}
	if(hal_GetBit(SensorStatus_L, USLV)) {
		cJSON_AddNumberToObject(pJsonRoot,"Real",DataPointer->WRainData.Real);
		cJSON_AddNumberToObject(pJsonRoot,"Ultrasonic",DataPointer->WRainData.Ultrasonic);
		cJSON_AddNumberToObject(pJsonRoot,"Height",DataPointer->WRainData.Height);
	}

	cJSON_AddNumberToObject(pJsonRoot,"CSQ",DataPointer->TransMethodData.CSQ);
	cJSON_AddNumberToObject(pJsonRoot,"Period",DataPointer->TerminalInfoData.SendPeriod);
	cJSON_AddNumberToObject(pJsonRoot,"BatteryPercentage",DataPointer->TerminalInfoData.PowerQuantity);
	cJSON_AddNumberToObject(pJsonRoot,"Version",DataPointer->TerminalInfoData.Version);

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

    p = cJSON_Print(pJsonRoot);
    if(NULL == p)
    {
      //convert json list to string faild, exit
      //because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
      cJSON_Delete(pJsonRoot);
      return NULL;
    }

    cJSON_Delete(pJsonRoot);
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
    AppDataPointer = &(App.Data);     //DataStruct * AppDataPointer; 定义结构体类型的指针 ,只是一个名字，具体要定义指针的地址

	//只初始化要用到的数据
#if (TRANSMIT_TYPE == GPRS_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_off;
	AppDataPointer->TransMethodData.GPRSNet = 0;
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
	AppDataPointer->TransMethodData.NBStatus = NB_Power_off;
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)
	AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF;
    AppDataPointer->TransMethodData.LoRaStatus = LoRa_Power_off;
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

	/************************地理信息*******************************************/
	for(i=0;i<8;i++){
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+32,infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Longitude = TransferData.Data;
	for(i=0;i<8;i++){
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+40,infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Latitude = TransferData.Data;
	for(i=0;i<8;i++){
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+48,infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Altitude = TransferData.Data;


	/************************DeviceID******************************************/
	App.Data.TerminalInfoData.DeviceID = Hal_getDeviceID();
	Send_Buffer[1] = (App.Data.TerminalInfoData.DeviceID>>16) & 0xFF;
	Send_Buffer[2] = (App.Data.TerminalInfoData.DeviceID>>8) & 0xFF;
	Send_Buffer[3] = App.Data.TerminalInfoData.DeviceID & 0xFF;
	/**************************出厂编号******************************************/
	App.Data.TerminalInfoData.SerialNumber = Hal_getSerialNumber();
	/**************************生产日期******************************************/
	App.Data.TerminalInfoData.PD = Hal_getManufactureDate();
	/**************************TerminalIndex 终端类型****************************/
	App.Data.TerminalInfoData.DeviceType = PRODUCT_TYPE;
	Send_Buffer[4] = App.Data.TerminalInfoData.DeviceType;
	/**************************SendPeriod***************************************/
	App.Data.TerminalInfoData.SendPeriod = Hal_getTransmitPeriod();  //发送周期
	Send_Buffer[31] = (App.Data.TerminalInfoData.SendPeriod>>8) & 0x00FF;
	Send_Buffer[32] = App.Data.TerminalInfoData.SendPeriod & 0x00FF;

	SensorRecord = Hal_getSensorRecord();	/* 获取传感器记录标志 */
	g_Printf_info("SensorRecord flag = %d\r\n", (uint32_t)SensorRecord);
	/**************************Version******************************************/
	App.Data.TerminalInfoData.Version = Hal_getFirmwareVersion();    //软件版本
	Send_Buffer[34] = App.Data.TerminalInfoData.Version;

	App.Data.WRainData.Height = Hal_getSensorHeight();

	/**************************未读取Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorReadStatus = SENSOR_STATUS_READ_NOTYET;
	/**************************未写入Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorWriteStatus = SENSOR_STATUS_WRITE_NOTYET;
	/**************************允许同步时间状态***********************/
	App.Data.TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_ENABLE;
	/**************************初始化允许雨量采集*********************/
    App.Data.WRainData.RainGaugeScadaStatus = RAINGAUGE_SCADA_ENABLE;

#if (TRANSMIT_TYPE == GPRS_Mode)
	#ifdef SIM800C
	// Socket_5V_ON;            //GPRS  PowerON-P5.0 //传输板上插GPRS模块时供电
	// delay_sec(5); 			 //wj20180511  为了稳定5V电源一段时间
	// ResetCommunication();    //模块复位管脚复位,对于GPRS模块电压需要达到5V,单片输出管脚电压只有3.3V
	g_Device_Usart0_Init(9600);	     //根据所选通信方式选择初始化波特率  GPRS
	// InitSim800C();           //初始化GPRS
	#endif
	#ifdef AIR202
	g_Device_Usart0_Init(9600);        //根据所选通信方式选择初始化波特率
	// Socket_5V_ON;            //GPRS  PowerON-P5.0 //传输板上插GPRS模块时供电
	// Socket_3V_ON;
	// delay_sec(2);
	// AIR202_Power_ON;
	// delay_sec(2);
	// AIR202_Power_OFF;
	if(Hal_getProductKey(App.Data.TerminalInfoData.ProductKey) != 0){
		return ;
	}
	g_Printf_info("ProductKey:%s\r\n",App.Data.TerminalInfoData.ProductKey);

	if(Hal_getDeviceName(App.Data.TerminalInfoData.DeviceName) != 0){
		return ;
	}
	g_Printf_info("DeviceName:%s\r\n",App.Data.TerminalInfoData.DeviceName);

	if(Hal_getDeviceSecret(App.Data.TerminalInfoData.DeviceSecret) != 0){
		return ;
	}
	g_Printf_info("DeviceSecret:%s\r\n",App.Data.TerminalInfoData.DeviceSecret);

	// HashValueSet();
	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Waitfor_SMSReady;
	#endif
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode || TRANSMIT_TYPE == NBIoT_AEP)
	// OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);	  // PowerON-P4.3 //传输板上插LoRa模块时供电
    // OSTimeDly(500);  //节拍2ms
	// ResetCommunication();    		      	//模块复位管脚复位
	g_Device_Usart0_Init(9600);	     	//根据所选通信方式选择初始化波特率   NBIOT
	// g_Device_NB_Init();
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)

	g_Device_Usart0_Init(115200);      //根据所选通信方式选择初始化波特率   LoRa
#if LoRa_QunDeng
	g_Device_Usart0_Init(115200);      //根据所选通信方式选择初始化波特率   LoRa
	LoRaDevEui = OSBsp.Device.InnerFlash.innerFLASHRead(9,infor_ChargeAddr);
	LoRaDevEui=LoRaDevEui<<8;
	LoRaDevEui += OSBsp.Device.InnerFlash.innerFLASHRead(10,infor_ChargeAddr);
	LoRa_Deveui[27]= LoRaDevEui/1000 + 0x30;
	LoRa_Deveui[28]= LoRaDevEui%1000/100 + 0x30;
	LoRa_Deveui[29]= LoRaDevEui%100/10 + 0x30;
	LoRa_Deveui[30]= LoRaDevEui%10 + 0x30;
#endif
	// InitLoRa_F8L10D();        //初始化LoRa
#elif (TRANSMIT_TYPE == LoRa_OM402_Mode)
	OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);;	         //LoRa  PowerON-P4.3 //传输板上插LoRa模块时供电
	hal_Delay_ms(100);			 //wj20180511
	OSBsp.Device.IOControl.ResetWirelesModule();    //模块复位管脚复位
	g_Device_Usart0_Init(9600);        //根据所选通信方式选择初始化波特率   LoRa
	InitLoRa_OM402();        //初始化门思LoRa模块
#elif (TRANSMIT_TYPE == LoRa_M100C_Mode)
	g_Device_Usart0_Init(9600);      //根据所选通信方式选择初始化波特率   LoRa
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
	Send_Buffer[55] = 0x00;
	Send_Buffer[56] = 0x00;
	Send_Buffer[57] = 0x00;
	Send_Buffer[58] = 0x00;

	App.Data.WRainData.Ultrasonic = 0.0;
	App.Data.WRainData.Real = 0.0;
	App.Data.WRainData.RainGauge = 0.0;
	App.Data.WRainData.RainGaugeH = 0.0;
	App.Data.WRainData.RainGaugeD = 0.0;
}



#endif //(PRODUCT_TYPE == Water_Station)

