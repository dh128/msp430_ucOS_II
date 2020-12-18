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
* Filename      : g_Voc_Station.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <hal_layer_api.h>
#include  <bsp.h>

#if (PRODUCT_TYPE == Flowmeter_Station)

#define SensorNum			12
#define CMDLength        	8
#define SensorKind          0b111111111111

AppStruct  App;
DataStruct *AppDataPointer;

//uint32_t Send_Buffer[34] = {0xaa,0x00,0x00,0x01,0x01,0x00,0x01,0x7F,0xFF,0x7F,0xFF,0x7F,
//0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0xff};


uint32_t Send_Buffer[60] = {0xaa,0x00,0x00,0x01,0x01,0x00,0x00,
                            0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,
                            0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff};
					//      /-------/ /--/ /--/ /-----------------/ /-----------------/ /-----------------/ /-------/
					//        period  电量  Ver       timestamp            Lng经度             lat纬度          海拔

const uint8_t ScadaFlow1[CMDLength]  = {0x01,0x03,0x00,0x00,0x00,0x0E,0xC4,0x0E};	          //超声波流量计


#define WQ_FlowD_Num 		5
#define WQ_FlowS_Num 		5

static float  WQ_FlowD[WQ_FlowD_Num]; //申请每一次用于水温的记录，最多读WQ_FlowD_Num次
static uint8_t FlowDtimes = 0; //存储水温的临时变量
static float  WQ_FlowS[WQ_FlowS_Num]; //申请每一次用于水温的记录，最多读WQ_FlowS_Num次
static uint8_t FlowStimes = 0; //存储水温的临时变量
float wq_FlowD_sum = 0.0;
float wq_FlowS_sum = 0.0;
float wq_FlowD = 0.0;
float wq_FlowS = 0.0;

float sensorFloatCahe = 0.0;
uint32_t sensorCahe = 0;
uint32_t ssensorCahe = 0;
float SimulationSensorFloatCahe = 0.0;
uint32_t SimulationSensorIntCahe = 0;
static uint8_t SensorStatus_H;
static uint8_t SensorStatus_L;
static uint8_t SensorReviseStatus_H;      //修正
static uint8_t SensorReviseStatus_L;
static uint8_t SensorSimulationStatus_H;  //模拟
static uint8_t SensorSimulationStatus_L;

//static uint8_t  SensorStatusBuff[2];       //传感器状态数组

/*******************************************************************************
* 描述	    	: 4字节16进制转浮点数  结构体
*******************************************************************************/
// typedef union
// {
// 	uint8_t Hex[4];
// 	float Data;
// }Hex2Float;
Hex2Float SensorData;

/*******************************************************************************
* 函数名		: AnalyzeComand
* 描述	    	: 解析传感器指令
* 输入参数  	: data,len
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
			   case 0x01:		//流量计1
			   //水深1
				SensorData.Hex[0] = data[6];      //大端模式，高位字节存放在后面
				SensorData.Hex[1] = data[5];
				SensorData.Hex[2] = data[4];
				SensorData.Hex[3] = data[3];
				//存储采集数据，用于算术平均
				if (FlowDtimes <= (WQ_FlowD_Num-1))
				{
				   WQ_FlowD[FlowDtimes] = SensorData.Data;
//				   g_Printf_dbg("Get a ZSValue Tempture: %f\r\n",WQ_WaterTemp[watertemprectimes]);
				   if((WQ_FlowD[FlowDtimes] <= -10.0) || (WQ_FlowD[FlowDtimes] >= 50.0))
				   {
					   WQ_FlowD[FlowDtimes] = AppDataPointer->FlowmeterData.Depth1Vlaue;
					}
				   FlowDtimes++;
				}

				hal_SetBit(SensorStatus_H, 3);   //传感器状态位置1
				AppDataPointer->FlowmeterData.Depth1Vlaue = SensorData.Data;
				Send_Buffer[7]=(int)(AppDataPointer->FlowmeterData.Depth1Vlaue*1000)/256;
				Send_Buffer[8]=(int)(AppDataPointer->FlowmeterData.Depth1Vlaue*1000)%256;

				//瞬时流速1
				SensorData.Hex[0] = data[30];      //大端模式，高位字节存放在后面
				SensorData.Hex[1] = data[29];
				SensorData.Hex[2] = data[28];
				SensorData.Hex[3] = data[27];
				//存储采集数据，用于算术平均
				if (FlowStimes <= (WQ_FlowS_Num-1))
				{
				   WQ_FlowS[FlowStimes] = SensorData.Data;
			//				   g_Printf_dbg("Get a ZSValue Tempture: %f\r\n",WQ_WaterTemp[watertemprectimes]);
				  if((WQ_FlowS[FlowStimes] <= -10.0) || (WQ_FlowS[FlowStimes] >= 50.0))
				   {
					   WQ_FlowS[FlowStimes] = AppDataPointer->FlowmeterData.Speed1Vlaue;
				   }
					 FlowStimes++;
				 }
				hal_SetBit(SensorStatus_H, 2);   //传感器状态位置1
				AppDataPointer->FlowmeterData.Speed1Vlaue = SensorData.Data;
				Send_Buffer[9]=(int)(AppDataPointer->FlowmeterData.Speed1Vlaue*1000)/256;
				Send_Buffer[10]=(int)(AppDataPointer->FlowmeterData.Speed1Vlaue*1000)%256;
				break;
				default:
					break;
			}//switch(data[0]) END
			} //(data[1]==0x03)  END
			Send_Buffer[55] = SensorReviseStatus_H;
			Send_Buffer[56] = SensorReviseStatus_L;
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

		// SensorStatusBuff[0] = SensorStatus_H;
		// SensorStatusBuff[1] = SensorStatus_L;
		// AppDataPointer->TerminalInfoData.SensorStatus = (uint16_t)SensorStatus_H*256 + (uint16_t)SensorStatus_L;

		// Clear_CMD_Buffer(dRxBuff,dRxNum);
		// dRxNum=0;
		// Len = 0;
	}else{
		Clear_CMD_Buffer(dRxBuff,dRxNum);
		dRxNum=0;
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
    uint32_t temp =0;

	temp = AppDataPointer->TerminalInfoData.SensorStatusSimulation;
	Itoa(temp,str,2); //2进制输出
	g_Printf_dbg("%s binary format: %s\n",__func__,str);

    if(AppDataPointer->TerminalInfoData.SensorStatusSimulation != AppDataPointer->TerminalInfoData.SensorStatusSimulation_Old) {
		infor_ChargeAddrBuff[24] = (uint8_t)(temp >> 8);
		infor_ChargeAddrBuff[25] = (uint8_t)temp;
		// OSBsp.Device.InnerFlash.innerFLASHWrite(&infor_ChargeAddrBuff,(uint8_t *)(infor_ChargeAddr+0),32);
		OSBsp.Device.InnerFlash.innerFLASHWrite(infor_ChargeAddrBuff,(uint8_t *)(infor_ChargeAddr+0),32);
	}
    AppDataPointer->TerminalInfoData.SensorStatusSimulation_Old = AppDataPointer->TerminalInfoData.SensorStatusSimulation;

	for(simulationIndex=1;simulationIndex<=SensorNum;simulationIndex++)  //SensorNum = 12
	{
		// sensorNOTExistStatus = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) & 0x0001;
		// AppDataPointer->TerminalInfoData.SensorStatusSimulation = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) >> 1;
		// if(sensorNOTExistStatus == 1)
		sensorNOTExistStatus = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) & 0x0800;
		AppDataPointer->TerminalInfoData.SensorStatusSimulation = (AppDataPointer->TerminalInfoData.SensorStatusSimulation) << 1;
		if(sensorNOTExistStatus == 0x0800)
		{
			switch(simulationIndex)
			{
				case 1:
					/**************水深**************///
					 SimulationSensorFloatCahe = 10.21 + (float)(rand()%4)/10 - (float)(rand()%4)/10;
					 if (SimulationSensorFloatCahe < 0.0)
					  {
					 	SimulationSensorFloatCahe = 10.21;
					   }
					 AppDataPointer->FlowmeterData.Depth1Vlaue = SimulationSensorFloatCahe;
					 hal_SetBit(SensorStatus_H, 3);             //传感器状态位置1
                     hal_SetBit(SensorSimulationStatus_H, 3);   //传感器模拟状态位置1
					 Send_Buffer[7] = (uint32_t)(SimulationSensorFloatCahe*100) / 256;
					 Send_Buffer[8] = (uint32_t)(SimulationSensorFloatCahe*100) % 256;
					break;
				case 2:
					/**************流速**************///
					SimulationSensorFloatCahe = 0.31 + (float)(rand()%4)/10 - (float)(rand()%4)/10;
					if (SimulationSensorFloatCahe < 0.0)
					{
					    SimulationSensorFloatCahe = 0.31;
					}
					AppDataPointer->FlowmeterData.Speed1Vlaue = SimulationSensorFloatCahe;
					hal_SetBit(SensorStatus_H, 2);             //传感器状态位置1
					hal_SetBit(SensorSimulationStatus_H, 2);   //传感器模拟状态位置1
					Send_Buffer[9] = (uint32_t)(SimulationSensorFloatCahe*100) / 256;
					Send_Buffer[10] = (uint32_t)(SimulationSensorFloatCahe*100) % 256;
					break;
				case 3:

					break;
				case 4:
					/**************CO2**************///
					break;
				case 5:
					/**************H2S**************///
					break;
				case 6:
					break;
				case 7:
					break;
				case 8:
				    /**************PH**************///7.21~7.61
					// SimulationSensorFloatCahe = 7.21 + (float)(rand()%4)/10;
					// AppDataPointer->WaterData.PHValue = SimulationSensorFloatCahe;
					// hal_SetBit(SensorStatus_L, 4);             //传感器状态位置1
					// hal_SetBit(SensorSimulationStatus_L, 4);   //传感器模拟状态位置1
					// Send_Buffer[21] = (uint32_t)(SimulationSensorFloatCahe*100) / 256;
					// Send_Buffer[22] = (uint32_t)(SimulationSensorFloatCahe*100) % 256;
					break;
				case 9:
					/**************CHL************/
					break;
				case 10:
					break;
				case 11:
					/**************光照**************///
					break;
				case 12:
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
* 函数名		: InqureSensor
* 描述	    	: 采集传感器数据；分析数据值
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void FilteringSensor(void)   //wj20200215 这个只能保证传感器都能读取到数据后，解析后，有滤波的功能。压根就没读取到这个传感器数据的情况不在这里处理
{
	  uint8_t i=0;
//平均法
		for (i=0; i<FlowDtimes; i++)
		{
			wq_FlowD_sum += WQ_FlowD[i];
		}
		wq_FlowD = wq_FlowD_sum / (float)(FlowDtimes);
		AppDataPointer->FlowmeterData.Depth1Vlaue = wq_FlowD;

		for (i=0; i<FlowStimes; i++)
		{
			wq_FlowS_sum += WQ_FlowS[i];
		}
			wq_FlowS = wq_FlowS_sum / (float)(FlowStimes);
			AppDataPointer->FlowmeterData.Speed1Vlaue = wq_FlowS;


}

/*******************************************************************************
* 函数名		: InqureSensor
* 描述	    	: 采集传感器数据；分析数据值
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void InqureSensor(void)
{
	//COD EC DO NH4 | Temp ORP ZS PH | CHL WL WS XX
	volatile char scadaIndex;
	volatile uint16_t sensorExistStatus = 0;
	volatile uint8_t sensorSN = 0;    //传感器编号，按照协议顺序排列
	volatile uint16_t sensorStatus;   //0000 0011 1100 0000     Do,氨氮，温度，ORP

	if(AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_NOTYET) {
		AppDataPointer->TerminalInfoData.SensorFlashReadStatus = SENSOR_STATUS_READFLASH_ALREADY;
		AppDataPointer->TerminalInfoData.SensorStatus = SensorKind;
		// Teminal_Data_Init();   //数据初始化
	} else if ( (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_ALREADY)
	         || (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_OK) ) {
		AppDataPointer->TerminalInfoData.SensorFlashReadStatus = SENSOR_STATUS_READFLASH_OK;
		AppDataPointer->TerminalInfoData.SensorFlashStatus = Hal_getSensorFlashStatus(); //wj20200217把上面一行改成了这一行
		AppDataPointer->TerminalInfoData.SensorStatus = AppDataPointer->TerminalInfoData.SensorFlashStatus; //这里是不是写反了或者上面的应该是SensorFlashStatus？？
	}
	if(AppDataPointer->TerminalInfoData.SensorStatus != 0) {
		SensorStatus_H = 0;
		SensorStatus_L = 0;
		for(scadaIndex=1;scadaIndex<=SensorNum;scadaIndex++)  //SensorNum = 12
		{
			// sensorExistStatus = (AppDataPointer->TerminalInfoData.SensorStatus) & 0x0001;
			// AppDataPointer->TerminalInfoData.SensorStatus = (AppDataPointer->TerminalInfoData.SensorStatus) >> 1;
			// if(sensorExistStatus == 1)

			memset(dRxBuff,0x0,dRxLength);
			dRxNum=0;
			sensorExistStatus = (AppDataPointer->TerminalInfoData.SensorStatus) & 0x0800;
			AppDataPointer->TerminalInfoData.SensorStatus = (AppDataPointer->TerminalInfoData.SensorStatus) << 1;
			if(sensorExistStatus == 0x0800)
			{
				Send_485_Enable;
				hal_Delay_ms(5);
				switch(scadaIndex)
				{
					case 1:
						sensorSN = 1;
						// hal_ResetBit(SensorStatus_H, 3);
						OSBsp.Device.Usart3.WriteNData(ScadaFlow1,CMDLength);
						break;
					case 2:
						sensorSN = 2;
						// hal_ResetBit(SensorStatus_H, 2);
						OSBsp.Device.Usart3.WriteNData(ScadaFlow1,CMDLength);
						break;
					case 3:
						sensorSN = 3;
						// hal_ResetBit(SensorStatus_H, 1);
						break;
					case 4:
						sensorSN = 4;
						// hal_ResetBit(SensorStatus_H, 0);
						break;
					case 5:
						sensorSN = 5;
						// hal_ResetBit(SensorStatus_L, 7);
						break;
					case 6:
						sensorSN = 6;
						// hal_ResetBit(SensorStatus_L, 6);
						break;
					case 7:
						sensorSN = 7;
						// hal_ResetBit(SensorStatus_L, 5);
						break;
					case 8:
						sensorSN = 8;
						// hal_ResetBit(SensorStatus_L, 4);
						break;
					case 9:
						sensorSN = 9;
						// hal_ResetBit(SensorStatus_L, 3);
						break;
					case 10:
						sensorSN = 10;
						// hal_ResetBit(SensorStatus_L, 2);
						break;
					case 11:
						sensorSN = 11;
						// hal_ResetBit(SensorStatus_L, 1);
						break;
					case 12:
						sensorSN = 12;
						// hal_ResetBit(SensorStatus_L, 0);
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
				int ret = AnalyzeComand(dRxBuff,dRxNum);
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

		AppDataPointer->TerminalInfoData.SensorStatus = (uint16_t)SensorStatus_H*256 + (uint16_t)SensorStatus_L;
		if(AppDataPointer->TerminalInfoData.SensorFlashWriteStatus == SENSOR_STATUS_WRITEFLASH_NOTYET)
		{
			AppDataPointer->TerminalInfoData.SensorFlashWriteStatus = SENSOR_STATUS_WRITEFLASH_ALREADY;
			//++++测试专用+++if+++++++//
			// infor_ChargeAddrBuff[21] = 0b00000011;       //0000 0011 1100 0000     Do,氨氮，温度，ORP               //++++++
			// infor_ChargeAddrBuff[22] = 0b11000000;                                                                 //++++++
			// OSBsp.Device.InnerFlash.innerFLASHWrite(&infor_ChargeAddrBuff,(uint8_t *)(infor_ChargeAddr+0),32);     //++++++
			//++++实际专用+++else+++++++//
			if(OSBsp.Device.InnerFlash.innerFLASHRead(20,infor_ChargeAddr) == 0x01) //0x01才允许修改Flash
			{
				infor_ChargeAddrBuff[21] = SensorStatus_H;
				infor_ChargeAddrBuff[22] = SensorStatus_L;
				// OSBsp.Device.InnerFlash.innerFLASHWrite(&infor_ChargeAddrBuff,(uint8_t *)(infor_ChargeAddr+0),32);
				OSBsp.Device.InnerFlash.innerFLASHWrite(infor_ChargeAddrBuff,(uint8_t *)(infor_ChargeAddr+0),32);
			}
			AppDataPointer->TerminalInfoData.SensorFlashWriteStatusPrintf = SENSOR_STATUS_WRITEFLASH_PRINTF_ENABLE;
		}

		//根据标志位判断是否需要模拟数据
		if (AppDataPointer->TerminalInfoData.SensorFlashReadStatus == SENSOR_STATUS_READFLASH_OK)
		{
			if(OSBsp.Device.InnerFlash.innerFLASHRead(23,infor_ChargeAddr) == 0x01)
			// if(OSBsp.Device.InnerFlash.innerFLASHRead(23,infor_ChargeAddr) == 0xFF)
			{
				AppDataPointer->TerminalInfoData.SensorStatusSimulation = (AppDataPointer->TerminalInfoData.SensorStatus) ^ (AppDataPointer->TerminalInfoData.SensorFlashStatus);
				//传感器损坏，无数据才支持补发
				if(AppDataPointer->TerminalInfoData.SensorStatusSimulation != 0) {
					SimulationSensorData();
					// dRxNum=0;
				}
			}
		}

		AppDataPointer->TerminalInfoData.ReviseSimulationCode = ((uint32_t)SensorReviseStatus_H*256 + (uint32_t)SensorReviseStatus_L)<<16;
		AppDataPointer->TerminalInfoData.ReviseSimulationCode = AppDataPointer->TerminalInfoData.ReviseSimulationCode + (uint32_t)SensorSimulationStatus_H*256 + (uint32_t)SensorSimulationStatus_L;

	}else {
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

    cJSON *pJsonRoot = mymalloc(512*sizeof(cJSON *));
	cJSON *pSubJson = mymalloc(128*sizeof(cJSON *));;
	char *p;

    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        cJSON_Delete(pJsonRoot);
        return NULL;
    }

    cJSON_AddNumberToObject(pJsonRoot, "SN",DataPointer->TerminalInfoData.SerialNumber);
    cJSON_AddNumberToObject(pJsonRoot, "DeviceID",DataPointer->TerminalInfoData.DeviceID);
    cJSON_AddNumberToObject(pJsonRoot, "SeqNum",DataPointer->TransMethodData.SeqNumber);
	if(REGRST != 0 ){
		cJSON_AddNumberToObject(pJsonRoot, "reboot",REGRST);
		REGRST = 0;
	}

    pSubJson = NULL;
    pSubJson = cJSON_CreateObject();
    if(NULL == pSubJson)
    {
      //create object faild, exit
      cJSON_Delete(pSubJson);
      return NULL;
    }

	if(hal_GetBit(SensorStatus_H, 3)) {
		cJSON_AddNumberToObject(pSubJson, "Flowl",DataPointer->FlowmeterData.Depth1Vlaue);
	}
	if(hal_GetBit(SensorStatus_H, 2)) {
		cJSON_AddNumberToObject(pSubJson, "Flows",DataPointer->FlowmeterData.Speed1Vlaue);
	}
	if(hal_GetBit(SensorStatus_H, 1)) {
	}
	if(hal_GetBit(SensorStatus_H, 0)) {

	}
	if(hal_GetBit(SensorStatus_L, 7)) {
	}
	if(hal_GetBit(SensorStatus_L, 6)) {
	}
	if(hal_GetBit(SensorStatus_L, 5)) {
	}
	if(hal_GetBit(SensorStatus_L, 4)) {
	}
	if(hal_GetBit(SensorStatus_L, 3)) {
	}
	if( (hal_GetBit(SensorStatus_L, 2))&&(hal_GetBit(SensorStatus_L, 1)) )
	{
	}
	if(hal_GetBit(SensorStatus_L, 0)) {
	}
	cJSON_AddItemToObject(pJsonRoot,"FlowmeterData", pSubJson);
#if (TRANSMIT_TYPE == GPRS_Mode)
	cJSON_AddStringToObject(pJsonRoot, "CSQ",CSQBuffer);
#endif
#if (TRANSMIT_TYPE == NBIoT_BC95_Mode)
	cJSON_AddNumberToObject(pJsonRoot,"RSRP",DataPointer->TransMethodData.RSRP);
	cJSON_AddNumberToObject(pJsonRoot,"SINR",DataPointer->TransMethodData.SINR);
	cJSON_AddNumberToObject(pJsonRoot,"PCI",DataPointer->TransMethodData.PCI);
#endif
	cJSON_AddNumberToObject(pJsonRoot,"SendPeriod",DataPointer->TerminalInfoData.SendPeriod);
	cJSON_AddNumberToObject(pJsonRoot,"Quanity",DataPointer->TerminalInfoData.PowerQuantity);
	cJSON_AddNumberToObject(pJsonRoot,"Version",DataPointer->TerminalInfoData.Version);

	// uint8_t date[8];
	// char Uptime[18];
	// char filestore[18];
	// memset(date,0x0,8);
	// memset(Uptime,0x0,18);
	// memset(filestore,0x0,20);
	// OSBsp.Device.RTC.ReadExtTime(date,RealTime);
	// g_Device_RTCstring_Creat(date,Uptime);
	// g_Printf_info("Uptime:%s\r\n",Uptime);
	// cJSON_AddStringToObject(pJsonRoot, "Uptime",Uptime);
	uint8_t date[8];
	char Uptime[19] = "2019-09-01 00:00:00";
	char filestore[19];
	memset(date,0x0,8);
	memset(Uptime,0x0,19);
	memset(filestore,0x0,19);
	OSBsp.Device.RTC.ReadExtTime(date,RealTime);
	g_Device_RTCstring_Creat(date,Uptime);
	g_Printf_info("Uptime:%s\r\n",Uptime);
	cJSON_AddStringToObject(pJsonRoot,"Uptime",Uptime);
	cJSON_AddNumberToObject(pJsonRoot,"UnixTimeStamp",UnixTimeStamp);
	cJSON_AddNumberToObject(pJsonRoot,"ReSiC",DataPointer->TerminalInfoData.ReviseSimulationCode);

    p = cJSON_Print(pJsonRoot);
    if(NULL == p)
    {
      //convert json list to string faild, exit
      //because sub json pSubJson han been add to pJsonRoot, so just delete pJsonRoot, if you also delete pSubJson, it will coredump, and error is : double free
      cJSON_Delete(pJsonRoot);
      return NULL;
    }

    cJSON_Delete(pJsonRoot);
	cJSON_Delete(pSubJson);

#if HAVE_SDCARD_SERVICE
	OSBsp.Device.IOControl.PowerSet(SDCard_Power_On);
	OSTimeDly(500);
	g_SD_FileName_Creat("0:/",date,filestore);
	g_SD_File_Write(filestore,p);
	g_SD_File_Write(filestore,"\r\n");     //数据换行
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
    AppDataPointer = &(App.Data);     //DataStruct * AppDataPointer; 定义结构体类型的指针 ,只是一个名字，具体要定义指针的地址

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
	for(i=0;i<32;i++){
		infor_ChargeAddrBuff[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i,infor_ChargeAddr);
	}

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
	/**************************Version******************************************/
	App.Data.TerminalInfoData.Version = Hal_getFirmwareVersion();    //软件版本
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
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode)
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
	SensorReviseStatus_H = 0x00;      //修正
	SensorReviseStatus_L = 0x00;
	Send_Buffer[55] = 0x00;
	Send_Buffer[56] = 0x00;
	SensorSimulationStatus_H = 0x00;  //模拟
	SensorSimulationStatus_L = 0x00;
	Send_Buffer[57] = 0x00;
	Send_Buffer[58] = 0x00;
	App.Data.TerminalInfoData.ReviseSimulationCode = 0;

	AppDataPointer->FlowmeterData.Depth1Vlaue = 0.0;
	AppDataPointer->FlowmeterData.Speed1Vlaue = 0.0;

}



#endif //(PRODUCT_TYPE == g_Flowmeter_Station)

