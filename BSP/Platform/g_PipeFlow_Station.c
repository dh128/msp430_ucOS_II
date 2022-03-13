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
* Filename      : g_PipeFlow_Station.c
* Version       : V1.00
* Programmer(s) : Dingh
* Change Logs:
* Date			Author		Notes
* 2022-03-12	dingh		update sensor record
*********************************************************************************************************
*/
#include  <hal_layer_api.h>
#include  <bsp.h>

#if (PRODUCT_TYPE == PipeFlow_Station)

#define SensorNum			2
#define CMDLength        	8
#define SensorKind          0x03
#define INQPERIOD			5	/* 默认5分钟采集一次，发送周期由外部设定，5的倍数 */
//定义管道类型数据
typedef struct {
	uint8_t type;		//类型
	float width;		//宽度，单位米
	float height;		//高度，圆形管道等于width 单位米
	float high;			//传感器安装高度，单位米
}shape_t;
shape_t shape;		//定义管道
#define PI 3.14159		//圆周率

/* Sensor Exist flag */
#define Speed			0
#define Deep			1
uint8_t SensorRecord = 0;	/* 传感器记录标志，1--记录，0--不记录 */
uint16_t SensorExist = SensorKind;

AppStruct  App;
DataStruct *AppDataPointer;

uint32_t Send_Buffer[60] = {0xaa,0x00,0x00,0x01,0x01,0x00,0x00,
                            0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,0x7F,0xFF,
							0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff};
//                          /-------/ /--/ /-----------------/ /-----------------/ /-----------------/ /-------/ /-------/ /-------/ /-------/ /-------/ /-------/ /--/
//                            Period   ver		timestamp            Lng经度             lat纬度          海拔      RSRP      SINR       PCI      保留		模拟  	 保留

const uint8_t ScadaSpeed[CMDLength]	= {0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};   //瞬时流速 m/s
const uint8_t ScadaDeep[CMDLength] 	= {0x01,0x03,0x00,0x26,0x00,0x0C,0xA4,0x04};	//水温、液位、流向

#define WQ_FlowD_Num 		3
#define WQ_FlowS_Num 		3

static float  WQ_FlowD[WQ_FlowD_Num]; //申请每一次用于水深的记录，最多读WQ_FlowD_Num次
static uint8_t FlowDtimes = 0; //存储水深的临时变量
static float  WQ_FlowS[WQ_FlowS_Num]; //申请每一次用于流速的记录，最多读WQ_FlowS_Num次
static uint8_t FlowStimes = 0; //存储流速的临时变量

static uint8_t SensorStatus_H;
static uint8_t SensorStatus_L;


/*******************************************************************************
* 描述	    	: 4字节16进制转浮点数  结构体
*******************************************************************************/
Hex2Float SensorData;

/*******************************************************************************
* 函数名		: AnalyzeComand
* 描述	    	: 解析传感器指令
* 输入参数  	: data,len
* 返回参数  	: 无
*******************************************************************************/
static int AnalyzeComand(uint8_t *data,uint8_t Len)
{
	int i = 0;
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
				switch(data[2])
				{
				case 0x04:		//返回4字节，瞬时流速
					hal_SetBit(SensorStatus_L, Speed);   //传感器状态位置1
					SensorData.Hex[0] = data[4];      //MCU是小端模式，低位字节存放在低位，传感器数据0xCDAB
					SensorData.Hex[1] = data[3];
					SensorData.Hex[2] = data[6];
					SensorData.Hex[3] = data[5];

					AppDataPointer->FlowData.IFlowS = SensorData.Data;
					break;
				case 0x18:	//返回24字节，水温，液位，流向
					hal_SetBit(SensorStatus_L, Deep);   //传感器状态位置1
					/* 水温 */
					SensorData.Hex[0] = data[4];      //MCU是小端模式，低位字节存放在低位，传感器数据0xCDAB
					SensorData.Hex[1] = data[3];
					SensorData.Hex[2] = data[6];
					SensorData.Hex[3] = data[5];
					AppDataPointer->FlowData.WaterTemp = SensorData.Data;
					/* 超声液位，单位m */
					SensorData.Hex[0] = data[8];
					SensorData.Hex[1] = data[7];
					SensorData.Hex[2] = data[10];
					SensorData.Hex[3] = data[9];
					if(FlowDtimes < WQ_FlowD_Num){
						WQ_FlowD[FlowDtimes++] = SensorData.Data;
					}else{
						for(i=0; i<WQ_FlowD_Num-1; i++){
							WQ_FlowD[i] = WQ_FlowD[i+1];
						}
						WQ_FlowD[FlowDtimes] = SensorData.Data;
					}
					AppDataPointer->FlowData.IFlowL = SensorData.Data;
					/* 方向1或-1 */
					SensorData.Hex[0] = data[24];
					SensorData.Hex[1] = data[23];
					SensorData.Hex[2] = data[26];
					SensorData.Hex[3] = data[25];
					/* 流速加上方向 */
					AppDataPointer->FlowData.IFlowS *= SensorData.Data;
					if(FlowStimes < WQ_FlowD_Num){
						WQ_FlowS[FlowStimes++] = AppDataPointer->FlowData.IFlowS;
					}else{
						for(i=0; i<WQ_FlowD_Num-1; i++){
							WQ_FlowS[i] = WQ_FlowS[i+1];
						}
						WQ_FlowS[FlowStimes] = AppDataPointer->FlowData.IFlowS;
					}
					// AppDataPointer->FlowData.Directions = SensorData.Data;
					break;
				default:
						break;
				}//switch(data[0]) END
			} //(data[1]==0x03)  END
			Clear_CMD_Buffer(dRxBuff,dRxNum);
			dRxNum=0;
			Len = 0;
			return 1;
		}else{	//CRC 校验出错
			Clear_CMD_Buffer(dRxBuff,dRxNum);
			dRxNum=0;
			Len = 0;
			return -2;
		}
	}else{//CRC Len 不对
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
		SensorStatus_H = 0;
		SensorStatus_L = 0;
		for(scadaIndex=1;scadaIndex<=SensorNum;scadaIndex++)  //SensorNum = 2
		{
			memset(dRxBuff,0x0,dRxLength);
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
						OSBsp.Device.Usart3.WriteNData((uint8_t *)ScadaSpeed,CMDLength);
						break;
					case 2:
						sensorSN = 2;
						OSBsp.Device.Usart3.WriteNData((uint8_t *)ScadaDeep,CMDLength);
						break;
					default:
						break;
				}
				hal_Delay_ms(2);//高波特率降低延时为1-2ms，否则容易丢包；低波特率增加延时，如4800延时10ms，否则容易丢包
				Recive_485_Enable;
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

		/* 上电后第一次检查哪些传感器在线 */
		if (AppDataPointer->TerminalInfoData.SensorWriteStatus == SENSOR_STATUS_WRITE_NOTYET)
		{
			AppDataPointer->TerminalInfoData.SensorWriteStatus = SENSOR_STATUS_WRITE_ALREADY;
			if(SensorRecord){
				SensorExist = (uint16_t)SensorStatus_H * 256 + (uint16_t)SensorStatus_L; //本次读取到的传感器置位
			}
		}
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

	if(hal_GetBit(SensorStatus_L, Speed)) {
		cJSON_AddNumberToObject(pJsonRoot,"IFlowS",DataPointer->FlowData.IFlowS); /* 瞬时流速 */
	}
	if(hal_GetBit(SensorStatus_L, Deep)) {
		cJSON_AddNumberToObject(pJsonRoot,"WaterTemp",DataPointer->FlowData.WaterTemp);
		cJSON_AddNumberToObject(pJsonRoot,"IFlowL",DataPointer->FlowData.IFlowL);		/* 瞬时液位 */
		cJSON_AddNumberToObject(pJsonRoot,"ICCFlowVal",DataPointer->FlowData.ICCFlowVal);	/* 瞬时流量 */
		cJSON_AddNumberToObject(pJsonRoot,"ACCFlowVal",DataPointer->FlowData.ACCFlowVal);
		cJSON_AddNumberToObject(pJsonRoot,"FlowType",shape.type);
		cJSON_AddNumberToObject(pJsonRoot,"FlowHeight",shape.high);	/* 传感器安装高度 */
		if(shape.type == 1){	/* 矩形 */
			cJSON_AddNumberToObject(pJsonRoot,"FlowRWidth",shape.width);	/* 管道宽度 */
			cJSON_AddNumberToObject(pJsonRoot,"FlowRHeight",shape.height);	/* 管道高度 */
		}else if(shape.type == 0){	/* 圆形 */
			cJSON_AddNumberToObject(pJsonRoot,"FlowCDiameter",shape.width);	/* 管道直径 */
		}else{	/* 梯形暂未实现 */

		}
		/* 累积流量周期发送后清零 */
		DataPointer->FlowData.ACCFlowVal = 0;

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
	uint8_t tempBuff[7] = {0};
	Hex2Double TransferData;
	char temp[20];			//打印数据buffer

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
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+36,infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Latitude = TransferData.Data;
	for(i=0;i<8;i++){
		TransferData.Hex[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+40,infor_ChargeAddr);
	}
	App.Data.TerminalInfoData.Altitude = TransferData.Data;

	/************************管道参数信息*******************************************/
	for(i=0;i<7;i++){
		tempBuff[i] = OSBsp.Device.InnerFlash.innerFLASHRead(i+48,infor_ChargeAddr);
	}
	if(tempBuff[0] <3)	//有效数据
	{
		shape.type = tempBuff[0];
		shape.width = ((float)tempBuff[1]*256+tempBuff[2])/100;		//单位换算成m
		if(shape.width == 0)
		{
			shape.width = 1;		//不可以为0
			g_Printf_info("width set err!!!\r\n");
		}
		shape.height = ((float)tempBuff[3]*256+tempBuff[4])/100;		//单位换算成m
		shape.high = ((float)tempBuff[5]*256+tempBuff[6])/100;		//单位换算成m
		sprintf(temp,"type %d\t",shape.type);
		g_Printf_info(temp);
		sprintf(temp,"width %.2f\t",shape.width);
		g_Printf_info(temp);
		sprintf(temp,"height %.2f\t",shape.height);
		g_Printf_info(temp);
		sprintf(temp,"high %.2f\r\n",shape.high);
		g_Printf_info(temp);
	}
	else
	{
		shape.type = 1;
		shape.width = 1.0;
		shape.height = 1.0;
		shape.high = 0.1;
		g_Printf_info("no shape data!!!\r\n");
	}


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
	App.Data.FlowData.InqurePeriod = INQPERIOD; /* 单独采集周期 */
	App.Data.TerminalInfoData.SendPeriod = Hal_getTransmitPeriod();  //发送周期
	if(App.Data.TerminalInfoData.SendPeriod <= 10)
		App.Data.TerminalInfoData.SendPeriod = 10;

	SensorRecord = Hal_getSensorRecord();	/* 获取传感器记录标志 */
	g_Printf_info("SensorRecord flag = %d\r\n", (uint32_t)SensorRecord);
	/**************************Version******************************************/
	App.Data.TerminalInfoData.Version = Hal_getFirmwareVersion();    //软件版本
	Send_Buffer[34] = App.Data.TerminalInfoData.Version;
	/**************************未读取Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorReadStatus = SENSOR_STATUS_READ_NOTYET;
	/**************************未写入Flash中存储的传感器状态***********************/
	App.Data.TerminalInfoData.SensorWriteStatus = SENSOR_STATUS_WRITE_NOTYET;
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

	AppDataPointer->FlowData.ACCFlowVal = 0.0;
	AppDataPointer->FlowData.IFlowL = 0.0;
	AppDataPointer->FlowData.IFlowS = 0.0;
	AppDataPointer->FlowData.ICCFlowVal = 0.0;
	AppDataPointer->FlowData.TotalPeiod = 0;
}

/*******************************************************************************
* 函数名		: CalcFlow
* 描述	    	: 计算流量，根据传感器测试水深和流速，以及系统配置管道参数、安装高度计算流量
				  目前支持圆形和矩形
* 输入参数  	: sl--传感器测试水深，speed--传感器测试流速
* 返回参数 		: 流量
*******************************************************************************/
float CalcFlow(float sl, float speed)
{
//	float wl = 0.0;		//水高度
	float s = 0.0;		//流量
	float hu = 0.0;		//弧度
	float dis = 0.0;		//液面与圆心距离

//	wl = sl + shape.high;	//计算管道水深度

	if(shape.type==1)//矩形直接计算流量
	{
		s = sl * shape.width * speed;
	}
	else if(shape.type==0)	//圆形
	{
		if(sl>shape.width)	//水位高于圆半径
		{
			dis = sl - shape.width;		//计算液位-圆心距离
			hu = 2 * PI - 2 * acos(dis / shape.width);//计算弧度
			s = pow(shape.width, 2) * (hu / 2) + dis * shape.width * sin(hu / 2);//计算截面积
			s = s * speed;	//计算流量
		}
		else//水位低于圆半径
		{
			dis = shape.width - sl;//计算页面-圆心距离
			hu = 2*acos(dis / shape.width);//计算弧度
			s = pow(shape.width,2) * (hu / 2) - dis*shape.width * sin(hu / 2);//计算截面积
			s = s * speed;	//计算流量
		}
	}
	else	//其他
	{
		s = 0;
	}
	return s;
}

/*******************************************************************************
* 函数名		: CalcData
* 描述	    	: 处理流速，液位，流量
* 输入参数  	: 无
* 返回参数 		: 无
*******************************************************************************/
void CalcData(void)
{
	uint8_t i;
	float temp = 0.0;
	//计算深度均值
	for(i=0;i<FlowDtimes;i++)
	{
		temp += WQ_FlowD[i];
		WQ_FlowD[i] = 0;
	}
	if(FlowDtimes > 0)
	{
		AppDataPointer->FlowData.IFlowL = temp/FlowDtimes;// + shape.high;
		FlowDtimes = 0;
	}


	if(AppDataPointer->FlowData.IFlowL <= 0.001)	//判断异常值
	{
		AppDataPointer->FlowData.IFlowL = 0;
	}
	else if(AppDataPointer->FlowData.IFlowL > (shape.height-shape.high))	//超过上限，(管道高度-安装高度)
	{
		AppDataPointer->FlowData.IFlowL = shape.height;
	}
	else		//正常范围+安装高度
	{
		AppDataPointer->FlowData.IFlowL += shape.high;
	}

	// //计算流速均值
	temp = 0.0;
	for(i=0;i<FlowStimes;i++)
	{
		temp += WQ_FlowS[i];
		WQ_FlowS[i] = 0;
	}
	if(FlowStimes > 0)
	{
		AppDataPointer->FlowData.IFlowS = temp/FlowStimes;
		FlowStimes = 0;
	}

	//计算流量
	temp = CalcFlow(AppDataPointer->FlowData.IFlowL, AppDataPointer->FlowData.IFlowS);
	/* 单词采集计算瞬时流量m3/s */
	AppDataPointer->FlowData.ICCFlowVal = temp;
	/* 总流量 = 流量m3/s*时间s = m3 */
	AppDataPointer->FlowData.ACCFlowVal += temp * AppDataPointer->FlowData.InqurePeriod * 60;
}



#endif //(PRODUCT_TYPE == g_PipeFlow_Station)

