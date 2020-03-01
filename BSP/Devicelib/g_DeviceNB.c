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
* Filename      : g_DeviceGprs.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <bsp.h>

#if (TRANSMIT_TYPE == NBIoT_BC95_Mode)

static int g_has_response = 0;
static char g_response[256];

//                                          [0]   [1]  [2]  [3]  [4]  [5]  [6]  [7]  [8]  [9] [10] [11] [12] [13] [14] [15]                                                                            
uint32_t Send_Buffer_CTwing_NBSignal[16] = {0x02,0x00,0x02,0x00,0x0B,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};   //test
//                                      |数据上报|平台服务ID| 数据长度|   RSRP  |   SNR   |   PCI   | ECL |     CELL ID     

//                                            [0]   [1]  [2]  [3]  [4]  [5]  [6]  [7]  [8]  [9] [10] [11] [12]                                                                         
uint32_t Send_Buffer_CTwing_NBSoildata[13] = {0x02,0x00,0x02,0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; 
//                                        |数据上报|平台服务ID| 数据长度|     Soil_Temp     |   Soil_Humi       |                                   

//                                              [0]   [1]  [2]  [3]  [4]  [5]  [6]                                                                        
uint32_t Send_Buffer_CTwing_NBWeatherdata[7] = {0x02,0x00,0x01,0x00,0x02,0x00,0x00}; 
//                                         |数据上报|平台服务ID| 数据长度 |Temp|Humi|    

//断点续传使用
uint16_t BackupIndex = 0;
uint16_t StartFile = 1;
uint8_t FullFlag = 0;
char RespFile[10];
// char Data_Backup[70];
char Data_Backup[120];
uint8_t ResendData = 0;
uint8_t cacheBuf[7];


static unsigned char Singal_data[6]={0};
static unsigned char SINR_data[5]={0};
static unsigned char PCI_data[5]={0};
static unsigned char ECL_data[5]={0};
static unsigned char CellID_data[10]={0};

//PCP测试
char NB_Fota = 0;		//Fota状态，开始时置1，结束时置0
int PackageNum = 0;		//当前获取数据包计数，获取成功后+1
char codeFile[11]={"0:/1106.txt"};
uint32_t newVersion = 0;		//Fota 临时存储版本
/* CRC计算前把CRC校验码位清零，然后计算整个CRC结果，填充在对应位置上 */
uint32_t report[25]={0xFF,0xFE,0x01,0x13,0x00,0x00,0x00,0x11,0x00,0x30,0x30,0x32,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
							//	  版本命令  CRC      数据长度  结果码  数据002
uint32_t getBuffer[26]={0xFF,0xFE,0x01,0x15,0x00,0x00,0x00,0x12,0x31,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
								//	 版本命令  CRC      数据长度  数据版本+包计数
uint8_t reportCommand[63]="AT+NMGS=25,FFFE0113164700110056322E31300000000000000000000000\r\n";
uint8_t getCommand[65]="AT+NMGS=26,FFFE0115CFC00012310000000000000000000000000000000000\r\n";
unsigned char Receive_data[1024]={0};
uint16_t PackageSize;
uint16_t PackageLen;
uint8_t CRCFlag = 0;
static long addr_write = FOTA_ADDR_START;
unsigned long readAddr = 0;     //SPI_Flash 读写地址
// static unsigned char ECL_data=0;

//static char TimeString[20] = "20170804 16:00:00";
/*******************************************************************************
* 函数名		: NB_Config
* 描述	    	: 配置NB模组参数
* 输入参数  	: *c--需要配置的指令，m--指令等待时间（m*100ms），t--配置失败重新尝试次数
* 返回参数  	: 0--失败，1--成功
*******************************************************************************/
unsigned char NB_Config(unsigned char *c , unsigned char m, unsigned char t)
{
	unsigned char x,i=0;
	i=0;
	Clear_Buffer(aRxBuff,&aRxNum);
	while (!Hal_CheckString(aRxBuff,"OK")  & (i < t))
	{
		Clear_Buffer(aRxBuff,&aRxNum);
	 	User_Printf(c);
	 	g_Printf_dbg(c);		//debug口同步打印
		for(x=0;x<m;x++)
			OSTimeDly(20);

		i++;	
	}
	if(i>=t)
	{
		Clear_Buffer(aRxBuff,&aRxNum);
		return 0;
	}
	else 
	{
		Clear_Buffer(aRxBuff,&aRxNum);
		return 1;
	}
}
/*******************************************************************************
* 函数名    : HexStrToByte
* 描述	  	: 16进制字符串转成16进制数据
* 输入参数  : resource---目标字符串，dest---目标数组， sourceLen----字符串长度
* 返回参数  : 无
*******************************************************************************/
void HexStrToByte(unsigned char* resource, unsigned char* dest)
{
	uint16_t	i =0;
	uint8_t temp = 0;
	unsigned int stringLen = strlen(resource)/2;
	while(i < stringLen)
	{
		//提取高八位
		if(*resource >= '0' && *resource <= '9')
			temp = *resource - 0x30;
		else if(*resource >= 'a' && *resource <= 'f')
			temp = 0x0A + (*resource - 'a');
		else if(*resource >= 'A' && *resource <= 'F')
			temp = 0x0A + (*resource - 'A');
		dest[i] = temp;
		resource ++;
		//提取低八位
		if(*resource >= '0' && *resource <= '9')
			temp = *resource - 0x30;
		else if(*resource >= 'a' && *resource <= 'f')
			temp = 0x0A + (*resource - 'a');
		else if(*resource >= 'A' && *resource <= 'F')
			temp = 0x0A + (*resource - 'A');
		dest[i] = (dest[i] <<4) |temp;
		resource ++;
		i++;
	}
	dest[i] = '\0';		//结束符
}
/*******************************************************************************
* 函数名		: g_Device_NB_Restart
* 描述	    	: NB模块重启---BC35G，配置不自动连接
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_NB_Restart(void)
{
	NB_Config("AT+NCONFIG=AUTOCONNECT,FALSE\r\n",2,5); //关闭自动连接
	OSTimeDly(200);

	User_Printf("AT+NRB\r\n");
	OSTimeDly(5000);
	
	if(NB_Config("AT\r\n",5,5))
	{
		AppDataPointer->TransMethodData.NBStatus = NB_Boot;
	}
	else
	{
		AppDataPointer->TransMethodData.NBStatus = NB_Power_on;
	}
}

/*******************************************************************************
* 函数名		: g_Device_NB_Init
* 描述	    	: NB模块初始化---BC35G
* 输入参数  	: 无
* 返回参数  	: 初始化结果  0----失败；1----成功
*******************************************************************************/
char g_Device_NB_Init(void)
{
	uint8_t ii = 0;
	//ML
	unsigned char *a;
	unsigned char i=0,n=0,m=0;
	unsigned char nb_Timedata[22]={0};
	uint8_t time_buf[8];
	uint8_t time_buf_bcd[8];
	if(AppDataPointer->TransMethodData.NBStatus == NB_Boot)
	{
		NB_Config("AT+CFUN=0\r\n",5,5);
		OSTimeDly(100);
		NB_Config("ATE1\r\n",5,5);
		OSTimeDly(100);
		NB_Config("AT+NNMI=2\r\n",5,5);
		OSTimeDly(100);
		NB_Config("AT+CGSN=1\r\n",5,5);  //IMEI
		OSTimeDly(100);
		NB_Config("AT+NCDP=180.101.147.115\r\n",5,5);         //电信物联网中心平台
		// NB_Config("AT+NCDP=221.229.214.202,5683\r\n",5,5); //CTWing
		OSTimeDly(100);
		NB_Config("AT+CFUN=1\r\n",100,5);
		OSTimeDly(100);
		NB_Config("AT+CIMI\r\n",5,5);    //USIM卡IMSI号
		OSTimeDly(100);
		NB_Config("AT+CGDCONT=1,\"IP\",\"CTNB\"\r\n",5,5);
		OSTimeDly(100);
		NB_Config("AT+CGATT=1\r\n",5,5);
		OSTimeDly(100);
		Clear_Buffer(aRxBuff,&aRxNum);
		//等待入网
		while ((aRxNum<35) & (ii < 60))
		{
			// if(String_Chk(aRxBuff,"+QLWEVTIND:0") == 1 && String_Chk(aRxBuff,"+QLWEVTIND:3") ==1)
			// {
			// 	AppDataPointer->TransMethodData.NBStatus = NB_Registered;
			// 	break;
			// }
			Clear_Buffer(aRxBuff,&aRxNum);
			User_Printf("AT+CGPADDR\r\n");
			g_Printf_dbg("AT+CGPADDR\r\n");
			OSTimeDly(1000);
		//		System.Device.Usart2.WriteString(dRxBuff);
			ii++;

		}
		if(ii > 60)
		{
			// AppDataPointer->TransMethodData.NBStatus = NB_Power_on;
			g_Printf_dbg("GET IP Failed!\r\n");
		}
		else
		{
			// AppDataPointer->TransMethodData.NBStatus = NB_Get_IP;
			g_Printf_dbg("GET IP Succeed!\r\n");
			AppDataPointer->TransMethodData.NBNet = 1;
		}
		OSTimeDly(100);
		NB_Config("AT+CSQ\r\n",5,5);   //CSQ
		OSTimeDly(500);
		NB_Config("AT+NCONFIG=AUTOCONNECT,TRUE\r\n",2,5); //打开自动连接
		OSTimeDly(500);
		AppDataPointer->TransMethodData.NBStatus = NB_Init_Done;
		//********ML同步时间-20191111************//
		if(AppDataPointer->TerminalInfoData.AutomaticTimeStatus == AUTOMATIC_TIME_ENABLE)
		{
			AppDataPointer->TransMethodData.GPRSTime = 0;
			AppDataPointer->TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_DISABLE;  //禁止时间同步
		User_Printf("AT+CCLK?\r\n");
		OSTimeDly(200);			//等待串口接收
        a=strstr(aRxBuff,"+CCLK:");
	    if(a!=NULL)
		{
			while(*(a+6)!='\r')
			{
				nb_Timedata[i]=*(a+6);
				i++;
				a++;
			}	
			nb_Timedata[i]='\n';	
			
			time_buf[1]=(nb_Timedata[0]-0x30)*10+(nb_Timedata[1]-0x30)*1;	       //年
			time_buf[2]=(nb_Timedata[3]-0x30)*10+(nb_Timedata[4]-0x30)*1;	       //月
			time_buf[3]=(nb_Timedata[6]-0x30)*10+(nb_Timedata[7]-0x30)*1;	       //日
			time_buf[4]=(nb_Timedata[9]-0x30)*10+(nb_Timedata[10]-0x30)*1+8;	   //时+时区
			time_buf[5]=(nb_Timedata[12]-0x30)*10+(nb_Timedata[13]-0x30)*1;	       //分
			time_buf[6]=(nb_Timedata[15]-0x30)*10+(nb_Timedata[16]-0x30)*1;	       //秒
			for(m=1;m<7;m++) {
				time_buf_bcd[m]= HexToBCD(time_buf[m]);    //存“年月日时分秒”
			}
			OSBsp.Device.RTC.ConfigExtTime(time_buf_bcd,RealTime);
				AppDataPointer->TransMethodData.GPRSTime = 1;
			g_Printf_dbg("NB Automatic Time OK\r\n");
		}
		}
		//*****************同步时间END************//
		return 1;
	}
	return 0;
}
/*******************************************************************************
* 函数名		: g_Device_NB_GetIP
* 描述	    	: NB模块入网  BC35G
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_NB_GetIP(void)
{
//	uint8_t ii = 0;
	//等待注册成功信息
	//+QLWEVTIND:0
	//+QLWEVTIND:3 
	
	AppDataPointer->TransMethodData.NBStatus = NB_Registered;
}
/*******************************************************************************
* 函数名		: g_Device_NB_Send
* 描述	    	: NB上报数据Hex数组
* 输入参数  	: *data--准备上报内容，length--数据长度
* 返回参数  	: code
*******************************************************************************/
void g_Device_NB_Send(uint32_t *data ,uint8_t length)
{
	static unsigned char ii = 0;
	char buff[15];
	AppDataPointer->TransMethodData.NBSendStatus = 0;
	Clear_Buffer(aRxBuff,&aRxNum);
	sprintf(buff,"AT+NMGS=%d,",length);
	User_Printf(buff);
    // User_Printf("AT+NMGS=34,");
    for(ii=0;ii<length;ii++)
    {
        if(data[ii] < 0x10)       //Printf下小于16进行补0打印
        {
        	User_Printf("0%X",data[ii] & 0xff);			// & 0xff防止出现数据溢出
        }
        else
        {
        	User_Printf("%X",data[ii] & 0xff);			// & 0xff防止出现数据溢出
        }
    }
    User_Printf("\r\n");
	OSTimeDly(200);
}
/*******************************************************************************
* 函数名		: g_Device_NB_Send_Str
* 描述	    	: NB上报数据Hex数组
* 输入参数  	: *data--准备上报内容，length--数据长度
* 返回参数  	: code
*******************************************************************************/
void g_Device_NB_Send_Str(char *data ,uint8_t length)
{
//	static unsigned char ii = 0;
	char buff[15];
	AppDataPointer->TransMethodData.NBSendStatus = 0;
	Clear_Buffer(aRxBuff,&aRxNum);
	sprintf(buff,"AT+NMGS=%d,",length);
	User_Printf(buff);
	User_Printf(data);
    User_Printf("\r\n");
	OSTimeDly(200);
}
/*******************************************************************************
* 函数名		: g_Device_NBSignal
* 描述	    	: NB查看信号质量
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_NBSignal(void)
{
	unsigned char *a;
	unsigned char i=0,n=0;
	int32_t dataTemp=0;
	int32_t dataTemp2=0;
	static uint16_t complement=0;
	static uint8_t Singal_Less=0;
	Clear_Buffer(aRxBuff,&aRxNum);

	User_Printf("AT+NUESTATS\r\n");
	OSTimeDly(200);			//等待串口接收
	
	//处理信号值，若多次无信号，则复位NB，重新入网
	//get the RSRP
	a=strstr(aRxBuff,"Signal power:");//用于判断字符串str2是否是str1的子串。如果是，则该函数返回str2在str1中首次出现的地址；否则，返回NULL。
	if(a!=NULL)
	{
		while(*(a+13)!='\r')
		{
			Singal_data[i]=*(a+13);
			i++;
			a++;
		}
		Singal_data[i]='\n';
		dataTemp=0;
		for(n=1;n<i;n++)
		{
			dataTemp=dataTemp*10 + Singal_data[n]-0x30;
		}
		/*CTwing发送数组*/
		dataTemp2 = (int32_t)((float)dataTemp/10);
		complement = 0XFFFF - dataTemp2 + 0X01;
		Send_Buffer_CTwing_NBSignal[5] = complement/256;
		Send_Buffer_CTwing_NBSignal[6] = complement%256;
		/**********************************************/
		dataTemp=0-dataTemp;
		AppDataPointer->TransMethodData.RSRP = (float)dataTemp/10;
		Send_Buffer[49] = (dataTemp & 0xFF00) >> 8;
		Send_Buffer[50] = dataTemp & 0xFF;
	}
	if(dataTemp==-32768)		//无信号
	{
		Singal_Less++;
	}
	if(Singal_Less>=2)
	{
		Singal_Less=0;
		AppDataPointer->TransMethodData.NBNet = 0;
		AppDataPointer->TransMethodData.NBStatus = NB_Power_on;
		g_Device_NB_Init();
	}
	//get the SINR
	i=0;
	dataTemp = 0;
	a=strstr(aRxBuff,"SNR:");
	if(a!=NULL)
	{
		while(*(a+4)!='\r')
		{
			SINR_data[i]=*(a+4);
			i++;
			a++;
		}
		SINR_data[i]='\n';
		dataTemp=0;
		if(SINR_data[0]=='-')
		{
			for(n=1;n<i;n++)
			{
				dataTemp=dataTemp*10 + SINR_data[n]-0x30;
			}
			/*CTwing发送数组*/
			dataTemp2 = (int32_t)((float)dataTemp/10);
			complement = 0XFFFF - dataTemp2 + 0X01;
			Send_Buffer_CTwing_NBSignal[7] = complement/256;
			Send_Buffer_CTwing_NBSignal[8] = complement%256;
			/**********************************************/
			complement = 0XFFFF - dataTemp + 0X01;
			dataTemp=0-dataTemp;
			AppDataPointer->TransMethodData.SINR = (float)dataTemp/10;
			Send_Buffer[51] = (complement & 0xFF00) >> 8;
			Send_Buffer[52] = complement & 0xFF;
		}
		else
		{
			for(n=0;n<i;n++)
			{
				dataTemp=dataTemp*10 + SINR_data[n]-0x30;
			}
			/*CTwing发送数组*/
			dataTemp2 = (int32_t)((float)dataTemp/10);
			Send_Buffer_CTwing_NBSignal[7] = dataTemp2/256;
			Send_Buffer_CTwing_NBSignal[8] = dataTemp2%256;
			/**********************************************/
		AppDataPointer->TransMethodData.SINR = (float)dataTemp/10;
		Send_Buffer[51] = (dataTemp & 0xFF00) >> 8;
		Send_Buffer[52] = dataTemp & 0xFF;
		}
	}
		//get the PCI
		i=0;
		dataTemp = 0;
		a=strstr(aRxBuff,"PCI:");
		if(a!=NULL)
		{
			while(*(a+4)!='\r')
			{
				PCI_data[i]=*(a+4);
				i++;
				a++;
			}
			PCI_data[i]='\n';
			dataTemp=0;
			for(n=0;n<i;n++)
			{
				dataTemp=dataTemp*10 + PCI_data[n]-0x30;
			}
		/*CTwing发送数组*/
		Send_Buffer_CTwing_NBSignal[9] = dataTemp/256;
		Send_Buffer_CTwing_NBSignal[10] = dataTemp%256;
		/**********************************************/
			AppDataPointer->TransMethodData.PCI = dataTemp;
			Send_Buffer[53] = (dataTemp & 0xFF00) >> 8;
			Send_Buffer[54] = dataTemp & 0xFF;
		}
	//get the ECL
	i=0;
	dataTemp = 0;
	a=strstr(aRxBuff,"ECL:");
	if(a!=NULL)
	{
		while(*(a+4)!='\r')
		{
			ECL_data[i]=*(a+4);
			i++;
			a++;
		}
		ECL_data[i]='\n';
		dataTemp=0;
		for(n=0;n<i;n++)
		{
			dataTemp=dataTemp*10 + ECL_data[n]-0x30;
		}
		/*CTwing发送数组*/
		Send_Buffer_CTwing_NBSignal[11] = dataTemp;
		/**********************************************/
		AppDataPointer->TransMethodData.ECL = dataTemp;
	}
	//get the Cell ID
	i=0;
	dataTemp = 0;
	a=strstr(aRxBuff,"Cell ID:");
	if(a!=NULL)
	{
		while(*(a+8)!='\r')
		{
			CellID_data[i]=*(a+8);
			i++;
			a++;
		}
		CellID_data[i]='\n';
		dataTemp=0;
		for(n=0;n<i;n++)
		{
			dataTemp=dataTemp*10 + CellID_data[n]-0x30;
		}
		/*CTwing发送数组*/
		Send_Buffer_CTwing_NBSignal[12] = (dataTemp & 0xFF000000) >> 24;
		Send_Buffer_CTwing_NBSignal[13] = (dataTemp & 0x00FF0000) >> 16;
		Send_Buffer_CTwing_NBSignal[14] = (dataTemp & 0x0000FF00) >> 8;
		Send_Buffer_CTwing_NBSignal[15] = dataTemp & 0x000000FF;
		/**********************************************/
		AppDataPointer->TransMethodData.CELLID = dataTemp;
	}
}
/*******************************************************************************
* 函数名    : ProcessCommand
* 描述	  	: 处理Command指令数据,下发指令
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void ProcessCommand()
{
	uint8_t *CommandBuff;
	uint8_t CommandBuffData[50];
	uint8_t CommandBuffNum;

	uint16_t Temp_SendPeriod;
	unsigned char Flash_Tmp[14];  //flash操作中间变量
	unsigned char TimebuffNum=0;
	unsigned char TimeBuff_Hex[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //16进制的时间Buffer  2018年3月15号 20时50分00秒 星期4

	if(Hal_CheckString(aRxBuff,"FF0102")) //修改上报周期
	{
		CommandBuff = strstr(aRxBuff,"FF0102");         //判断接收到的数据是否有效
		while(*(CommandBuff+6) != 0x0A)
		{
			CommandBuffData[CommandBuffNum] = *(CommandBuff+6);
			CommandBuffNum++;
			CommandBuff++;
		}
		Temp_SendPeriod = (CommandBuffData[0]-0x30)*1000 + (CommandBuffData[1]-0x30)*100
							+ (CommandBuffData[2]-0x30)*10 + (CommandBuffData[3]-0x30)*1;
		if( (Temp_SendPeriod >= 5) && (Temp_SendPeriod <= 240) )
		{
			App.Data.TerminalInfoData.SendPeriod = (unsigned char)(Temp_SendPeriod & 0x00FF);
			g_Printf_info("NB Set SendPeriod OK\r\n");
			//将发送周期的信息存入Flash
			// delay_ms(10);
			OSTimeDly(5);
			Flash_Tmp[11] = App.Data.TerminalInfoData.SendPeriod;//上传周期（min）
			OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp[11], 1, infor_ChargeAddr, 11);//把终端信息写入FLASH
		}
		else
		{
			g_Printf_info("NB Set SendPeriod Failed！\r\n");
		}
	}
	else if(Hal_CheckString(aRxBuff,"FF0208")) //同步设备时间
	{
		// System.Device.Usart2.WriteString("Time Set Done!\r\n");
		CommandBuff = strstr(aRxBuff,"FF0208");         //判断接收到的数据是否有效
		while(*(CommandBuff+6) != 0x0A)
		{
			CommandBuffData[CommandBuffNum] = *(CommandBuff+6);
			CommandBuffNum++;
			CommandBuff++;
		}

		for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++)
		{
			TimeBuff_Hex[TimebuffNum] = ((CommandBuffData[TimebuffNum*2]-0x30)<<4) + (CommandBuffData[TimebuffNum*2+1]-0x30);
		}
		if( (TimeBuff_Hex[0]==0x20) && (TimeBuff_Hex[1]>=0x18) && (TimeBuff_Hex[2]>=1) && (TimeBuff_Hex[2]<=0x12) && (TimeBuff_Hex[3]>=1) && (TimeBuff_Hex[3]<=0x31)
			&& (TimeBuff_Hex[4]<0x24)  && (TimeBuff_Hex[5]<0x60) && (TimeBuff_Hex[6]<0x60) && (TimeBuff_Hex[7]>=1) && (TimeBuff_Hex[7]<=0x7) )
		{
			OSBsp.Device.RTC.ConfigExtTime(TimeBuff_Hex,0);   //写入时间
			g_Printf_info("NB Time Set Done!\r\n");
		}
		else
		{
			g_Printf_info("NB Time Set Failed!\r\n");
		}

	}
	else if(Hal_CheckString(aRxBuff,"FF0301")) //复位设备
	{
		g_Printf_info("NB Reset Device OK!\r\n");
		OSTimeDly(500);
		hal_Reboot(); //******软件复位*******//
	}
	else
	{
		g_Printf_info("Unknown command\r\n");
	}
}
/*******************************************************************************
* 函数名    : GetCode
* 描述	  	: 获取PCP升级包
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void GetCode(int num)
{
	uint16_t temp1=0;	
	uint16_t ii = 0;
	
	if(num < PackageLen)	//分包获取数据包
	{
		//CRC
		getBuffer[24] =num/256;
		getBuffer[25] =num%256;
		getBuffer[4] = getBuffer[5] = 0;
		temp1 = CRC16CCITT(getBuffer,26);
		getBuffer[4] =temp1/256;
		getBuffer[5] =temp1%256;
		//Send
		Hex2Str(getCommand,getBuffer,26,11);
		for(ii=0;ii<65;ii++)
		{
			g_Device_SendByte_Uart0(getCommand[ii]);		//获取数据指令
			// delay_ms(10);
		}
		
	}
	else		//上报下载完成
	{
		User_Printf("AT+NMGS=9,FFFE0116850E000100\r\n");
	}
//	System.Device.Timer.Start(5,TimerSystick,200,Get_Data);
}
/*******************************************************************************
* 函数名    : ProcessPCP
* 描述	  	: 处理PCP指令数据,软件升级
* 输入参数  : 下行数据包
* 返回参数  : 无
*******************************************************************************/
void ProcessPCP(unsigned char *p)
{
	uint8_t PCPData[512];
	uint16_t CRCtemp;
//	uint16_t temp2=0;
	uint16_t ret = 0;	
//	uint16_t bai,shi,ge;
	long ii;
	long m;
	long length;
	uint8_t TestData[3]={0};
	uint8_t Flash_Tmp[3];					//flash操作中间变量
//	char d_t[530000];
	HexStrToByte(p,PCPData);
	switch (PCPData[3])
	{
	case 0x13:			//回复查询版本指令
		g_Printf_info("get 0x13 command\r\n");
		//version 获取
		report[9]=App.Data.TerminalInfoData.Version/100 +0x30;
		report[10]=App.Data.TerminalInfoData.Version%100/10 +0x30;
//		report[11]=App.Data.TerminalInfoData.Version%10 +0x30;
		report[11]=(App.Data.TerminalInfoData.Version-1)%10 +0x30;	//测试用，否则固件版本相同
		//CRC
		report[3]= 0x13;
		report[4] = report[5] = 0;
		CRCtemp = CRC16CCITT(report,25);
		report[4] =CRCtemp/256;
		report[5] =CRCtemp%256;
		//Send
		Hex2Str(reportCommand,report,25,11);
		for(ii=0;ii<63;ii++)
		{
			g_Device_SendByte_Uart0(reportCommand[ii]);		//获取数据指令
			// delay_ms(10);
		}
		//User_Printf("AT+NMGS=25,FFFE0113164700110056322E31300000000000000000000000\r\n");
		OSTimeDly(5000);
		g_Printf_info(" 0x13 delay over\r\n");
		break;
	case 0x14:			//新版本通知
		g_Printf_info("get 0x14 command\r\n");
		//提取版本号
		newVersion = (PCPData[8]-0x30)*100 + (PCPData[9]-0x30)*10 + PCPData[10] - 0x30;
		getBuffer[8] = PCPData[8];
		getBuffer[9] = PCPData[9];
		getBuffer[10] = PCPData[10];
		//获取没报长度
		PackageSize = PCPData[24];
		PackageSize = PackageSize*256 + PCPData[25];
		//获取总包数
		PackageLen = PCPData[26];
		PackageLen = PackageLen*256 + PCPData[27];
		//擦除SPI
		W25Q16_CS_HIGH();
		OSTimeDly(50);
		Base_3V3_ON;
		// P4OUT |= BIT0;
		W25Q16_Init();
		readAddr = FOTA_ADDR_START;
		for (ii=0;ii<6;ii++)
		{
			SPI_Flash_Erase_Block(readAddr);
			readAddr += 0x10000;
		}
		g_Printf_info("Erase SPI_Flash\r\n");
		User_Printf("AT+NMGS=9,FFFE0114D768000100\r\n");		//允许升级
		// System.Device.Timer.Stop(5);
		// UpdateFlag = 1;
		OSTimeDly(100);
		NB_Fota = 1;
//		User_Printf("AT+NMGS=26,FFFE0115CFC00012310000000000000000000000000000000000\r\n");
		GetCode(PackageNum);
		//System.Device.Timer.Start(6,TimerSystick,20,GetCode);
	break;
	case 0x15:			//请求升级包，不用
	//	byte[9],byte[10]为数据包数，应该等于PackageNum
		ret = PCPData[9]*256 + PCPData[10];
		length = strlen(p)/2-11;		//去除包头FFFE等11个字节数据
		CRCtemp = PCPData[4]*256+PCPData[5];
		PCPData[4] = PCPData[5] = 0;
		if(CRCtemp == CRC16CCITT_Byte(PCPData,length+11))
			CRCFlag = 1;
		else
			CRCFlag = 0;
		// CRC校验通过、设备包正确则存储继续获取否则重新获取
		if((CRCFlag == 1) && (ret == PackageNum))	
		{
			PackageNum ++;
			for(m=0;m<length;m++)
			{
				SPI_Flash_Write_Data(PCPData[m+11],addr_write++);
			}
			aRxNum = 0;
			User_Printf("AT+NMGR\r\n");		//防止出现重复缓存
			OSTimeDly(100);
		}
		else
		{
			g_Printf_info("CRCFlah= %d,packGet= %d, PackageNum= %d\r\n",(uint32_t)CRCFlag,(uint32_t)ret,(uint32_t)PackageNum);
		}
		GetCode(PackageNum);
	break;
	case 0x016:			//上报下载情况
		
	break;
	case 0x017:			//执行升级		重启后
		g_Printf_info("get 0x17 command\r\n");
		PackageNum = 0;		//结束下载，清零计数
		User_Printf("AT+NMGS=9,FFFE0117B725000100\r\n");
		OSTimeDly(500);		//上报升级成功
		//CRC
		report[3]= 0x18;
		report[4] = report[5] = 0;
		CRCtemp = CRC16CCITT(report,25);
		report[4] =CRCtemp/256;
		report[5] =CRCtemp%256;
		//Send
		Hex2Str(reportCommand,report,25,11);
		for(ii=0;ii<63;ii++)
		{
			g_Device_SendByte_Uart0(reportCommand[ii]);		//获取数据指令
			// delay_ms(10);
		}
	//	User_Printf("AT+NMGS=25,FFFE0118835A00110031000000000000000000000000000000\r\n");
		// UpdateFlag = 0;
	break;
	case 0x18:			//上报升级结果	重启后
		g_Printf_info("get 0x18 command\r\n");
		NB_Fota = 0;
		g_Printf_info("%d version code printf begin:\r\n",newVersion);
		// add_temp = FOTA_ADDR_START;
		// lenth = 
		for(m=FOTA_ADDR_START;m<addr_write;m++)
		{
			// d_t[m]=Read_Byte(m);
			// OSBsp.Device.Usart2.WriteData(d_t[m]);
			OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(m));
		}	

		//判断存储数据头尾是否正确 然后配置启动标志位存放于infor_BootAddr
		TestData[0] = SPI_Flash_ReadByte(addr_write-3);
		TestData[1] = SPI_Flash_ReadByte(FOTA_ADDR_START+1);
		if(TestData[0] == 'q' && TestData[1] == 'c')	//确认@c400和q\r\n,存储结束后addr_writer值为\n后面一位
		{	        
			g_Printf_info("Enter %s and System will goto bootloader\r\n",__func__);
			loop8:
				Flash_Tmp[0] = 0x02; 		//置位Flash 标志位	//把infor_BootAddr写0x02，建立FOTA升级标志位
				Flash_Tmp[1] = (uint8_t)newVersion;
				OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp, 2, infor_BootAddr, 0);
				hal_Delay_ms(10);
				if(OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_BootAddr) == 0x02 && OSBsp.Device.InnerFlash.innerFLASHRead(1, infor_BootAddr) == newVersion)
					hal_Reboot();			//重启MCU
				else
					goto loop8;	
		}
		else
		{
			g_Printf_info("Error code!\r\n");
		}
		
	break;
	default:
		g_Printf_info("Error pcp data\r\n");
		break;
	}
}
/*******************************************************************************
* 函数名		: g_Device_NB_SendCheck
* 描述	    	: NB发送确认
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_NB_SendCheck(void)
{
	if(Hal_CheckString(aRxBuff,"OK"))
	{
		// AppDataPointer->TransMethodData.NBStatus = NB_Send_Done;
		AppDataPointer->TransMethodData.NBSendStatus = 1;
		g_Printf_info("NB send data ok\r\n");
		Clear_Buffer(aRxBuff,&aRxNum);
	}
	// AppDataPointer->TransMethodData.NBStatus = NB_Idel;
}
/*******************************************************************************
* 函数名		: g_Device_NB_GetReceive
* 描述	    	: NB查询接收数据
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_NB_GetReceive(void)
{
	uint8_t *Uart0_RxBuff;
//	uint8_t Uart0_RxBuff_data[50];
//	uint8_t Uart0_RxBuff_Num;
	uint16_t i = 0;
	//Uart0_RxBuff_Num = 0;

	// Clear_Buffer(aRxBuff,&aRxNum);
	Clear_CMD_Buffer(aRxBuff,1050);
	aRxNum = 0;
	// OSTimeDly(4000);		//等待4000ms		使用下面的循环判断延时，待测试
	if(NB_Fota)				//进入Fota后每次获取PCP数据延时，避免数据下发不及时
	{
		while(i < 40)
		{
			i++;
			OSTimeDly(250);		//500ms
			if(Hal_CheckString(aRxBuff,"+NNMI"))
				break;			
			OSTimeDly(250);		//500ms
		}
	}
	
	User_Printf("AT+NMGR\r\n");
	OSTimeDly(1000);		//等待2000ms
	if(Hal_CheckString(aRxBuff,",FFFE"))							//获取到PCP消息
	{
		//提取PCP消息
		g_Printf_info("Get PCP data!\r\n");
		Uart0_RxBuff = strstr(aRxBuff,",FFFE");         //判断接收到的数据是否有效
		i = 0;		//清零用于存储下行数据包
		while(*(Uart0_RxBuff+1)!='\r')
		{
			Receive_data[i]=*(Uart0_RxBuff+1);
			i++;
			Uart0_RxBuff++;
		}
		Receive_data[i] = '\0';
		// Uart0_RxBuff = strstr(aRxBuff,"FFFE");         //判断接收到的数据是否有效
		// while(*(Uart0_RxBuff) != '\r')
		// {
		// 	Uart0_RxBuff_data[Uart0_RxBuff_Num] = *Uart0_RxBuff;
		// 	Uart0_RxBuff_Num++;
		// 	Uart0_RxBuff++;
		// }
		OSTimeDly(100);
		//处理PCP消息
		ProcessPCP(Receive_data);
	}
	else if(Hal_CheckString(aRxBuff,"FF") & Hal_CheckString(aRxBuff,"AA"))  //获取到用户下发指令
	{
		g_Printf_info("Get Command data!\r\n");
		ProcessCommand();
		aRxNum = 0;
	}
	else														//无下发数据
	{
		//配置NB状态位Idle态
		AppDataPointer->TransMethodData.NBStatus = NB_Idel;
		g_Printf_info("No (correct) message downloaded!\r\n");
		Clear_Buffer(aRxBuff,&aRxNum);
	}

	
}
/*******************************************************************************
* 函数名  : CreatFileNum
* 描述	  : 生成存储文件名称
* 输入参数  : x	1——序号加；0——序号减
* 返回参数  : 无
*******************************************************************************/
void CreatFileNum(char x)
{
  if(x ==1)	//序号加
  {
    if(FullFlag == 0)
    {
		BackupIndex++;
		if(BackupIndex > MaxLength)
		{
			BackupIndex = 1;
			StartFile = BackupIndex + 1;
			FullFlag = 1;
		}
    }
    else
    {
		BackupIndex ++;
		if(BackupIndex > MaxLength)
		{
			BackupIndex = 1;
		}
		StartFile = BackupIndex + 1;
		if(StartFile > MaxLength)
		{
			StartFile = 1;
		}
    }
  }
  else			//序号减
  {
      if(FullFlag == 1)
      {
		BackupIndex--;
		if(BackupIndex == 0)
		{
			BackupIndex = MaxLength;
			FullFlag = 0;
		}
      }
      else
      {
//	  if(BackupIndex >= StartFile)
	  {
	      if(BackupIndex == StartFile)
	      {
			BackupIndex = 0;
			StartFile = 1;
	      }
	      else	//BackupIndex ！= StartFile 只能是大于关系
	      {
		  	BackupIndex --;
	      }
	  }
      }
  }
}
/*******************************************************************************
* 函数名  : WriteStoreData
* 描述	  : 发送失败时存储发送数据
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void WriteStoreData(void)
{
//	uint8_t tempBuffer[20];
	
	CreatFileNum(1);		//参数1   BackupIndex++;
	ltoa( (long)BackupIndex , RespFile);
	strcat(RespFile , ".txt");
	if(FullFlag)		//循环时需要先删除原来文件
	{
		del_txt("0:/INDEX",RespFile);
		// sprintf(tempBuffer,"Del file:%s\r\n",RespFile);
		// g_Printf_dbg(tempBuffer);
	}
	if(Write_ToDirTxt("0:/INDEX",RespFile,Data_Backup))			//临时存储，用于补发数据,需要存储成功
	{
		cacheBuf[0] = BackupIndex/256;
		cacheBuf[1] = BackupIndex%256;
		cacheBuf[2] = StartFile/256;
		cacheBuf[3] = StartFile%256;
		cacheBuf[4] = FullFlag%256;
		OSBsp.Device.InnerFlash.FlashRsvWrite(cacheBuf,5,infor_ChargeAddr,18);	//存储成功后保存文件名序号至Flash
		g_Printf_dbg(Data_Backup);
		g_Printf_dbg("\r\n");
		g_Printf_dbg(RespFile);
		g_Printf_dbg(" Write backup to SD\r\n");
	}
	else			//若存储失败则退回原来序号
	{
		if(FullFlag==0)
		{
			if(BackupIndex >= 1)
				BackupIndex --;
		}
		else
		{
			if(BackupIndex == 1)
				BackupIndex = MaxLength;	//超过一轮需转回最大值
			else
				BackupIndex --;
		}

	}
}
/*******************************************************************************
* 函数名  : GetStoreData
* 描述	  : 获取发送失败时存储的发送数据
* 输入参数  : 无
* 返回参数  : 无
*******************************************************************************/
void GetStoreData(void)
{
	uint8_t temp = 0;
	while(BackupIndex >=1)
	{
		ltoa( (long)BackupIndex , RespFile);
		strcat(RespFile , ".txt");
		temp = Get_String("0:/INDEX" , RespFile , Data_Backup , 70);
		if( temp == 1)		
		{
			// BackupIndex--;
			ResendData = 1;		//补发数据标志位
			Data_Backup[68] = '\0';
			AppDataPointer->TransMethodData.NBStatus = NB_Init_Done;
			break;		//退出循环，准备发送数据
		}
		else			//没取到字符串
		{
			CreatFileNum(0);		//参数0   BackupIndex--;
		}
	}
	
}
/*******************************************************************************
* 函数名  : TransmitTaskStart
* 描述	  : 发送任务函数
* 输入参数  : *p_arg
* 返回参数  : 无
*******************************************************************************/
void  TransmitTaskStart (void *p_arg)
{
    (void)p_arg;   
    OSTimeDlyHMSM(0u, 0u, 0u, 100u);      
    g_Printf_info("%s ... ...\n",__func__);           
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.*/
        if(Hal_getCurrent_work_Mode() == 0){
            if(AppDataPointer->TransMethodData.NBStatus == NB_Power_off)
			{
				//NB-IoT 第一次开机时对NB上电操作，后续进入低功耗不关电
				g_Printf_dbg("Turn on NB power\r\n");
				g_Printf_info("\r\n\r\nNB-IoT Fota Test version 3\r\n\r\n");	//测试打印
				OSTimeDly(500);
                OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);		//打开NB电源
				//reset脚电平
                OSTimeDly(5000);
                // OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
                AppDataPointer->TransMethodData.NBStatus = NB_Power_on;
				g_Printf_dbg("NB power on\r\n");
                
            }
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Power_on)
			{
				g_Device_NB_Restart();
            }
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Boot)
			{
                g_Device_NB_Init();
            }
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Init_Done)
			{
                if( AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER)
				{
					g_Printf_info("Scan over,data uploading\r\n");
					if(ResendData == 0)		//正常上报数据,需要累加SeqNum,采集电压，本地存储
					{
						char *data = Hal_Malloc(512*sizeof(char *));
						char response[128];
						//SeqNumber ++
						if(App.Data.TerminalInfoData.DeviceFirstRunStatus == DEVICE_STATUS_FIRSTRUN_BEGIN) {
							App.Data.TerminalInfoData.DeviceFirstRunStatus = DEVICE_STATUS_FIRSTRUN_OVER;
							App.Data.TransMethodData.SeqNumber = 0;
						}else {
							App.Data.TransMethodData.SeqNumber++;
							if(AppDataPointer->TransMethodData.SeqNumber >= 65535)
								AppDataPointer->TransMethodData.SeqNumber = 1;
						}
						Send_Buffer[5] = AppDataPointer->TransMethodData.SeqNumber/256;
						Send_Buffer[6] = AppDataPointer->TransMethodData.SeqNumber%256;
						//Voltage
						GetADCValue();
						//检查信号质量
						g_Device_NBSignal();
						data = MakeJsonBodyData(AppDataPointer);		//组包json并存储SD卡
						g_Printf_info("data:%s\r\n",data);
						memset(response,0x0,128);
						Hex2Str(Data_Backup,Send_Buffer,60,0);					
						g_Printf_info("Hexdata:%s\r\n",Data_Backup);    //打印输出16进制发送数据
					}
					//发送数据
					if(AppDataPointer->TransMethodData.NBNet == 1)
					{
						if(ResendData == 1)		//补发数据
						{
							g_Device_NB_Send_Str(Data_Backup,120);
							// ResendData = 0;
						}
						else					//正常上报数据
						{
							g_Device_NB_Send(Send_Buffer,60);
							//上报CTwing
							// g_Device_NB_Send(Send_Buffer_CTwing_NBSignal,16);
							// g_Device_NB_Send(Send_Buffer_CTwing_NBSoildata,13);		
							// g_Device_NB_Send(Send_Buffer_CTwing_NBWeatherdata,7);							
						}
						
						OSTimeDly(2500);	//等待5s
						g_Device_NB_SendCheck();
						if(AppDataPointer->TransMethodData.NBSendStatus == 1)	//确认帧发送成功,发送数据前会置0
						{
							if(ResendData)
							{
								ResendData = 0;
								del_txt("0:/INDEX",RespFile);				//删除临时存储，同时更改存储BackupIndex值
								CreatFileNum(0);		//参数0   BackupIndex++;
								cacheBuf[0] = BackupIndex/256;
								cacheBuf[1] = BackupIndex%256;
								cacheBuf[2] = StartFile/256;
								cacheBuf[3] = StartFile%256;
								cacheBuf[4] = FullFlag;
								OSBsp.Device.InnerFlash.FlashRsvWrite(cacheBuf,5,infor_ChargeAddr,18);
							}	
							if(BackupIndex >=1)
								GetStoreData();
							else
								AppDataPointer->TransMethodData.NBStatus = NB_Send_Over;

							// if(NB_Fota)		//NB_Fota中断时，重新获取
							// {
							// 	GetCode(PackageNum);
							// }
						}
						else
						{
							WriteStoreData();
							AppDataPointer->TransMethodData.NBStatus = NB_Idel;			//发送失败进Idles
						}
					}
					else
					{
						//存储数据等待下次补传
						WriteStoreData();
						AppDataPointer->TransMethodData.NBStatus = NB_Send_Over;
					}      
                }    
            }
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Send_Over)		//不在线直接进Idle
			{
				g_Device_NB_GetReceive();
			}
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Idel)
			{
                // OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
                Hal_EnterLowPower_Mode();
            }
            OSTimeDlyHMSM(0u, 0u, 0u, 200u);  
        }
    }
}

#endif //(TRANSMIT_TYPE == NBIoT_BC95_Mode)

