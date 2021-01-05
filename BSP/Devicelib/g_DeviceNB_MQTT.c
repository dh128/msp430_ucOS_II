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

#if (TRANSMIT_TYPE == NBIoT_MQTT_Ali)


//断点续传使用
uint16_t BackupIndex = 0;
uint16_t StartFile = 1;
uint8_t FullFlag = 0;
char RespFile[10];
// char Data_Backup[70];
char Data_Backup[122];
uint8_t ResendData = 0;
uint8_t cacheBuf[7];

//NB信号使用
static unsigned char Singal_data[6]={0};
static unsigned char SINR_data[5]={0};
static unsigned char PCI_data[5]={0};
static unsigned char ECL_data[5]={0};
static unsigned char CellID_data[10]={0};


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
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	while (!Hal_CheckString(aRxBuff,"OK")  & (i < t))
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	 	User_Printf((char *)c);
	 	//g_Printf_dbg((char *)c);		//debug口同步打印
		for(x=0;x<m;x++)
			OSTimeDly(50);

		i++;	
	}
	if(i>=t)
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		return 0;
	}
	else 
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		return 1;
	}
}

/*******************************************************************************
* 函数名		: NB_MQTT_Config
* 描述	    	: 配置NB模组MQTT参数
* 输入参数  	: *c--需要配置的指令，m--指令等待时间（m*100ms），t--配置失败重新尝试次数，r--等待指令
* 返回参数  	: 0--失败，1--成功
*******************************************************************************/
unsigned char NB_MQTT_Config(unsigned char *c , unsigned char m, unsigned char t, unsigned char *r)
{
	unsigned char x,i=0;
	i=0;
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	while (!Hal_CheckString(aRxBuff,r)  & (i < t))
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	 	User_Printf(c);
	 	//g_Printf_dbg(c);		//debug口同步打印
		for(x=0;x<m;x++)
			OSTimeDly(500);

		i++;	
	}
	if(i>=t)
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		return 0;
	}
	else 
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
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
	NB_Config("AT+QREGSWT=2\r\n",2,5); //禁用IoT平台注册功能
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
* 函数名		: SyncTime
* 描述	    	: NB网络校时
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void SyncTime(void)
{
	RtcStruct BjTime;
	char *a;
	unsigned char i=0,m=0;
	unsigned char nb_Timedata[22]={0};
	uint8_t time_buf[8];
	uint8_t time_buf_bcd[8];
	if(AppDataPointer->TerminalInfoData.AutomaticTimeStatus == AUTOMATIC_TIME_ENABLE)
	{
		AppDataPointer->TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_DISABLE;  //禁止时间同步
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
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
			Rtctime.Year = 2000+(nb_Timedata[0] - 0x30) * 10 + (nb_Timedata[1] - 0x30) * 1;	       //年
			Rtctime.Month = (nb_Timedata[3] - 0x30) * 10 + (nb_Timedata[4] - 0x30) * 1;	       //月
			Rtctime.Day = (nb_Timedata[6] - 0x30) * 10 + (nb_Timedata[7] - 0x30) * 1;	       //日
			Rtctime.Hour = (nb_Timedata[9] - 0x30) * 10 + (nb_Timedata[10] - 0x30) * 1;			//时
			Rtctime.Minute = (nb_Timedata[12] - 0x30) * 10 + (nb_Timedata[13] - 0x30) * 1;	       //分
			Rtctime.Second = (nb_Timedata[15] - 0x30) * 10 + (nb_Timedata[16] - 0x30) * 1;	       //秒
			UnixTimeStamp = covBeijing2UnixTimeStp(&Rtctime);		//get UTC
			UnixTimeStamp += 28800;									//UTC + 8hours
			covUnixTimeStp2Beijing(UnixTimeStamp, &BjTime);			//get Beijing time
			time_buf[1]= BjTime.Year - 2000;	   //年
			time_buf[2]= BjTime.Month;	       //月
			time_buf[3]= BjTime.Day;	       //日
			time_buf[4]= BjTime.Hour;	   	   //时
			time_buf[5]= BjTime.Minute;	       //分
			time_buf[6]= BjTime.Second;	       //秒
			for(m=1;m<7;m++) {
				time_buf_bcd[m]= HexToBCD(time_buf[m]);    //存“年月日时分秒”
			}
			OSBsp.Device.RTC.ConfigExtTime(time_buf_bcd,RealTime);
			Write_info_RTC(time_buf_bcd);
			g_Printf_dbg("NB Automatic Time OK\r\n");
		}
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
	
	if(AppDataPointer->TransMethodData.NBStatus == NB_Boot)
	{
		NB_Config("AT+CFUN=0\r\n",5,5);
		OSTimeDly(100);
		NB_Config("ATE1\r\n",5,5);
		OSTimeDly(200);
		NB_Config("AT+CGSN=1\r\n",5,5);  //IMEI
		OSTimeDly(100);
		NB_Config("AT+CFUN=1\r\n",100,5);
		OSTimeDly(100);
		NB_Config("AT+CIMI\r\n",5,5);    //USIM卡IMSI号
		OSTimeDly(100);
		NB_Config("AT+CGATT=1\r\n",5,5);
		OSTimeDly(100);
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		//等待入网
		while (ii < 60)
		{
			Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
			User_Printf("AT+CGPADDR\r\n");
			g_Printf_dbg("AT+CGPADDR\r\n");
			OSTimeDly(1000);
			if(Hal_CheckString(aRxBuff,"+CGPADDR:0,") || Hal_CheckString(aRxBuff,"+CGPADDR:1,"))
			{
				// AppDataPointer->TransMethodData.NBStatus = NB_Registered;
				break;
			}
		//		System.Device.Usart2.WriteString(dRxBuff);
			ii++;

		}
		if(ii >= 60)
		{
			// AppDataPointer->TransMethodData.NBStatus = NB_Power_on;
			g_Printf_dbg("GET IP Failed!\r\n");
			User_Printf("AT+NCSEARFCN\r\n");		//入网失败清除频点
			g_Printf_dbg("AT+NCSEARFCN\r\n");
			OSTimeDly(100);
			AppDataPointer->TransMethodData.NBStatus = NB_Init_Error;
			return 0;
		}
		else
		{
			// AppDataPointer->TransMethodData.NBStatus = NB_Get_IP;
			g_Printf_dbg("GET IP Succeed!\r\n");
			AppDataPointer->TransMethodData.NBNet = 1;
		}
		OSTimeDly(100);
		NB_Config("AT+CMEE=1\r\n",5,5);   //CMEE
		OSTimeDly(100);
		NB_Config("AT+CSQ\r\n",5,5);   //CSQ
		OSTimeDly(500);
		NB_Config("AT+NCONFIG=AUTOCONNECT,TRUE\r\n",2,5); //打开自动连接
		OSTimeDly(500);
		AppDataPointer->TransMethodData.NBStatus = NB_Init_Done;
		//********ML同步时间-20191111************//
		SyncTime();
		
		//*****************同步时间END************//
		return 1;
	}
	return 0;
}
char NB_MQTT_SendCheck(void)
{
	unsigned char i=0;
	while (!Hal_CheckString(aRxBuff,"+QMTPUB: 0,0,0")  & (i < 200))
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		//for(x=0;x<m;x++)
		OSTimeDly(500);
		i++;	
	}
	if(i>=200)
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		return 0;
	}
	else 
	{
		AppDataPointer->TransMethodData.NBSendStatus = 1;
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
		return 1;
	}
}

/*******************************************************************************
* 函数名		: g_Device_NB_register
* 描述	    	: NB模块注册阿里云---BC35G
* 输入参数  	: 无
* 返回参数  	: 初始化结果  0----失败；1----成功
*******************************************************************************/
uint8_t g_Device_NB_register()
{
	char temp[128];
	sprintf(temp,"AT+QMTCFG=\"aliauth\",0,\"%s\",\"%s\",\"%s\"\r\n",
	App.Data.TerminalInfoData.ProductKey,App.Data.TerminalInfoData.DeviceName,App.Data.TerminalInfoData.DeviceSecret);
	NB_Config((unsigned char*)temp,5,5);	//配置参数，回复OK
	OSTimeDly(100);
	NB_MQTT_Config("AT+QMTOPEN=0,\"iot-as-mqtt.cn-shanghai.aliyuncs.com\",1883\r\n",5,5,"+QMTOPEN: 0,0");//创建客户端 回OK +QMTOPEN: 0,0
	OSTimeDly(100);
	NB_MQTT_Config("AT+QMTCONN=0,\"SeeperIoT\"\r\n",5,5,"+QMTCONN: 0,0,0");	//连接服务器 回OK +QMTCONN: 0,0,0
	OSTimeDly(100);
	memset(temp,0,128);
	sprintf(temp,"AT+QMTSUB=0,1,\"/sys/%s/%s/thing/deviceinfo/update_reply\",2\r\n",
					App.Data.TerminalInfoData.ProductKey,App.Data.TerminalInfoData.DeviceName);
	NB_MQTT_Config((unsigned char*)temp,5,5,"+QMTSUB: 0,1,0,1");	//订阅资源 回OK +QMTSUB: 0,1,0,1
	OSTimeDly(100);
	AppDataPointer->TransMethodData.NBStatus = NB_Registered;
	return 1;
}
void g_Device_NB_MQTT_PUB(char* dat)
{
	char sta = 0;
	char temp[128];
	AppDataPointer->TransMethodData.NBSendStatus = 0;
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	sprintf(temp,"AT+QMTPUB=0,0,0,0,\"/sys/%s/%s/thing/event/property/post\"\r\n",
					App.Data.TerminalInfoData.ProductKey,App.Data.TerminalInfoData.DeviceName);
	NB_MQTT_Config((unsigned char*)temp,1,5,">");	//订阅资源 回OK +QMTSUB: 0,1,0,1
	OSTimeDly(100);
//	g_Printf_info(dat);
	OSBsp.Device.Usart0.WriteString(dat);
	OSBsp.Device.Usart0.WriteData(0x1A);

	OSTimeDly(100);
	sta = NB_MQTT_SendCheck();	//正常返回1
	if(!sta){
		User_Printf("AT+QMTDISC=0\r\n");
		OSTimeDly(100);
		AppDataPointer->TransMethodData.NBStatus = NB_Init_Done;
	}
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
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
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
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
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
	char *a;
	unsigned char i=0,n=0;
	int32_t dataTemp=0;
	static uint8_t Singal_Less=0;
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);

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

		/**********************************************/
		dataTemp=0-dataTemp;
		AppDataPointer->TransMethodData.RSRP = (float)dataTemp/10;
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
			/**********************************************/
			dataTemp=0-dataTemp;
			AppDataPointer->TransMethodData.SINR = (float)dataTemp/10;
		}
		else
		{
			for(n=0;n<i;n++)
			{
				dataTemp=dataTemp*10 + SINR_data[n]-0x30;
			}
			
			/**********************************************/
			AppDataPointer->TransMethodData.SINR = (float)dataTemp/10;
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
		/**********************************************/
		AppDataPointer->TransMethodData.PCI = dataTemp;
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
		/**********************************************/
		AppDataPointer->TransMethodData.CELLID = dataTemp;
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
		temp = Get_String("0:/INDEX" , RespFile , Data_Backup , 122);
		if( temp == 1)		
		{
			// BackupIndex--;
			ResendData = 1;		//补发数据标志位
			Data_Backup[120] = '\0';
			AppDataPointer->TransMethodData.NBStatus = NB_Init_Done;
			break;		//退出循环，准备发送数据
		}
		else			//没取到字符串
		{
			CreatFileNum(0);		//参数0   BackupIndex--;
		}
	}
	
}
char *MakeAliJsonBody(DataStruct *DataPointer)
{
    mallco_dev.init();

    cJSON * pJsonRoot = NULL;
	cJSON * pSubJson;
//	cJSON * pSub_SubJson;
//	cJSON * pBack;
	char * p;
//	char mqttpub[256];
//	uint16_t mqttpub_len = 0;
//	uint16_t p_len = 0;
//	static uint16_t p_m = 0;
	char SeqNum[10];
	memset(SeqNum,0x0,10);
	sprintf(SeqNum,"%d",DataPointer->TransMethodData.SeqNumber); 
    pJsonRoot = cJSON_CreateObject();
    if(NULL == pJsonRoot)
    {
        cJSON_Delete(pJsonRoot);
        return NULL;
    }
	cJSON_AddStringToObject(pJsonRoot,"id",SeqNum);
	cJSON_AddStringToObject(pJsonRoot,"version","1.0");

	pSubJson = NULL;
	pSubJson = cJSON_CreateObject();
	if(NULL == pSubJson)
	{
	  //create object faild, exit
	  cJSON_Delete(pJsonRoot);
	  return NULL;
	}
	cJSON_AddNumberToObject(pSubJson, "Status",(int)(DataPointer->SeeperData.LVValue));     //水深
	cJSON_AddNumberToObject(pSubJson, "Data",DataPointer->TerminalInfoData.BATVoltage);
	
	cJSON_AddItemToObject(pJsonRoot, "params", pSubJson);
	cJSON_AddStringToObject(pJsonRoot, "method","thing.event.property.post");

	// p = cJSON_PrintUnformatted(pJsonRoot);
	// p_len = strlen(p);

	// memset(mqttpub,0x0,256);
	// mqttpub_len = 0;
	// for(p_m=0;p_m<p_len;p_m++){
	// 	if(p[p_m] == 0x22){		//"  换成  \"
	// 		mqttpub[mqttpub_len++] = 0x5c;
	// 		mqttpub[mqttpub_len++] = 0x32;
	// 		mqttpub[mqttpub_len++] = 0x32;
	// 	}else{
	// 		mqttpub[mqttpub_len++] = p[p_m];
	// 	}
	// }
	// System.Device.Usart2.WriteString(mqttpub);
	// Mqtt_Publish_Property(mqttpub,1);
	
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
* 函数名  : TransmitTaskStart
* 描述	  : 发送任务函数
* 输入参数  : *p_arg
* 返回参数  : 无
*******************************************************************************/
void  TransmitTaskStart (void *p_arg)
{
//	uint8_t i;
	char *data;
//	uint8_t initRetry = 0;

    (void)p_arg;   
    OSTimeDlyHMSM(0u, 0u, 0u, 100u);      
    g_Printf_info("%s ... ...\n",__func__);           
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.*/
        if(Hal_getCurrent_work_Mode() == 0){
			TaskRefreshWTD(EventWtFlag , WTD_BIT_TRANSMIT);
            if(AppDataPointer->TransMethodData.NBStatus == NB_Power_off)
			{
				//NB-IoT 第一次开机时对NB上电操作，后续进入低功耗不关电
				g_Printf_dbg("Turn on NB power\r\n");
				// g_Printf_info("\r\n\r\nNB-IoT Fota Test version 21\r\n\r\n");	//测试打印
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
				g_Device_NB_register();
			}
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Registered)
			{
                if( AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER)
				{
					g_Printf_info("Scan over,data uploading\r\n");
					if(ResendData == 0)		//正常上报数据,需要累加SeqNum,采集电压，本地存储
					{

						//char response[128];
						//SeqNumber ++
						if(App.Data.TerminalInfoData.DeviceFirstRunStatus == DEVICE_STATUS_FIRSTRUN_BEGIN) {
							App.Data.TerminalInfoData.DeviceFirstRunStatus = DEVICE_STATUS_FIRSTRUN_OVER;
							App.Data.TransMethodData.SeqNumber = 0;
						}else {
							App.Data.TransMethodData.SeqNumber++;
							if(AppDataPointer->TransMethodData.SeqNumber >= 65535)
								AppDataPointer->TransMethodData.SeqNumber = 1;
						}						
						//Voltage
						GetADCValue();
						//检查信号质量
						g_Device_NBSignal();
					//	data = MakeMQTTJsonBody(AppDataPointer);		//组包json并存储SD卡
						data = MakeAliJsonBody(AppDataPointer);		//组包json并存储SD卡
						
					//	g_Printf_info("data:%s\r\n",data);
						//memset(response,0x0,128);
						// Hex2Str(Data_Backup,Send_Buffer,60,0);					
						// g_Printf_info("Hexdata:%s\r\n",Data_Backup);    //打印输出16进制发送数据
					}
					//发送数据
					if(AppDataPointer->TransMethodData.NBNet == 1)
					{
						// if(App.Data.TransMethodData.SeqNumber == 0){		//开机第一次不上报数据
						// 	AppDataPointer->TransMethodData.NBStatus = NB_Idel;
						// }else{
							g_Device_NB_MQTT_PUB(data);
							// g_Device_NB_Send_Str(Data_Backup,60);	
							// OSTimeDly(5000);	//等待10s
							// g_Device_NB_SendCheck();
							if(AppDataPointer->TransMethodData.NBSendStatus == 1)	//确认帧发送成功,发送数据前会置0
							{
								AppDataPointer->TransMethodData.NBStatus = NB_Send_Over;
							}
							else
							{
								WriteStoreData();
								AppDataPointer->TransMethodData.NBStatus = NB_Send_Error;			//发送失败
							}
						// }
					}
					else
					{						
						AppDataPointer->TransMethodData.NBStatus = NB_Init_Error;
					}
				}      
            }
			else if(AppDataPointer->TransMethodData.NBStatus == NB_Send_Over)		//不在线直接进Idle
			{
				//g_Device_NB_GetReceive();
				AppDataPointer->TransMethodData.NBStatus = NB_Idel;
			}
			else if((AppDataPointer->TransMethodData.NBStatus == NB_Idel) || (AppDataPointer->TransMethodData.NBStatus == NB_Init_Error)
					|| (AppDataPointer->TransMethodData.NBStatus == NB_Send_Error))	{//发送完成或入网失败，关闭NB电源，进入低功耗，退出低功耗后重新上电初始化
				if( AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER)
					Hal_EnterLowPower_Mode();
			}  

            OSTimeDlyHMSM(0u, 0u, 0u, 200u);  
        }
    }
}

#endif //(TRANSMIT_TYPE == NBIoT_BC95_Mode)

