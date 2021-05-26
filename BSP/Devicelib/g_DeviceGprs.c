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

#if (TRANSMIT_TYPE == GPRS_Mode)
 const char *g_30000IoT_HOST = "172.17.1.109:8082";
 const char *g_30000IoT_PATH = "";
//const char *g_30000IoT_HOST = "30000iot.cn:9001";
//const char *g_30000IoT_PATH = "/api/Upload/data/";
//const char *g_30000IoT_HOST = "47.111.88.91:6096";
//const char *g_30000IoT_PATH = "/iot/data/receive";

//unsigned long readAddr = 0;     //SPI_Flash 读写地址
//uint8_t buf0[16]={0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x0D,0x0A},buf1[256] = {0},buf2[1024] ={0};

uint32_t newVersion = 0;		//Fota 临时存储版本
#define TestD 0
#ifdef TestD
long addr_write = FOTA_ADDR_START;

unsigned long readAddr = 0;     //SPI_Flash 读写地址
uint8_t buf0[16]={0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x0D,0x0A},buf1[256] = {0},buf2[1024] ={0};
  
#endif // TestD

enum CoordinateSystem{
	WGS_84 = 1,
	GCJ_02 = 2
};

static char gprs_tick = 0;
static char gprs_over_tick = 0;
static uint32_t HTTP_Status_Code = 0;
static int g_has_response = 0;
static char g_response[256];
char g_ftp_allow_get = 0;
char g_ftp_allow_storage = 0;
char *StartString = NULL;
char *EndString  = NULL;
char CSQBuffer[15]={'0'};

uint8_t download_data_1[1536];
uint16_t data1_len = 0;
//long addr_write = FOTA_ADDR_START;


//static char TimeString[20] = "20170804 16:00:00";

/*******************************************************************************
* 函数名		: g_Device_GPRS_Init
* 描述	    	: GPRS模块初始化
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_GPRS_Init(void)
{
	if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Preinit){
		if((AppDataPointer->TransMethodData.GPRSNet == 0)&&
				(AppDataPointer->TransMethodData.GPRSAttached == 0)){
			g_Printf_dbg("AT+CSQ\r\n");
			User_Printf("AT+CSQ\r\n");   //查询信号质量
			gprs_tick ++;
			OSTimeDly(1500);
			if(gprs_tick == 5){
				gprs_tick = 0;
				g_Printf_info("%s failed : Poor signal quality\r\n",__func__);
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Init_Failed;
			}	
		}else if((AppDataPointer->TransMethodData.GPRSNet == 1)&&
				(AppDataPointer->TransMethodData.GPRSAttached == 0)){
			g_Printf_dbg("AT+CGATT?\r\n");
			User_Printf("AT+CGATT?\r\n"); 		//查询GPRS附着状态 ，0表示分离，1表示附着
			gprs_tick ++;
			OSTimeDly(1000);	
			if((gprs_tick == 8)&&(AppDataPointer->TransMethodData.GPRSAttached == 0)){
				gprs_tick = 0;
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Init_Failed;
				OSTimeDly(1000);
			}	
		}
		else if((AppDataPointer->TransMethodData.GPRSNet == 1)
		       &&(AppDataPointer->TransMethodData.GPRSAttached == 1)
			   &&(AppDataPointer->TerminalInfoData.AutomaticTimeStatus == AUTOMATIC_TIME_ENABLE)){
			AppDataPointer->TransMethodData.GPRSTime = 0;
			AppDataPointer->TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_DISABLE;  //禁止时间同步
			memset(aRxBuff,0x0,256);

//			g_Printf_dbg("AT+CNTP=\"203.107.6.88\",\"8\"\r\n");
//			User_Printf("AT+CNTP=\"203.107.6.88\",\"8\"\r\n");  //同步网络时间
//			OSTimeDly(2000);//2ms
//			g_Printf_dbg("AT+CNTP\r\n");
//			User_Printf("AT+CNTP\r\n");  //同步网络时间
//			OSTimeDly(2000);//2ms
//			g_Printf_dbg("AT+CCLK?\r\n");
//			User_Printf("AT+CCLK?\r\n");  //获取基站定位和日期
			OSTimeDly(2000);//2ms
		}
		else if((AppDataPointer->TransMethodData.GPRSNet == 1)&&
				(AppDataPointer->TransMethodData.GPRSAttached == 1)){

			OSTimeDly(2000);
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Init_Done;
#ifdef AIR202
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Preinit;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
#endif
		}	
		else
		{
			gprs_tick ++;
			if (gprs_tick >= 10)
			{
				gprs_tick = 0;
				g_Printf_info("%s failed : GPRS_Preinit Wrong!\r\n",__func__);
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Init_Failed;
			}
		}
	}
#ifdef AIR202
	else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Preinit){
		gprs_tick = 0;
		while(gprs_tick<7)
		{
			if(gprs_tick == 0){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
					User_Printf("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 1;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 1){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+SAPBR=3,1,\"APN\",\"\"\r\n");
					User_Printf("AT+SAPBR=3,1,\"APN\",\"\"\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 2;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 2){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+SAPBR=1,1\r\n");
					User_Printf("AT+SAPBR=1,1\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 3;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_GetIP;
				}
			}else if(gprs_tick == 3){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_GetIP){
					AppDataPointer->TransMethodData.Http_Cid = 0;
					memset(aRxBuff,0x0,256);
					aRxNum = 0;
					g_Printf_dbg("AT+SAPBR=2,1\r\n");
					User_Printf("AT+SAPBR=2,1\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 4;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 4){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+HTTPINIT\r\n");
					User_Printf("AT+HTTPINIT\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 5;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 5){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+HTTPPARA=\"CID\",%d\r\n",AppDataPointer->TransMethodData.Http_Cid);
					User_Printf("AT+HTTPPARA=\"CID\",%d\r\n",AppDataPointer->TransMethodData.Http_Cid);
					OSTimeDly(1000);
				}else{
					gprs_tick ++;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
					AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Init_Done;
					g_Printf_info("%s done and ready to post\r\n",__func__);
				}
			}


				gprs_over_tick++;
				//wj20200215-问题在于gprs_over_tick > 12，如果要加这段代码，12要改成13以上；入网时有可能失败，故防止死循环
				// if(gprs_over_tick > 3)
				if(gprs_over_tick > 15)
				{
					//AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Err;  //重新进入此程序
					gprs_tick = 10;  //>7即可
				}

		}
		gprs_tick = 0;
		gprs_over_tick = 0;
	}
#endif
}

/*******************************************************************************
* 函数名		: g_Device_Establish_TCP_Connection
* 描述	    	: 建立TCP链接
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_Establish_TCP_Connection(const char *ip,uint32_t port)
{
//	//Saas
//	User_Printf("AT+CIPSTART=\"TCP\",\"114.55.93.183\",\"8081\"\r\n"); //建立TCP连接
	//OneNet      OneBoxLabDevice
//	User_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",ip,port);   //建立TCP连接
}

/*******************************************************************************
* 函数名		: GprsHttpPost
* 描述	    	: http上报数据
* 输入参数  	: host,path,apikey,data,response,timeout(sec)
* 返回参数  	: code
*******************************************************************************/
int16_t g_Device_http_post(const char *host,const char* path,const char *apikey,const char *data,
                      char *response,int timeout)
{
//	int timer = 0;


    uint32_t datalen = 0;
	datalen = strlen(data);

	g_Printf_dbg("AT+CIPSTART=\"TCP\",\"172.17.1.109\",\"8082\"\r\n");
	User_Printf("AT+CIPSTART=\"TCP\",\"172.17.1.109\",\"8082\"\r\n");   //建立TCP连接
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	g_Printf_dbg("AT+CIPSEND\r\n");
	User_Printf("AT+CIPSEND\r\n");
	hal_Delay_ms(100);
	hal_Delay_ms(100);


	User_Printf("POST /saveWaterQuality HTTP/1.1\r\n");
	User_Printf("Host:172.17.1.109:8082\r\n");
	User_Printf("Accept:*/*\r\n");
	User_Printf("Content-Length:%d\r\n",datalen);
	User_Printf("SendData:");
	User_Printf("%s\r\n",data);
	User_Printf("Connection:close\r\n");
	User_Printf("\r\n");

	hal_Delay_ms(100);
	OSBsp.Device.Usart0.WriteData(0x1A);      //发送数据完成结束符0x1A,16进制发送
	hal_Delay_ms(100);
	g_Printf_dbg("AT+HTTPREAD\r\n");
	User_Printf("AT+HTTPREAD\r\n");
//	User_Printf("\r\n");
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);
	hal_Delay_ms(100);

//    OSBsp.Device.Usart2.WriteString(data);
}

char data_write[300];
void g_Device_GPRS_Fota_Start(void)
{
	long i = 0;
	int m;
	char *StartString = NULL;
	char *EndString  = NULL;
	char tempLen[10];
	// long add_temp;
	// char d_t[7524];
	uint8_t TestData[3]={0};
	uint8_t Flash_Tmp[3];					//flash操作中间变量
	int length=0;
	
	if(gprs_tick == 0){
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
			User_Printf("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"\r\n");
			OSTimeDly(1000);
		}else{
			gprs_tick = 1;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 1){
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
			User_Printf("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
			OSTimeDly(1000);
		}else{
			gprs_tick = 2;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 2){
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+SAPBR=1,1\r\n");
			User_Printf("AT+SAPBR=1,1\r\n");
			OSTimeDly(1000);
		}else{
			gprs_tick = 3;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_GetIP;
		}
	}else if(gprs_tick == 3){         
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_GetIP){
			AppDataPointer->TransMethodData.Ftp_Cid = 0;
			g_Printf_dbg("AT+SAPBR=2,1\r\n");
			User_Printf("AT+SAPBR=2,1\r\n");
			OSTimeDly(1000);
		}else{
			gprs_tick = 4;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 4){          
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPCID=%d\r\n",AppDataPointer->TransMethodData.Ftp_Cid);
			User_Printf("AT+FTPCID=%d\r\n",AppDataPointer->TransMethodData.Ftp_Cid);
			OSTimeDly(1000);
		}else{
			gprs_tick = 5;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			g_Printf_info("%s done and ready to get file\r\n",__func__);
		}
	}else if(gprs_tick == 5){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPTYPE=\"%s\"\r\n","I");
			User_Printf("AT+FTPTYPE=\"%s\"\r\n","I");
			OSTimeDly(1000);
		}else{
			gprs_tick = 6;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 6){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPSERV=\"%s\"\r\n",AppDataPointer->FotaInfor.ip);
			User_Printf("AT+FTPSERV=\"%s\"\r\n",AppDataPointer->FotaInfor.ip);
			OSTimeDly(1000);
		}else{
			gprs_tick = 7;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 7){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPPORT=%d\r\n",AppDataPointer->FotaInfor.port);
			User_Printf("AT+FTPPORT=%d\r\n",AppDataPointer->FotaInfor.port);
			OSTimeDly(1000);
		}else{
			gprs_tick = 8;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 8){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPUN=\"%s\"\r\n",AppDataPointer->FotaInfor.username);
			User_Printf("AT+FTPUN=\"%s\"\r\n",AppDataPointer->FotaInfor.username);
			OSTimeDly(1000);
		}else{
			gprs_tick = 9;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 9){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPPW=\"%s\"\r\n",AppDataPointer->FotaInfor.password);
			User_Printf("AT+FTPPW=\"%s\"\r\n",AppDataPointer->FotaInfor.password);
			OSTimeDly(1000);
		}else{
			gprs_tick = 10;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 10){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPGETNAME=\"%s\"\r\n",AppDataPointer->FotaInfor.imgname);
			User_Printf("AT+FTPGETNAME=\"%s\"\r\n",AppDataPointer->FotaInfor.imgname);
			OSTimeDly(1000);
		}else{
			gprs_tick = 11;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 11){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_dbg("AT+FTPGETPATH=\"%s\"\r\n",AppDataPointer->FotaInfor.imgpath);
			User_Printf("AT+FTPGETPATH=\"%s\"\r\n",AppDataPointer->FotaInfor.imgpath);
			OSTimeDly(1000);
		}else{
			gprs_tick = 12;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}else if(gprs_tick == 12){      
		if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
			g_Printf_info("AT+FTPGET=1\r\n");
			User_Printf("AT+FTPGET=1\r\n");
			g_Printf_info("start download ==");
			OSTimeDly(1000);
		}else{
			gprs_tick = 13;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}
	//存储固件
	if(g_ftp_allow_get == 1){
		User_Printf("AT+FTPGET=2,1024\r\n");
		OSTimeDly(1000);		//延时2秒，等待读取完毕
		g_ftp_allow_get = 0;
		g_ftp_allow_storage = 0;
		
		//计算数据长度
		StartString = strstr(download_data_1,"FTPGET: 2, ");
		EndString = strstr(StartString,"\r\n");
		if(EndString > StartString && StartString != NULL)
		{
			memcpy(tempLen, StartString+11, EndString - StartString - 10);
		}
		length = atoi(tempLen);	
		
		for(m=0;m<length;m++)
		{
			SPI_Flash_Write_Data(EndString[2+m],addr_write++);
		//	hal_Delay_us(3);
		}
		OSTimeDly(100);
		memset(download_data_1,0x0,1536);
		data1_len = 0;
		StartString = NULL;
		EndString = NULL;
			
	}else if(g_ftp_allow_get == 2){
		g_ftp_allow_get = 0;
		g_ftp_allow_storage = 0;
		g_Printf_info("code printf begin:");
		for(i=FOTA_ADDR_START;i<addr_write;i++)
		{
			
			OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(i));
			// hal_Delay_ms(1);
		}
		OSTimeDly(1000);	//延时2s
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
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;
		}
	}	
	
}

void g_Device_check_Response(char *res)
{
	uint8_t *a;
	uint8_t i=0,m=0;
	uint8_t gpts_Timedata[25]={0};
	uint8_t time_buf[8];
	uint8_t time_buf_bcd[8];

	//static uint8_t res_len = 0;

	if(g_has_response == -1){
		g_has_response = 0;
		memset(g_response,0x0,256);
		strcpy(g_response,res);
	}


//	OSBsp.Device.Usart2.WriteString(res);

#ifdef AIR202
	if( (AppDataPointer->TransMethodData.GPRSStatus == GPRS_Power_on) ||
			(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Waitfor_SMSReady) ){
		if(Hal_CheckString(res,"SMS Ready")){
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Preinit;
		}
	}else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Preinit){
		if(Hal_CheckString(res,"+CGATT:"))
		{
			if (Hal_CheckString(res,"0"))
			{
				g_Printf_dbg("Attached Faild!\r\n");
				AppDataPointer->TransMethodData.GPRSAttached = 0;
			}else
			{
				g_Printf_dbg("Attached OK!\r\n");
				AppDataPointer->TransMethodData.GPRSAttached = 1;
			}
		}else if(Hal_CheckString(res,"+CSQ:")){
			if (Hal_CheckString(res," 0,0")){
				AppDataPointer->TransMethodData.GPRSNet = 0;
			}else{
				AppDataPointer->TransMethodData.GPRSNet = 1;
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Preinit;
				memset(CSQBuffer, '\0', 15);	//清空buffer
				StartString = strstr(aRxBuff,"+CSQ:");
				EndString  = strstr(aRxBuff, ",");
				memcpy(CSQBuffer, StartString + 6, EndString - StartString - 6);	//复制CSQ值
			}
		}else if(Hal_CheckString(res,"+CCLK:")){   //日期
        	a=strstr(res,"+CCLK:");
			if((a!=NULL)&&(*(a+8)!="."))
			{
				while(*(a+8)!='\r')
				{
					gpts_Timedata[i]=*(a+8);
					i++;
					a++;
				}	
				gpts_Timedata[i]='\n';	
				
				time_buf[1]=(gpts_Timedata[0]-0x30)*10+(gpts_Timedata[1]-0x30)*1;	       //年
				time_buf[2]=(gpts_Timedata[3]-0x30)*10+(gpts_Timedata[4]-0x30)*1;	       //月
				time_buf[3]=(gpts_Timedata[6]-0x30)*10+(gpts_Timedata[7]-0x30)*1;	       //日
				time_buf[4]=(gpts_Timedata[9]-0x30)*10+(gpts_Timedata[10]-0x30)*1;	       //时
				time_buf[5]=(gpts_Timedata[12]-0x30)*10+(gpts_Timedata[13]-0x30)*1;	       //分
				time_buf[6]=(gpts_Timedata[15]-0x30)*10+(gpts_Timedata[16]-0x30)*1;	       //秒
				for(m=1;m<7;m++) {
					time_buf_bcd[m]= HexToBCD(time_buf[m]);    //存“年月日时分秒”
				}
				OSBsp.Device.RTC.ConfigExtTime(time_buf_bcd,RealTime);
				Write_info_RTC(time_buf_bcd);
				g_Printf_dbg("Gprs Automatic Time OK\r\n");
				AppDataPointer->TransMethodData.GPRSTime = 1;
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Preinit;
			}
		}
	}else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Init_Done){
		if(Hal_CheckString(res,"OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_OK;
		}else if(Hal_CheckString(res,"DOWNLOAD")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_Download;
		}else if(Hal_CheckString(res,"+HTTPACTION: 1")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_HTTPACT;
			HTTP_Status_Code = (res[15]-0x30)*100+(res[16]-0x30)*10+(res[17]-0x30);
			g_has_response = res[19]-0x30;
		}else if(Hal_CheckString(res,"iotToken")){
			// memcpy(iotTokenBuf,response,strlen(response));
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_OK;
		}else if(Hal_CheckString(res,"+HTTPREAD:")){
			g_has_response = -1;
		}

	}else if((AppDataPointer->TransMethodData.GPRSStatus == GPRS_Mqtt_Preinit)
	||(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Preinit)||(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Init_Done)){
		if(Hal_CheckString(res,"CONNECT OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Connect_OK;
		}else if(Hal_CheckString(res,"CONNACK OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Connack_OK;
		}else if(Hal_CheckString(res,"OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_OK;
		}else if(Hal_CheckString(res,"+HTTPREAD:")){
			memset(res,0x0,256);
		}else if(Hal_CheckString(res,"+SAPBR:")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			AppDataPointer->TransMethodData.Http_Cid = res[8] - 0x30;
		}else if(Hal_CheckString(res,"DOWNLOAD")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_Download;
		}else if(Hal_CheckString(res,"+HTTPACTION: 1")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_HTTPACT;
		}
//		else if(Hal_CheckString(res,"HTTP/1.1 200 OK")){
//			OSBsp.Device.Usart2.WriteString(res);  //****************************//
//		}
	}
	else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Mqtt_Init_Done){
		if(Hal_CheckString(res,"SUBACK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_Suback;
		}else if(Hal_CheckString(res,"OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_OK;
		}
	}else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_MQTT_Alive){
		if(Hal_CheckString(res,"PUBACK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_MQTT_Enable_Pulish;
		}else if(Hal_CheckString(res,"+MSUB: ")){
			// memset(mqttbuf,0x0,512);
			// memcpy(mqttbuf,response,strlen(response));
		}
	}else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Fota_Process){
		if(Hal_CheckString(res,"OK")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Get_OK;
			if(g_ftp_allow_storage == 1){
				g_ftp_allow_storage = 0;
			}
		}else if(Hal_CheckString(res,"+SAPBR:")){
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			AppDataPointer->TransMethodData.Ftp_Cid = res[8] - 0x30;
		}else if(Hal_CheckString(res,"+FTPGET: 1,1")){
			g_ftp_allow_get = 1;
			g_ftp_allow_storage = 1;
		}else if(Hal_CheckString(res,"+FTPGET: 2")){
			// g_ftp_allow_storage = 1;
		}else if(Hal_CheckString(res,"+FTPGET: 1,0")){
			g_ftp_allow_get = 2;
		}else if(Hal_CheckString(res,"+FTPGET: 1,")) {//除了+FTPGET: 1,1和+FTPGET: 1,0之外异常情况
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;	//FTP过程出现故障退出，恢复到正常上报流程
		}
	}
#endif
}
/*******************************************************************************
* 函数名		: CAT_Config
* 描述	    	: 配置NB模组参数
* 输入参数  	: *c--需要配置的指令，m--指令等待时间（m*1000ms），t--配置失败重新尝试次数
* 返回参数  	: 0--失败，1--成功
*******************************************************************************/
unsigned char CAT_Config(unsigned char *c , unsigned char m, unsigned char t)
{
	unsigned char x,i=0;
	i=0;
	Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	while (!Hal_CheckString(aRxBuff,"OK")  & (i < t))
	{
		Clear_Buffer((unsigned char *)aRxBuff,&aRxNum);
	 	User_Printf((char *)c);
	 	g_Printf_dbg((char *)c);		//debug口同步打印
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
void  TransmitTaskStart (void *p_arg)
{
	long i = 0;
	uint32_t datalen = 0;
//	uint8_t i=0;
//	uint8_t scadaADCIndex = 0,scadaBATIndex = 0;
	static uint8_t idle_times = 0;
	static uint8_t failed_times = 0;

    (void)p_arg;   
    OSTimeDlyHMSM(0u, 0u, 0u, 100u);      
    g_Printf_info("%s ... ...\n",__func__);      
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.       */
        if(Hal_getCurrent_work_Mode() == 0){
			TaskRefreshWTD(EventWtFlag , WTD_BIT_TRANSMIT);
            if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Power_off){
				OSTimeDly(1000); 
                OSBsp.Device.IOControl.PowerSet(BaseBoard_Power_On);
                OSBsp.Device.IOControl.PowerSet(SIM800C_Power_On);
                OSTimeDly(500); 
                OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
				OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);
                AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_on;
                //上电后延时一段时间
                OSTimeDly(2500);OSTimeDly(2500);OSTimeDly(2500);OSTimeDly(2500);

				CAT_Config("AT\r\n",1,5);
                OSTimeDly(1000);
                g_Printf_dbg("ATE0\r\n");
                User_Printf("ATE0\r\n");         //关闭回显，初始化阶段关闭
				OSTimeDly(1000);
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Preinit;
            }

			else if((AppDataPointer->TransMethodData.GPRSStatus > GPRS_Waitfor_SMSReady)&&
                    	(AppDataPointer->TransMethodData.GPRSStatus < GPRS_Http_Init_Done)){
				idle_times = 0;
                g_Device_GPRS_Init();
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Init_Done){
                if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER){
					if (App.Data.TerminalInfoData.DeviceFirstRunStatus == DEVICE_STATUS_FIRSTRUN_BEGIN) {
						App.Data.TerminalInfoData.DeviceFirstRunStatus = DEVICE_STATUS_FIRSTRUN_OVER;
						App.Data.TransMethodData.SeqNumber = 0;
					}else {
						App.Data.TransMethodData.SeqNumber++;
					}
					failed_times = 0;	//Initdone 之后清零失败次数
					//************电量处理Begin************//
	                GetADCValue();
                    //************电量处理End*************//

					char response[512];
					char data[512];
					uint32_t datalen = snprintf(data,512,MakeJsonBodyData(AppDataPointer));
					g_Printf_info("datalen:%d\ndata:%s\r\n",datalen,data);
				
                   
                	memset(response,0x0,512);
					int16_t code = 0;
                    code = g_Device_http_post(g_30000IoT_HOST,g_30000IoT_PATH,null,data,response,30);//时间延长至30s

                    if(code == 200){
                        g_Printf_info("response : %s \r\n",response);   //对response解析，可以执行配置或ota操作
						AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;
                    }else{    //这里可以做失败重发操作
					    //AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;  //ML 20190828
                    	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Err;
                        g_Printf_dbg("http_post failed\r\n");
                    }                        
                }    
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Fota_Process){
				g_Device_GPRS_Fota_Start();
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Post_Done){
                // OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
            	if(App.Data.TerminalInfoData.SendPeriod > NO_LOWPER_PERIOD)
            	{
            		Hal_EnterLowPower_Mode();//上传频率大于5min，才进低功耗模式，对应取消RTC_ISR
            	}
            	else
            	{  //不进入低功耗模式，用延时函数来实现上传周期
            		g_Printf_dbg("POST OK!\r\n");
            		g_Printf_dbg("No LowPower!\r\n");
            		AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Preinit;
            		//AppDataPointer->TerminalInfoData.DeviceStatus = DEVICE_STATUS_POWER_OFF; //在RTC里设置过了
            		//恢复到模块http初始化完成的时候，就不必二次初始化了。
            		if (App.Data.TerminalInfoData.SendPeriod == 1)  //最好不要设置成1
            		{
						OSTimeDlyHMSM(0u, 0u, 27u, 0u);//GPRS上传全部完成需要30s
            		}
            		else if (App.Data.TerminalInfoData.SendPeriod == 2)
            		{
            			OSTimeDlyHMSM(0u, 1u, 27u, 0u);
            		}
            		else
            		{
            			OSTimeDlyHMSM(0u, 2u, 27u, 0u);
            		}
            	}
            }
            else if (AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Err)
            {  //入网遇到问题，本包丢，并做模块重启处理，重新上传
            	if(App.Data.TerminalInfoData.SendPeriod > NO_LOWPER_PERIOD)
            	{
            		g_Printf_dbg("POST ERR!\r\n");

            	    Hal_EnterLowPower_Mode();//上传频率大于5min，才进低功耗模式，对应取消RTC_ISR
            	}
            	else
            	{  //用模块重启来实现故障自恢复
					g_Printf_dbg("AT+RESET\r\n");
                	User_Printf("AT+RESET\r\n");
					OSTimeDly(1000);OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); 
					OSTimeDly(1000);OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000);  //2s
					AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_off;
					AppDataPointer->TransMethodData.GPRSNet = 0;
					AppDataPointer->TransMethodData.GPRSAttached =0;
					AppDataPointer->TransMethodData.GPRSATStatus = 0;
            	}

            }
			else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Init_Failed){

			 	g_Printf_info("Gprs init failed\r\n");
			 	failed_times++;
			 	if (failed_times < 5) //第一次初始化失败，重启GPRS模块
			 	{
					g_Printf_dbg("AT+RESET\r\n");
                	User_Printf("AT+RESET\r\n");
					OSTimeDly(1000);OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); 
					OSTimeDly(1000);OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000); OSTimeDly(1000);  //2s
					//清零
					AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_off;
					AppDataPointer->TransMethodData.GPRSNet = 0;
					AppDataPointer->TransMethodData.GPRSAttached =0;
					AppDataPointer->TransMethodData.GPRSATStatus = 0;
			 	}
			 	else // 9次初始化失败，重启，只存在于<NO_LOWPER_PERIOD的逻辑里
			 	{
					failed_times = 0;	//结束计数，清零，进入低功耗
					g_Printf_dbg("GPRS Init Retry vaild, abandon!\r\n");
					Hal_EnterLowPower_Mode();	//进低功耗模式
			 	}

            }
            else  //GPRS unkonw err, such as GPRS_Wait_Idle
            {
            	idle_times++;
            	OSTimeDlyHMSM(0u, 0u, 2u, 0u);
            	if (idle_times == 60)  //120s 空跑2min
            	{
            		g_Printf_dbg("GPRS Unkonw error,try reboot!\r\n");
            		idle_times = 0;
					Hal_EnterLowPower_Mode();	//进低功耗模式
					// App.Data.TransMethodData.SeqNumber = 0;
					// App.Data.TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_ENABLE;  //允许时间同步
					// OSBsp.Device.IOControl.PowerSet(AIR202_Power_Off);
					// OSBsp.Device.IOControl.PowerSet(LPModule_Power_Off);
					// OSBsp.Device.IOControl.PowerSet(Motor_Power_Off);

					// hal_Reboot();  //复位 是否需要主机都复位？或者只需要
            	}
            }

            OSTimeDlyHMSM(0u, 0u, 0u, 200u);  
        }
        else
        {
        	g_Printf_dbg("TransmitTaskStart ERR!\r\n");
            OSTimeDlyHMSM(0u, 0u, 0u, 200u);  
			hal_Reboot();  //任务启动失败，必须重启
        }
    }
}

#endif //(TRANSMIT_TYPE == GPRS_Mode)

