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
// const char *g_30000IoT_HOST = "30000iot.cn:9001";    
// const char *g_30000IoT_PATH = "/api/Upload/data/";
const char *g_30000IoT_HOST = "47.111.88.91:6096"; 
const char *g_30000IoT_PATH = "/iot/data/receive";

//unsigned long readAddr = 0;     //SPI_Flash 读写地址
//uint8_t buf0[16]={0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x37,0x36,0x35,0x34,0x33,0x32,0x0D,0x0A},buf1[256] = {0},buf2[1024] ={0};

char codeFile[11]={"0:/1106.txt"};

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
			OSTimeDly(1000);
			if(gprs_tick == 5){
				gprs_tick = 0;
				g_Printf_info("%s failed : Poor signal quality\r\n",__func__);
			}	
		}else if((AppDataPointer->TransMethodData.GPRSNet == 1)&&
				(AppDataPointer->TransMethodData.GPRSAttached == 0)){
			g_Printf_dbg("AT+CGATT?\r\n");
			User_Printf("AT+CGATT?\r\n"); 		//查询GPRS附着状态 ，0表示分离，1表示附着
			gprs_tick ++;
			OSTimeDly(1000);	
			if((gprs_tick == 5)&&(AppDataPointer->TransMethodData.GPRSAttached == 0)){
				gprs_tick = 0;
				g_Printf_info("Manual gain access to network\r\n");
				g_Printf_dbg("AT+CGATT=1\r\n"); //手动附着GPRS网络
				User_Printf("AT+CGATT=1\r\n");
				OSTimeDly(1000);
			}	
		}else if((AppDataPointer->TransMethodData.GPRSNet == 1)&&
				(AppDataPointer->TransMethodData.GPRSAttached == 1)){
#ifdef SIM800C
			g_Printf_dbg("AT+CIPSHUT\r\n");
			User_Printf("AT+CIPSHUT\r\n");
#endif
			OSTimeDly(2000);
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Init_Done;
#ifdef AIR202
			AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Preinit;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
#endif
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
					g_Printf_dbg("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
					User_Printf("AT+SAPBR=3,1,\"APN\",\"CMNET\"\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 2;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 2){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+SAPBR=5,1\r\n");
					User_Printf("AT+SAPBR=5,1\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 3;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 3){
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+SAPBR =1,1\r\n");
					User_Printf("AT+SAPBR =1,1\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 4;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_GetIP;
				}
			}else if(gprs_tick == 4){         
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_GetIP){
					AppDataPointer->TransMethodData.Http_Cid = 0;
					memset(aRxBuff,0x0,256);
					aRxNum = 0;
					g_Printf_dbg("AT+SAPBR=2,1\r\n");
					User_Printf("AT+SAPBR=2,1\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 5;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 5){         
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
					g_Printf_dbg("AT+HTTPINIT\r\n");
					User_Printf("AT+HTTPINIT\r\n");
					OSTimeDly(1000);
				}else{
					gprs_tick = 6;
					AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				}
			}else if(gprs_tick == 6){          
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
		}
		gprs_tick = 0;
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
	User_Printf("AT+CIPSTART=\"TCP\",\"%s\",\"%d\"\r\n",ip,port);   //建立TCP连接
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
	int timer = 0;
#ifdef SIM800C
    uint32_t datalen = 0;
	datalen = strlen(data);

	User_Printf("AT+CIPSEND\r\n");
	hal_Delay_ms(100);
	hal_Delay_ms(100);

    //Saas*******************************//
	User_Printf("POST %s HTTP/1.1\r\n",path);
	if(apikey != null){
		User_Printf("api-key:%s\r\n",apikey);
	}
	User_Printf("Host:%s\r\n",host);

	User_Printf("Accept:*/*\r\n");
	User_Printf("Content-Length:%d\r\n",datalen);
	User_Printf("Content-Type:text/html\r\n");
	User_Printf("Connection:close\r\n");
	User_Printf("\r\n");

    User_Printf("%s\r\n",data);

	hal_Delay_ms(100);
	OSBsp.Device.Usart0.WriteData(0x1A);      //发送数据完成结束符0x1A,16进制发送
	User_Printf("\r\n");

    OSBsp.Device.Usart2.WriteString(data);

	static char timetick = 0;
	while();
#endif

#ifdef AIR202
	uint32_t datalen = 0;
	int16_t g_err = 0;
	struct hal_timeval now;		
	struct hal_timeval tmp_tv;
	int32_t	sub_timeout_sec;
	Hal_GetTimeOfDay(&now);
	while(g_err == 0){
		if(gprs_tick == 0){      //设置Http会话参数
			if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
				g_Printf_dbg("AT+HTTPPARA=\"URL\",\"%s%s\"\r\n",host,path);
				User_Printf("AT+HTTPPARA=\"URL\",\"%s%s\"\r\n",host,path);
				OSTimeDly(1000);
			}else{
				gprs_tick = 1;
				AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			}
		}else if(gprs_tick == 1){      //
			if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
				datalen = strlen(data);
				g_Printf_dbg("AT+HTTPDATA=%d,100000\r\n",datalen);
				User_Printf("AT+HTTPDATA=%d,100000\r\n",datalen);
				OSTimeDly(1000);
				if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_Download){
					User_Printf("%s\r\n",data);
					gprs_tick = 2;
					OSTimeDly(1000);
				}

				AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			}
		}else if(gprs_tick == 2){      //
			if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
				g_Printf_dbg("AT+HTTPACTION=1\r\n");
				User_Printf("AT+HTTPACTION=1\r\n");
				OSTimeDly(1500);               //注意是否需要增加延时
			}else if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Get_HTTPACT){
				gprs_tick = 3;
				g_Printf_info("%s HTTP STATUS CODE = %d\r\n",__func__,HTTP_Status_Code);
				if((HTTP_Status_Code == 200)&&(g_has_response > 0)){  //？？？？+++++++++++++++++++++++++++++++++++++++++++++++++++++
					g_Printf_dbg("AT+HTTPREAD\r\n");
					User_Printf("AT+HTTPREAD\r\n");
					OSTimeDly(1000);
				}
				AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
			}
		}else if(gprs_tick == 3){
			if(AppDataPointer->TransMethodData.GPRSATStatus == GPRS_Waitfor_OK){
				if(strlen(g_response) > 0){
					memcpy(response,g_response,strlen(g_response));
				}
				g_Printf_dbg("AT+HTTPTERM\r\n");
				User_Printf("AT+HTTPTERM\r\n");    //根据需要看是否需结束Http服务
				OSTimeDly(1000);
			}else{
				gprs_tick = 0;
				// HttpStart = 0;
				AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				AppDataPointer->TransMethodData.GPRSStatus = GPRS_Init_Done;

				g_err = HTTP_Status_Code;
			}
		}

		Hal_GetTimeOfDay(&tmp_tv);
		sub_timeout_sec = tmp_tv.tv_sec - now.tv_sec;
		if(sub_timeout_sec > timeout){
			now.tv_sec = tmp_tv.tv_sec;
			g_Printf_info("%s timerout ",__func__);	
			if((gprs_tick < 2)&&(gprs_tick > 0)){
				g_Printf_info(": para set timerout\r\n");	
				g_err = -1;					
			}else if(gprs_tick == 2){
				g_Printf_info(": failed.http timerout\r\n");
				gprs_tick = 3;
				User_Printf("AT+HTTPTERM\r\n");   //结束Http服务
				OSTimeDly(1000);
				AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
				g_err = -2;
			}
		}
	}

	return g_err;
#endif
}
char data_write[300];
void g_Device_GPRS_Fota_Start(void)
{
	long i = 0;
	long j=0;
	int m;
	// char tempdata[10];
	// long add_temp;
	// char d_t[7524];
	uint8_t Flash_Tmp[3];					//flash操作中间变量
	int length=0;
	if(g_ftp_allow_get == 1){
		g_ftp_allow_get = 0;
		if(data1_len != 0){
			hal_Delay_sec(2);			//添加测试，等待接收完全结束

			
			length = strlen(download_data_1);
			/**************************************************************************/
			#if TestD
//				OS_ENTER_CRITICAL();// Disable interrupts
			// _DINT();
				UCA0IE &= ~UCRXIE; 		//使能串口0中断
				OSTimeDly(50);
				// __disable_interrupt();
				// UCB2CTL1 |= UCSWRST;               		    // Enable SW reset
				// UCB2BR0 = 0;								//6��Ƶ
				// UCB2BR1 = 0;
				// UCB2CTL1 &= ~UCSWRST;

				for(i=0;i<16;i++)                                          //填充数据
				{
					memcpy(&buf1[i*16], buf0,16);
				} 
				

				g_Printf_info("check erase\r\n");

				for(i=FOTA_ADDR_START;i<0xB0000;i++)
				{
					
					OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(i));
					// hal_Delay_ms(1);
				} 
				g_Printf_info("write flash\r\n");   
				// WDTCTL = WDTPW + WDTHOLD;             //CloseWatchDog
				// 				SFRIE1 &= 0;   
				for (i=0;i<768;i++)
				{
					SPI_Flash_Write_Page(buf1, addr_write,256);
					// SPI_Flash_Write_Page(download_data_1+j,addr_write,256);
					// hal_Delay_ms(3);
					addr_write += 256;
					// j += 256;
				}
				// WDTCTL  = WDT_MDLY_32;
				// 					SFRIE1 |= 1;
				g_Printf_info("check write\r\n");
				for(i=FOTA_ADDR_START;i<addr_write;i++)
				{					
					OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(i));
					// hal_Delay_ms(1);
				}
				// __enable_interrupt();
				UCA0IE |= UCRXIE; 		//使能串口0中断
//				OS_EXIT_CRITICAL();  // Enable  interrupts
				// _EINT();
				while(1)
				{
					LED_ON
					hal_Delay_sec(1);
					LED_OFF;
					hal_Delay_sec(1);
				}
				#endif	

				/**************************************************************************/
//			__disable_interrupt();
					// WDTCTL = WDTPW + WDTHOLD;             //CloseWatchDog
   					// SFRIE1 &= 0; 
			UCA0IE &= ~UCRXIE; 		//失能串口0中断，避免中断干扰SPI时序
			// RTCCTL0 &= ~RTCTEVIE;//关闭每分钟中断标志
			OSTimeDly(50);
			j = 1;
			for (i=0;i<4;i++)
			{
				// SPI_Flash_Write_Page(buf1, addr_write,256);
				SPI_Flash_Write_Page(download_data_1+j,addr_write,256);
				// hal_Delay_ms(3);
				addr_write += 256;
				j += 256;
			}
			UCA0IE |= UCRXIE; 		//使能串口0中断
			// RTCCTL0 |= RTCTEVIE;//打开每分钟中断标志
			// hal_Delay_sec(1);
			// g_Printf_info("write Done\r\n");
//			__enable_interrupt();
			// SPI_Flash_Write_Page(download_data_1+1,addr_write,256);
			// for(m=1;m<length-5;m++)
			// {
			// 	// OSBsp.Device.Usart2.WriteData(download_data_1[m]);
			// 	SPI_Flash_Write_Data(download_data_1[m],addr_write++);
			// 	hal_Delay_ms(3);
			// }
		
			
			/****************测试按页写***********************/
			// SPI_Flash_Write_Page(download_data_1+1,addr_write,1024);
			// addr_write += 1024;
			/****************测试按页写***********************/
//			OSTimeDly(100);
			// download_data_1[length-5] = '\0';
			// g_SD_File_Write(codeFile,download_data_1+1);
			// g_SD_File_Write(codeFile,"Hello world\r\n");
			// int addr = 0;
			// char *fm = mymalloc(1050*sizeof(char));
			// memset(fm,0x0,1050);
			// fm = strtok(download_data_1,"OK");
			// g_Printf_info("fm size = %d\r\n",strlen(fm));
			// int lenth = 1025;
			// OSBsp.Device.Usart2.WriteString(fm);
			// for(m=0;m<5;m++){
			// 	if(lenth > 256){
			// 		memset(data_write,0x0,300);
			// 		strncpy(data_write,&download_data_1[addr],256);
			// 		addr += 256;
			// 		g_SD_File_Write("0:/fm1.txt",data_write);
			// 		lenth -= 256;
			// 	}else{
			// 		memset(data_write,0x0,300);
			// 		strncpy(data_write,&download_data_1[addr],lenth);
			// 		g_SD_File_Write("0:/fm1.txt",data_write);
			// 		break;
			// 	}
			// }
			memset(download_data_1,0x0,1536);
			data1_len = 0;
			
			// addr_write = g_MTD_spiflash_writeSector(addr_write,fm,1024);
		}

		// g_Printf_info("AT+FTPGET=2,1024\r\n");	
		User_Printf("AT+FTPGET=2,1024\r\n");	
		
		OSTimeDly(100);
	}else if(g_ftp_allow_get == 2){
		g_ftp_allow_get = 0;

		if(data1_len != 0){
			length = strlen(download_data_1);
			download_data_1[length]  = '\0';		//加入结束符，待测试
			
			UCA0IE &= ~UCRXIE; 		//使能串口0中断
			OSTimeDly(50);
			//  hal_Delay_ms(10);
			for(m=1;m<length-5;m++)
			{
				// OSBsp.Device.Usart2.WriteData(download_data_1[m]);
				SPI_Flash_Write_Data(download_data_1[m],addr_write++);
				// hal_Delay_us(100);
			}
			// hal_Delay_ms(10);
			
			// download_data_1[length-5] = '\0';
			// g_SD_File_Write(codeFile,download_data_1+1);
			
			g_Printf_info("code printf begin:");
			
			for(i=FOTA_ADDR_START;i<addr_write;i++)
			{
				
				OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(i));
				// hal_Delay_ms(1);
			}
			UCA0IE |= UCRXIE; 		//使能串口0中断
			//两次读取打印对比
			// g_Printf_info("\r\ncode printf begin:");
			// for(i=FOTA_ADDR_START;i<addr_write;i++)
			// {
			// 	OSBsp.Device.Usart2.WriteData(SPI_Flash_ReadByte(i));
			// }	
			
		//	tempdata = Read_Byte(FOTA_ADDR_START+1);
			g_Printf_info("code head:");
			// OSBsp.Device.Usart2.WriteData(tempdata[0]);
			//判断存储数据头尾是否正确 然后配置启动标志位存放于infor_BootAddr
			if(SPI_Flash_ReadByte(addr_write-1) == 'q' && SPI_Flash_ReadByte(FOTA_ADDR_START+1) == 'c')	//确认C400和q
			{	        
				g_Printf_info("Enter %s and System will goto bootloader\r\n",__func__);
				loop8:
					Flash_Tmp[0] = 0x02; 		//置位Flash 标志位	//把infor_BootAddr写0x02，建立FOTA升级标志位
					OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp, 1, infor_BootAddr, 0);
					hal_Delay_ms(10);
					if(OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_BootAddr) == 0x02)
						hal_Reboot();			//重启MCU
					else
						goto loop8;	
			}
			// hal_Reboot();
			// char *fm;
			// int addr = 0;
			// fm = strtok(download_data_1,"OK");
			// strncpy(data_write,fm,strlen(download_data_1)-3);
			// int lenth = strlen(download_data_1)-3;
			// OSBsp.Device.Usart2.WriteString(fm);
			// for(m=0;m<4;m++){
			// 	if(lenth > 256){
			// 		memset(data_write,0x0,300);
			// 		strncpy(data_write,&fm[addr],256);
			// 		addr += 256;
			// 		g_SD_File_Write("0:/fm1.txt",data_write);
			// 		lenth -= 256;
			// 	}else{
			// 		memset(data_write,0x0,300);
			// 		strncpy(data_write,&fm[addr],lenth);
			// 		g_SD_File_Write("0:/fm1.txt",data_write);
			// 		break;
			// 	}
			// }
			// OSBsp.Device.Usart2.WriteString(fm);
			// addr_write = g_MTD_spiflash_writeSector(addr_write,fm,strlen(download_data_1)-3);
			// g_Printf_info("==>> end addr 0x%04x\r\n",addr_write);
			memset(download_data_1,0x0,1536);
			data1_len = 0;
		}
		
		OSTimeDly(100);
	}
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

			
			// //擦除SPI
			W25Q16_CS_HIGH();
			OSTimeDly(50);
			Base_3V3_ON;
			// P4OUT |= BIT0;
			W25Q16_Init();
			readAddr = FOTA_ADDR_START;
			for (i=0;i<6;i++)
			{
				SPI_Flash_Erase_Block(readAddr);
				readAddr += 0x10000;
			}
			g_Printf_info("Erase SPI_Flash\r\n");
			//擦除结束
			g_Printf_info("AT+FTPGET=1\r\n");
			User_Printf("AT+FTPGET=1\r\n");
			g_Printf_info("start download ==");
			OSTimeDly(1000);
		}else{
			gprs_tick = 13;
			AppDataPointer->TransMethodData.GPRSATStatus = GPRS_Waitfor_OK;
		}
	}	
	
}

void g_Device_check_Response(char *res)
{
	static uint8_t res_len = 0;
	if(g_has_response == -1){
		g_has_response = 0;
		memset(g_response,0x0,256);
		strcpy(g_response,res);
	}
#ifdef SIM800C
	if(Hal_CheckString(response,"CONNECT OK")){
		g_Printf_dbg("Connect OK\r\n");
		AppDataPointer->TransMethodData.GPRSConnect = 1;
	}else if(Hal_CheckString(response,"SEND OK")){
		// App.Data.GprsSendStatus = 1;
		g_Printf_dbg("Send OK\r\n");
	} else if(Hal_CheckString(response,"CLOSED")){
		g_Printf_dbg("Web Closed\r\n");
		// AllowSend=0;
		// GPRSReset;
		// Ctr_GPRS_OFF;  //鍏抽棴GPRS
	}else if(Hal_CheckString(response,"+CSQ:")){
		if (Hal_CheckString(response,"0,0")){
			g_Printf_dbg("CSQ Faild!\r\n");
			AppDataPointer->TransMethodData.GPRSNet = 0;
		}else{
			g_Printf_dbg("CSQ OK!\r\n");
			AppDataPointer->TransMethodData.GPRSNet = 1;
			gprs_tick = 0;
		}
	}else if(Hal_CheckString(response,"+CGATT:")){
		if (Hal_CheckString(response,"0")){
			g_Printf_dbg("Attached Faild!\r\n");
			AppDataPointer->TransMethodData.GPRSAttached = 0;
		}else{
			g_Printf_dbg("Attached OK!\r\n");
			AppDataPointer->TransMethodData.GPRSAttached = 1;
			gprs_tick = 0;
		}
	}
#endif

#ifdef AIR202
	if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Power_on){
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
				memset(CSQBuffer, '\0', 15);	//清空buffer
				StartString = strstr(aRxBuff,"+CSQ:");
				EndString  = strstr(aRxBuff, ",");
				memcpy(CSQBuffer, StartString + 6, EndString - StartString - 6);	//复制CSQ值
				AppDataPointer->TransMethodData.GPRSNet = 1;
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
		}else if(Hal_CheckString(res,"+FTPGET: 2")){
			g_ftp_allow_storage = 1;
		}else if(Hal_CheckString(res,"+FTPGET: 1,0")){
			g_ftp_allow_get = 2;
		}
	}
#endif
}

void  TransmitTaskStart (void *p_arg)
{
	long i = 0;
	uint32_t datalen = 0;
//	uint8_t i=0;
	uint8_t scadaADCIndex = 0,scadaBATIndex = 0;
    (void)p_arg;   
    OSTimeDlyHMSM(0u, 0u, 0u, 100u);      
    g_Printf_info("%s ... ...\n",__func__);      
	for(i=0;i<16;i++)                                          //填充数据
	{
	   memcpy(&buf1[i*16], buf0,16);
	}     
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.       */
        if(Hal_getCurrent_work_Mode() == 0){
            if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Power_off){
                OSBsp.Device.IOControl.PowerSet(BaseBoard_Power_On);
                OSBsp.Device.IOControl.PowerSet(SIM800C_Power_On);
                OSTimeDly(500); 
                OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
				OSBsp.Device.IOControl.PowerSet(LPModule_Power_On);
                AppDataPointer->TransMethodData.GPRSStatus = GPRS_Power_on;
                //上电后延时一段时间
                OSTimeDly(2500);
                g_Printf_dbg("AT\r\n");
                User_Printf("AT\r\n");           //AT同步，与主机端同步波特率
                OSTimeDly(1000);
                g_Printf_dbg("ATE0\r\n");
                User_Printf("ATE0\r\n");         //关闭回显，初始化阶段关闭
                OSTimeDly(1500);
                AppDataPointer->TransMethodData.GPRSStatus = GPRS_Preinit;
            }else if((AppDataPointer->TransMethodData.GPRSStatus >= GPRS_Power_on)&&
                    	(AppDataPointer->TransMethodData.GPRSStatus < GPRS_Http_Init_Done)){
                g_Device_GPRS_Init();
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Init_Done){
                if(AppDataPointer->TerminalInfoData.DeviceStatus == DEVICE_STATUS_POWER_SCAN_OVER){



					if (App.Data.TerminalInfoData.DeviceFirstRunStatus == DEVICE_STATUS_FIRSTRUN_BEGIN) {
						App.Data.TerminalInfoData.DeviceFirstRunStatus = DEVICE_STATUS_FIRSTRUN_OVER;
						App.Data.TransMethodData.SeqNumber = 0;
					}else {
						App.Data.TransMethodData.SeqNumber++;
					}
					//************电量处理Begin************//
	                GetADCValue();
                    //************电量处理End*************//

					char response[256];
					char data[512];
					uint32_t datalen = snprintf(data,512,MakeJsonBodyData(AppDataPointer));
					g_Printf_info("datalen:%d\ndata:%s\r\n",datalen,data);
				
                   
                	memset(response,0x0,128);
					int16_t code = 0;
                    code = g_Device_http_post(g_30000IoT_HOST,g_30000IoT_PATH,null,data,response,20);//时间延长至20s
                    if(code == 200){
                        g_Printf_info("response : %s \r\n",response);   //对response解析，可以执行配置或ota操作

						cJSON *payload;
						cJSON *pMsg = NULL;
						cJSON *pVersion;
						payload = cJSON_Parse(response);
						// pVersion = cJSON_GetObjectItem(payload,"Version");
						// if(pVersion->valueint != AppDataPointer->TerminalInfoData.Version){
						pMsg = cJSON_GetObjectItem(payload,"msg");
						if(pMsg != NULL){
							// cJSON *pIP = NULL;
							cJSON *pPort = NULL;
							// cJSON *pUsername = NULL;
							// cJSON *pPassword = NULL;
							// cJSON *pImgpath = NULL;
							// cJSON *pImgName = NULL;
							// pIP = cJSON_GetObjectItem(payload,"ip");
							// pPort = cJSON_GetObjectItem(payload,"port");
							// pUsername = cJSON_GetObjectItem(payload,"username");
							// pPassword = cJSON_GetObjectItem(payload,"password");
							// pImgpath = cJSON_GetObjectItem(payload,"imgpath");
							// pImgName = cJSON_GetObjectItem(payload,"imgname");

							// if(pIP != NULL){
							// 	strcpy(AppDataPointer->FotaInfor.ip,pIP->valuestring);
							//  AppDataPointer->FotaInfor.port = pPort->valueint;
							// 	strcpy(AppDataPointer->FotaInfor.username,pUsername->valuestring);
							// 	strcpy(AppDataPointer->FotaInfor.password,pPassword->valuestring);
							// 	strcpy(AppDataPointer->FotaInfor.imgpath,pImgpath->valuestring);
							// 	strcpy(AppDataPointer->FotaInfor.imgname,pImgName->valuestring);

							// 	AppDataPointer->TransMethodData.GPRSStatus = GPRS_Fota_Process;
							// }

							strcpy(AppDataPointer->FotaInfor.ip,"114.55.93.183");
							AppDataPointer->FotaInfor.port = 21;
							strcpy(AppDataPointer->FotaInfor.username,"sanwanwulianw");
							strcpy(AppDataPointer->FotaInfor.password,"e1KTMWXeujEEGr8iQpk");
							strcpy(AppDataPointer->FotaInfor.imgpath,"/sanwanwulianw/web/");
							strcpy(AppDataPointer->FotaInfor.imgname,"dh_msp430_ucOS_II.txt");

							AppDataPointer->TransMethodData.GPRSStatus = GPRS_Fota_Process;

							g_Printf_info("FotaInfor.ip:%s\r\n",AppDataPointer->FotaInfor.ip);
							g_Printf_info("FotaInfor.port:%d\r\n",AppDataPointer->FotaInfor.port);
							g_Printf_info("FotaInfor.username:%s\r\n",AppDataPointer->FotaInfor.username);
							g_Printf_info("FotaInfor.password:%s\r\n",AppDataPointer->FotaInfor.password);
							g_Printf_info("FotaInfor.imgpath:%s\r\n",AppDataPointer->FotaInfor.imgpath);
							g_Printf_info("FotaInfor.imgname:%s\r\n",AppDataPointer->FotaInfor.imgname);

							//擦除SPI
							W25Q16_CS_HIGH();
							OSTimeDly(50);
							Base_3V3_ON;
							// P4OUT |= BIT0;
							W25Q16_Init();
							readAddr = FOTA_ADDR_START;
							for (i=0;i<6;i++)
							{
								SPI_Flash_Erase_Block(readAddr);
								readAddr += 0x10000;
							}
							g_Printf_info("Erase SPI_Flash\r\n");
							
						}else{
							 AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;
						}
                    }else{    //这里可以做失败重发操作
					    AppDataPointer->TransMethodData.GPRSStatus = GPRS_Http_Post_Done;  //ML 20190828
                        g_Printf_dbg("http_post failed\r\n");
                    }                        
                }    
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Fota_Process){
								
				g_Device_GPRS_Fota_Start();
            }else if(AppDataPointer->TransMethodData.GPRSStatus == GPRS_Http_Post_Done){
                // OSBsp.Device.IOControl.PowerSet(AIR202_Power_On);
                Hal_EnterLowPower_Mode();
            }
            OSTimeDlyHMSM(0u, 0u, 0u, 200u);  
        }
    }
}

#endif //(TRANSMIT_TYPE == GPRS_Mode)

