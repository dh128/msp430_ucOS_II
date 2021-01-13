
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
* Filename      : g_DeviceConfig.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <bsp.h>

char *Uart0_RxBuff;     //+++++++++++++//
unsigned int  Uart0_RxBuff_Num=0;
unsigned char Uart0_RxBuff_data[1024];	
unsigned char TimebuffNum=0;
unsigned char TimeBuff_Hex[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //16杩涘埗鐨勬椂闂碆uffer  2018骞�3鏈�15鍙� 20鏃�50鍒�00绉� 鏄熸湡4

static OS_STK   UartRecTaskStartStk[MINIMUM_TASK_STK_SIZE];
Queue_t  g_ConfigQueue;
void *QConfiMsgTb[QConfigMsgTb_Size];
struct hal_message ConfigMsg;


static void g_Device_WirelessUpload_Config(g_Device_Config_CMD uploadCmd)   //通信模块无线指令下发配置(串口1)
{
	if(uploadCmd.cmdLenth != 0)
	{
#if (TRANSMIT_TYPE == GPRS_Mode)
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)
#elif (TRANSMIT_TYPE == LoRa_F8L10D_Mode)
		if(Hal_CheckString(uploadCmd.strcmd,"+RCV") & Hal_CheckString(uploadCmd.strcmd,"FF") & Hal_CheckString(uploadCmd.strcmd,"AA"))
		{
			char *Uart0_RxBuff;     						 //+++++++++++++//
			char Uart0_RxBuff_data[50];
			uint8_t Uart0_RxBuff_Num = 0;
			if(Hal_CheckString(uploadCmd.strcmd,"FF0102")) //修改上报周期
			{

				Uart0_RxBuff = strstr(uploadCmd.strcmd,"FF0102");         //判断接收到的数据是否有效
				while(*(Uart0_RxBuff+6) != ',')
				{
					Uart0_RxBuff_data[Uart0_RxBuff_Num] = *(Uart0_RxBuff+6);
					Uart0_RxBuff_Num++;
					Uart0_RxBuff++;
				}
				uint16_t Temp_SendPeriod = (Uart0_RxBuff_data[0]-0x30)*1000 + (Uart0_RxBuff_data[1]-0x30)*100
									+ (Uart0_RxBuff_data[2]-0x30)*10 + (Uart0_RxBuff_data[3]-0x30)*1;
				if( (Temp_SendPeriod >= 5) && (Temp_SendPeriod <= 240) )
				{
					uint8_t Flash_Tmp[14];  //flash操作中间变量
					App.Data.TerminalInfoData.SendPeriod = (uint8_t)(Temp_SendPeriod & 0x00FF);
					OSBsp.Device.Usart2.WriteString("LoRa Set SendPeriod OK\r\n");
					//将发送周期的信息存入Flash
					Flash_Tmp[0] = OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_ChargeAddr);
					Flash_Tmp[1] = OSBsp.Device.InnerFlash.innerFLASHRead(1, infor_ChargeAddr);
					Flash_Tmp[2] = OSBsp.Device.InnerFlash.innerFLASHRead(2, infor_ChargeAddr);
					Flash_Tmp[3] = OSBsp.Device.InnerFlash.innerFLASHRead(3, infor_ChargeAddr);
					Flash_Tmp[4] = OSBsp.Device.InnerFlash.innerFLASHRead(4, infor_ChargeAddr);
					Flash_Tmp[5] = OSBsp.Device.InnerFlash.innerFLASHRead(5, infor_ChargeAddr);
					Flash_Tmp[6] = OSBsp.Device.InnerFlash.innerFLASHRead(6, infor_ChargeAddr);
					Flash_Tmp[7] = OSBsp.Device.InnerFlash.innerFLASHRead(7, infor_ChargeAddr);//终端类型
					Flash_Tmp[8] = OSBsp.Device.InnerFlash.innerFLASHRead(8, infor_ChargeAddr);//传输方式
					Flash_Tmp[9] = OSBsp.Device.InnerFlash.innerFLASHRead(9, infor_ChargeAddr);//DevEUI_H(高八位)
					Flash_Tmp[10] = OSBsp.Device.InnerFlash.innerFLASHRead(10, infor_ChargeAddr);//DevEUI_L(低八位)
					Flash_Tmp[11] = App.Data.TerminalInfoData.SendPeriod;//上传周期（min）
					OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp, 12, infor_ChargeAddr, 0);//把终端信息写入FLASH
	//							//*******返回ACK成功 字符串数组 BEGIN*******//
	////										delay_ms(10);
	//							User_Printf("AT+TXH=15,AA0101FF\r\n");	//TXH 发送16进制数，15代表通道号，TXA 发送字符串
	//							//*******返回ACK成功 字符串数组  END*********//
				}else{
					OSBsp.Device.Usart2.WriteString("LoRa Set SendPeriod Failed！\r\n");
				}
			}
			if(Hal_CheckString(uploadCmd.strcmd,"FF0208")) //同步设备时间
			{
				uint8_t TimeBuff_Hex[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //16进制的时间Buffer  2018年3月15号 20时50分00秒 星期4
				OSBsp.Device.Usart2.WriteString("Time Set Done!\r\n");
				Uart0_RxBuff = strstr(aRxBuploadCmd.strcmduff,"FF0208");         //判断接收到的数据是否有效
				while(*(Uart0_RxBuff+6) != ',')
				{
					Uart0_RxBuff_data[Uart0_RxBuff_Num] = *(Uart0_RxBuff+6);
					Uart0_RxBuff_Num++;
					Uart0_RxBuff++;
				}
				uint8_t TimebuffNum=0;
				uint8_t time_buf[8];
				for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++){
					TimeBuff_Hex[TimebuffNum] = (Uart0_RxBuff_data[TimebuffNum*2]-0x30)*10 + (Uart0_RxBuff_data[TimebuffNum*2+1]-0x30)*1;
				}
				if( (TimeBuff_Hex[0]==20) && (TimeBuff_Hex[1]>=18) && (TimeBuff_Hex[2]>=1) && (TimeBuff_Hex[2]<=12) && (TimeBuff_Hex[3]>=1) && (TimeBuff_Hex[3]<=31)
					&& (TimeBuff_Hex[4]<24)  && (TimeBuff_Hex[5]<60) && (TimeBuff_Hex[6]<60) && (TimeBuff_Hex[7]>=1) && (TimeBuff_Hex[7]<=7) )
				{
					for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++)
					{
						time_buf[TimebuffNum]= HexToBCD(TimeBuff_Hex[TimebuffNum]);    //存“年月日时分秒周”
					}
					OSBsp.Device.RTC.ConfigExtTime(time_buf,RealTime);   //写入时间
					OSBsp.Device.Usart2.WriteString("LoRa Time Set Done\r\n");
				}else{
					OSBsp.Device.Usart2.WriteString("LoRa Time Set Failed！\r\n");
				}

			}
			if(Hal_CheckString(uploadCmd.strcmd,"FF0301")) //复位设备
			{
				OSBsp.Device.Usart2.WriteString("LoRa Reset Device OK!\r\n");
				hal_Delay_ms(100);hal_Delay_ms(100);hal_Delay_ms(100);
				hal_Reboot(); //******软件复位*******//
			}
		}
#elif (TRANSMIT_TYPE == NBIoT_BC95_Mode)
		if(Hal_CheckString(uploadCmd.strcmd,"+NNMI") & Hal_CheckString(uploadCmd.strcmd,"FF") & Hal_CheckString(uploadCmd.strcmd,"AA"))
		{
			if(Hal_CheckString(uploadCmd.strcmd,"FF0102")) //修改上报周期
			{
				Uart0_RxBuff = strstr(aRxBuff,"FF0102");         //判断接收到的数据是否有效
				while(*(Uart0_RxBuff+6) != 0x0A)
				{
					Uart0_RxBuff_data[Uart0_RxBuff_Num] = *(Uart0_RxBuff+6);
					Uart0_RxBuff_Num++;
					Uart0_RxBuff++;
				}
				uint16_t Temp_SendPeriod = (Uart0_RxBuff_data[0]-0x30)*1000 + (Uart0_RxBuff_data[1]-0x30)*100
									+ (Uart0_RxBuff_data[2]-0x30)*10 + (Uart0_RxBuff_data[3]-0x30)*1;
				if( (Temp_SendPeriod >= 5) && (Temp_SendPeriod <= 240) )
				{
					uint8_t Flash_Tmp[14];  //flash操作中间变量
					App.Data.TerminalInfoData.SendPeriod = (uint8_t)(Temp_SendPeriod & 0x00FF);
					OSBsp.Device.Usart2.WriteString("NB Set SendPeriod OK\r\n");
					//将发送周期的信息存入Flash
//					Flash_Tmp[0] = OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_ChargeAddr);
//					Flash_Tmp[1] = OSBsp.Device.InnerFlash.innerFLASHRead(1, infor_ChargeAddr);
//					Flash_Tmp[2] = OSBsp.Device.InnerFlash.innerFLASHRead(2, infor_ChargeAddr);
//					Flash_Tmp[3] = OSBsp.Device.InnerFlash.innerFLASHRead(3, infor_ChargeAddr);
//					Flash_Tmp[4] = OSBsp.Device.InnerFlash.innerFLASHRead(4, infor_ChargeAddr);
//					Flash_Tmp[5] = OSBsp.Device.InnerFlash.innerFLASHRead(5, infor_ChargeAddr);
//					Flash_Tmp[6] = OSBsp.Device.InnerFlash.innerFLASHRead(6, infor_ChargeAddr);
//					Flash_Tmp[7] = OSBsp.Device.InnerFlash.innerFLASHRead(7, infor_ChargeAddr);//终端类型
//					Flash_Tmp[8] = OSBsp.Device.InnerFlash.innerFLASHRead(8, infor_ChargeAddr);//传输方式
//					Flash_Tmp[9] = OSBsp.Device.InnerFlash.innerFLASHRead(9, infor_ChargeAddr);//DevEUI_H(高八位)
//					Flash_Tmp[10] = OSBsp.Device.InnerFlash.innerFLASHRead(10, infor_ChargeAddr);//DevEUI_L(低八位)
					Flash_Tmp[11] = App.Data.TerminalInfoData.SendPeriod;//上传周期（min）
					OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp[11], 1, infor_ChargeAddr, 11);
				}else{
					OSBsp.Device.Usart2.WriteString("NB Set SendPeriod Failed!\r\n");
				}
			}
			if(Hal_CheckString(uploadCmd.strcmd,"FF0208")) //同步设备时间
			{
				OSBsp.Device.Usart2.WriteString("Time Set Done!\r\n");
				Uart0_RxBuff = strstr(uploadCmd.strcmd,"FF0208");         //判断接收到的数据是否有效
				while(*(Uart0_RxBuff+6) != 0x0A)
				{
					Uart0_RxBuff_data[Uart0_RxBuff_Num] = *(Uart0_RxBuff+6);
					Uart0_RxBuff_Num++;
					Uart0_RxBuff++;
				}

				uint8_t TimebuffNum;
				uint8_t time_buf[8];
				for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++)
				{
					TimeBuff_Hex[TimebuffNum] = (Uart0_RxBuff_data[TimebuffNum*2]-0x30)*10 + (Uart0_RxBuff_data[TimebuffNum*2+1]-0x30)*1;
				}
				if( (TimeBuff_Hex[0]==20) && (TimeBuff_Hex[1]>=18) && (TimeBuff_Hex[2]>=1) && (TimeBuff_Hex[2]<=12) && (TimeBuff_Hex[3]>=1) && (TimeBuff_Hex[3]<=31)
					&& (TimeBuff_Hex[4]<24)  && (TimeBuff_Hex[5]<60) && (TimeBuff_Hex[6]<60) && (TimeBuff_Hex[7]>=1) && (TimeBuff_Hex[7]<=7) )
				{
					for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++)
					{
						time_buf[TimebuffNum]= HexToBCD(TimeBuff_Hex[TimebuffNum]);    //存“年月日时分秒周”
					}
					OSBsp.Device.RTC.ConfigExtTime(time_buf,RealTime);   //写入时间
					Write_info_RTC(time_buf);		//同步写入MCU内部RTC
					OSBsp.Device.Usart2.WriteString("NB Time Set Done\r\n");
				}else{
					OSBsp.Device.Usart2.WriteString("NB Time Set Failed！\r\n");
				}

			}
			if(Hal_CheckString(uploadCmd.strcmd,"FF0301")) //复位设备
			{
				OSBsp.Device.Usart2.WriteString("NB Reset Device OK!\r\n");
				hal_Delay_ms(100);hal_Delay_ms(100);hal_Delay_ms(100);
				hal_Reboot(); //******软件复位*******//
			}
		}
#endif
	}
}

// #if ((ACCESSORY_TYPR == RS485_Mode)||(ACCESSORY_TYPR == RS232_Mode))
// static void g_Device_WiredUpload_Config(g_Device_Config_CMD uploadCmd)  //485/232有线指令下发配置(串口1)
// {
// 	if(uploadCmd.cmdLenth != 0)
// 	{
// 		if( (uploadCmd.hexcmd[0] == 'F')&&(bRxBuff[1] == 'F')&&
// 				(uploadCmd.hexcmd[uploadCmd.cmdLenth-2] == 'A')&&(uploadCmd.hexcmd[uploadCmd.cmdLenth-1] == 'A') ) //判断包头包尾
// 		{
// 			if((uploadCmd.hexcmd[2] == '0')&&(uploadCmd.hexcmd[3] == '1')
// 					&&(uploadCmd.hexcmd[4] == '0')&&(uploadCmd.hexcmd[5] == '2')) //更改上传周期
// 			{
// 				uint16_t Temp_SendPeriod = (uploadCmd.hexcmd[6]-0x30)*1000 + (uploadCmd.hexcmd[7]-0x30)*100 + (uploadCmd.hexcmd[8]-0x30)*10 + (uploadCmd.hexcmd[9]-0x30)*1;
// 				if( (Temp_SendPeriod >= 5) && (Temp_SendPeriod <= 240) ){
// 					uint8_t Flash_Tmp[14];  //flash操作中间变量
// 					App.Data.TerminalInfoData.SendPeriod = (uint8_t)(Temp_SendPeriod & 0x00FF);
// 					OSBsp.Device.Usart2.WriteString("RS485/RS232 Set SendPeriod OK\r\n");
// 					//将发送周期的信息存入Flash
// 					Flash_Tmp[0] = OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_ChargeAddr);
// 					Flash_Tmp[1] = OSBsp.Device.InnerFlash.innerFLASHRead(1, infor_ChargeAddr);
// 					Flash_Tmp[2] = OSBsp.Device.InnerFlash.innerFLASHRead(2, infor_ChargeAddr);
// 					Flash_Tmp[3] = OSBsp.Device.InnerFlash.innerFLASHRead(3, infor_ChargeAddr);
// 					Flash_Tmp[4] = OSBsp.Device.InnerFlash.innerFLASHRead(4, infor_ChargeAddr);
// 					Flash_Tmp[5] = OSBsp.Device.InnerFlash.innerFLASHRead(5, infor_ChargeAddr);
// 					Flash_Tmp[6] = OSBsp.Device.InnerFlash.innerFLASHRead(6, infor_ChargeAddr);
// 					Flash_Tmp[7] = OSBsp.Device.InnerFlash.innerFLASHRead(7, infor_ChargeAddr);		//终端类型
// 					Flash_Tmp[8] = OSBsp.Device.InnerFlash.innerFLASHRead(8, infor_ChargeAddr);		//传输方式
// 					Flash_Tmp[9] = OSBsp.Device.InnerFlash.innerFLASHRead(9, infor_ChargeAddr);		//DevEUI_H(高八位)
// 					Flash_Tmp[10] = OSBsp.Device.InnerFlash.innerFLASHRead(10, infor_ChargeAddr);	//DevEUI_L(低八位)
// 					Flash_Tmp[11] = App.Data.TerminalInfoData.SendPeriod;							//上传周期（min）
// 					OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp, 12, infor_ChargeAddr, 0);		//把终端信息写入FLASH
// 				}else{
// 					OSBsp.Device.Usart2.WriteString("RS485/RS232 Set SendPeriod Failed！\r\n");
// 				}
// 			}
// 			if((uploadCmd.hexcmd[2] == '0')&&(uploadCmd.hexcmd[3] == '2')&&
// 					(uploadCmd.hexcmd[4] == '0')&&(uploadCmd.hexcmd[5] == '8')) //同步时钟
// 			{
// 				uint8_t TimebuffNum=0;
// 				uint8_t time_buf[8];
// 				uint8_t TimeBuff_Hex[8] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}; //16进制的时间Buffer  2018年3月15号 20时50分00秒 星期4
// 				for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++){
// 					TimeBuff_Hex[TimebuffNum] = (uploadCmd.hexcmd[6+TimebuffNum*2]-0x30)*10 + (uploadCmd.hexcmd[7+TimebuffNum*2]-0x30)*1;
// 				}
// 				if( (TimeBuff_Hex[0]==20) && (TimeBuff_Hex[1]>=18) && (TimeBuff_Hex[2]>=1) && (TimeBuff_Hex[2]<=12) && (TimeBuff_Hex[3]>=1) && (TimeBuff_Hex[3]<=31)
// 				    && (TimeBuff_Hex[4]<24)  && (TimeBuff_Hex[5]<60) && (TimeBuff_Hex[6]<60) && (TimeBuff_Hex[7]>=1) && (TimeBuff_Hex[7]<=7) )
// 				{
// 					for(TimebuffNum=0;TimebuffNum<8;TimebuffNum++){
// 						time_buf[TimebuffNum]= HexToBCD(TimeBuff_Hex[TimebuffNum]);    //存“年月日时分秒周”
// 					}
// 					OSBsp.Device.RTC.ConfigExtTime(time_buf,RealTime);   //写入时间
// 					OSBsp.Device.Usart2.WriteString("RS485/RS232 Time Set Done\r\n");
// 				}else{
// 					OSBsp.Device.Usart2.WriteString("RS485/RS232 Time Set Failed！\r\n");
// 				}
// 			}
// 			if((uploadCmd.hexcmd[2] == '0')&&(uploadCmd.hexcmd[3] == '3')&&
// 					(uploadCmd.hexcmd[4] == '0')&&(uploadCmd.hexcmd[5] == '1')) //复位
// 			{
// 				if((uploadCmd.hexcmd[6] == '0')&&(uploadCmd.hexcmd[7] == '1')) //复位
// 				{
// 					OSBsp.Device.Usart2.WriteString("RS485/RS232 Reset Device OK！\r\n");
// 					hal_Delay_ms(50);
// 					hal_Reboot(); //******软件复位*******//
// 				}
// 			}
// 		}else{
// 			OSBsp.Device.Usart2.WriteString("RS485/RS232 Wrong Command！\r\n");
// 		}
// 	}
// }
// #endif


#if (ACCESSORY_TYPR == GPS_Mode)
static void g_Device_GPS_Config(g_Device_Config_CMD uploadCmd)
{
	uint8_t i=0,ii=0;
	float latTemp,lngTemp;
    char *a,*c;
	uint8_t GPSLng_data[20];      //longitude  //经度
	uint8_t GPSLat_data[20];      //latitude   //纬度
	static char GPSPowerOffNum = 0;

	if(uploadCmd.cmdLenth != 0)
	{
		a=strstr(uploadCmd.strcmd,",A,");         //判断接收到的数据是否有效
        //strstr(str1,str2) 函数用于判断字符串str2是否是str1的子串;如果是，则该函数返回str2在str1中首次出现的地址；否则，返回NULL。
		if(a!=null){
			// b=strstr(uploadCmd.strcmd,"$GNGLL,");  //接收到的“纬度”数据有效
			while(*(a+3) != ',')
			{
				GPSLat_data[i] = *(a+3);        //3200.6362
				i++;
				a++;
			}
			c=strstr(uploadCmd.strcmd,",N,");      //接收到的“经度”数据有效
			while(*(c+3) != ',')
			{
				GPSLng_data[ii] = *(c+3);       //11846.8892
				ii++;
				c++;
			}
			//********清0上组GPS定位信息*********//
			AppDataPointer->TransMethodData.GPSLat_Point = 0; //纬度
			AppDataPointer->TransMethodData.GPSLng_Point = 0; //经度
		//将GPS得到的“WGS-84数据”转换“GCJ-02”
		//公式：abcde.fghi————abc+(de/60)+(fghi/600000)
		    //*****纬度    3200.6362,N*****//
		latTemp = atof(GPSLat_data);
		lngTemp = atof(GPSLng_data);
		AppDataPointer->TransMethodData.GPSLat_Point = (int)(latTemp/100) + (latTemp/100.0 - (int)(latTemp/100)) *100.0 / 60.0;
    	AppDataPointer->TransMethodData.GPSLng_Point = (int)(lngTemp/100) + (lngTemp/100.0 - (int)(lngTemp/100)) *100.0 / 60.0;
		g_Printf_dbg("lat:%lf,lng:%lf\r\n",AppDataPointer->TransMethodData.GPSLat_Point,AppDataPointer->TransMethodData.GPSLng_Point);
		g_Printf_dbg("GPS positioning ok\r\n"); //串口打印定位成功信息
			////关闭GPS_3V3 电源
		GPSPowerOffNum++;
		if(GPSPowerOffNum>=3)       //设置GPS定位15s一次，成功定位3次后停止
		{
			GPSPowerOffNum = 0;
			OSBsp.Device.IOControl.PowerSet(GPS_Power_Off);
//			Flag_GPSPositionOK = 1;
		}
		}else{
		    g_Printf_dbg("GPS positioning failed!\r\n"); //串口打印定位失败信息
			////关闭GPS_3V3 电源
// 			GPSPowerOffNum++;
// 			if(GPSPowerOffNum>=8)       //GPS定位15s一次，成功失败10次后停止
// 			{
// 				GPSPowerOffNum = 0;
// 				OSBsp.Device.IOControl.PowerSet(GPS_Power_Off);
// //				Flag_GPSPositionOK = 0;
// 			}
		}

	}
}
#endif


static int FirmCMD_Receive(uint8_t *RxBuff, uint8_t RxNum)
{
	uint8_t Flash_Tmp[12];//flash操作中间变量
	uint8_t f_tmp[18];
//	 uint8_t Send_Tmp[35];
	//uint32_t Send_Tmp[34];
//	uint8_t Send_Tmp_String[70];
	int i;
	__disable_interrupt();
	if((RxNum > 10)&&(RxNum == (RxBuff[2]+4))&&(RxBuff[RxNum-1]==0x0D)){
		if(RxBuff[1] == 0x01){
			OSBsp.Device.InnerFlash.FlashRsvWrite(&RxBuff[2], RxBuff[2]+1, Infor_AliAddr, ProductKey_Addr);//把终端信息写入FLASH
		}else if(RxBuff[1] == 0x02){
			OSBsp.Device.InnerFlash.FlashRsvWrite(&RxBuff[2], RxBuff[2]+1, Infor_AliAddr, DeviceName_Addr);//把终端信息写入FLASH
		}else if(RxBuff[1] == 0x03){
			OSBsp.Device.InnerFlash.FlashRsvWrite(&RxBuff[2], RxBuff[2]+1, Infor_AliAddr, DeviceSecret_Addr);//把终端信息写入FLASH
		}
		g_Printf_info("Enter %s Set OK\r\n",__func__);
		return 0;
	}else if((RxNum==12)&& (RxBuff[0]==0x0D) && (RxBuff[RxNum-1]==0x0D)){
		for(i=0;i<RxNum;i++)
			g_Printf_info("0x%x ",RxBuff[i]);
		hal_Delay_ms(10);
		if(RxBuff[1] == 0xEF){	        //固件升级请求指令 存放于infor_BootAddr
			g_Printf_info("Enter %s and System will goto bootloader\r\n",__func__);
			loop5:
				Flash_Tmp[0] = 0x01;
				OSBsp.Device.InnerFlash.FlashRsvWrite(Flash_Tmp, 1, infor_BootAddr, 0);//把infor_BootAddr写0x01，建立固件升级标志位
				hal_Delay_ms(10);
				if(OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_BootAddr) == 0x01)
					hal_Reboot();
				else
					goto loop5;
			return 0;	
		}else if(RxBuff[1] == 0xF0){		//终端信息查询指令
			g_Printf_info("Enter %s and Check device Info now\r\n",__func__);
			OSTimeDly(100); 
			f_tmp[0] = 0xFF;
			f_tmp[1] = OSBsp.Device.InnerFlash.innerFLASHRead(1,infor_BootAddr);
			for(i = 0; i < 13; i ++)
			{
				f_tmp[i+2] = OSBsp.Device.InnerFlash.innerFLASHRead(i,infor_ChargeAddr);
			}
			f_tmp[15] = OSBsp.Device.InnerFlash.innerFLASHRead(20,infor_ChargeAddr);
			f_tmp[16] = OSBsp.Device.InnerFlash.innerFLASHRead(23,infor_ChargeAddr);
			f_tmp[17]=0xFF;
			OSBsp.Device.Usart2.WriteNData(f_tmp,18);
			return 0;
		}
		else if(RxBuff[1] == 0xF5){	    //时钟同步
			uint8_t time_buf[8];
			for(i = 1; i < 8; i++){
				time_buf[i]=RxBuff[i+1];	//存“年月日时分秒周”
			}
			OSBsp.Device.RTC.ConfigExtTime(time_buf,RealTime);
			g_Printf_info("Enter %s and Time config done\r\n",__func__);
			return 0;
		}else if(RxBuff[1] == 0xFA){		//设置终端信息
			loop6:
				g_Printf_info("Enter %s and Set device type\r\n",__func__);
				infor_ChargeAddrBuff[9] = RxBuff[2];  //终端类型
				infor_ChargeAddrBuff[10] = RxBuff[3]; //传输方式
				infor_ChargeAddrBuff[11] = RxBuff[4]; //上传周期（min）(高八位)
				infor_ChargeAddrBuff[12] = RxBuff[5]; //上传周期（min）(低八位)
				infor_ChargeAddrBuff[20] = RxBuff[6]; //FLASH修改标志位       01允许修改  FF禁止修改
				infor_ChargeAddrBuff[23] = RxBuff[7]; //模拟数据标志位        01允许修改  FF禁止修改
				OSBsp.Device.InnerFlash.FlashRsvWrite(&infor_ChargeAddrBuff[9], 4, infor_ChargeAddr, 9);//把终端信息写入FLASH
				hal_Delay_ms(10);		//连续存储添加延时
				OSBsp.Device.InnerFlash.FlashRsvWrite(&infor_ChargeAddrBuff[20], 1, infor_ChargeAddr, 20);
				hal_Delay_ms(10);		//连续存储添加延时
				OSBsp.Device.InnerFlash.FlashRsvWrite(&infor_ChargeAddrBuff[23], 1, infor_ChargeAddr, 23);
				// App.Data.TerminalInfoData.SendPeriod = Hal_getTransmitPeriod();
				App.Data.TerminalInfoData.SendPeriod = infor_ChargeAddrBuff[11]*256 + infor_ChargeAddrBuff[12];
				Send_Buffer[31] = (App.Data.TerminalInfoData.SendPeriod>>8) & 0x00FF;
				Send_Buffer[32] = App.Data.TerminalInfoData.SendPeriod & 0x00FF;
				hal_Delay_ms(10);
				if( OSBsp.Device.InnerFlash.innerFLASHRead(9, infor_ChargeAddr) == RxBuff[2] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(10, infor_ChargeAddr) == RxBuff[3] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(11, infor_ChargeAddr) == RxBuff[4] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(12, infor_ChargeAddr) == RxBuff[5] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(20, infor_ChargeAddr) == RxBuff[6] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(23, infor_ChargeAddr) == RxBuff[7]) {
					return 0;
				}else
					goto loop6;
		}else if(RxBuff[1] == 0xFB){		//设置管道参数
			loop4:
				g_Printf_info("Enter %s and Set flow para\r\n",__func__);
				Flash_Tmp[0] = RxBuff[2];  //类型
				Flash_Tmp[1] = RxBuff[3]; //高度/半径_H
				Flash_Tmp[2] = RxBuff[4]; //高度/半径_L
				Flash_Tmp[3] = RxBuff[5]; //宽度_H
				Flash_Tmp[4] = RxBuff[6]; //宽度_L
				Flash_Tmp[5] = RxBuff[7]; //安装宽度_H
				Flash_Tmp[6] = RxBuff[8]; //安装宽度_L
				OSBsp.Device.InnerFlash.FlashRsvWrite(&Flash_Tmp[0], 7, infor_ChargeAddr, 48);//把终端信息写入FLASH
				hal_Delay_ms(10);

				if( OSBsp.Device.InnerFlash.innerFLASHRead(48, infor_ChargeAddr) == RxBuff[2] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(49, infor_ChargeAddr) == RxBuff[3] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(50, infor_ChargeAddr) == RxBuff[4] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(51, infor_ChargeAddr) == RxBuff[5] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(52, infor_ChargeAddr) == RxBuff[6] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(53, infor_ChargeAddr) == RxBuff[7] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(54, infor_ChargeAddr) == RxBuff[8]) {
					return 0;
				}else
					goto loop4;
		}else if(RxBuff[1] == 0xFE){	    //设置出厂信息
			loop7:
				g_Printf_info("Enter %s and Set device product date\r\n",__func__);
				infor_ChargeAddrBuff[0] = RxBuff[2];//出厂日期_年
				infor_ChargeAddrBuff[1] = RxBuff[3];//出厂日期_月
				infor_ChargeAddrBuff[2] = RxBuff[4];//出厂日期_日
				infor_ChargeAddrBuff[3] = RxBuff[5];//出厂编号高八位
				infor_ChargeAddrBuff[4] = RxBuff[6];//出厂编号中八位
				infor_ChargeAddrBuff[5] = RxBuff[7];//出厂编号低八位
				infor_ChargeAddrBuff[6] = RxBuff[8];//设备编号高八位
				infor_ChargeAddrBuff[7] = RxBuff[9];//设备编号中八位
				infor_ChargeAddrBuff[8] = RxBuff[10];//设备编号低八位
				OSBsp.Device.InnerFlash.FlashRsvWrite(infor_ChargeAddrBuff, 9, infor_ChargeAddr, 0);//把终端信息写入FLASH
				hal_Delay_ms(10);
				if( OSBsp.Device.InnerFlash.innerFLASHRead(0, infor_ChargeAddr) == RxBuff[2] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(1, infor_ChargeAddr) == RxBuff[3] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(2, infor_ChargeAddr) == RxBuff[4] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(3, infor_ChargeAddr) == RxBuff[5] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(4, infor_ChargeAddr) == RxBuff[6] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(5, infor_ChargeAddr) == RxBuff[7] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(6, infor_ChargeAddr) == RxBuff[8] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(7, infor_ChargeAddr) == RxBuff[9] && 
					OSBsp.Device.InnerFlash.innerFLASHRead(8, infor_ChargeAddr) == RxBuff[10]) {
					return 0;
				}else
					goto loop7;
		}else if(RxBuff[1] == 0xFD){	    //复位终端
			g_Printf_info("Enter %s and System will reboot\r\n",__func__);
			hal_Delay_ms(100);
			hal_Reboot();
			return 0;
		}
	}else if((RxNum == 27)&&(RxBuff[1] == 0xD1)&&(RxBuff[RxNum-1]==0x0D)){
		g_Printf_info("Enter %s Set OK\r\n",__func__);
		return 0;
	}
	__enable_interrupt();
	return -1;
}

/*******************************************************************************
* 函数名		: g_Device_Board_Config
* 描述	    	: 配置板载参数，主要由上位机设置；通信模块AT指令透传
* 输入参数  	: ClientCmd
* 返回参数  	: 无
*******************************************************************************/
static void g_Device_Board_Config(g_Device_Config_CMD ClientCmd)
{
	volatile uint8_t i;
	/*********************串口Debug串口数据，时钟同步或LoRa配置**********************************/
	if(ClientCmd.cmdLenth != 0)
	{
		if((ClientCmd.hexcmd[0]=='A' && ClientCmd.hexcmd[1]=='T' && 
				ClientCmd.hexcmd[ClientCmd.cmdLenth-2]==0x0D && ClientCmd.hexcmd[ClientCmd.cmdLenth-1]==0x0A))
		{
			g_Printf_info((char *)ClientCmd.hexcmd);
			g_Printf_info("ah receive client set AT cmd\r\n");
			User_Printf((char *)ClientCmd.hexcmd);           //AT指令直接转发给模块
		}else{
			if(FirmCMD_Receive(ClientCmd.hexcmd, ClientCmd.cmdLenth) < 0)//上位机指令解析
			{
				g_Printf_info("ah receive client setpra cmd\r\n");
			}
		}
		// memset(&cRxBuff,0x0,sizeof(g_Device_Config_CMD));
	}
}


void ManagerTaskStart(void *p_arg)
{
	(void)p_arg;    
	OS_CPU_SR  cpu_sr = 0u;
	OSTimeDlyHMSM(0u, 0u, 0u, 150u);
//	static int index = 0;
	g_ConfigQueue = Hal_QueueCreate(QConfiMsgTb,QConfigMsgTb_Size);    
	g_Printf_info("%s ... ...\n",__func__);     
	Hal_ThreadCreate(UartRecTaskStart,
				(void *)"UartRecTaskStart",
				&UartRecTaskStartStk[MINIMUM_TASK_STK_SIZE-1u],
				UART_REC_TASK_TASK_PRIO);
    while (DEF_TRUE) {               /* Task body, always written as an infinite loop.*/
        if(Hal_getCurrent_work_Mode() == 0){    //非低功耗状态
			TaskRefreshWTD(EventWtFlag , WTD_BIT_MANAGER);
			
			memset(&ConfigMsg,0x0,sizeof(struct hal_message));  //清空结构体数组变量
			int ret = Hal_QueueRecv(g_ConfigQueue,&ConfigMsg,1000);
			if(ret == 0) {
				OS_ENTER_CRITICAL();
				g_Printf_dbg("Recv message type %d\r\n",ConfigMsg.what);
				g_Printf_dbg("Recv message content %s\r\n",(char *)ConfigMsg.content);
				OS_EXIT_CRITICAL();
				if (ConfigMsg.what == G_WIRELESS_UPLAOD){
					char *cmdType = (char *)ConfigMsg.content;
					if(strcmp(cmdType,"SerialBus") == 0){
#if ((ACCESSORY_TYPR == RS485_Mode)||(ACCESSORY_TYPR == RS232_Mode))
						g_Device_Config_CMD g_ConfigCMD;
						memset(&g_ConfigCMD,0x0,sizeof(g_Device_Config_CMD));
						OSTimeDlyHMSM(0u, 0u, 0u, 10u);	        //延时等待接收完成
						g_ConfigCMD = g_Device_Usart_UserCmd_Copy(Usart1);
						g_Device_WiredUpload_Config(g_ConfigCMD);
#endif
					}
					else if(strcmp(cmdType,"Wireless") == 0){
						hal_Delay_ms(50);	        //延时等待接收完成
					}
					else if(strcmp(cmdType,"GPS_Info") == 0){
						// g_Printf_info("Recv message type %d\r\n",ConfigMsg.what);
						g_Device_Config_CMD g_ConfigCMD;
						memset(&g_ConfigCMD,0x0,sizeof(g_Device_Config_CMD));
						g_ConfigCMD = g_Device_Usart_UserCmd_Copy(Usart1);
						g_Device_GPS_Config(g_ConfigCMD);
					}
				}else if (ConfigMsg.what == G_CLIENT_CMD){
					char *cmdType = (char *)ConfigMsg.content;
					if(strcmp(cmdType,"ClientCMD") == 0){
						g_Device_Config_CMD g_ConfigCMD;
						memset(&g_ConfigCMD,0x0,sizeof(g_Device_Config_CMD));
						g_ConfigCMD = g_Device_Usart_UserCmd_Copy(Usart2);
						g_Device_Board_Config(g_ConfigCMD);
					}
				}
			}

			OSTimeDly(100);
			// GetADCValue();
        }
		else
        {
        	g_Printf_dbg("ManagerTaskStart ERR!\r\n");
        	OSTimeDlyHMSM(0u, 0u, 0u, 300u);
        }
    }
}



int g_Device_Config_QueuePost(uint32_t type,void *state)
{
	struct hal_message msg;
	
	msg.what = type;
	msg.freecb = null;
	msg.content = state;	
	
	// g_Printf_dbg("Send message type %d\r\n",msg.what);
	// g_Printf_dbg("Send message content %s\r\n",(char *)msg.content);
	// OSTimeDly(50);
	if (Hal_QueueSend(g_ConfigQueue,&msg, 10) < 0){
		g_Printf_dbg("%s message failed!\r\n",__func__);
		return -1;
	}
	OSTimeDly(100);		//添加延时，避免连续发送出错
	return 0;
}









