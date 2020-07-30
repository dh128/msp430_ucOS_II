/*
*********************************************************************************************************
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
*
*                                      Texas Instruments MSP430
*                                               on the
*                                          MSP-EXP430F5259LP
*                                          Evaluation Board
*
* Filename      : g_DeviceRTC.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include <bsp.h>

static uint32_t g_MinuteTick = 0;
static uint32_t g_MinuteTimeTick = 0;
static uint32_t g_HourTimeTick = 0;
//static uint32_t g_DayTimeTick = 0;

RtcStruct Rtctime;


//***********************************************************************
//                      DS1302_write_byte
//***********************************************************************
static void DS1302_write_byte(uint8_t addr, uint8_t d) 
{
	uint8_t i;
	RESET_SET;				    //??????DS1302??????
	/*??????????????ddr*/
	IO_OUT;
	addr = addr & 0xFE;         //???????????�
	for (i = 0; i < 8; i ++)
	{
		if (addr & 0x01)
		{
			IO_SET;
		}
		else
		{
			IO_CLR;
		}
		SCK_SET;
		SCK_CLR;
		addr = addr >> 1;
	}
	/*???????????d*/
	IO_OUT;
	for (i = 0; i < 8; i ++)
	{
		if (d & 0x01)
		{
			IO_SET;
		}
		else
		{
			IO_CLR;
		}
		SCK_SET;
		SCK_CLR;
		d = d >> 1;
	}
	RESET_CLR;				//??DS1302??????
}
//***********************************************************************
//                   DS1302_read_byte
//***********************************************************************
static uint8_t DS1302_read_byte(uint8_t addr)
{
	uint8_t i;
	uint8_t temp;
	RESET_SET;					//??????DS1302??????
	/*??????????????ddr*/
	IO_OUT;
	addr = addr | 0x01;         //?????????�
	for (i = 0; i < 8; i ++)
	{
		if (addr & 0x01)
		{
		    IO_SET;
		}
		else
		{
		    IO_CLR;
		}
		SCK_SET;
		SCK_CLR;
		addr = addr >> 1;
	}
	/*????????????emp*/
	IO_IN;
	for (i = 0; i < 8; i ++)
	{
		temp = temp >> 1;
		if (IO_R)
		{
		    temp |= 0x80;
		}
		else
		{
		    temp &= 0x7F;
		}
		SCK_SET;
		SCK_CLR;
	}
	RESET_CLR;				//??DS1302??????
	return temp;
}

//***********************************************************************
//                   向DS302写入时钟数据
//***********************************************************************
void DS1302_write_time(uint8_t *data,uint8_t dataType)
{
	DS1302_write_byte(DS1302_control_add,0x00);		//???????�
	DS1302_write_byte(DS1302_sec_add,0x80);	        //??? 
	//DS1302_write_byte(DS1302_charger_add,0xa9);		//?????? 
	switch(dataType){
		case RealTime:{
			DS1302_write_byte(DS1302_year_add,data[1]);	//?�
			DS1302_write_byte(DS1302_month_add,data[2]);	//?�
			DS1302_write_byte(DS1302_day_add,data[3]);	//?�
			DS1302_write_byte(DS1302_hr_add,data[4]);		//?�
			DS1302_write_byte(DS1302_min_add,data[5]);	//?�
			DS1302_write_byte(DS1302_sec_add,data[6]);	//?�
			DS1302_write_byte(DS1302_week_add,data[7]);	//?�
		}
		break;
		case AlarmData:{
			DS1302_write_byte(DS1302_clock_hr_add,data[0]);	//????�
			DS1302_write_byte(DS1302_clock_min_add,data[1]);	//????�
		}
		break;
		case InnerRam:{
			DS1302_write_byte(DS1302_FLASH,data[0]);
		}
		break;
	}

	DS1302_write_byte(DS1302_control_add,0x80);		//???????�
}

//***********************************************************************
//                     ??S302?????????
//***********************************************************************

void DS1302_read_time(uint8_t *data,uint8_t dataType)
{ 
	switch(dataType){
		case RealTime:{
			data[1]=DS1302_read_byte(DS1302_year_add);	    //?�
			data[2]=DS1302_read_byte(DS1302_month_add);	    //?�
			data[3]=DS1302_read_byte(DS1302_day_add);	    //?�
			data[4]=DS1302_read_byte(DS1302_hr_add);		    //?�
			data[5]=DS1302_read_byte(DS1302_min_add);		    //?�
			data[6]=(DS1302_read_byte(DS1302_sec_add))&0x7F;  //?�
			data[7]=DS1302_read_byte(DS1302_week_add);		    //?�
		}
		break;
		case AlarmData:{
			data[0]=DS1302_read_byte(DS1302_clock_hr_add);	//clock?�
			data[1]=DS1302_read_byte(DS1302_clock_min_add);	//clock?�
		}
		break;
		case InnerRam:{
			data[0]=DS1302_read_byte(DS1302_FLASH);
		}
		break;
	}
}

//***********************************************************************
//               DS302初始化
//***********************************************************************
void g_Device_ExtRTC_Init(void) 
{
	RESET_CLR;			                        //RESET脚置低
	SCK_CLR;			                        //SCK脚置低
	RESET_OUT;			                        //RESET脚设置为输出
	SCK_OUT;			                        //SCK脚设置为输出
	DS1302_write_byte(DS1302_sec_add,0x00);	    //启动

	OSBsp.Device.RTC.ReadExtTime = DS1302_read_time;
	OSBsp.Device.RTC.ConfigExtTime = DS1302_write_time;
}

//***********************************************************************
//生成时间戳函数
//***********************************************************************
void g_Device_RTCstring_Creat(uint8_t *datetime,char *t_str)
{
	t_str[0]='2';	                            //year
	t_str[1]='0';   	                        //
	t_str[2]=((datetime[1]>>4)&0x0f)+0x30;	    //
	t_str[3]=(datetime[1]&0x0f)+0x30;			//
    t_str[4]='-';                               //-
	t_str[5]=((datetime[2]>>4)&0x0f)+0x30;		//month
	t_str[6]=(datetime[2]&0x0f)+0x30; 			//
	t_str[7]='-';                               //-
	t_str[8]=((datetime[3]>>4)&0x0f)+0x30;		//day
	t_str[9]=(datetime[3]&0x0f)+0x30; 			//
	t_str[10]=' ';
	t_str[11]=((datetime[4]>>4)&0x0f)+0x30;		//hour
	t_str[12]=(datetime[4]&0x0f)+0x30; 			//
	t_str[13]=':';
	t_str[14]=((datetime[5]>>4)&0x0f)+0x30;	 	//min
	t_str[15]=(datetime[5]&0x0f)+0x30; 		    //
	t_str[16]=':';
	t_str[17]=((datetime[6]>>4)&0x0f)+0x30;	    //sec
	t_str[18]=(datetime[6]&0x0f)+0x30; 			//

	// App.Data.RtcData.Year = 2000 + datetime[1];
	// App.Data.RtcData.Month = datetime[2];
	// App.Data.RtcData.Day = datetime[3];
	// App.Data.RtcData.Hour = datetime[4];
	// App.Data.RtcData.Minute = datetime[5];
	// App.Data.RtcData.Second = datetime[6];

	Rtctime.Year = 2000 + ((datetime[1]>>4)&0x0f)*10 + (datetime[1]&0x0f);
	Rtctime.Month = ((datetime[2]>>4)&0x0f)*10 + (datetime[2]&0x0f);
	Rtctime.Day = ((datetime[3]>>4)&0x0f)*10 + (datetime[3]&0x0f);
	Rtctime.Hour = ((datetime[4]>>4)&0x0f)*10 + (datetime[4]&0x0f);
	Rtctime.Minute = ((datetime[5]>>4)&0x0f)*10 + (datetime[5]&0x0f);
	Rtctime.Second = ((datetime[6]>>4)&0x0f)*10 + (datetime[6]&0x0f);

	UnixTimeStamp = covBeijing2UnixTimeStp(&Rtctime);
	Send_Buffer[35] = (UnixTimeStamp>>24) & 0xFF;
	Send_Buffer[36] = (UnixTimeStamp>>16) & 0xFF;
	Send_Buffer[37] = (UnixTimeStamp>>8) & 0xFF;
	Send_Buffer[38] = UnixTimeStamp & 0xFF;
}

void g_Device_InnerRTC_Init(uint8_t *date)
{
    RTCCTL01 = RTCMODE + RTCBCD + RTCHOLD + RTCTEV_0 + RTCSSEL_0;		//  Min. changed interrupt
    RTCYEAR 	= 0x2000 + date[1];		//初始时间：从硬件时钟读取
	RTCMON  	= date[2];
    RTCDAY  	= date[3];
	RTCHOUR 	= date[4];        
    RTCMIN  	= date[5];
    RTCSEC  	= date[6];
	RTCADOWDAY  = date[7];    
    RTCCTL01 &= ~RTCHOLD;//启动实时时钟

    RTCCTL0 |= RTCTEVIE;//打开每分钟中断标志
//    RTCCTL0 |= RTCRDYIE;//打开每秒钟中断标志
}

//????????TC???
void Write_info_RTC(uint8_t *time)
{
    RTCCTL01 |= RTCHOLD;//关闭实时时钟

    RTCYEAR_L   = time[1];
    RTCMON      = time[2];
    RTCDAY      = time[3];
    RTCHOUR     = time[4];
    RTCMIN      = time[5];
    RTCSEC      = time[6];
    RTCADOWDAY  = time[7];

    RTCCTL01 &= ~RTCHOLD;//启动实时时钟
//    RTCCTL0 |= RTCTEVIE;//打开每分钟中断标志
//    RTCCTL0 |= RTCRDYIE;//打开每秒钟中断标志
}

//读系统自带的RTC函数
void Read_info_RTC(uint8_t *time)
{
    time[1] = (uint8_t)RTCYEAR;
    time[2] = RTCMON;
    time[3] = RTCDAY;
    time[4] = RTCHOUR;
    time[5] = RTCMIN;
    time[6] = RTCSEC;
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//
static uint8_t month_day[12]={31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //平年
static uint8_t Leap_month_day[12]={31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31}; //闰年
const uint16_t dayPerYear[4] = {365, 365, 365, 366};
uint32_t UnixTimeStamp=0;




// 判断是否是闰年
// year: 需要判断的年
// return：1：闰年
//	  	   0: 平年
uint8_t isLeapYear(uint16_t year)
{
	uint8_t res=0;
	
	if(year%4 == 0) // 能够被4整除
	{
		if((year%100 == 0) && (year%400 != 0))	//能够被100整除，但是不能够被400整除
		{
			res = 0;
		}
		else
		{
			res = 1;
		}
	}
	return res;
}

// 将Unix时间戳转换为北京时间
// unixTime: 需要判断的Unix时间戳
// *tempBeijing:返回的北京时间
// return：none
// note：没对输入参数正确性做判断
void covUnixTimeStp2Beijing(uint32_t unixTime, RtcStruct *tempBeijing)
{
   uint32_t totleDaynum=0, totleSecNum=0;
   uint16_t remainDayofYear;
   uint8_t *pr, tempYear=0;


   totleDaynum = unixTime/(86400); //总天数(注意加括号)
   totleSecNum = unixTime%(86400); //当天剩余的秒速

   memset(tempBeijing, 0x00, sizeof(RtcStruct));
   //1.计算哪一年
   tempBeijing->Year = 1970 + (totleDaynum/FOURYEARDAY)*4;
   remainDayofYear = totleDaynum%FOURYEARDAY+1;
   while(remainDayofYear >= dayPerYear[tempYear]){
       remainDayofYear -= dayPerYear[tempYear];
       tempBeijing->Year++;
       tempYear++;
   }

   //2.计算哪一月的哪一天
   pr = isLeapYear(tempBeijing->Year)?Leap_month_day:month_day;
   while(remainDayofYear > *(pr+tempBeijing->Month))
   {
		remainDayofYear -= *(pr+tempBeijing->Month);
       tempBeijing->Month++;
   }
   tempBeijing->Month++; //month
   tempBeijing->Day = remainDayofYear; //day

   //3.计算当天时间
   tempBeijing->Hour = totleSecNum/3600;
   tempBeijing->Minute = (totleSecNum%3600)/60; //error：变量搞错
   tempBeijing->Second = (totleSecNum%3600)%60;

   //4.时区调整
   tempBeijing->Hour +=TIMEZONE;
   if(tempBeijing->Hour>23){
       tempBeijing->Hour -= 24;
       tempBeijing->Day++;
   }
}
 
// 将北京时间转换为Unix时间戳
// year: 需要判断的年
// return：Unix时间戳（从1970/1/1 00:00:00 到现在的秒数)
// note：没对输入参数正确性做判断
uint32_t covBeijing2UnixTimeStp(RtcStruct *beijingTime)
{
	uint32_t daynum=0,secNum=0,secDayNum=0,secHourNum=0,secMinNum=0,secSecNum=0; //保存北京时间到起始时间的天数
	uint16_t tempYear=1970, tempMonth=0;
 
 
	//1.年的天数
	while(tempYear < beijingTime->Year) 
	{
		if(isLeapYear(tempYear)){
			daynum += 366;
		}
		else{
			daynum += 365;
		}
		tempYear++;
	}
	//2.月的天数
	if(beijingTime->Month > 1){
	 	while(tempMonth < (beijingTime->Month-1))
	 	{
	        if(isLeapYear(beijingTime->Year)){ //闰年
	            daynum += Leap_month_day[tempMonth];
	        }
	        else{
			    daynum += month_day[tempMonth];
	        }
			tempMonth++;
		}
	}
    //3.天数
	if(beijingTime->Day > 1){
		daynum += (beijingTime->Day)-1;
	}
 
    //4.时分秒
    secDayNum = daynum*24*60*60; //s    
    secHourNum = (beijingTime->Hour)*60*60;    
    secMinNum = (beijingTime->Minute)*60;    
    secSecNum = beijingTime->Second;
 
 	secNum = secDayNum + secHourNum + secMinNum + secSecNum;

    //5.时区调整
    secNum -= TIMEZONE*60*60;

    return secNum;
}



#pragma vector=RTC_VECTOR
__interrupt void RTC_ISR(void)
{
	uint16_t min;
    switch (__even_in_range(RTCIV, RTC_RT1PSIFG))
    {
        case RTC_NONE:
            break;

        case RTC_RTCTEVIFG:		//分钟
			{
				min = BCDToHEX(RTCMIN);
				if(App.Data.TerminalInfoData.SendPeriod > NO_LOWPER_PERIOD) //上传频率大于5min才具备低功耗模式
				{
					if((min % App.Data.TerminalInfoData.SendPeriod == 0) && (Hal_getCurrent_work_Mode() == 1)){ 	 //当前为低功耗状态
						__bic_SR_register_on_exit(LPM0_bits);
						TBCTL |= MC_1;     //start timerB
						Hal_ExitLowPower_Mode(Rtc_Int);
					}
					//g_MinuteTick ++;
					// g_Printf_dbg("g_MinuteTick wakeup %d times\r\n",g_MinuteTick);	
					//强行进低功耗会影响FOTA接收dh202029
					// if(g_MinuteTick == (App.Data.TerminalInfoData.SendPeriod-1)){       //ML 20191121强行进低功耗
					// 	if(Hal_getCurrent_work_Mode() == 0){          //当前为未进入低功耗状态
					// 		Hal_EnterLowPower_Mode();
					// 	}
					// }
					// if(g_MinuteTick >= App.Data.TerminalInfoData.SendPeriod){
					// 	g_MinuteTick = 0;
					// 	// if(Hal_getCurrent_work_Mode() == 1){          //当前为低功耗状态
					// 		__bic_SR_register_on_exit(LPM0_bits);
					// 		// WDTCTL  = WDT_MDLY_32;
					// 		// SFRIE1 |= 1;  
					// 		TBCTL |= MC_1;     //start timerB
					// 			// Hal_ExitLowPower_Mode();
					// 		Hal_ExitLowPower_Mode(Rtc_Int);
					// 	// }		
					// }
				}

				g_MinuteTimeTick ++;
				if(g_MinuteTimeTick == 60)
				{
					g_MinuteTimeTick = 0;
					g_HourTimeTick ++;
					if(g_HourTimeTick == 24)//TEST
					{
						g_HourTimeTick = 0;
						App.Data.TransMethodData.SeqNumber = 0;
						App.Data.TerminalInfoData.AutomaticTimeStatus = AUTOMATIC_TIME_ENABLE;  //允许时间同步
						//复位前把模块电源控制都关闭
						#if (TRANSMIT_TYPE == GPRS_Mode)
							OSBsp.Device.IOControl.PowerSet(AIR202_Power_Off);
							OSBsp.Device.IOControl.PowerSet(LPModule_Power_Off);
							OSBsp.Device.IOControl.PowerSet(Motor_Power_Off);
						#endif
					}
				}
			} 
        	break;
        case  RTC_RTCRDYIFG:	//分钟
			{
        
			}
        	break;
        default:
            break;
    }
}
