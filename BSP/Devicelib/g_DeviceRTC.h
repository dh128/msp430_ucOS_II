#ifndef   G_DEVICERTC_H
#define   G_DEVICERTC_H

#include <stdint.h>

/*DS1302管脚定义*/
/*DS1302复位脚*/
#define RESET_CLR   	P1OUT &= ~BIT2            //电平置低
#define RESET_SET	    P1OUT |=  BIT2            //电平置高
#define RESET_IN	    P1DIR &= ~BIT2            //方向输入
#define RESET_OUT	    P1DIR |=  BIT2            //方向输出
/*DS1302双向数据*/
#define IO_CLR	        P1OUT &= ~BIT1            //电平置低
#define IO_SET	        P1OUT |=  BIT1            //电平置高
#define IO_R	        P1IN & BIT1               //电平读取
#define IO_IN	        P1DIR &= ~BIT1            //方向输入
#define IO_OUT	        P1DIR |=  BIT1            //方向输出
/*DS1302时钟信号*/
#define SCK_CLR	        P1OUT &= ~BIT0            //时钟信号
#define SCK_SET	        P1OUT |=  BIT0            //电平置高
#define SCK_IN	        P1DIR &= ~BIT0            //方向输入
#define SCK_OUT	        P1DIR |=  BIT0            //方向输出
/*DS1302命令与相关地址*/
#define DS1302_sec_add		    0x80		      //秒数据地址
#define DS1302_min_add		    0x82		      //分数据地址
#define DS1302_hr_add		    0x84		      //时数据地址
#define DS1302_day_add		    0x86		      //日数据地址
#define DS1302_month_add	    0x88		      //月数据地址
#define DS1302_week_add		    0x8a		      //星期数据地址
#define DS1302_year_add		    0x8c		      //年数据地址
#define DS1302_clock_min_add	0xc3		      //分数据地址_闹钟
#define DS1302_clock_hr_add	    0xc5		      //时数据地址_闹钟
#define DS1302_FLASH            0XC7
#define DS1302_control_add	    0x8e		      //控制数据地址
#define DS1302_charger_add	    0x90
#define DS1302_clkburst_add	    0xbe

enum EXT_RTC_DATA_TYPE {
	RealTime = 0,
    AlarmData = 1,
    InnerRam
};


void g_Device_ExtRTC_Init(void);
void g_Device_RTCstring_Creat(uint8_t *datetime,char *t_str);

void g_Device_InnerRTC_Init(void);
void Write_info_RTC(uint8_t *time);
void Read_info_RTC(uint8_t *time);


#define FOURYEARDAY (365+365+365+366)  //4年一个周期内的总天数（1970~2038不存在2100这类年份，故暂不优化）
#define TIMEZONE    8                  //北京时区调整

typedef struct
{
	uint32_t  Second;
	uint32_t  Minute;
	uint32_t  Hour;
	uint32_t  Day;
	uint32_t  Week;
	uint32_t  Month;
	uint32_t Year;
}RtcStruct;

extern RtcStruct Rtctime;
extern uint32_t UnixTimeStamp;
uint8_t isLeapYear(uint16_t year);
void covUnixTimeStp2Beijing(uint32_t unixTime, RtcStruct *tempBeijing);
uint32_t covBeijing2UnixTimeStp(RtcStruct *beijingTime);

#endif

