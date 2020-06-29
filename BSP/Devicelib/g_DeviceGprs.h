#ifndef G_DEVICEGPRS_H
#define G_DEVICEGPRS_H

#include <hal_layer_api.h>
#include <g_Platform.h>

#if (TRANSMIT_TYPE == GPRS_Mode)

extern char CSQBuffer[15];

enum GprsStatus
{
	PowerOff     = 0,
	InitDone     = 1,
	SyncNetRtc   = 2,
	NormalPost   = 3,
	OnNet        = 4,
	DisConnect   = 5
};

enum GPRS_STATUS {
	GPRS_Power_off = 0,
	GPRS_Power_on = 1,
	GPRS_Waitfor_SMSReady = 2,
	GPRS_Preinit = 3,
	GPRS_Init_Done,
	GPRS_Mqtt_Preinit,
	GPRS_Mqtt_Init_Done,
	GPRS_Http_Preinit,
	GPRS_Http_Init_Done,
	GPRS_Init_Failed,
	GPRS_Waitfor_OK,
	GPRS_Waitfor_Download,
	GPRS_Waitfor_Token,
	GPRS_Get_Token,
	GPRS_Get_OK,
	GPRS_Waitfor_Connect,
	GPRS_Connect_OK,
	GPRS_Waitfor_Connack,
	GPRS_Connack_OK,
	GPRS_Waitfor_Suback,
	GPRS_MQTT_Alive,
	GPRS_MQTT_Enable_Pulish,
	GPRS_Waitfor_GetIP,
	GPRS_Get_HTTPACT,
	GPRS_Http_Post_Done,
	GPRS_Fota_Process,
	GPRS_Wait_Idle,
	GPRS_Http_Err
};


void g_Device_GPRS_Init(void);
void g_Device_Establish_TCP_Connection(const char *ip,uint32_t port);
int16_t g_Device_http_post(const char *host,const char* path,const char *apikey,const char *data,
                           char *response,int timeout);
void g_Device_check_Response(char *res);
void  TransmitTaskStart (void *p_arg);
#endif

#endif

