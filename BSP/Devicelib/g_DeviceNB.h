#ifndef G_DEVICENB_H
#define G_DEVICENB_H

#include <hal_layer_api.h>
#include <g_Platform.h>

#if (TRANSMIT_TYPE == NBIoT_BC95_Mode)

//�ϵ�����ʹ��
#define MaxLength	5
extern uint16_t BackupIndex;
extern uint16_t StartFile;
extern uint8_t FullFlag;
extern char RespFile[10];
extern char NB_Fota;

extern uint32_t Send_Buffer_CTwing_NBSignal[16];
extern uint32_t Send_Buffer_CTwing_NBSoildata[13];
extern uint32_t Send_Buffer_CTwing_NBWeatherdata[7]; 

enum NB_STATUS {
	NB_Power_off = 0,
	NB_Power_on = 1,
	NB_Boot,
	NB_Init_Done,
	NB_Init_Error,
	NB_Get_IP,
	NB_Registered,
	NB_Send_Over,
	NB_Send_Error,
	NB_Get_PCP,
	NB_Check_Receive,
	NB_Idel
};

unsigned char NB_Config(unsigned char *c , unsigned char m, unsigned char t);
void g_Device_NB_Restart(void);
void SyncTime(void); 
char  g_Device_NB_Init(void);
void g_Device_NB_GetIP(void);
void g_Device_NB_Send(uint32_t *data ,uint8_t length);
void g_Device_NBSignal(void);
void g_Device_NB_Receive(void);
void g_Device_check_Response(char *res);
// void Hex2Str(unsigned char *d,uint32_t *p,unsigned char Len, unsigned char offset);
// void Hex2Str(unsigned char *d,uint8_t *p,unsigned char Len, unsigned char offset);
void  TransmitTaskStart (void *p_arg);
#endif

#endif

