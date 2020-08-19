#ifndef G_DEVICELORA_H
#define G_DEVICELORA_H

#include <hal_layer_api.h>
#include <g_Platform.h>

#if (TRANSMIT_TYPE == LoRa_F8L10D_Mode)

//�ϵ�����ʹ��
#define MaxLength	4
extern uint16_t BackupIndex;
extern uint16_t StartFile;
extern uint16_t FullFlag;
extern char RespFile[10];


enum LoRa_STATUS {
	LoRa_Power_off = 0,
	LoRa_Power_on = 1,
	LoRa_Boot,
	LoRa_TimeSynchronization,
	LoRa_Init_Done,
	LoRa_Join_Over,
	LoRa_Send_Done,
	LoRa_Send_Error,
	LoRa_Get_Receive,
	LoRa_Check_Receive,
	LoRa_Idel
	
};

unsigned char LoRa_Config(unsigned char *c , unsigned char m, unsigned char t);
void g_Device_LoRa_Init(void);
void g_Device_LoRa_GetJoined(void);
void g_Device_LoRa_Send(uint32_t *data , uint8_t length , uint8_t port);
void g_Device_LoRa_Receive(void);
void g_Device_check_Response(char *res);

void  TransmitTaskStart (void *p_arg);
#endif

#endif

