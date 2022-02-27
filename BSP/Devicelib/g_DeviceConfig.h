#ifndef G_DEVICECONFIG_H_
#define G_DEVICECONFIG_H_


#define QConfigMsgTb_Size               10

typedef struct{
	uint8_t hexcmd[32];
    char strcmd[128];
    uint8_t cmdLenth;
}g_Device_Config_CMD;

enum G_CONFIG_TYPE{
	G_WIRELESS_UPLAOD = 0x1,		
	G_CLIENT_CMD
};

void ManagerTaskStart(void *p_arg);
int g_Device_Config_QueuePost(uint32_t type,void *state);
#if (ACCESSORY_TYPR == GPS_Mode)
static void g_Device_GPS_Config(g_Device_Config_CMD uploadCmd);
#endif



#endif /* G_DEVICECONFIG_H_ */
