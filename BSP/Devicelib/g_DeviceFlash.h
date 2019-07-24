/*******************************************************************************
*文件名     : DeviceFlash.h
*作用         : Flash操作
*创建时间 : 2018/07/05
********************************************************************************
*/
#ifndef G_DEVICEFLASH_H
#define G_DEVICEFLASH_H

#define infor_BootAddr		0x1800
#define infor_ChargeAddr	0x1900

//inforFLASH存储分布
/**************************************************************************************************
 *   Boot 	 Vision |
 *   0x1800  0x1801 |
 **************************************************************************************************
 *   DevID_H DevID_L| DevSN_H DevSN_L|  PD_H   PD_M   PD_L  |�ն� ���� | ͨѶ��ʽ  |Deveui_H  Deveui_L|
 *   0x1900  0x1901 | 0x1902  0x1903 | 0x1904 0x1905 0x1906 | 0x1907 |  0x1908 | 0x1909   0x190A  |
 **************************************************************************************************
 *   Period    		|
 *   0x190B  0x190C |
 *
 */

extern uint8_t Terminal_Info[14];



void g_Device_InnerFlash_Init(void);
//void InitTerminal(void);
void InforOperate(uint8_t data);





#endif


