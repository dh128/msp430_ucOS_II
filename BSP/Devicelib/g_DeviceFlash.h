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

#define ProductKey_Addr         0x39
#define DeviceName_Addr         0x49
#define DeviceSecret_Addr       0x59

// 0x1800——升级标志位    01
// 0x1801——当前版本号
// 0x1802——版本号缓存
extern uint8_t infor_ChargeAddrBuff[32];

//inforFLASH存储分布
/**************************************************************************************************
 *   Boot 	 Vision |
 *   0x1800  0x1801 |
 **************************************************************************************************
 *   PD_H   PD_M   PD_L   | DevID_H Dev_ID_M DevID_L| DevSN_H DevSN_M DevSN_L|  Type  | Comm    |
 *   0x1900 0x1901 0x1902 |  0x1903 0x1904 0x1905   | 0x1906   0x1907 0x1908 | 0x1909 | 0x190A  |
 **************************************************************************************************
 *   Per_H   Per_L	| qantity|
 *   0x190B  0x190C | 0x190D | 0x190E |0x190F |
 *   
 *  管道类型 | 管道宽度      | 管道高度         |  传感器安装高度 |
 *  0x1930  | 0x1931 0x1932 |  0x1933 0x1934  | 0x1935 0x1936  |
 * 
 * 
 */



void g_Device_InnerFlash_Init(void);





#endif


