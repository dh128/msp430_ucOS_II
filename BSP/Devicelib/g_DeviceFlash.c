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
* Filename      : g_DeviceFlash.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include  <bsp.h>
#include "MSP430F5xx_6xx\flashctl.h"


uint8_t infor_ChargeAddrBuff[32] = {0x00};

/*******************************************************************************
* Function Name  : infoFLASH_read
* Description    :
* Input para     : index add
* Output para    : value
*******************************************************************************/
uint8_t infoFLASH_read(uint8_t index,uint32_t add)
{
	uint32_t *Flash_ptrD;
	Flash_ptrD = (uint32_t *)add;
	uint8_t *flash_ptr =  ((uint8_t *)Flash_ptrD) + index;

	return *flash_ptr;
}

/*******************************************************************************
* 函数名      	: infoFLASH_write
* 描述	  		: Flash写操作
* 输入参数  	: *data_ptr--写入数据，*flashAddr--写入地址，count--长度
* 返回参数  	: 无
*******************************************************************************/
void infoFLASH_write(uint8_t *data_ptr,uint8_t *flashAddr,uint16_t count)
{
	__disable_interrupt();
	FlashCtl_eraseSegment(flashAddr);

	if(FlashCtl_performEraseCheck(flashAddr,count) == 0x01)
	{
		FlashCtl_write8(data_ptr,flashAddr,count);
	}
	__enable_interrupt();
}
/*******************************************************************************
* 函数名      	: InforFlash_Write
* 描述	  		: Flash写操作
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void InforFlash_Write(unsigned char dat)
{
	char *Flash_ptr;		                     	//Flash指针
	Flash_ptr = (char *) infor_ChargeAddr;			//初始化Flash指针
	FCTL3 = FWKEY;						       		//清除锁定控制位
	FCTL1 = FWKEY+ERASE;				    		//段擦除
	_DINT();										//Flash操作器件不允许任何中断
	*Flash_ptr = 0;									//空写，启动擦除
	while(FCTL3 & BUSY);							//等待擦除操作完成
	FCTL1 = FWKEY+WRT;								//采用字节/字写入模式
	*Flash_ptr = dat;								//写入数据
	while(FCTL3 & BUSY);							//等待写入完成
	_EINT();										//启动全局中断
	FCTL1 = FWKEY;									//Flash退出写模式
	FCTL3 = FWKEY + LOCK;							//恢复Flash锁定位，保护数据

}
/*******************************************************************************
* 函数名      	: FlashRsvWrite
* 描述	  		: Flash指定位置写入指定数据，整页
* 输入参数  	: *data_ptr要写入的数据，indux--写入数据长度，infostartaddr--存储起始地址，num--存储地址偏移位
* 返回参数  	: 无
*******************************************************************************/
void FlashRsvWrite(uint8_t *data_ptr,uint8_t index,uint32_t infostartaddr,uint8_t num)
{
	static uint8_t m;
	uint8_t cacheBuf[128];

	for(m=0;m<128;m++)
	{
		cacheBuf[m] = infoFLASH_read(m,infostartaddr);
	}

	for(m=0;m<index;m++)
		cacheBuf[m+num] = *data_ptr++;

	infoFLASH_write(cacheBuf,(uint8_t *)infostartaddr,128);
}
/*******************************************************************************
* 函数名      	: g_Device_InnerFlash_Init
* 描述	  		: 初始化内部Flash操作
* 输入参数  	: 无
* 返回参数  	: 无
*******************************************************************************/
void g_Device_InnerFlash_Init(void)
{
//	uint8_t TerminalNum = 0x07;

	OSBsp.Device.InnerFlash.innerFLASHRead  = infoFLASH_read;
	OSBsp.Device.InnerFlash.innerFLASHWrite = infoFLASH_write;
	OSBsp.Device.InnerFlash.FlashRsvWrite   = FlashRsvWrite;

	hal_Delay_ms(10);

//	OSBsp.Device.InnerFlash.innerFLASHWrite(&TerminalNum,(uint8_t *)infor_ChargeAddr,1);
}

