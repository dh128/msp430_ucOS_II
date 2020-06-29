/*
*********************************************************************************************************
*                                         BOARD SUPPORT PACKAGE
*
*                            (c) Copyright 2014; Micrium, Inc.; Weston, FL
*
*               All rights reserved. Protected by international copyright laws.
*
*               BSP is provided in source form to registered licensees ONLY.  It is
*               illegal to distribute this source code to any third party unless you receive
*               written permission by an authorized Micrium representative.  Knowledge of
*               the source code may NOT be used to develop a similar product.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                        BOARD SUPPORT PACKAGE
*
*                                 Texas Instruments MSP-EXP430F5529LP
*                                               LaunchPad
*
* Filename      : g_DeviceSPIFlash.h
* Version       : V1.00
* Programmer(s) : HS
*********************************************************************************************************
*/
#ifndef _G_DEVICESPIFLASH_H
#define _G_DEVICESPIFLASH_H


//固件地址划分
#define UPDATE_ADDR       0x0
#define FOTA_ADDR_START   0x80000

/* W25Q16片选 */
#define W25Q16_CS_LOW()           P2OUT &=~BIT2		//SPI falsh cs put low
#define W25Q16_CS_HIGH()          P2OUT |= BIT2		//SPI falsh cs put high


/* W25X16系列ID是0xEF14 */
#define W25Q16_ID  0xEF14

/* W25Q16指令表	*/
#define W25X_WriteEnable		  	0x06
#define W25X_WriteDisable		  	0x04
#define W25X_ReadStatusReg			0x05
#define W25X_WriteStatusReg			0x01
#define W25X_ReadData			    0x03
#define W25X_FastReadData		 	0x0B
#define W25X_FastReadDual		  	0x3B
#define W25X_PageProgram		  	0x02
#define W25X_BlockErase			  	0xD8
#define W25X_SectorErase		  	0x20
#define W25X_ChipErase			  	0xC7
#define W25X_PowerDown			  	0xB9
#define W25X_ReleasePowerDown		0xAB
#define W25X_DeviceID			    0xAB
#define W25X_ManufactDeviceID		0x90
#define W25X_JedecDeviceID			0x9F

/* Private Function*/
uint8_t W25Q16_Init(void);
void SPI_Flash_Read(uint8_t* pBuffer,long ReadAddr,uint16_t NumByteToRead);
void SPI_Flash_Write_Page(uint8_t* pBuffer,long WriteAddr,uint16_t NumByteToWrite);
void SPI_Flash_Write_NoCheck(uint8_t * pbuf,long WriteAddr,uint16_t Len);
void SPI_Flash_Erase_Chip(void);
void SPI_Flash_Erase_Sector(long Sector_Addr);
void SPI_Flash_Erase_Block(long Block_Addr);
void WriteHZ16(void);
uint8_t SPI_Flash_ReadSR(void);
void SPI_Flash_WriteSR(void);
void SPI_FLASH_Write_Enable(void);
void SPI_Flash_Write_Data(uint8_t pBuffer,long WriteAddr);
unsigned char SPI_Flash_ReadByte(long ReadAddr);
/**************************************old code*********************************************************/
// #define PageSize            255
// #define PagesinSector       16
// #define SectorsinBlock      16

// //////W25X16 命令指令表定义
// #define WRITE_ENABLE      0X06        //写使能，设置状态寄存器
// #define WRITE_DISABLE     0X04        //写禁止
// #define READ_STATUSREG    0X05        //读状态寄存器
// #define WRITE_STATUSREG   0X01        //写状态寄存器
// #define READ_DATE         0X03        //读取存储器数据
// #define READ_FAST         0X0B        //快速读取存储器数据
// #define READ_DOUBLEFAST   0X3B        //快速双端口输出方式读取存储器数据
// #define WRITE_PAGE        0X02        //页面编程--写数据
// #define CLEAR_BLOCK       0XD8        //块擦除
// #define CLEAR_PAGE        0X20        //扇区擦除
// #define CLEAR_SHIP        0XC7        //片擦除
// #define POWER_OFF         0XB9        //掉电模式
// #define POWER_ON          0XAB        //退出掉电模式、设备ID信息
// #define SHIP_ID           0X90        //读取制造厂商ID信息和设备ID信息
// #define JEDEC_ID          0X9F        //JEDEC的ID信息

// #define FOTA_ADDR_START   0x80000

// ///////函数声明
// extern void Write_Enable(void);
// extern void Write_Disable(void);
// extern uint8_t Read_StatReg(void);
// extern void Write_StatReg(uint8_t com);
// void SPI_Flash_Wait_Busy(void);
// extern void Erase_Page(long address);
// void Erase_Block(long address);
// extern uint8_t Read_Byte(long address);
// extern void Read_Data(long address,uint8_t Date_Buf[],uint16_t size);
// extern void Write_Byte(long address,uint8_t byte);
// extern void Write_Date(long address,uint8_t Date_Buf[],uint8_t size);
// long g_MTD_spiflash_writeSector(long address,uint8_t Date_Buf[],uint16_t size);
// void g_MTD_spiflash_EraseBlock(long block_start);
// void SPI_Flash_Write_Data(uint8_t pBuffer,uint32_t WriteAddr);


#endif
