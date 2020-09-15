/****************SPI_Flash.c*************************************/
#include <bsp.h>



/**************************************************************************************
** Function name:      void SPI_Flash_Read(uint8_t* pBuffer,long ReadAddr,uint16_t NumByteToRead)
** Descriptions:       在指定地址开始读取指定长度的数据
** input parameters:   ReadAddr: 开始读取的地址(24bit)
**                     NumByteToRead :要读取的字节数(最大65535)
** output parameters:  无
** Returned value:     pBuffer: 指向的数组
***************************************************************************************/
void SPI_Flash_Read(uint8_t* pBuffer,long ReadAddr,uint16_t NumByteToRead)
{
 	uint16_t i;
	W25Q16_CS_LOW();                            //使能器件
  OSBsp.Device.Spi2.WriteReadData(W25X_ReadData);          //发送读取命令
  OSBsp.Device.Spi2.WriteReadData((uint8_t)((ReadAddr)>>16));   //发送24bit地址
  OSBsp.Device.Spi2.WriteReadData((uint8_t)((ReadAddr)>>8));
  OSBsp.Device.Spi2.WriteReadData((uint8_t)ReadAddr);
  for(i=0;i<NumByteToRead;i++)
	{
    pBuffer[i]=OSBsp.Device.Spi2.WriteReadData(0XFF);       //循环读数
  }
	W25Q16_CS_HIGH();                            //取消片选
}
/**************************************************************************************
** Function name:      SPI_Flash_ReadByte
** Descriptions:       在指定地址开始读取指定一个字节的数据
** input parameters:   ReadAddr: 读取的地址(24bit)
** Returned value:     读取到数据
***************************************************************************************/
unsigned char SPI_Flash_ReadByte(long ReadAddr)
{
 	uint8_t temp;
	W25Q16_CS_LOW();                            //使能器件
	OSBsp.Device.Spi2.WriteReadData(W25X_ReadData);          //发送读取命令
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((ReadAddr)>>16));   //发送24bit地址
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((ReadAddr)>>8));
	OSBsp.Device.Spi2.WriteReadData((uint8_t)ReadAddr);
	temp=OSBsp.Device.Spi2.WriteReadData(0XFF);       //循环读数
  	W25Q16_CS_HIGH();                            //取消片选
	return temp;
}

/**************************************************************************************
** Function name:      uint8_t SPI_Flash_ReadSR(void)
** Descriptions:       读取SPI_FLASH的状态寄存器
** input parameters:   无
** output parameters:  无
** Returned value:     忙标记位(1,忙;0,空闲)
***************************************************************************************/
uint8_t SPI_Flash_ReadSR(void)
{
	uint8_t byte=0;
	W25Q16_CS_LOW();                             //使能器件
	OSBsp.Device.Spi2.WriteReadData(0x05);                    //发送读取状态寄存器命令
	byte=OSBsp.Device.Spi2.WriteReadData(0Xff);               //读取一个字节
	W25Q16_CS_HIGH();                            //取消片选
	//  printf("READ_STATUSREG = %x\r\n",byte);
	return byte;
}

void SPI_Flash_WriteSR(void)
{
//	uint8_t byte=0;
	SPI_FLASH_Write_Enable();
	W25Q16_CS_LOW();                             //使能器件
	OSBsp.Device.Spi2.WriteReadData(0x01);                    //发送读取状态寄存器命令
	OSBsp.Device.Spi2.WriteReadData(0X00);               //读取一个字节
	W25Q16_CS_HIGH();                            //取消片选
//	 printf("READ_STATUSREG = %x\r\n",byte);
//	return byte;
}
/**************************************************************************************
** Function name:      void SPI_Flash_Wait_Busy(void)
** Descriptions:       等待空闲
** input parameters:   无
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_Flash_Wait_Busy(void)
{
	while ((SPI_Flash_ReadSR()&0x01)==0x01);   // 等待BUSY位清空
}


/**************************************************************************************
** Function name:      void SPI_FLASH_Write_Enable(void)
** Descriptions:       SPI_FLASH写使能
** input parameters:   无
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_FLASH_Write_Enable(void)
{
	W25Q16_CS_LOW();                             //使能器件
	// hal_Delay_us(10);
	OSBsp.Device.Spi2.WriteReadData(W25X_WriteEnable);        //发送写使能
	// hal_Delay_us(10);
	W25Q16_CS_HIGH();                            //取消片选
}

/**************************************************************************************
** Function name:      void SPI_Flash_Erase_Chip(void)
** Descriptions:       擦除整个芯片整片擦除时间:(擦除时间来源于手册，有待验证)
**		                 W25X16:25s
**		                 W25X32:40s
**		                 W25X64:40s
**		                 等待时间超长...
** input parameters:   无
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_Flash_Erase_Chip(void)
{
  SPI_FLASH_Write_Enable();                    //SET WEL
  SPI_Flash_Wait_Busy();
  W25Q16_CS_LOW();                             //使能器件
  OSBsp.Device.Spi2.WriteReadData(W25X_ChipErase);          //发送片擦除命令
	W25Q16_CS_HIGH();                            //取消片选
	SPI_Flash_Wait_Busy();   				             //等待芯片擦除结束
}

/**************************************************************************************
** Function name:      void SPI_Flash_Erase_Sector(long Sector_Addr)
** Descriptions:       擦除W25Q16的一个扇区 (4K)
** input parameters:   Sector_Addr:扇区地址(范围是0~511)
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_Flash_Erase_Sector(long Sector_Addr)
{
//  Sector_Addr*=4096;

	SPI_FLASH_Write_Enable();                                     // 写使能
	SPI_Flash_Wait_Busy();
	W25Q16_CS_LOW();                                              //使能SPI通信
	OSBsp.Device.Spi2.WriteReadData(W25X_SectorErase);                         //发送扇区擦除指令
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((Sector_Addr)>>16));                //发送24bit地址
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((Sector_Addr)>>8));
	OSBsp.Device.Spi2.WriteReadData((uint8_t)Sector_Addr);
	W25Q16_CS_HIGH();                                             //禁能SPI通信
	SPI_Flash_Wait_Busy();                                        //等待操作完成
}

/**************************************************************************************
** Function name:      void SPI_Flash_Erase_Block(long Block_Addr)
** Descriptions:       擦除W25Q16的一个块(64K)
** input parameters:   Block_Addr:块地址(范围是0~31)
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_Flash_Erase_Block(long Block_Addr)
{
	// Block_Addr*=65536;

  SPI_FLASH_Write_Enable();                                     // 写使能
  SPI_Flash_Wait_Busy();
  W25Q16_CS_LOW();
  OSBsp.Device.Spi2.WriteReadData(W25X_BlockErase);                          //发送块区擦除指令
  OSBsp.Device.Spi2.WriteReadData((uint8_t)((Block_Addr)>>16));                //发送24bit地址
  OSBsp.Device.Spi2.WriteReadData((uint8_t)((Block_Addr)>>8));
  OSBsp.Device.Spi2.WriteReadData((uint8_t)Block_Addr);
	W25Q16_CS_HIGH();
  SPI_Flash_Wait_Busy();
}

/**************************************************************************************
** Function name:      void SPI_Flash_Write_Page(uint8_t* pBuffer,long WriteAddr,uint16_t NumByteToWrite)
** Descriptions:       在指定地址开始写入最大256字节的数据
** input parameters:   pBuffer: 数据存储区
**                     WriteAddr: 开始写入的地址
**                     NumByteToWrite: 要写入的字节数(最大256)，该数不应该超过该页的剩余字节数！！！
** output parameters:  无
** Returned value:     无
***************************************************************************************/
void SPI_Flash_Write_Page(uint8_t* pBuffer,long WriteAddr,uint16_t NumByteToWrite)
{
 	uint16_t i;
  	SPI_FLASH_Write_Enable();                                   //SET WEL
	W25Q16_CS_LOW();                                            //使能器件
	hal_Delay_us(10);
	OSBsp.Device.Spi2.WriteReadData(W25X_PageProgram);                       //发送写页命令
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>16));                  //发送24bit地址
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>8));
	// if(NumByteToWrite == 256)
	// 	OSBsp.Device.Spi2.WriteReadData(0);
	// else
	OSBsp.Device.Spi2.WriteReadData((uint8_t)WriteAddr);
	OSBsp.Device.Spi2.WriteNData(pBuffer,NumByteToWrite);
	// for(i=0;i<NumByteToWrite;i++) 
	// {
	// 	OSBsp.Device.Spi2.WriteReadData(pBuffer[i]);//循环写数
	// }
	W25Q16_CS_HIGH();                                            //取消片选
	// hal_Delay_ms(5);											//等待写数据完成3-5ms
	SPI_Flash_Wait_Busy();					                             //等待写入结束
}

//无检验写SPI FLASH
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能
//在指定地址开始写入指定长度的数据
//pbuf:数据存储区
//WriteAddr:开始写入的地址(24bit)
//Len:要写入的字节数(最大65535)
void SPI_Flash_Write_NoCheck(uint8_t * pbuf,long WriteAddr,uint16_t Len)
{
    uint16_t PageLen;                  // 页内写入字节长度
    PageLen=256-WriteAddr%256;    // 单页剩余的字节数 （单页剩余空间）
    if(Len<=PageLen) PageLen=Len; // 不大于256 个字节
    while(1)
    {
		// if(WriteAddr % 0x10000 == 0)
		// 	hal_Delay_sec(1);
		// else
		hal_Delay_us(10);
        SPI_Flash_Write_Page(pbuf,WriteAddr,PageLen);
        if(PageLen==Len)break;   // 写入结束了
        else
        {
            pbuf+=PageLen;
            WriteAddr+=PageLen;
            Len-=PageLen;              //  减去已经写入了的字节数
            if(Len>256)
				PageLen=256;   // 一次可以写入256 个字节
            else 
				PageLen=Len;          // 不够256 个字节了
        }
    }
}
void SPI_Flash_Write_Data(uint8_t pBuffer,long WriteAddr)
{
 //	uint16_t i;
  	SPI_FLASH_Write_Enable();                                   //SET WEL
	W25Q16_CS_LOW();                                            //使能器件
	// hal_Delay_us(10);
	OSBsp.Device.Spi2.WriteReadData(W25X_PageProgram);                       //发送写页命令
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>16));                  //发送24bit地址
	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>8));
	OSBsp.Device.Spi2.WriteReadData((uint8_t)WriteAddr);
	OSBsp.Device.Spi2.WriteReadData(pBuffer);//循环写数
	// hal_Delay_us(10);
	W25Q16_CS_HIGH();                                            //取消片选
	SPI_Flash_Wait_Busy();					                             //等待写入结束
}
/**************************************************************************************
** Function name:      uint16_t SPI_Flash_ReadID(void)
** Descriptions:       读取芯片ID  W25X16的ID:0XEF14
** input parameters:   无
** output parameters:  无
** Returned value:     ChipID:器件ID 正确为0xEF14
***************************************************************************************/
uint16_t SPI_Flash_ReadID(void)
{
	uint16_t ChipID = 0;
	W25Q16_CS_LOW();
	OSBsp.Device.Spi2.WriteReadData(0x90);                               //发送读取ID命令
	OSBsp.Device.Spi2.WriteReadData(0x00);
	OSBsp.Device.Spi2.WriteReadData(0x00);
	OSBsp.Device.Spi2.WriteReadData(0x00);
	ChipID|=OSBsp.Device.Spi2.WriteReadData(0xFF)<<8;
	ChipID|=OSBsp.Device.Spi2.WriteReadData(0xFF);
	W25Q16_CS_HIGH();
	return ChipID;
}

/**************************************************************************************
** Function name:      uint8_t W25Q16_Init(void)
** Descriptions:       w25q16初始化
** input parameters:   无
** output parameters:  无
** Returned value:     0:success 1:error
***************************************************************************************/
uint8_t W25Q16_Init(void)
{
	uint16_t type,status;
	char CHIP_ID[20];
	type=SPI_Flash_ReadID();
  	sprintf(CHIP_ID,"W25Q16 ID is 0x%x",type);
	status = SPI_Flash_ReadSR();
	if(status > 3)		//出现保护位
	{
		g_Printf_info("ststus=%d Clear protect bit\r\n",status);
		SPI_Flash_WriteSR();
	}

	if(type==W25Q16_ID)
	{
		g_Printf_info(CHIP_ID);
		g_Printf_info("Init W25Q16 OK\r\n");
		return 1;
	}
	else
	{
		g_Printf_info("Init W25Q16 Failed\r\n");
		return 0;
	}
}

/********************************old code*********************************************/
//芯片写使能
// void Write_Enable(void)
// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(WRITE_ENABLE);  
//   hal_Delay_us(10);
//   CS_1;
// }

// ///芯片禁止写入
// void Write_Disable(void)
// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(WRITE_DISABLE);
//   hal_Delay_us(10);
//   CS_1;
// }

// //读取芯片状态寄存器
// uint8_t Read_StatReg(void)
// {
//   uint32_t temp;
//   // CS_1;
//   // SD_CS_High();
//   // hal_Delay_us(20);
//   CS_0;  
//   // hal_Delay_us(20);
//   OSBsp.Device.Spi2.WriteReadData(READ_STATUSREG);
//   temp = OSBsp.Device.Spi2.WriteReadData(0xff);
//   // hal_Delay_us(20);
//  // g_Printf_info("READ_STATUSREG = %02x\r\n",temp);
//   CS_1;  
//   return temp;
// }
// void SPI_Flash_Wait_Busy(void)   
// {   
// 	while ((Read_StatReg()&0x01)==0x01);   // 等待BUSY位清空
// }
 
// //写状态寄存器
// void Write_StatReg(uint8_t com)
// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(WRITE_STATUSREG);  
//   OSBsp.Device.Spi2.WriteReadData(com);  
//   hal_Delay_us(10);
//   CS_1;
// }

// void Erase_Page(long address)
// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   uint8_t H,M,L;
//   H = address>>16;  
//   M = address>>8;  
//   L = address&0xff;  
//   Write_Enable(); //��ִ��дʹ��
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(CLEAR_PAGE);  
//   OSBsp.Device.Spi2.WriteReadData(H);  
//   OSBsp.Device.Spi2.WriteReadData(M);  
//   OSBsp.Device.Spi2.WriteReadData(L); 
//   hal_Delay_us(10);
//   CS_1;
// }

// void Erase_Block(long address)
// {
//   unsigned char H,M,L;
//   H = address>>16;
//   M = address>>8;
//   L = address&0xff;
//   Write_Enable(); //先执行写使能
//   CS_0;
//  hal_Delay_us(1);
//   OSBsp.Device.Spi2.WriteReadData(CLEAR_BLOCK);
//   OSBsp.Device.Spi2.WriteReadData(H);
//   OSBsp.Device.Spi2.WriteReadData(M);
//   OSBsp.Device.Spi2.WriteReadData(L);
//  hal_Delay_us(1);
//   CS_1;
// }
// //在任意地址写入一个字节
// void Write_Byte(long address,uint8_t byte)

// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   uint8_t H,M,L;
//   H = address>>16;  
//   M = address>>8; 
//   L = address&0xff;  
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(WRITE_PAGE);  
//   OSBsp.Device.Spi2.WriteReadData(H);  
//   OSBsp.Device.Spi2.WriteReadData(M);  
//   OSBsp.Device.Spi2.WriteReadData(L);  
//   OSBsp.Device.Spi2.WriteReadData(byte);
//   hal_Delay_us(10);
//   CS_1;
// }
// void SPI_Flash_Write_Data(uint8_t pBuffer,uint32_t WriteAddr)
// {
//  //	u16 i;  
//   Write_Enable();                                   //SET WEL 
// 	CS_0;                                            //使能器件
// 	OSBsp.Device.Spi2.WriteReadData(WRITE_PAGE);                       //发送写页命令
// 	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>16));                  //发送24bit地址
// 	OSBsp.Device.Spi2.WriteReadData((uint8_t)((WriteAddr)>>8));   
// 	OSBsp.Device.Spi2.WriteReadData((uint8_t)WriteAddr);   
// 	OSBsp.Device.Spi2.WriteReadData(pBuffer);//循环写数
// 	CS_1;                                            //取消片选
// 	SPI_Flash_Wait_Busy();					                             //等待写入结束
// }
// //在任意地址开始写入一个数据包（最大长度不超过256个字节）
// void Write_Date(long address,uint8_t Date_Buf[],uint8_t size)
// {  
//   SD_CS_Low();
//   hal_Delay_us(20);
//   uint8_t i;
//   uint8_t H,M,L;
//   H = address>>16;  
//   M = address>>8;  
//   L = address&0xff;  
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(WRITE_PAGE);  
//   OSBsp.Device.Spi2.WriteReadData(H);  
//   OSBsp.Device.Spi2.WriteReadData(M);  
//   OSBsp.Device.Spi2.WriteReadData(L);
//   for(i=0;i<size;i++)  
//   {  
//     OSBsp.Device.Spi2.WriteReadData(Date_Buf[i]);
//   }  
//   hal_Delay_us(10);
//   CS_1;
// }

// //在任意地址读出一个字节
// uint8_t Read_Byte(long address)
// {
//   // SD_CS_Low();
//   // hal_Delay_us(20);
//   uint8_t temp;
//   uint8_t H,M,L;
//   H = address>>16; 
//   M = address>>8;  
//   L = address&0xff; 
//   CS_0;  
//   // hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(READ_DATE);  
//   OSBsp.Device.Spi2.WriteReadData(H);  
//   OSBsp.Device.Spi2.WriteReadData(M);  
//   OSBsp.Device.Spi2.WriteReadData(L);  
//   temp = OSBsp.Device.Spi2.WriteReadData(0xff);  
//   // hal_Delay_us(10);
//   CS_1;  
//   return temp;
// }

// //从任意地址开始读出数据
// void Read_Data(long address,uint8_t Date_Buf[],uint16_t size)
// {
//   SD_CS_Low();
//   hal_Delay_us(20);
//   int i;
//   uint8_t H,M,L;
//   H = address>>16;  
//   M = address>>8;  
//   L = address&0xff;  
//   CS_0;  
//   hal_Delay_us(10);
//   OSBsp.Device.Spi2.WriteReadData(READ_DATE);  
//   OSBsp.Device.Spi2.WriteReadData(H);  
//   OSBsp.Device.Spi2.WriteReadData(M); 
//   OSBsp.Device.Spi2.WriteReadData(L); 
//   for(i=0;i<size;i++) 
//   {  
//     Date_Buf[i] = OSBsp.Device.Spi2.WriteReadData(0xff);  
//     // g_Printf_info("%s",&Date_Buf[i]);
//     OSBsp.Device.Usart2.WriteData(Date_Buf[i]);
//   }  
//   hal_Delay_us(10);
//   CS_1;
// }

// long g_MTD_spiflash_writeSector(long address,uint8_t Date_Buf[],uint16_t size)
// {
//   static int m = 0;
//   uint16_t size_to_write = 256;
//   uint16_t remain_size = size;
//   long addr = address;
//   while(Read_StatReg()&0x01);
// 	Write_Enable();

//   for(m=0;m<PagesinSector;m++){
//     if(remain_size <= PageSize){
//       Write_Date(addr,&Date_Buf[size_to_write],size);
//       addr += size;
//       break;
//     }else{
//       Write_Date(addr,&Date_Buf[size_to_write],PageSize);
//       remain_size -= PageSize;
//       addr += PageSize;
//       size_to_write += PageSize;
//     }
//   }
//   Write_Disable();

//   return addr;
// }

// void g_MTD_spiflash_EraseBlock(long block_start)
// {
//   static int m;
//   long addr = block_start;
//   for(m=0;m<256;m++){
//     Erase_Page(addr);
//     addr += PageSize;
//   }
// }

// void g_MTD_spiflash_readSector(long address,uint8_t *Date_Buf)
// {
//   static int m = 0;
//   uint16_t size_to_write = 256;
//   long addr = address;

//   for(m=0;m<PagesinSector;m++){

//   }
// }
