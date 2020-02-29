/*
*********************************************************************************************************
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
*
*                                      Texas Instruments MSP430
*                                               on the
*                                          MSP-EXP430F5259LP
*                                          Evaluation Board
*
* Filename      : g_DeviceSD.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include <bsp.h>


uint8_t SD_Type=0;              //SD��������

unsigned long cardSize = 0;     //SD������
uint8_t status = 1;             //״̬
uint8_t timeout = 0;            //����
uint8_t SD_Status = 1;
//*************************************************//
//      SD_Storage_DebugLog
//*************************************************//
uint8_t date[8];
char fileNameStr[12] = {"000000.txt"};

char SD_DebugLogStr[5][30] = {
                                {"\r\n"},                      //0-\r\n
                                {"2019-09-01 00:00:03   "},    //1-Uptime
                                {"Init module ok; "},          //2
                                {"Init module error; "},       //3
                                {"Scan sensor ok; "}           //4
                                }; 
//*************************************************//
void SD_Storage_DebugLog(uint8_t index)
{
    OSBsp.Device.RTC.ReadExtTime(date,RealTime);
    fileNameStr[0]=((date[1]>>4)&0x0f)+0x30;	     //1           
    fileNameStr[1]=(date[1]&0x0f)+0x30;			     //9
    fileNameStr[2]=((date[2]>>4)&0x0f)+0x30;		 //1
    fileNameStr[3]=(date[2]&0x0f)+0x30; 		     //0
    fileNameStr[4]=((date[3]>>4)&0x0f)+0x30;		 //2
    fileNameStr[5]=(date[3]&0x0f)+0x30; 		     //5
    g_Device_RTCstring_Creat(date,SD_DebugLogStr[1]);// 

    Write_ToDirTxt("0:/DebugLog",fileNameStr,SD_DebugLogStr[index]);//在DebugLog文件夹中，添加191025.TXT文档，存放SD_DebugLogStr字符串
}


//*************************************************//
//      g_Device_SDCard_Check
//*************************************************//
uint8_t g_Device_SDCard_Check(void)
{
	/***********Test SD Card*******************/
	while((status != 0)&(timeout < 100))          //初始化50次MMC/SD卡，如果依然返回错误代码，则初始化失败，可能是没有SD卡或损坏
	{
		status = g_MMC_init();
		timeout++;
	}
	if(status != 0){
	    g_Printf_info("Init Failure,Please Check SD!\r\n");
	}else
	{
	    g_Printf_info("Init SD Success!\r\n");
		cardSize =  MMC_ReadCardSize();                 //从SD卡寄存器读取MMC/SD卡容量大小，返回长整型
		// SDsizeDisplay(cardSize);
		g_Printf_info("SD Size: %d MB",cardSize/1048576);
	}

	 return status;
	/***********End*******************/
}

//***********************************************************************
//            SD卡容量处理及显示函数
//***********************************************************************
void SDsizeDisplay(unsigned long Capacity)
{
	static unsigned int SizeQ,SizeB,SizeS,SizeG;
	unsigned long Size;
	OSBsp.Device.Usart2.WriteString("SD Size:");          //显示SD卡容量
	Size=Capacity/1024/1024;
	SizeQ=Size/1000;
	SizeB=(Size%1000)/100;
	SizeS=(Size%1000)%100/10;
	SizeG=(Size%1000)%100%10;
	OSBsp.Device.Usart2.WriteData(SizeQ+'0');
	OSBsp.Device.Usart2.WriteData(SizeB+'0');
	OSBsp.Device.Usart2.WriteData(SizeS+'0');
	OSBsp.Device.Usart2.WriteData(SizeG+'0');
	OSBsp.Device.Usart2.WriteString("MB\r\n");  		   //显示SD卡容量单位
}
/*******************************************************************************
* 函数名	: SD_GetResponse
* 描述	: SD卡获取设备响应
* 输入参数  : Response
* 返回参数  : MSD_RESPONSE_FAILURE 或 MSD_RESPONSE_NO_ERROR
*******************************************************************************/
uint8_t SD_GetResponse(uint8_t Response)
{
	uint16_t ResponseTick = 0xFFF;		//等待次数

	while ((OSBsp.Device.Spi2.WriteReadData(0XFF) != Response) && ResponseTick)
		ResponseTick--;				//等待得到准确的回应

	if (ResponseTick==0)
		return MSD_RESPONSE_FAILURE;    //得到回应失败
	else
		return MSD_RESPONSE_NO_ERROR;   //正确回应
}

/*******************************************************************************
* 函数名	: SD_GetResponse
* 描述	: 等待SD卡写入完成
* 输入参数  : 无
* 返回参数  : 0，成功
*******************************************************************************/
uint8_t SD_WaitDataReady(void)
{
	uint8_t r1 = MSD_DATA_OTHER_ERROR;
	uint32_t retry = 0;

    do
    {
        r1 = OSBsp.Device.Spi2.WriteReadData(0xFF) & 0X1F;             //读到回应
        if(retry==0xfffe)
        	return 1;

		retry++;

		switch (r1)
		{					   
			case MSD_DATA_OK:               //数据接收正确了
				r1=MSD_DATA_OK;
				break;  
			case MSD_DATA_CRC_ERROR:        //CRC校验错误
				return MSD_DATA_CRC_ERROR;  
			case MSD_DATA_WRITE_ERROR:      //数据写入错误
				return MSD_DATA_WRITE_ERROR;  
			default:                        //未知错误
				r1=MSD_DATA_OTHER_ERROR;
				break;	 
		}   
    }while(r1==MSD_DATA_OTHER_ERROR);       //数据错误时一直等待

	retry=0;
	while(OSBsp.Device.Spi2.WriteReadData(0XFF)==0)      //读到数据为0,则数据还未写完成
	{
		retry++;
		//delay_us(10);                     //SD卡写等待需要较长的时间
		if(retry >= 0XFFFFFFFE)
			return 0XFF;                    //等待失败
	};	    

    return 0;                               //成功
}	 

/*******************************************************************************
* 函数名	: SD_SendCommand
* 描述	: 向SD卡发送一个命令
* 输入参数  :  cmd  命令
			 arg  命令参数
			 crc  crc校验值
* 返回参数  : SD卡返回的响应
*******************************************************************************/
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc)
{
	uint8_t r1;
	uint8_t Retry=0;

	P5OUT |= BIT5;

    OSBsp.Device.Spi2.WriteReadData(0xff);			//高速写命令延时
	OSBsp.Device.Spi2.WriteReadData(0xff);     
 	OSBsp.Device.Spi2.WriteReadData(0xff);

    //片选端置低，选中SD卡
    SD_CS_Low();
    //发送

    OSBsp.Device.Spi2.WriteReadData(cmd | 0x40);		//分别写入命令
    OSBsp.Device.Spi2.WriteReadData(arg >> 24);
    OSBsp.Device.Spi2.WriteReadData(arg >> 16);
    OSBsp.Device.Spi2.WriteReadData(arg >> 8);
    OSBsp.Device.Spi2.WriteReadData(arg);
    OSBsp.Device.Spi2.WriteReadData(crc); 

   //等待响应，或超时退出
    while((r1=OSBsp.Device.Spi2.WriteReadData(0xFF))==0xFF)
    {
        Retry++;	    
        if(Retry>200)break; 
    }   
    //关闭片选
    SD_CS_High();
    //在总线上额外增加8个时钟，让SD卡完成剩下的工作
    OSBsp.Device.Spi2.WriteReadData(0xFF);
    //返回状态值
    return r1;
}

/*******************************************************************************
* 函数名	: SD_SendCommand_NoDeassert
* 描述	: 向SD卡发送一个命令(结束是不失能片选，还有后续数据传来）
* 输入参数  :  cmd  命令
			 arg  命令参数
			 crc  crc校验值
* 返回参数  : SD卡返回的响应
*******************************************************************************/
uint8_t SD_SendCommand_NoDeassert(uint8_t cmd, uint32_t arg, uint8_t crc)
{
	uint8_t Retry=0;
	uint8_t r1;

    OSBsp.Device.Spi2.WriteReadData(0xff);//高速写命令延时
	OSBsp.Device.Spi2.WriteReadData(0xff);  	 	 
    SD_CS_Low();//片选端置低，选中SD卡
    //发送
    OSBsp.Device.Spi2.WriteReadData(cmd | 0x40); //分别写入命令
    OSBsp.Device.Spi2.WriteReadData(arg >> 24);
    OSBsp.Device.Spi2.WriteReadData(arg >> 16);
    OSBsp.Device.Spi2.WriteReadData(arg >> 8);
    OSBsp.Device.Spi2.WriteReadData(arg);
    OSBsp.Device.Spi2.WriteReadData(crc);   

    //等待响应，或超时退出
    while((r1=OSBsp.Device.Spi2.WriteReadData(0xFF))==0xFF)
    {
        Retry++;	    
        if(Retry>200)break; 
    }  	  
   //返回响应值
    return r1;
}

/*******************************************************************************
* 函数名	: SD_Idle_Sta
* 描述	: 把SD卡设置到挂起模式
* 输入参数  : 无
* 返回参数  : SD卡返回的响应
*******************************************************************************/
uint8_t SD_Idle_Sta(void)
{
	uint16_t kk;
	uint8_t retry;
    for(kk=0;kk<0xf00;kk++);     	//纯延时，等待SD卡上电完成
    								//先产生>74个脉冲，让SD卡自己初始化完成
    for(kk=0;kk<10;kk++)
    	OSBsp.Device.Spi2.WriteReadData(0xFF);

    //-----------------SD卡复位到idle开始-----------------
    //循环连续发送CMD0，直到SD卡返回0x01,进入IDLE状态
    //超时则直接退出
    retry = 0;
    do
    {	   
        //发送CMD0，让SD卡进入IDLE状态
        kk = SD_SendCommand(CMD0, 0, 0x95);
        retry++;
    }while((kk!=0x01)&&(retry<200));

    //跳出循环后，检查原因：初始化成功？or 重试超时？
    if(retry==200)
    	return 1; 		//失败

	return 0;			//成功
}														    

/*******************************************************************************
* 函数名	: g_Device_SD_Init
* 描述	: SD卡初始化
* 输入参数  : 无
* 返回参数  : SD卡返回的响应
*******************************************************************************/
uint8_t g_Device_SD_Init(void)
{								 
	uint8_t r1;     		// 存放SD卡的返回值
	uint16_t retry;  		// 用来进行超时计数
	uint8_t buff[6];

	g_Device_SPI2_Init(); 
	SD_CS_High();
    if(SD_Idle_Sta())
    	return 1;			//超时返回1 设置到idle 模式失败
   //-----------------SD卡复位到idle结束-----------------
    //获取卡片的SD版本信息
// 	SD_CS_Low();
    SD_CS_Low();
	r1 = SD_SendCommand_NoDeassert(8, 0x1aa,0x87);	     
    //如果卡片版本信息是v1.0版本的，即r1=0x05，则进行以下初始化
    if(r1 == 0x05)
    {
        //设置卡类型为SDV1.0，如果后面检测到为MMC卡，再修改为MMC
        SD_Type = SD_TYPE_V1;	   
        //如果是V1.0卡，CMD8指令后没有后续数据
        //片选置高，结束本次命令
        SD_CS_High();
        //多发8个CLK，让SD结束后续操作
        OSBsp.Device.Spi2.WriteReadData(0xFF);	  
        //-----------------SD卡、MMC卡初始化开始-----------------
        //发卡初始化指令CMD55+ACMD41
        // 如果有应答，说明是SD卡，且初始化完成
        // 没有回应，说明是MMC卡，额外进行相应初始化
        retry = 0;
        do
        {
             //先发CMD55，应返回0x01；否则出错
            r1 = SD_SendCommand(CMD55, 0, 0);
            if(r1 == 0XFF)return r1;//只要不是0xff,就接着发送
            //得到正确响应后，发ACMD41，应得到返回值0x00，否则重试200次
            r1 = SD_SendCommand(ACMD41, 0, 0);
            retry++;
        }while((r1!=0x00) && (retry<400));
        // 判断是超时还是得到正确回应
        // 若有回应：是SD卡；没有回应：是MMC卡
        //----------MMC卡额外初始化操作开始------------
        if(retry==400)
        {
            retry = 0;
           //发送MMC卡初始化命令（没有测试）
            do
            {
                r1 = SD_SendCommand(1,0,0);
                retry++;
            }while((r1!=0x00)&& (retry<400));
            if(retry==400)return 1;   //MMC卡初始化超时
             //写入卡类型
            SD_Type = SD_TYPE_MMC;
        }
        //----------MMC卡额外初始化操作结束------------
     
          
		OSBsp.Device.Spi2.WriteReadData(0xFF);	 
        //禁止CRC校验
		r1 = SD_SendCommand(CMD59, 0, 0x95);
        if(r1 != 0x00)return r1;   //命令错误，返回r1
        //设置Sector Size
        r1 = SD_SendCommand(CMD16, 512, 0x95);
        if(r1 != 0x00)return r1;  //命令错误，返回r1
        //-----------------SD卡、MMC卡初始化结束-----------------

    }//SD卡为V1.0版本的初始化结束
     //下面是V2.0卡的初始化
     //其中需要读取OCR数据，判断是SD2.0还是SD2.0HC卡
    else if(r1 == 0x01)
    {
        //V2.0的卡，CMD8命令后会传回4字节的数据，要跳过再结束本命令
        buff[0] = OSBsp.Device.Spi2.WriteReadData(0xFF);  //should be 0x00
        buff[1] = OSBsp.Device.Spi2.WriteReadData(0xFF);  //should be 0x00
        buff[2] = OSBsp.Device.Spi2.WriteReadData(0xFF);  //should be 0x01
        buff[3] = OSBsp.Device.Spi2.WriteReadData(0xFF);  //should be 0xAA	    
        SD_CS_High();
        OSBsp.Device.Spi2.WriteReadData(0xFF);//the next 8 clocks			 
        //判断该卡是否支持2.7V-3.6V的电压范围
        //if(buff[2]==0x01 && buff[3]==0xAA) //不判断，让其支持的卡更多
        {	  
            retry = 0;
            //发卡初始化指令CMD55+ACMD41
    		do
    		{
    			r1 = SD_SendCommand(CMD55, 0, 0);
    			if(r1!=0x01)return r1;	   
    			r1 = SD_SendCommand(ACMD41, 0x40000000, 0);
                if(retry>200)return r1;  //超时则返回r1状态
            }while(r1!=0);		  
           //初始化指令发送完成，接下来获取OCR信息
            //-----------鉴别SD2.0卡版本开始-----------
            r1 = SD_SendCommand_NoDeassert(CMD58, 0, 0);
            if(r1!=0x00)
			{
				SD_CS_High();//释放SD片选信号
				return r1; //如果命令没有返回正确应答，直接退出，返回应答
			}//读OCR指令发出后，紧接着是4字节的OCR信息
            buff[0] = OSBsp.Device.Spi2.WriteReadData(0xFF);
            buff[1] = OSBsp.Device.Spi2.WriteReadData(0xFF); 
            buff[2] = OSBsp.Device.Spi2.WriteReadData(0xFF);
            buff[3] = OSBsp.Device.Spi2.WriteReadData(0xFF);		 
            //OCR接收完成，片选置高
            SD_CS_High();
            OSBsp.Device.Spi2.WriteReadData(0xFF);	   
            //检查接收到的OCR中的bit30位（CCS），确定其为SD2.0还是SDHC
            //如果CCS=1：SDHC   CCS=0：SD2.0
            if(buff[0]&0x40)SD_Type = SD_TYPE_V2HC;    //检查CCS
            else SD_Type = SD_TYPE_V2;	    
            //-----------鉴别SD2.0卡版本结束-----------
            
        }	    
    }
    return r1;
}	 																			   

/*******************************************************************************
* 函数名	: SD_ReceiveData
* 描述	: 从SD卡中读回指定长度的数据，放置在给定位置
* 输入参数  : *data(存放读回数据的内存>len)
			len(数据长度）
			release(传输完成后是否释放总线CS置高 0：不释放 1：释放）
* 返回参数  : 0：NO_ERR
*******************************************************************************/
uint8_t SD_ReceiveData(uint8_t *data, uint16_t len, uint8_t release)
{
    // 启动一次传输
    SD_CS_Low();
	if(SD_GetResponse(0xFE))//等待SD卡发回数据起始令牌0xFE
	{	  
		SD_CS_High();
		return 1;
	}
    while(len--)//开始接收数据
    {
        *data=OSBsp.Device.Spi2.WriteReadData(0xFF);
        data++;
    }
    //下面是2个伪CRC（dummy CRC）
    OSBsp.Device.Spi2.WriteReadData(0xFF);
    OSBsp.Device.Spi2.WriteReadData(0xFF);
    if(release==RELEASE)//按需释放总线，将CS置高
    {
        SD_CS_High();//传输结束
        OSBsp.Device.Spi2.WriteReadData(0xFF);
    }

    return 0;
}

/*******************************************************************************
* 函数名	: SD_GetCID
* 描述	: 获取SD卡的CID信息，包括制造商信息
* 输入参数  : *cid_data(存放CID的内存，至少16Byte）
* 返回参数  : 0：NO_ERR
			1：TIME_OUT
			other：错误信息
*******************************************************************************/
uint8_t SD_GetCID(uint8_t *cid_data)
{
	uint8_t r1;
    //发CMD10命令，读CID
    r1 = SD_SendCommand(CMD10,0,0xFF);
    if(r1 != 0x00)
    	return r1;  						//没返回正确应答，则退出，报错

    SD_ReceiveData(cid_data,16,RELEASE);	//接收16个字节的数据

    return 0;
}

/*******************************************************************************
* 函数名	: SD_GetCSD
* 描述	: 获取SD卡的CSD信息，包括容量和速度信息
* 输入参数  : *cid_data(存放CID的内存，至少16Byte）
* 返回参数  : 0：NO_ERR
			1：TIME_OUT
			other：错误信息
*******************************************************************************/
uint8_t SD_GetCSD(uint8_t *csd_data)
{
	uint8_t r1;
    r1=SD_SendCommand(CMD9,0,0xFF);				//发CMD9命令，读CSD
    if(r1)
    	return r1; 							 	//没返回正确应答，则退出，报错

    SD_ReceiveData(csd_data, 16, RELEASE);		//接收16个字节的数据

    return 0;
}  

/*******************************************************************************
* 函数名	: SD_GetCapacity
* 描述	: 获取SD卡的容量（字节）
* 输入参数  : 无
* 返回参数  : SD卡容量
*******************************************************************************/
uint32_t SD_GetCapacity(void)
{
	uint8_t csd[16];
	uint32_t Capacity;
	uint8_t r1;
	uint16_t kk;
	uint16_t temp;

	//取CSD信息，如果期间出错，返回0
    if(SD_GetCSD(csd)!=0)
    	return 0;
    //如果为SDHC卡，按照下面方式计算
    if((csd[0]&0xC0)==0x40)
    {									  
	    Capacity=((uint32_t)csd[8])<<8;
		Capacity+=(uint32_t)csd[9]+1;
        Capacity = (Capacity)*1024;//得到扇区数
		Capacity*=512;//得到字节数
    }
    else
    {		    
    	kk = csd[6]&0x03;
    	kk <<=8;
    	kk += csd[7];
    	kk <<=2;
    	kk += ((csd[8]&0xc0)>>6);
        //C_SIZE_MULT
    	r1 = csd[9]&0x03;
    	r1<<=1;
    	r1 += ((csd[10]&0x80)>>7);	 
    	r1+=2;//BLOCKNR
    	temp = 1;
    	while(r1)
    	{
    		temp *= 2;
    		r1 --;
    	}
    	Capacity = ((uint32_t)(kk+1))*((uint32_t)temp);
        // READ_BL_LEN
    	kk = csd[5]&0x0f;
        //BLOCK_LEN
    	temp = 1;
    	while(kk)
    	{
    		temp*=2;
    		kk--;
    	}
        //The final result
    	Capacity *= (uint32_t)temp;   //字节为单位
    }

    return (uint32_t)Capacity;
}	   																			    

/*******************************************************************************
* 函数名	: SD_ReadSingleBlock
* 描述	: 读SD卡的一个block
* 输入参数  : sector 取地址（sector值，非物理地址）
			*buffer 数据存储地址（大小至少512byte）
* 返回参数  : 0： 成功
	     	other：失败
*******************************************************************************/
uint8_t SD_ReadSingleBlock(uint32_t sector, uint8_t *buffer)
{
	uint8_t r1;
  		   
    //如果不是SDHC，给定的是sector地址，将其转换成byte地址
    if(SD_Type!=SD_TYPE_V2HC)
    {
        sector = sector<<9;
    } 
	r1 = SD_SendCommand(CMD17, sector, 0);//读命令
	if(r1 != 0x00)
		return r1;

	r1 = SD_ReceiveData(buffer, 512, RELEASE);

	if(r1 != 0)
		return r1;   //读数据出错！
    else
    	return 0;
}


////////////////////////////下面2个函数为USB读写所需要的/////////////////////////
//定义SD卡的块大小
#define BLOCK_SIZE 512 


/*******************************************************************************
* 函数名	: MSD_WriteBuffer
* 描述	: 写入MSD/SD数据
* 输入参数  : pBuffer:数据存放区
			ReadAddr:写入的首地址
			NumByteToRead:要写入的字节数
* 返回参数  : 0： 写入完成
	     	other：失败
*******************************************************************************/
uint8_t MSD_WriteBuffer(uint8_t *pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
	uint32_t i,NbrOfBlock = 0, Offset = 0;
	uint32_t sector;
	uint8_t r1;

   	NbrOfBlock = NumByteToWrite / BLOCK_SIZE;//得到要写入的块的数目

    SD_CS_Low();
	while (NbrOfBlock--)			//写入一个扇区
	{
		sector=WriteAddr+Offset;
		if(SD_Type==SD_TYPE_V2HC)
			sector>>=9;				//执行与普通操作相反的操作
		r1=SD_SendCommand_NoDeassert(CMD24,sector,0xff);//写命令
 		if(r1)
		{
			SD_CS_High();

			return 1;//应答不正确，直接返回
	    }

	    OSBsp.Device.Spi2.WriteReadData(0xFE);//放起始令牌0xFE
	    //放一个sector的数据
	    for(i=0;i<512;i++)
	    	OSBsp.Device.Spi2.WriteReadData(*pBuffer++);
	    //发2个Byte的dummy CRC
	    OSBsp.Device.Spi2.WriteReadData(0xff);
	    OSBsp.Device.Spi2.WriteReadData(0xff); 
 		if(SD_WaitDataReady())//等待SD卡数据写入完成
		{
			SD_CS_High();
			return 2;    
		}
		Offset += 512;	   
	}	    
    //写入完成，片选置1
    SD_CS_High();
    OSBsp.Device.Spi2.WriteReadData(0xff);	 

    return 0;
}

/*******************************************************************************
* 函数名	: MSD_ReadBuffer
* 描述	: 读取MSD/SD数据
* 输入参数  : pBuffer:数据存放区
			ReadAddr:读取的首地址
			NumByteToRead:要读取的字节数
* 返回参数  : 0： 完成
	     	other：失败
*******************************************************************************/
uint8_t MSD_ReadBuffer(uint8_t *pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	uint32_t NbrOfBlock=0,Offset=0;
	uint32_t sector=0;
	uint8_t r1=0;
  	NbrOfBlock=NumByteToRead/BLOCK_SIZE;	  
    SD_CS_Low();

	while (NbrOfBlock --)
	{	
		sector=ReadAddr+Offset;
		if(SD_Type==SD_TYPE_V2HC)
			sector>>=9;										//执行与普通操作相反的操作
		r1=SD_SendCommand_NoDeassert(CMD17,sector,0xff);	//读命令
		if(r1)//命令发送错误
		{
    		SD_CS_High();
			return r1;
		}	   							  
		r1=SD_ReceiveData(pBuffer,512,RELEASE);		 
		if(r1)//读数错误
		{
    		SD_CS_High();
			return r1;
		}
		pBuffer+=512;	 					    
	  	Offset+=512;				 	 
	}	 	 
    SD_CS_High();
    OSBsp.Device.Spi2.WriteReadData(0xff);

    return 0;
}

/*******************************************************************************
* 函数名	: SD_WriteSingleBlock
* 描述	: 写入SD卡的一个block(未实际测试过)
* 输入参数  : sector 扇区地址（sector值，非物理地址）
			*buffer 数据存储地址（大小至少512byte）
* 返回参数  : 0： 完成
	     	other：失败
*******************************************************************************/
uint8_t SD_WriteSingleBlock(uint32_t sector, const uint8_t *data)
{
	uint8_t r1;
	uint16_t kk;
	uint16_t retry;


    if(SD_Type != SD_TYPE_V2HC)
    {
        sector = sector<<9;
    }   
    r1 = SD_SendCommand(CMD24, sector, 0x00);
    if(r1 != 0x00)
    {
        return r1;  //应答不正确，直接返回
    }
    
    //开始准备数据传输
    SD_CS_Low();
    //先放3个空数据，等待SD卡准备好
    OSBsp.Device.Spi2.WriteReadData(0xff);
    OSBsp.Device.Spi2.WriteReadData(0xff);
    OSBsp.Device.Spi2.WriteReadData(0xff);
    //放起始令牌0xFE
    OSBsp.Device.Spi2.WriteReadData(0xFE);

    //放一个sector的数据
    for(kk=0;kk<BLOCK_SIZE;kk++)
    {
        OSBsp.Device.Spi2.WriteReadData(*data++);
    }
    //发2个Byte的dummy CRC
    OSBsp.Device.Spi2.WriteReadData(0xff);
    OSBsp.Device.Spi2.WriteReadData(0xff);
    
    //等待SD卡应答
    r1 = OSBsp.Device.Spi2.WriteReadData(0xff);
    if((r1&0x1F)!=0x05)
    {
        SD_CS_High();
        return r1;
    }
    
    //等待操作完成
    retry = 0;
    while(!OSBsp.Device.Spi2.WriteReadData(0xff))
    {
        retry++;
        if(retry>0xfffe)        //如果长时间写入没有完成，报错退出
        {
            SD_CS_High();
            return 1;           //写入超时返回1
        }
    }	    
    //写入完成，片选置1
    SD_CS_High();
    OSBsp.Device.Spi2.WriteReadData(0xff);

    return 0;
}				           

/*******************************************************************************
* 函数名	: SD_ReadMultiBlock
* 描述	: 读SD卡的多个block(实际测试过)
* 输入参数  : sector 扇区地址（sector值，非物理地址）
			*buffer 数据存储地址（大小至少512byte）
			count 连续读count个block
* 返回参数  : 0： 完成
	     	other：失败
*******************************************************************************/
uint8_t SD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint8_t count)
{
	uint8_t r1;
    //SPIx_SetSpeed(SPI_SPEED_HIGH);//设置为高速模式
 	//如果不是SDHC，将sector地址转成byte地址
    if(SD_Type!=SD_TYPE_V2HC)sector = sector<<9;  
    //SD_WaitDataReady();
    //发读多块命令
	r1 = SD_SendCommand(CMD18, sector, 0);//读命令
	if(r1 != 0x00)
		return r1;
    do//开始接收数据
    {
        if(SD_ReceiveData(buffer, 512, NO_RELEASE) != 0x00)
        	break;

        buffer += 512;
    } while(--count);		 
    //全部传输完毕，发送停止命令
    SD_SendCommand(CMD12, 0, 0);
    //释放总线
    SD_CS_High();
    OSBsp.Device.Spi2.WriteReadData(0xFF);    

    if(count != 0)
    	return count;   //如果没有传完，返回剩余个数
    else
    	return 0;
}											  

/*******************************************************************************
* 函数名	: SD_ReadMultiBlock
* 描述	: 写入SD卡的N个block(未实际测试过)
* 输入参数  : sector 扇区地址（sector值，非物理地址）
			*buffer 数据存储地址（大小至少512byte）
			count 写入的block数目
* 返回参数  : 0： 完成
	     	other：失败
*******************************************************************************/
uint8_t SD_WriteMultiBlock(uint32_t sector, const uint8_t *data, uint8_t count)
{
	uint8_t r1;
	uint16_t kk;
    //SPIx_SetSpeed(SPI_SPEED_HIGH);//设置为高速模式
    if(SD_Type != SD_TYPE_V2HC)sector = sector<<9;///如果不是SDHC，给定的是sector地址，将其转换成byte地址
    if(SD_Type != SD_TYPE_MMC) r1 = SD_SendCommand(ACMD23, count, 0x00);//如果目标卡不是MMC卡，启用ACMD23指令使能预擦除
    r1 = SD_SendCommand(CMD25, sector, 0x00);//发多块写入指令
    if(r1 != 0x00)return r1;  //应答不正确，直接返回
    SD_CS_Low();//开始准备数据传输
    OSBsp.Device.Spi2.WriteReadData(0xff);//先放3个空数据，等待SD卡准备好
    OSBsp.Device.Spi2.WriteReadData(0xff);   
    //--------下面是N个sector写入的循环部分
    do
    {
        //放起始令牌0xFC 表明是多块写入
        OSBsp.Device.Spi2.WriteReadData(0xFC);	  
        //放一个sector的数据
        for(kk=0;kk<BLOCK_SIZE;kk++)
        {
            OSBsp.Device.Spi2.WriteReadData(*data++);
        }
        //发2个Byte的dummy CRC
        OSBsp.Device.Spi2.WriteReadData(0xff);
        OSBsp.Device.Spi2.WriteReadData(0xff);
        
        //等待SD卡应答
        r1 = OSBsp.Device.Spi2.WriteReadData(0xff);
        if((r1&0x1F)!=0x05)
        {
            SD_CS_High();    //如果应答为报错，则带错误代码直接退出
            return r1;
        }		   
        //等待SD卡写入完成
        if(SD_WaitDataReady()==1)
        {
            SD_CS_High();    //等待SD卡写入完成超时，直接退出报错
            return 1;
        }	   
    }while(--count);//本sector数据传输完成
    //发结束传输令牌0xFD
    r1 = OSBsp.Device.Spi2.WriteReadData(0xFD);
    if(r1==0x00)
    {
        count =  0xfe;
    }		   
    if(SD_WaitDataReady()) //等待准备好
	{
		SD_CS_High();
		return 1;  
	}
    //写入完成，片选置1
    SD_CS_High();
    OSBsp.Device.Spi2.WriteReadData(0xff);  

    return count;   //返回count值，如果写完则count=0，否则count=1
}						  					  
//
//输入:
//返回值:0： 成功
//       other：失败
/*******************************************************************************
* 函数名	: SD_Read_Bytes
* 描述	: 在指定扇区,从offset开始读出bytes个字节
* 输入参数  : sector 扇区地址（sector值，非物理地址）
			*buf   数据存储地址（大小<=512byte）
			offset 在扇区里面的偏移量
			bytes  要读出的字节数
* 返回参数  : 0： 完成
	     	other：失败
*******************************************************************************/
uint8_t SD_Read_Bytes(uint32_t address,uint8_t *buf,uint16_t offset,uint16_t bytes)
{
	uint8_t r1;
	uint16_t kk=0;
    r1=SD_SendCommand(CMD17,address<<9,0);//发送读扇区命令
    if(r1)
	return r1;  			//应答不正确，直接返回

	SD_CS_Low();			//选中SD卡
	if(SD_GetResponse(0xFE))//等待SD卡发回数据起始令牌0xFE
	{
		SD_CS_High(); 		//关闭SD卡
	return 1;				//读取失败
	}	 
	for(kk=0;kk<offset;kk++)
		OSBsp.Device.Spi2.WriteReadData(0xff);		//跳过offset位
    for(;kk<offset+bytes;kk++)
    	*buf++=OSBsp.Device.Spi2.WriteReadData(0xff);//读取有用数据
    for(;kk<512;kk++)
    	OSBsp.Device.Spi2.WriteReadData(0xff); 	 	//读出剩余字节

    OSBsp.Device.Spi2.WriteReadData(0xff);				//发送伪CRC码
    OSBsp.Device.Spi2.WriteReadData(0xff);  
    SD_CS_High();						//关闭SD卡

	return 0;
}




