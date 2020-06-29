#ifndef G_DEVICESD_H
#define G_DEVICESD_H

#define SD_CS_High()	P7OUT |= BIT5		//SD_cs put high
#define SD_CS_Low()		P7OUT &=~BIT5		//SD_cs put low Enable

#define SD_DEBUG_NEXTLINE  0
#define SD_DEBUG_UPTIME    1
#define SD_DEBUG_INIT_OK   2
#define SD_DEBUG_INIT_ERR  3
#define SD_DEBUG_SCAN_OK   4


//SD传输数据结束后是否释放总线宏定义  
#define NO_RELEASE      0
#define RELEASE         1		 
// SD卡类型定义  
#define SD_TYPE_MMC     0
#define SD_TYPE_V1      1
#define SD_TYPE_V2      2
#define SD_TYPE_V2HC    4	   
// SD卡指令表  	   
#define CMD0    0       //卡复位
#define CMD1    1
#define CMD9    9       //命令9 ，读CSD数据
#define CMD10   10      //命令10，读CID数据
#define CMD12   12      //命令12，停止数据传输
#define CMD16   16      //命令16，设置SectorSize 应返回0x00
#define CMD17   17      //命令17，读sector
#define CMD18   18      //命令18，读Multi sector
#define ACMD23  23      //命令23，设置多sector写入前预先擦除N个block
#define CMD24   24      //命令24，写sector
#define CMD25   25      //命令25，写Multi sector
#define ACMD41  41      //命令41，应返回0x00
#define CMD55   55      //命令55，应返回0x01
#define CMD58   58      //命令58，读OCR信息
#define CMD59   59      //命令59，使能/禁止CRC，应返回0x00
//数据写入回应字意义
#define MSD_DATA_OK                0x05
#define MSD_DATA_CRC_ERROR         0x0B
#define MSD_DATA_WRITE_ERROR       0x0D
#define MSD_DATA_OTHER_ERROR       0xFF
//SD卡回应标记字
#define MSD_RESPONSE_NO_ERROR      0x00
#define MSD_IN_IDLE_STATE          0x01
#define MSD_ERASE_RESET            0x02
#define MSD_ILLEGAL_COMMAND        0x04
#define MSD_COM_CRC_ERROR          0x08
#define MSD_ERASE_SEQUENCE_ERROR   0x10
#define MSD_ADDRESS_ERROR          0x20
#define MSD_PARAMETER_ERROR        0x40
#define MSD_RESPONSE_FAILURE       0xFF



extern unsigned long cardSize;  //SD卡容量
extern unsigned char status;    //状态
extern uint8_t SD_Status;

void SD_Storage_DebugLog(uint8_t index);
uint8_t g_Device_SDCard_Check(void);
void SDsizeDisplay(unsigned long Capacity);
//函数申明区 
uint8_t SD_GetResponse(uint8_t Response);
uint8_t SD_WaitDataReady(void);
uint8_t SD_SendCommand(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t SD_SendCommand_NoDeassert(uint8_t cmd, uint32_t arg, uint8_t crc);
uint8_t g_Device_SD_Init(void);
uint8_t SD_Idle_Sta(void);

uint8_t SD_ReceiveData(uint8_t *data, uint16_t len, uint8_t release);
uint8_t SD_GetCID(uint8_t *cid_data);
uint8_t SD_GetCSD(uint8_t *csd_data);
uint32_t SD_GetCapacity(void);

//USB 读卡器 SD卡操作函数
uint8_t MSD_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);
uint8_t MSD_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);

uint8_t SD_ReadSingleBlock(uint32_t sector, uint8_t *buffer);
uint8_t SD_WriteSingleBlock(uint32_t sector, const uint8_t *buffer);
uint8_t SD_ReadMultiBlock(uint32_t sector, uint8_t *buffer, uint8_t count);
uint8_t SD_WriteMultiBlock(uint32_t sector, const uint8_t *data, uint8_t count);
uint8_t SD_Read_Bytes(uint32_t address,uint8_t *buf,uint16_t offset,uint16_t bytes);


#endif

