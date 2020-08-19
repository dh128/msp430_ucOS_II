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
* Filename      : FileTxt.c
* Version       : V1.00
* Programmer(s) : GLZ
*********************************************************************************************************
*/
#include <bsp.h>

int16_t i;
FATFS fs;
FRESULT res;
FILINFO fileInfo;
uint16_t br,bw;
DIR dir;
FIL fsrc;

//åˆ›å»ºæ–‡ä»¶è·¯å¾„
void g_SD_FileName_Creat(const char *docPath,uint8_t *date,char *filename)
{
	char date_string[7];
	date_string[0]=((date[1]>>4)&0x0f)+0x30;	    //1
	date_string[1]=(date[1]&0x0f)+0x30;			    //7
	date_string[2]=((date[2]>>4)&0x0f)+0x30;		//0
	date_string[3]=(date[2]&0x0f)+0x30; 		    //3
	date_string[4]=((date[3]>>4)&0x0f)+0x30;		//0
	date_string[5]=(date[3]&0x0f)+0x30; 		    //6

	sprintf(filename,"%s%s.txt",docPath,date_string);
	g_Printf_dbg("%s : %s\n\r",__func__,filename);
}

void g_SD_File_Write(const char *file_path,const char *dat)
{
	SD_CS_Low ();
	g_Printf_dbg("write file to %s......\n\r",file_path);
	res=f_mount(0, &fs);
	res=f_open(&fsrc,(char *)file_path, FA_OPEN_ALWAYS | FA_WRITE);
	if(res != FR_OK){
		uint32_t res_s = res;
		g_Printf_dbg("open file error = %d\n\r",res_s);
	}else{
		g_Printf_dbg("open file OK!\n\r");
		f_lseek(&fsrc,fsrc.fsize);                      //移动指针到末�?
		res = f_write(&fsrc, dat, strlen(dat), &bw);    /* Write it to the dst file */
		if(res == FR_OK){
			uint32_t num_w;
			num_w=strlen(dat);
			g_Printf_dbg("%d bytes has been writted!\r\n",num_w);
			g_Printf_dbg("write data ok! \n\r");
		}else{
			g_Printf_dbg("write data error \n\r");
		}
		/*close file */
		f_close(&fsrc);
	}
	f_mount(0,NULL);
	SD_CS_High ();
}
void del_txt(char *direction,char *file_path)//É¾³ýÎÄ¼þ
{
	res = f_mount(0, &fs);
	res = f_opendir(&dir , direction);
	if(res != FR_OK)
	{
		g_Printf_dbg("open dir error\r\n");

	}
	else
	{
		g_Printf_dbg("open dir success\r\n");
		f_chdir(direction);
	}

	res = f_unlink(file_path);
	if(res != FR_OK)
	{
		g_Printf_dbg("del file %s error\r\n",file_path);
	}
	else
	{
		g_Printf_dbg("del file %s  ok\r\n",file_path);
	}

	f_mount(0,NULL);
}
//ÏòÖ¸¶¨Ä¿Â¼ÖÐÐ´ÎÄ¼þ
char Write_ToDirTxt(char *direction,char *file_path,char *dat)
{
	char result = 0;
//	char *path[30] = {"0:/RESP/1.txt"};
	res = f_mount(0, &fs);
	res = f_opendir(&dir , direction);
	if(res != FR_OK)
	{
		g_Printf_dbg("open dir error\r\n");
		res = f_mkdir(direction);
		if(res != FR_OK)
		{
			g_Printf_dbg("make dir error\r\n");
			result = 0;
		}
		else
		{
			g_Printf_dbg("make dir success\r\n");
			res = f_opendir(&dir , direction);
			f_chdir(direction);
		}
	}
	else
	{
		g_Printf_dbg("open dir success\r\n");
		f_chdir(direction);
	}
//	strcpy(path , direction);
//	strcat(path , file_path);
	res = f_open(&fsrc,file_path, FA_OPEN_ALWAYS | FA_WRITE);

	if(res != FR_OK)
	{
		g_Printf_dbg("open file error\r\n");
		result = 0;
	}
	else
	{
		g_Printf_dbg("open file OK!\r\n");
		f_lseek(&fsrc,fsrc.fsize);                     		 // ÒÆ¶¯Ö¸Õëµ½Ä©Î²
		res = f_write(&fsrc, dat, strlen(dat), &bw);         // Write it to the dst file
//		res = f_write(&fsrc, "\r\n", 2, &bw);         // Write it to the dst file
//		if(res == FR_OK)
//		{
//			g_Printf_dbg("write data ok! \r\n");
//		}
//		else
//		{
//			g_Printf_dbg("write data error \r\n");
//		}
		/*close file */
		f_close(&fsrc);
		result = 1;
	}
	f_mount(0,NULL);
	return result;
}
//¶ÁÈ¡ÎÄ¼þÖÐ×Ö·û´®
char Get_String(char *direction,char *file_path,char *buffer,int len)
{
	char result = 0;
	res = f_mount(0, &fs);
	res = f_opendir(&dir , direction);
	if(res != FR_OK)
	{
		g_Printf_dbg("open dir error\r\n");
		result = 0;
	}
	else
	{
		g_Printf_dbg("open dir success\r\n");
		f_chdir(direction);
	}

	res = f_open(&fsrc,file_path, FA_OPEN_ALWAYS | FA_READ);

	if(res != FR_OK)
	{
		g_Printf_dbg("open file error\r\n");
		result = 0;
	}
	else
	{
		g_Printf_dbg("open file OK!\r\n");
//		f_lseek(&fsrc,fsrc.fsize);                     		 // ÒÆ¶¯Ö¸Õëµ½Ä©Î²
//		res = f_gets(buffer, len , &fsrc);         // Write it to the dst file
		if(f_gets(buffer, len , &fsrc) != NULL)
		{
			g_Printf_dbg("f_gets ok \r\n");
			g_Printf_dbg(buffer);
			result = 1;

		}
		else
		{
			g_Printf_dbg("f_gets error\r\n");

			result = 2;			//ÓÃÓÚÉ¾³ý±êÖ¾
//			f_chdir(direction);
		}
		/*close file */
		f_close(&fsrc);
		if(result == 2)
		{
			result = 0;			//É¾³ýºó·µ»Ø0£¬Ã»ÓÐ»ñÈ¡µ½×Ö·û´®
			res=f_unlink ((char *)file_path);
			if(res != FR_OK)
			{
				g_Printf_dbg("del file error\r\n");
			}
			else
			{
				g_Printf_dbg("del file ok\r\n");
			}
		}

	}
	f_mount(0,NULL);
	return result;
}

