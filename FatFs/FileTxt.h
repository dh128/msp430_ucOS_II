#ifndef FILETXT_H
#define FILETXT_H




void g_SD_FileName_Creat(const char *docPath,uint8_t *date,char *filename);
void g_SD_File_Write(const char *file_path,const char *dat);
void del_txt(char *direction,char *file_path);
char Write_ToDirTxt(char *direction,char *file_path,char *dat);
char Get_String(char *direction,char *file_path,char *buffer,int len);

#endif
