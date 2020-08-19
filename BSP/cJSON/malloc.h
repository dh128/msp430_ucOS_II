#ifndef __MALLOC_H
#define __MALLOC_H

#include  <bsp.h>


  
#ifndef NULL
#define NULL (void *)0
#endif

#define MEM_BLOCK_SIZE		    32  	  					            //内存块大小为32字节
#define MEM_MAX_SIZE		    8*1024  		  				        //最大管理内存 2K
#define MEM_ALLOC_TABLE_SIZE	MEM_MAX_SIZE/MEM_BLOCK_SIZE             //内存表大小
		 
//内存管理控制器
struct _m_mallco_dev
{
	void      (*init)(void);		    //初始化
	uint8_t   (*perused)(void);		  	//内存使用率
	uint8_t    *membase;				//内存池
	uint16_t   *memmap; 				//内存管理状态表
	uint8_t     memrdy; 				//内存管理是否就绪
};

extern struct _m_mallco_dev mallco_dev;	 //在mallco.c里面定义

void mymemset(void *s,uint8_t c,uint32_t count);	 //设置内存
void mymemcpy(void *des,void *src,uint32_t n);//复制内存 

void mem_init(void);					 //内存管理初始化函数
uint32_t mem_malloc(uint32_t size);		 //内存分配
uint8_t mem_free(uint32_t offset);		 //内存释放
uint8_t mem_perused(void);				 //获得内存使用率 
////////////////////////////////////////////////////////////////////////////////
//用户调用函数
void myfree(void *ptr);  			     //内存释放
void *mymalloc(uint32_t size);			 //内存分配
void *myrealloc(void *ptr,uint32_t size);//重新分配内存





#endif














