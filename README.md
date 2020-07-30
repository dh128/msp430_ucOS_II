	 ________         __            _________          
	/ ______|        |  |          |____    _|         
	| |   _____      |  |               /  /           
	| |  |_   _|     |  |              /  /            
	| |____| |       |  |____       __/  /____         
	\________|       |_______|     |__________|        
 Copyright (C) 2018 - present, liangzhiGuo, <lz_kwok@163.com>, et al 
 msp430_ucOS_II
## Note : ##
### 1. The branch of "master" is the original branch, so you can't upload code at will. You can test your own code in the branch of "develop" and Refer to the following steps: ###
### 2. git clone https://github.com/lz-kwok/msp430_ucOS_II.git ###
### 3. git checkout develop ###
### 4. git branch ###
to check if your local branch is "develop"
=============================================================================================================================================================================
## 1. hal_layer_api.c ##
### 1.1. int Hal_ThreadCreate(void (*func)(void *p_arg), void *funcname,OS_STK *TaskStk, int priority)  ###
Create tasks and set stack pointers, sizes, and priorities for tasks

### 1.2. int Hal_ThreadDestory(int priority)  ###
Destroy tasks, enter parameters as priority of tasks

### 1.3. int Hal_QueueCreate(void **start,int size)  ###
Create a message queue with parameters of array pointer and queue size. The size of array pointer should be no less than queue size.

### 1.4. int Hal_QueueDestory(Queue_t queue)  ###
Destroy message queue with input parameter as queue pointer to destroy

### 1.5. Hal_QueueSend(Queue_t queue, struct hal_message* msg, int timeout)  ###
Send message function. Send message is a structure pointer, which includes message type and message content.

### 1.6. int Hal_QueueNum_Waitfor_Pend(Queue_t queue) ###
query Number of messages waiting in the queue

### 1.7. Hal_QueueRecv(Queue_t queue, struct hal_message* msg, int timeout)  ###
Receiving message function, receiving message is a structure pointer, which includes message type and message content.

### 1.8. Mutex_t Hal_MutexCreate(int priority) ###
Create a mutually exclusive semaphore with input parameters as the priority of the semaphore

### 1.9. void Hal_MutexDestory(Mutex_t mutex) ###
Destroy mutually exclusive semaphore and input parameter is the pointer of the semaphore

### 1.10. void Hal_MutexLock(Mutex_t mutex) ###
Obtain the right to use semaphores

### 1.11. void Hal_MutexUnlock(Mutex_t mutex) ###
Release semaphore usage rights

### 1.12. void Hal_EnterLowPower_Mode(void) ###
Enter Low Power Mode

### 1.13. void Hal_ExitLowPower_Mode(void) ###
Exit Low Power Mode

### 1.14. void Hal_GetTimeOfDay(struct hal_timeval* tv) ###
Acquire relative time value based on system ticks.

## 2. app.c ##
There are four threads in the app.c,which are scada task, transmission task , management task and IDLE task
### 2.1. static  void  ScadaTaskStart (void *p_arg)  ###

2020-6-9
黄振修改后代码提交

2020-6-11	dingh
1、串口程序中添加bRxBuffer和bRxNum变量，接收GPS参数（GPS只接受$GNRMC数据），接收到“,A,”确认有效数据，则解析接收数据；
2、修改GPS解析程序，解析单位为度，SD卡中存储字符串格式，NB上报四字节小端模式浮点数；
3、进入低功耗断开GPS电源，修改串口TX IO,退出低功耗后打开GPS电源，初始化串口1;
4、修复NB程序中部分形参格式警告。
2020-6-23	dingh
1、修改系统时钟为TimerB0定时器，节拍依旧为2ms一次；
2、修改gMutex信号优先级，之前等级==15创建失败；
3、启用软件看门狗1小时8分16秒，并添加看门狗任务，用户任务中优先级最高，接收其他任务的喂狗标志组之后喂狗，否则等待；
4、组包Json数据时，添加复位变量组包功能，当复位变量不为0时，添加reboot字段。
2020-7-6	dingh
1、修改气象设备发送数据60字节初始内容；
2、修改已有设备数据解析要求字节长度超过2Bytes;
3、添加SD驱动中retry参数++，避免SD操作卡死；
4、json组包添加reboot字段。

2020-7-8	huangzh
1、添加水雨情设备类型；
2、修改气象设备模拟数据光照赋值bug。

2020-7-9	dingh
1、修改NB进入低功耗位置，低功耗下串口可以唤醒配置；
2、添加入网失败进入低功耗；

2020-7-10	dingh
1、修改NB发送数据，使用g_Device_NB_Send_Str函数；
2、添加MCU内部RTC同步时钟功能，避免外部始终异常；
3、修改UNIX时间戳函数，避免数值异常（如hour = 0）时异常；
4、修改传感器解析组包错误。
2020-7-15
1、g_Device_Config_QueuePost 测试发现连续发送串口消息会卡死，函数添加延时后正常
2、消息content定义为全局变量。

2020-7-22	黄振修改内容提交
1、美化g_Water_Station.c文件；
2、添加水雨情采集前清楚模拟参数；
3、更改NB入网方式鉴定。
2020-7-22	dingh
1、进入低功耗后关闭串口2，避免串口唤醒；
2、删除Scada_Idle态4min后重启；
3、添加Scada_Idle 态和Scan_Over态持续时间4min钟后进入低功耗；

2020-7-23	dingh
1、修改NB时钟同步，转UTC再转北京时间，避免出现大于24时情况。
2020-7-24	dingh
1、修复水雨情设备读取Flash存储SensorStatue错误，删除if(hal_GetBit(SensorStatus_L, 6));
2、设备初始化后SPI Flash的CS管脚置高，SD卡读写后CS置高；
3、删除RTC中断log输出；
4、添加NB发送数据组包reboot原因。

5、删除RTC中断判断是否处于低功耗，到时间直接执行退出低功耗，修改该任务状态出重启运行态。
6、添加NB发送数据失败和初始化失败情况，增加NB入网失败清除频点操作，断电进入低功耗，退出低功耗后重新上电；
7、修复不添加纪录传感器采集缺失BUG；
8、去掉NB入网判断中的数据包长度；
9、去掉补传功能；

2020-7-29	dingh
1、修改RTC中唤醒判断方式，采用分钟取余方法，可正点采集上报。
2、添加初始化时读取硬件时钟到MCU内部，增加同步时钟功能同步至内部RTC，无网络校时的时候可以使用硬件时钟同步MCU内部RTC。

