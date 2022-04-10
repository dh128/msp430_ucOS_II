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
### 2. git clone https://github.com/dh128/msp430_ucOS_II.git ###
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


6、添加NB发送数据失败和初始化失败情况，增加NB入网失败清除频点操作，断电进入低功耗，退出低功耗后重新上电；
7、修复不添加纪录传感器采集缺失BUG；
8、去掉NB入网判断中的数据包长度；
9、去掉补传功能；

2020-7-29	dingh
1、修改RTC中唤醒判断方式，采用分钟取余方法，可整点采集上报。
2、添加初始化时读取硬件时钟到MCU内部，增加同步时钟功能同步至内部RTC，无网络校时的时候可以使用硬件时钟同步MCU内部RTC。

2020-8-6	dingh
1、更改开机第一次不上报数据，等待周期唤醒后上报数据；
2、更改退出低功耗启动系统时钟位置到最后，等待所有配置全部完成后再启动；
3、独立NB网络校时功能为一个函数，且每24小时校时一次；
4、删除不需要退出低功耗的串口中断函数里的退出低功耗函数；
5、进入低功耗关闭传感器串口3接收中断，退出低功耗时再打开。

2020-8-9	dingh
1、删除看门狗任务函数中的log输出；
2、长时间处于DEVICE_STATUS_POWER_SCAN_OVER时判断是否处于升级状态，NB升级则不进入低功耗；
3、添加scada_over_times和scada_idle_times清零操作；
4、设置P2.3(对应NB模块的RI)为输入，RI为输出，0-3V；
5、添加W25Q16电源控制宏；
6、调整NB发送数据及接收数据处理过程，优化NB升级过程，获取数据过程禁止任务切换，读写SPI flash过程中添加临界态保护。

2020-8-12	dingh
1、修改水雨情设备传感器数量2，删除无用的采集，组包；
2、NB升级前判断电量是否大于50%，信号信噪比是否大于0，不符合条件的不更新；
3、NB升级过程中读取写入Flash数据，发现数据写入异常则停止升级操作；

2020-8-23	dingh
1、修改校时周期为1小时。

2020-8-24	dingh
1、修改SPI时钟为2M；
2、升级下载完成后输出code,等待10s,然后重启。
2020-8-25
1、添加goto语句，检验SPI写入数据是否正常；
2、升级过程中关闭其他电源；
3、W25x16改用分页写；
2020-8-27
1、进低功耗前不修改串口配置，仅关闭串口中断；
2、退出低功耗打开串口中断。
2020-9-3	dingh	WRain
1、关闭SVS 002.txt测试865057041800016
2020-9-4	dingh	WRain
1、NB升级时关闭SD卡电源，现有电路要么拔掉SD卡，要么去除三线上拉电阻；
2、添加升级中断后继续获取数据功能；
3、NB采用电池供电方式工作；

2020-9-9	dingh	WRain
1、NB接收固件时关闭传感器电源，确保电源相对稳定；
2、固件校验出错，增加W25x16初始化；
3、w25x16删除0x10000时的延时。经过验证，效果不太稳定；

2020-9-14	dingh	WRain
1、BSP\Devicelib\g_DeviceSPIFlash.c 取消w25x16驱动中也操作写数据，整页时地址清零；
2、BSP\Devicelib\g_DeviceNB.c 擦除Flash时添加临界态保护;
3、BSP\Devicelib\g_DeviceNB.c 添加下发修改周期后同步发送数据包周期字段。

2020-10-27	dingh	WRain
1、取消PMM_disableSvsLSvmL，测试发现无效；
2、24小时后seqno清零，同时软件重启模组，避免网络校时偏差；
3、修改LPM3回LPM0；
4、进低功耗前修改串口为低电平输出。

2020-11-03	dingh	WRain
1、取消退出低功耗时串口2恢复功能，测试7天正常。

2020-11-27	dingh	Cat1
1、从WRain程序中checkout到Cat1分支，开发cat1模组上报数据功能,打开Debug串口，用于接收log；
2、cat1模组采用上海合宙公司的Air724UG，修改GPRS程序中的部分指令配置，时钟读、存；
3、修改水质程序，删除GPS信息组包，取消外部时钟读取，直接采用内部时钟，天机默认WL字段上报，不然平台不显示；
4、修改GPRS程序中入网，校时，上报功能。

2020-11-30	dingh	Cat1
1、修改GPRS程序中cat1模组入网失败处理，CGATT？失败后，手动入网无效，所有失败均须重启模组。

2020-12-15	dingh	WRain
1、取消hal_layer_api.c文件中退出低功耗时使能雨量采集；
2、在RTC中断中增加min==0时，使能雨量采集，实现雨量整点采集上报。
2020-12-17	dingh	WRain
1、修改雨量位周期检测，每小时上报一次1小时内雨量，0点上报24小时内雨量;
2、修改周期雨量与液位传感器值存放位置；

2020-12-20	dingh	Flow
1、修改流量计设备采集，只保留温度、流速、流量、液位参数；
2、添加流量计安装参数配置，使用0xFB指令配置流量计管道参数（目前只支持圆和矩形）；
3、添加流量计算算法，根据配置参数计读取的流速、液位信息，计算流量（目前只支持圆和矩形）；
4、添加NB初始化失败后进低功耗前判断设备状态是否处于采集完，防止串口唤醒后立即又进低功耗。

2021-1-5	dingh	mqtt
1、添加NBIoT_MQTT_Ali 传输类型，用于NB对接阿里云平台
2、添加易涝点检测站，检测水位数据；
3、修改Flash存储数据区阿里云三元组位置为0x1939开始；
4、修改配置程序，将阿里云配置优先判断，避免数据长度一致冲突；

2021-1-8	dingh	mqtt
1、修改上报数据字段类型。

2021-1-11	dingh	develop
1、修改MQTT客户对接地址。

2021-1-13	dingh	develop
1、修改阿里云三元组存放位置为0x1880,每个长度32+1字节；
2、读取三元组时判断长度超过32则认为时非法数据。

2021-1-17	dingh	develop
1、修改设备温度取值方法：
	修改前：
		原先程序每次读取所有设备的温度值，每一轮读取完毕后使用读取的温度值去平均数；
	修改后:
		读取温度后修改温度标志位，低标志位的被高标志位替换。每一轮值留一个设备的温度值，最后通过滤波处理；
2、删除水质代码中无用、注释掉的代码；

2021-1-19	dingh	magic
1、添加MAGICSTCK设备类型；
2、修改NB对接电信IoT前发送自动注册指令；
3、添加定时器退出低功耗时设备等待120s再运行功能。

2021-1-25	dingh	magic
1、取消COD解析中数据值判断;
2、修改magic和water中COD数值NB上报精度为2位小数；
3、修复magic和water中ORP数据上报错误；

2021-1-26	dingh	magic
1、cond的值解析乘以1000，单位换算成是us/cm;
2、COD的延时需要增加到150s，避免出现0的问题。

2021-4-20	dingh	develop
1、修改气象设备多参数传感器采集程序，改用一次性读取全部数据；
2、雨量采用雨量桶传感器，添加小时雨量，日降雨量变量；
3、修改气象设备上报数据协议位置；
2021-4-21	dingh	develop
1、修改传感器标志位清零位置；
2、添加气象设备雨量数据清零log。
2021-4-22	dingh	develop
1、修改气象设备退出低功耗赋值雨量监测标志位，解决没有雨量小时、天数据问题。
2021-4-23	dingh	develop
1、修改日降雨量上报时间为8点。
2021-4-25
1、修改hal_layer_api.c中MQTT参数获取函数条件编译，GPRS_MODE参加编译。

2021-12-05	dingh	watergague
1、修改电子水尺传感器解析，读取时间，seeperStation，读取波特率4800；
2、添加电信AEP平台上报数据对接，物模型，json格式。
2021-12-07	dingh	watergague
1、添加AEP平台json下发解析；
2、seeperStation 添加json组包判断。
2021-12-17	dingh	AEP
1、修改AEP组包过程中while循环类型bug。
2021-12-22	dingh	AEP
1、添加AirStation，检测空气（RK一体式传感器：温湿度、PM、气压、噪声）+TVOC；
2、删除AEP.c中无用变量;
3、cJson中浮点数改用%g格式输出，小数后面的0舍去。
2021-12-23	dingh AEP
1、添加水质设备解析接入AEP平台。
2021-12-27	dingh AEP
1、删除滤波处理中TUB标志位置位。
2022-01-23	dingh	AEP (v007版本)
1、恢复上一版多注释的TemperatureStatus=0操作，恢复温度变化；
2、取消水质参数数据范围限制，直接上报采集值。

2022-02-19	dingh	AEP
1、[g_DeviceRTC.c] [covUnixTimeStp2Beijing] 月底最后一天0区16时之后day出错BUG修复；
2、[g_DeviceWrain_Station.c]超声波传感器地址改成1，液位字段修改为Ultrasonic，增加Real和height(下发高程数据)。
2022-2-27	dingh	AEP
1、增加g_PitWell_Station，窨井液位设备，检测投入式液位和超声波液位传感器；
2、关闭GPS配件，设备为ACCESSORY_TYPR 为 None;
3、修改App.c 中修改IntegratedPitWell时，检测超时时间为10s。
2022-3-9	dingh	AEP
1、增加g_PipeFlow_Station，在流量计的基础上实现窨井流量监测；
2、修改g_printf的打印buffer为512字节；
3、修改NB AEP发送循环为uint32_t类型。
2022-3-13	dingh	AEP
1、加入NB入网超时重启模组的功能;
2、重写传感器记录的代码，不保存在flash里，直接ram保存;
3、重写传感器记录初步测试，Water、WRain、PitWell、PipeFlow四类设备。
2022-3-19	dingh	AEP
1、添加管网流量设备下发管道参数，下发参数单位采用cm。
2022-4-10	dingh	AEP
1、修改窨井液位设备周期超过60min时，按正点上报数据。