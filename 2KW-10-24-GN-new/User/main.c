/*
*********************************************************************************************************
author:罗荣海
version:V1.0
data:2018.08.28

*********************************************************************************************************
*/
#include "includes.h"

#include "Task_Config.h"
#include "MT2000_protocol.h"
#include "my_protocol.h"
#include "hard_control.h"
#include "alarm.h"

#include "main.h"
#include "W5500.h"
#include "stand_wave_rate.h"
#include "firmware_upgrade.h"

/*
 * 外部声明
 *
 */

extern uint8_t test_hard_control_display;

/*
**********************************************************************************************************
											函数声明
**********************************************************************************************************
*/
static void vTimerCallback(xTimerHandle pxTimer);
static void vTimerCallback2(xTimerHandle pxTimer);
static void vTimerCallback3(xTimerHandle pxTimer);

static void AppTaskCreate (void);
static void AppObjCreate (void);

/*
**********************************************************************************************************
											变量声明
**********************************************************************************************************
*/
uint8_t g_fuc_cod[2];					//全局功能码
uint8_t g_fuc_codToPC[2];				//全局功能码

uint32_t			g_inquire_stamp = 0;	//查询包的时间戳
uint32_t			g_stamp_distance = 0;	//当前时间时间戳和系统的时间戳的距离

Sys_Status_t 	System;				//系统状态
Monitor_t			Monitor;			//硬件监控
Run_Diagram_data_t	Run_Diagram_data;	//获取到的运行图数据
Alarm_t				Alarm;				//报警

Alarm_threshold_t	Alarm_threshold;	//报警阈值				0101
PC_Cmd_t			PC_Cmd;				//PC指令				0301
Disalarm_t			Disalarm;			//PC解除报警			0501
Scan_Frq_t			Scan_Frq;			//扫描最佳工作频率		0901
Sacn_stop_t			Sacn_stop;			//停止扫频				0A01
Add_Power_t			Add_Power;			//功率增加				0B01
Sub_Power_t			Sub_Power;			//功率减小				0C01
Run_Diagram_t		Run_Diagram;		//运行图				0D01
Run_Diagram_buf_t	Run_Diagram_buf[10];//运行图缓存(100个)

/*---------------------------------------应答包-------------------------------------------------*/
Alarm_backPC_t		Alarm_backPC;		//报警参数初始化应答包	0102
Work_paraBack_t		Work_paraBack;		//查询应答包			0202
Trans_openBack_t	Trans_openBack;		//发射应答包			0302
Trans_stopBack_t	Trans_stopBack;		//停止发射应答包		0402
DisalarmBack_t		DisalarmBack;		//解除报警应答包		0502
Alarm_historyBack_t	Alarm_historyBack;	//历史报警应答包		0602
Power_onBack_t		Power_onBack;		//开机应答包			0702
Power_offBack_t		Power_offBack;		//关机应答包			0802
Scan_FrqBack_t		Scan_FrqBack;		//扫频应答包(扫频)		0902
Sacn_stopBack_t		Sacn_stopBack;		//停止扫频应答包		0A02
Add_PowerBack_t		Add_PowerBack;		//功率增加应答包		0B02
Sub_PowerBack_t		Sub_PowerBack;		//功率减小应答包		0C02
Run_DiagramBack_t	Run_DiagramBack;	//运行图应答包			0D02


Scan_FrqBack_t		Scan_FrqBack1;		//PC扫描最佳工作频率	(查询)0902	//3.2-4.9
Scan_FrqBack_t		Scan_FrqBack2;		//PC扫描最佳工作频率	(查询)0902	//5.0-6.9
Scan_FrqBack_t		Scan_FrqBack3;		//PC扫描最佳工作频率	(查询)0902	//7.0-8.9
Scan_FrqBack_t		Scan_FrqBack4;		//PC扫描最佳工作频率	(查询)0902	//9.0-10.9
Scan_FrqBack_t		Scan_FrqBack5;		//PC扫描最佳工作频率	(查询)0902	//11.0-12.9
Scan_FrqBack_t		Scan_FrqBack6;		//PC扫描最佳工作频率	(查询)0902	//13.0-14.9
Scan_FrqBack_t		Scan_FrqBack7;		//PC扫描最佳工作频率	(查询)0902	//15.0-16.9
Scan_FrqBack_t		Scan_FrqBack8;		//PC扫描最佳工作频率	(查询)0902	//17.0-18.9
Scan_FrqBack_t		Scan_FrqBack9;		//PC扫描最佳工作频率	(查询)0902	//19.0-20.9
Scan_FrqBack_t		Scan_FrqBack10;		//PC扫描最佳工作频率	(查询)0902	//21.0-22.9
Scan_FrqBack_t		Scan_FrqBack11;		//PC扫描最佳工作频率	(查询)0902	//23.0-24.9
Scan_FrqBack_t		Scan_FrqBack12;		//PC扫描最佳工作频率	(查询)0902	//25.0-26.1

//事件标志
SemaphoreHandle_t  xMutex = NULL;		//void  //App_printf(char *format,...);使用了信号量

static TimerHandle_t xTimers=NULL;
static TimerHandle_t xTimers2=NULL;
static TimerHandle_t xTimers3=NULL;

extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];		// ADC1转换的电压值通过MDA方式传到SRAM
float ADC_ConvertedValueLocal[NOFCHANEL];				// 用于保存转换计算后的电压值

struct rtc_time set_time;//定义RTC设置时间

/*时间结构体，默认时间2020-05-01 9:0:0*/
struct rtc_time systmtime=
{
    0, 0, 9, 1, 5, 2021
};

uint8_t adc1_zero_count = 0;		//反向功率很容易等于0
uint8_t adc3_zero_count = 0;		//电流很容易等于0

float ADC0_buff[AD_SAMPLE_NUM];
float ADC1_buff[AD_SAMPLE_NUM];
float ADC2_buff[AD_SAMPLE_NUM];
float ADC3_buff[AD_SAMPLE_NUM];

float		SWR_array[256];					//驻波比缓存
float		Forward_Power_array[256];		//正向功率
float		Reverse_Power_array[256];		//反向功率

uint32_t	freq_band_time_stamp[10];	//存储扫频结束的时间戳

uint8_t Hardware_Time_Count=Hardware_Time;				//硬件接管授时定时计数,默认刚进入硬件接管的时候更新一次时间
uint8_t Status_OPENING_Count=0;
void System_Status_Init(void)	//详见main.h
{
    /* 系统状态 */
    System.already_init = 0;
    System.already_swept = 0;

    /* 标志位 */
    System.open = 0;					//开机	0:关机状态	1:正在开机(保留)	2:已经开机
    System.close = 0;					//关机	1:正在关机，会开机失败
    System.sweep = 0;					//扫频	1:正在扫频	2:完成扫频，需写入到flash
    System.cancel_sweeping = 0;			//取消扫频
    System.Scan_Freq_flag = 0;			//扫频频段
    System.achieve_add_sub = 0;			//增加/减小功率状态	1:正在调节功率
    System.modify_power = 0;			//修改功率
    System.cancel_add_sub = 0;			//取消功率增减
    System.stop = 0;					//紧急停止
		System.time_update_flag = 0x00;	//是否更新了系统时间
		System.Power_Adjustment=0;		//增减功率执行标记
    /* 系统状态--MT2000_Rx更新 */
    System.status = SYSTEM_SHUTDOWN;
    System.Voltage = 0.0;
    System.Electricity = 0.0;
		System.Model[0]=Device_Version;
    System_Status_Clean();
}

void System_Status_Clean(void)
{
    System.emission = 0x00;
    System.mode = 0;
    System.method = 0;
    System.channel = 0;

    System.power[0] = 0;
    System.power[1] = 0;
    System.power[2] = 0;

    memset(System.freq1, 0, 10);
    memset(System.freq2, 0, 10);
    memset(System.freq3, 0, 10);

    memset(System.forward, 0, 4);
    memset(System.reverse, 0, 4);
    memset(System.swr, 0, 2);


    System.Forward_Power = 0;
    System.Reverse_Power = 0;
    System.Standing_wave_ratio = 0;

//		System.Voltage = 0.0;
//		System.Electricity = 0.0;		
}

void System_Status_Update(void)
{
    /*-----------------------系统状态更新----------------------------*/
//    uint8_t freq1_len=0, freq2_len=0, freq3_len=0;
	
    System.mode = GT2000_Rx.Gt2000_mode;   //系统工作模式是//1固频、2双频、3三频、4AM 
    System.method = 0;				//暂时没用//工作方式，FM
    System.channel = 1;
	
    if( System.sweep == 0x01 )
    {
			System.power[0] = 180;		//扫频时，只开一个频率，幅度值为8
			System.power[1] = 0;
			System.power[2] = 0;
    }
    else	if( Monitor.hard_control == 0x00 )		//不是硬件接管，直接覆盖为PC下发的功率等级
    {
        System.power[0] = PC_Cmd.power[0];
        System.power[1] = PC_Cmd.power[1];
        System.power[2] = PC_Cmd.power[2];
    }
		else if(Monitor.hard_control == 1)		//硬件接管时，取激励器的返回值
		{
			Range_Power(&GT2000_Rx);  /* 功率转化为系统幅度值,保存在System.power[0] */
			memset(System.power+1,0,2);	  //因为GT2000新机器只设置一个功率，只显示一个功率，默认清零power[1]、power[2]。
		}

    memset(System.freq1, 0, 10); //清零System.freq1[10]
    memset(System.freq2, 0, 10);
    memset(System.freq3, 0, 10);

//    freq1_len = strlen((const char*)GT2000_Rx.Gt2000_freq1);
//    freq2_len = strlen((const char*)GT2000_Rx.Gt2000_freq2);
//    freq3_len = strlen((const char*)GT2000_Rx.Gt2000_freq3);
    memcpy(System.freq1, GT2000_Rx.Gt2000_freq1, 2); //赋值查询到的发射机频率给上位机
		Trans_printf("main--GT2000_Rx.Gt2000_freq1= %x %x \n",GT2000_Rx.Gt2000_freq1[0],\
		GT2000_Rx.Gt2000_freq1[1]);

    if( System.mode == 2 )
    {
        memcpy(System.freq2, GT2000_Rx.Gt2000_freq2, 2);
			 Trans_printf("main--GT2000_Rx.Gt2000_freq2= %x %x  \n",GT2000_Rx.Gt2000_freq2[0],\
	   	GT2000_Rx.Gt2000_freq2[1]);
    }
    else	if( System.mode == 3 )
    {
        memcpy(System.freq2, GT2000_Rx.Gt2000_freq2, 2);
        memcpy(System.freq3, GT2000_Rx.Gt2000_freq3, 2);
//			Trans_printf("main--GT2000_Rx.Gt2000_freq2_3= %x %x %x %x  \n",GT2000_Rx.Gt2000_freq2[0],\
//	   	GT2000_Rx.Gt2000_freq2[1],GT2000_Rx.Gt2000_freq3[0],GT2000_Rx.Gt2000_freq3[1]);
    }


    if( GT2000_Rx.sys_emit_open == 1 )			//发射状态  //在MT2000_protocol.c中赋值
        System.emission = 0x01;
    else	if( GT2000_Rx.sys_emit_open == 0 )	//停止状态
        System.emission = 0x00;
//    Trans_printf(" sys_emit_open_To_System.emission = %02x GT2000_Rx.sys_emit_open =%02x\n",System.emission,GT2000_Rx.sys_emit_open );  //打印
		
    memcpy(System.forward, GT2000_Rx.forward_power, 4);
    memcpy(System.reverse, GT2000_Rx.reverse_power, 4);
//    memcpy(System.swr, GT2000_Rx.swr, 4);
		//		Trans_printf(" 发射机驻波比直接读取数字不对。 ");

		
		if(System.Launch_Switch_state==1)			//从发射机查询到的发射状态,待测试
		{
			if(System.status!=SYSTEM_SCAN&&System.status!=SYSTEM_ALARM)
			{
				System.status=SYSTEM_EMISSING;
				System.emission=1;	
			}				
		}
		else
		{
			System.emission=0;
		}
    
		System.Standing_wave_ratio = get_Standing_wave_ratio(System.Forward_Power, System.Reverse_Power);		//计算驻波比
    Trans_printf(" 1System.Standing_wave_ratio== %f \n",System.Standing_wave_ratio);
}

static void AppObjCreate (void)
{
    xMutex = xSemaphoreCreateMutex();   //创建互斥信号量

    if(xMutex == NULL)
    {
		#if	PRINTF_EN
//        printf("xSemaphoreCreateMutex failure!\r\n");
		#endif
		
        /* 没有创建成功，用户可以在这里加入创建失败的处理机制 */
        for(;;)
        {

        }
    }

}
/*
*********************************************************************************************************
*	函 数 名: main
*	功能说明: 标准c程序入口。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
int main(void)
{
	
#if	FIRMWARE_UPDATE_EN
		Set_Vector_Table();		
#endif
	__enable_irq();			//开启所有中断
	/*W5500初始化最好放在这里，不然容易卡程序*/	
#if	STM32_W5500_EN
	System_Initialization();	//W5500驱动相关初始化
	Load_Net_Parameters();		//装载网络参数	
	W5500_Hardware_Reset();		//硬件复位W5500
	W5500_Initialization();		//W5500初始化配置
#endif

    bsp_Init();
    AppObjCreate();   //创建互斥信号量
    System_Status_Init();		//系统状态初始化
    clean_all_alarm_flags();	//清除报警标志位
    GT2000_Init();				//MT2000_Tx初始化

#if	FIRMWARE_UPDATE_EN
	uint32_t	app_version_1, app_version_2;
	uint8_t 	version[4];

	APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
	app_version_1 = (APP_Version_1[0]<<24) + (APP_Version_1[1]<<16) + (APP_Version_1[2]<<8) + (APP_Version_1[3]<<0);
	app_version_2 = (APP_Version_2[0]<<24) + (APP_Version_2[1]<<16) + (APP_Version_2[2]<<8) + (APP_Version_2[3]<<0);
		
	if( (app_version_1 == 0xFFFFFFFF) && (app_version_2 == 0xFFFFFFFF) )
	{
		//版本无效
	}
	else	if( Get_APP_Version() == 0x01 )
	{
		if( app_version_2 == 0xFFFFFFFF )
		{
			//版本无效
			APP_Version_2[0] = (APP_VERSION>>0) & 0xFF;
			APP_Version_2[1] = (APP_VERSION>>8) & 0xFF;
			APP_Version_2[2] = (APP_VERSION>>16) & 0xFF;
			APP_Version_2[3] = (APP_VERSION>>24) & 0xFF;
			APP_Version_Write(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
			APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
		}
		else	if( (app_version_1 != 0xFFFFFFFF) && (app_version_1 > APP_VERSION) )	//现在是旧版本
		{
			version[0] = 0xFF;
			version[1] = 0xFF;
			version[2] = 0xFF;
			version[3] = 0xFF;
			
			APP_Version_Write(APPLICATION_VERSION_ADDRESS, version, APP_Version_2);
		}
	}
	else
	{
		if( app_version_1 == 0xFFFFFFFF )
		{
			//版本无效
			APP_Version_1[0] = (APP_VERSION>>0) & 0xFF;
			APP_Version_1[1] = (APP_VERSION>>8) & 0xFF;
			APP_Version_1[2] = (APP_VERSION>>16) & 0xFF;
			APP_Version_1[3] = (APP_VERSION>>24) & 0xFF;
			APP_Version_Write(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
			APP_Version_Read(APPLICATION_VERSION_ADDRESS, APP_Version_1, APP_Version_2);
		}
		else	if( (app_version_2 != 0xFFFFFFFF) && (app_version_2 > APP_VERSION) )	//现在是旧版本
		{
			version[0] = 0xFF;
			version[1] = 0xFF;
			version[2] = 0xFF;
			version[3] = 0xFF;
			
			APP_Version_Write(APPLICATION_VERSION_ADDRESS, APP_Version_1, version);
		}
	}

#endif
  	set_time.tm_year=systmtime.tm_year;
    set_time.tm_mon=systmtime.tm_mon;
    set_time.tm_mday=systmtime.tm_mday;
    set_time.tm_hour=systmtime.tm_hour;
    set_time.tm_min=systmtime.tm_min;
    set_time.tm_sec=systmtime.tm_sec;
		RTC_CheckAndConfig(&set_time);
		update_RTCtime(RTC_GetCounter(), &set_time);

#if	IWDOG_EN			//晶振不起振，会一直复位，等待起振之后再开启看门狗
	IWDG_Config(IWDG_Prescaler_64 ,625);	//IWDG 1s 超时溢出
#endif
	
    /*---------------------------------------------------------------------------------------------------*/

    Flash_to_AcceptAPP();		//读取报警参数，并置位 System.already_init = 1;
    Flash2_to_AcceptAPP();		//读取正向功率和驻波比
    Flash3_to_AcceptAPP();		//读取运行图的数量

    /*---------------------------------------------------------------------------------------------------*/

    if( System.already_init == 1 )	//已经完成初始化，打印初始化数据
    {
        
    }
    else	//未初始化，将接收查询包的CAN_ID作为System.CAN_ID(CAN_ID已经决定了该发射机只能接收到该ID的数据)
    {
        System.status = SYSTEM_UNINITIALIZE;		//未初始化状态

        //会在查询后，自适应CAN_ID，否则会一直显示CAN_ID不匹配，一直掉线
    }

    if(flash_3_once_flag==1)
    {
        if(flash3_Save<=10&&flash3_Save>0)
        {
            judg_read_flash(flash3_Save);
        }
        else
        {
            flash_3_once_flag=0;		//这样不进入定时器的周期性轮询，因为flash3_Save必须小于等于10而大于0
        }
    }


    xTimers= xTimerCreate("Timer",			/* 定时器名字 */
                          5000,				/* 定时器周期,单位时钟节拍 */
                          pdTRUE,			/* 周期性 */
                          (void *) 0,		/* 定时器ID */
                          vTimerCallback);	/* 定时器回调函数 */

    if(xTimers== NULL)
    {
			
    }
    else
    {
        /* 启动定时器，系统启动后才开始工作 */
        if(xTimerStart(xTimers, 100) != pdPASS)
        {
			#if	PRINTF_EN
//            printf("timer created failure!\r\n");
			#endif
			
            for(;;)
            {
            }
        }
    }

    xTimers2= xTimerCreate("Timer2",			/* 定时器名字 */
                           1000,				/* 定时器周期,单位时钟节拍 */
                           pdTRUE,				/* 周期性 */
                           (void *) 1,			/* 定时器ID */
                           vTimerCallback2);	/* 定时器回调函数 */

    if(xTimers2== NULL)
    {
		#if	PRINTF_EN
//        printf("timer2 created failure!\r\n");//串口都还没有初始化，不可调用printf
		#endif
    }
    else
    {
        /* 启动定时器，系统启动后才开始工作 */
        if(xTimerStart(xTimers2, 100) != pdPASS)
        {
			#if	PRINTF_EN
//            printf("timer2 created failure!\r\n");
			#endif
			
            for(;;)
            {
            }
        }
    }

    xTimers3= xTimerCreate("Timer3",			/* 定时器名字 */
                           50,					/* 定时器周期,单位时钟节拍 */
                           pdTRUE,				/* 周期性 */
                           (void *) 2,			/* 定时器ID */
                           vTimerCallback3);	/* 定时器回调函数 */

    if(xTimers3== NULL)
    {
		#if	PRINTF_EN
//        printf("timer3 created failure!\r\n");//串口都还没有初始化，不可调用printf
		#endif
    }
    else
    {
        /* 启动定时器，系统启动后才开始工作 */
        if(xTimerStart(xTimers3, 100) != pdPASS)
        {
			#if	PRINTF_EN
//            printf("timer3 created failure!\r\n");
			#endif
			
            for(;;)
            {
            }
        }
    }

    /* 创建任务 */
    AppTaskCreate();

    /* 启动调度，开始执行任务 */
    vTaskStartScheduler();

    /*
      如果系统正常启动是不会运行到这里的，运行到这里极有可能是用于定时器任务或者空闲任务的
      heap空间不足造成创建失败，此要加大FreeRTOSConfig.h文件中定义的heap大小：
      #define configTOTAL_HEAP_SIZE	      ( ( size_t ) ( 20 * 1024 ) )
    */
    while(1)
	{
		
	}
		
}

/*
*********************************************************************************************************
*	函 数 名: vTimerCallback
*	功能说明: 定时器回调函数
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
/*温湿度的固定读取指令，modbus协议*/
//110300000002C69B
//120300000002C6A8
//130300000002C779
//140300000002C6CE
//150300000002C71F
//010300000002C40B
uint8_t T_H_cmd[8]= {0x01,0x03,0x00,0x00,0x00,0x02,0xC4,0x0B};			//旧温湿度传感器	地址0x01	修改地址之后，还需要在TH_Analyze()修改地址。
//uint8_t T_H_cmd[8]= {0x15,0x03,0x00,0x00,0x00,0x02,0xC7,0x1F};		//新温湿度传感器	地址0x15
static void vTimerCallback(xTimerHandle pxTimer)	/* 5s */
{
    comSendBuf(COM2, T_H_cmd, 8);

    if( Alarm.alarm_history == 1 )
    {
        alarm_printf();
    }
	if(Hardware_Time_Count<=Hardware_Time)	//避免溢出
	{
		Hardware_Time_Count++;		//硬件接管授时定时计数
	}
}

/*
 *监控定时器--如果PC端60s无反应，则硬件自动控制
 */
static void vTimerCallback2(xTimerHandle pxTimer)	/* 1s */
{
    Monitor.no_respond_count++;
		
		if(System.status==SYSTEM_OPENING)		//检测该状态持续时间
		{
			Status_OPENING_Count++;
			if(Status_OPENING_Count>=30)
			{
				Status_OPENING_Count=0;
				System.status=SYSTEM_SHUTDOWN;
			}
		}
		else
		{
			Status_OPENING_Count=0;
		}
    if( System.time_update_flag == 0x00 )		//未更新时间，不进入硬件接管
    {
        Monitor.no_respond_count=0;
        Monitor.hard_control=0;
        Monitor.need_emit = 0;
        Monitor.need_open = 0;
        Monitor.need_close = 0;

        Run_Diagram_data.mode = '0';	
        Run_Diagram_data.power[0] = '0';
        Run_Diagram_data.power[1] = '0';
        Run_Diagram_data.power[2] = '0';
        memset(Run_Diagram_data.Freq, 0, sizeof(Run_Diagram_data.Freq));
    }
    else	if( Monitor.no_respond_count >= 60 )   //60
    {
        Monitor.no_respond_count = 60;
        Monitor.hard_control = 1;
    }
    else
    {
        Monitor.hard_control=0;
    }

    if( (Monitor.hard_control==0x01) && (flash_3_once_flag==1) )
    {
		    juge_need_open_close(flash3_Save);
        find_hard_control(flash3_Save);
    }
}

/*
 *系统状态指示定时器
 */
static void vTimerCallback3(xTimerHandle pxTimer)	/* 50ms */
{
    static uint8_t led1_counter = 0;
    static uint8_t led2_counter = 0;
	
#if	IWDOG_EN
        IWDG_Feed();		//喂狗
#endif
	
    if( System.emission == 1 )			//运行指示灯
    {
        LED1_ON;						//黄灯
    }
    else
    {
        led1_counter++;

        if( led1_counter >= 4 )
        {
            led1_counter = 0;
            LED1_TOGGLE;
        }
    }

    if( Alarm.alarm_history == 1)		//报警指示灯
    {
        led2_counter++;
        if( led2_counter >= 4 )
        {
            led2_counter = 0;
            LED2_TOGGLE;				//红灯
        }
    }
	
    else
    {
        led2_counter = 0;
        LED2_OFF;
    }
	
}

/*
*********************************************************************************************************
*	函 数 名: AppTaskCreate
*	功能说明: 创建应用任务
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
static void AppTaskCreate (void)
{
	xTaskCreate( W5500_APP, 						/* 任务函数  */
								 "W5500_APP",   					/* 任务名    */
								 256,            					/* 任务栈大小，单位word，也就是4字节 */
								 NULL,           					/* 任务参数  */
								 6,              					/* 任务优先级*/
								 &xW5500Task_APP );					/* 任务句柄  */
	
	xTaskCreate( Task_App,				  			/* 任务函数  */
                 "Task_App",   						/* 任务名    */
                 1536,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 7,              					/* 任务优先级*/
                 &xHandleTask_App );				/* 任务句柄  */
	
    xTaskCreate( Task_Upper_Computer,  				/* 任务函数  */
                 "Task_Upper_Computer",   			/* 任务名    */
                 640,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 5,              					/* 任务优先级*/
                 &xHandleTask_Upper_Computer );		/* 任务句柄  */

    xTaskCreate( Task_Hardware_Monitor,				/* 任务函数  */
                 "Task_Hardware_Monitor",   		/* 任务名    */
                 640,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 4,              					/* 任务优先级*/
                 &xHandleTask_Hardware_Monitor );	/* 任务句柄  */

    xTaskCreate( Task_MT2000,  						/* 任务函数  */
                 "Task_MT2000",   					/* 任务名    */
                 640,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 3,              					/* 任务优先级*/
                 &xHandleTask_MT2000 );				/* 任务句柄  */

    xTaskCreate( Task_TH,  							/* 任务函数  */
                 "Task_TH",   						/* 任务名    */
                 256,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 2,              					/* 任务优先级*/
                 &xHandleTask_TH );					/* 任务句柄  */

    xTaskCreate( Task_Alarm, 						/* 任务函数  */
                 "Task_Alarm",   					/* 任务名    */
                 256,            					/* 任务栈大小，单位word，也就是4字节 */
                 NULL,           					/* 任务参数  */
                 1,              					/* 任务优先级*/
                 &xHandleTask_Alarm );				/* 任务句柄  */
				 
	}



/***************************** (END OF FILE) *********************************/

