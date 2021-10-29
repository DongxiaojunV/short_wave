#ifndef	_MAIN_H_
#define	_MAIN_H_

#include <stdint.h>

#pragma		pack(1)

/*-----------------------------------新/旧机器协议------------------------------------------*/
#define		NEW_PROCOTOL			1			//新协议

/*-----------------------------------机器版本种类选择------------------------------------------*/
#define		Device_Version		0xB1		//代表是北京短波发射机,A1代表海华机器

/*---------------------------------------看门狗---------------------------------------------*/
#define		IWDOG_EN				1			//使能看门狗	在Task_TH()里喂狗

/*-------------------------------------打印调试信息-----------------------------------------*/
#define		PRINTF_EN				0			//是否打印调试信息

/*-------------------------------------打印调试信息-----------------------------------------*/
#define		printf_EN				1			//是否打印2KW调试信息

/*----------------------------------------W5500-----------------------------------------------*/
#define		STM32_W5500_EN			1			//使能网口W5500

/*----------------------------------Firmware_Updare-----------------------------------------*/
#define		FIRMWARE_UPDATE_EN		1			//使能固件升级

/*----------------------------------------TH------------------------------------------------*/
#define		NEW_TH_EN				1			//是否探头式温湿度变送器
#define   New_Th          1
/*--------------------------------设备无响应自动恢复----------------------------------------*/
#define		NO_RESPOND_RECOVER		1			//1:自动恢复

/*--------------------------------设备无响应最大次数----------------------------------------*/
#define		NO_RESPOND_MAX			3

/*------------------------------------保护调节----------------------------------------------*/
#define		PROTECT_ADJUST			0.95

/*------------------------------------AD采集参数--------------------------------------------*/
#define		SWR_ALARM_THRESHOLD 24		//驻波比报警临界值,扩大10倍便于比较
#define 	Debug_M_45I				(41.0)		//电流	(37.5)
#define 	Debug_M_45V				(10.0)		//电压	(45/4.76)
#define		AD_SAMPLE_NUM			50			//AD采集次数(求平均值)	开机后立即关机，会欠压报警，改100为10

/*------------------------------------硬件接管开关机时间--------------------------------------------*/
#define		BEFOREHAND_OPEN_TIME	5			//硬件接管，提前开机时间				(单位:分钟)
#define 	Hardware_Time 12		//硬件接管时，前往NTP询问的时间，1次5S，12次1分钟
/*------------------------------------扫频更新时间和扫频策略--------------------------------------------*/
#define		SCAN_BREAKPOINT_ADDRESS		0x08039000	//扫频断点地址，即下一个扫频频点的地址
#define		SCAN_BREAKPOINT_LEN			0x02		//扫频断点的数据长度

#define		SCAN_ALARM_SWR_THRESHOLD	4		//扫频时，严重报警，驻波比为4

#define		POWER_MAX				1200		//机器最大功率
#define   POWER_CHANGE_MAX 350
/*--------------------------------扫频数据有效时间，超过有效时间，则重新扫频-----------------------------*/
#define		FREQ_BAND_UPDATE_TIME	(7*24*3600)	//小于频段更新时间，则扫频时直接返回已扫频的数据		单位:s



/* 系统状态 */
#define		SYSTEM_EMISSING			0x01		//发射状态
#define		SYSTEM_STANDBY			0x02		//待机状态	已开机，但是没有执行任何操作
#define		SYSTEM_FREQUENCY		0x03		//调频状态	调谐状态
#define		SYSTEM_SHUTDOWN			0x04		//关机状态
#define		SYSTEM_SCAN				0x05		//扫频状态
#define		SYSTEM_OPENING			0x06		//正在开机

#define		SYSTEM_UNINITIALIZE		0x10		//未初始化
#define		SYSTEM_ALARM			0x11		//报警状态
#define		SYSTEM_HARDWARE			0x12		//硬件接管	PC占用了，不使用

/* 新旧协议功率等级 */
#define		NEW_FULL_POWER_LEVEL				'3'
#define		NEW_HALF_POWER_LEVEL				'2'
#define		NEW_QUARTER_POWER_LEVEL				'1'
#define		NEW_THREE_QUARTET_POWER_LEVEL		'4'
#define		OLD_FULL_POWER_LEVEL				0x00
#define		OLD_HALF_POWER_LEVEL				0x01
#define		OLD_QUARTER_POWER_LEVEL				0x02
#define		OLD_THREE_QUARTET_POWER_LEVEL		0x03


typedef struct
{	
	/* 系统信息 */
	uint8_t CAN_ID[1];				//发射机ID	
	uint8_t Model[1];						//发射机类型
	/* 标志位 */
	uint8_t already_init;			//初始化完成标志
	uint8_t already_swept;			//扫频完成标志		//断点扫描之后，该标志位已经无意义
	uint8_t time[6];				//北斗时间(从PC得到数据，更新到系统)
	uint8_t time_update_flag;		//更新时间标志位
	
	/* 系统状态 */
	uint8_t status;					//0x01发射状态	0x02待机状态	0x03调频状态	0x04关机	0x05扫频	0x10未初始化	0x11报警状态
	uint8_t open;					//开机标志位	0:关机	1:正在开机	2:已经开机
	uint8_t close;					//关机标志位	1:正在关机，不能检测AD，否则会低电压报警，关机之后，再清零
	
	uint8_t sweep;					//扫频标志位	1:正在扫频，2:扫频完成，更新status标志位，并将sweep清零		最后，保存在flash
	uint8_t cancel_sweeping;		//取消扫频		1:取消扫频
	uint8_t	Scan_Freq_flag;			//扫频频段
	
	uint8_t achieve_add_sub;			//增加/减小功率完成标志位	1:完成功率调整
	uint8_t modify_power;			//1:正在增减功率
	uint8_t cancel_add_sub;			//1:取消增减功率
	uint8_t protect_adjust;			//1:正在进行保护调节		0:不需要调节或调节结束(完成或失败)		(电流/驻波比 > 80%报警阈值)
	
	uint8_t stop;					//紧急停止标志位	1:紧急停止(正在增减功率，但是已经过了工作时间，需要紧急停止)	通知Task_MT2000(),BJ2000_Stop();MT2000_Cmd_Stop();才置位
	
	/* 新协议--MT2000_Rx接收信息，完成之后更新到系统状态 */
	uint8_t emission;				//发射/停止				(新协议)'T'/'P'		(旧协议)0x01发射/0x00停止
	uint8_t mode;					//固频、双频、三频		(新协议)'1','2','3'	(旧协议)0x01,0x02,0x03
	uint8_t INQUIRE_Mode;  //从机器上查询到的模式
	uint8_t method;					//工作方式，FM			(新协议)'F'			(旧协议)0x03
	uint8_t channel;				//信道					(新协议)'1'			(旧协议)0x01,0x02,0x03	(旧协议3->新协议1，但是新协议固定为'1'，暂只输出，无特别含义)
	uint8_t power[3];				//功率等级				(新协议)'1':1/4		'2':1/2		'3':1	(旧协议)0x00:全功率	0x01:1/2功率	0x02:1/4功率
									//旧协议是三个功率等级，新协议共用一个功率等级(调用update_system_power_level()，根据选择策略，返回功率等级)
									//(旧协议3->新协议实际上为1，但是新协议需要0602返回给PC，保留，并且另设置功率选择策略函数，返回值为需要发射的功率)
								
	//从PC收到发射指令，直接赋值给MT2000_Tx，
	//或者从flash获取之后，通过set_run_diagram_power_level()赋值给Run_Diagram_data，在Task_Hardware_Monitor()判断是否需要发射，最后赋值给MT2000_Tx，然后发送
	uint8_t freq1[10];				//固频频率		发射成功后，MT2000_Rx => System
	uint8_t freq2[10];				//双频频率		发射成功后，MT2000_Rx => System
	uint8_t freq3[10];				//三频频率		发射成功后，MT2000_Rx => System
	
	uint8_t forward[6];				//正向功率		//从MT2000_Rx更新数据后，会更新到0602，返回给PC
	uint8_t reverse[6];				//反向功率		//从MT2000_Rx更新数据后，会更新到0602，返回给PC
	uint8_t swr[4];					  //驻波比		//从MT2000_Rx更新数据后，会更新到0602，返回给PC
	
	uint8_t Control_Model;			//北京机器控制模式：自动/手动
	uint8_t Reverse_State;			//发射机反射状态
	uint8_t Absorber_State_1;			//滤波器状态，当前使用的滤波器
	uint8_t Absorber_State_2;			//滤波器状态
	uint16_t Amplifier_Temperature[4];		//功放温度检测，每个功放有一个，一共四个
	uint8_t	Launch_Switch_state;
	uint8_t	Attenuation;					//衰减值
	uint8_t Frequency_band_value;	//频断值
	uint8_t fbv_c;								//频段值核对变量
	/* 旧协议 */
	float ADC_Sum_F;				//ADC采样的正向功率和	/* 旧协议	浮点型数据(电压) */
	float ADC_Sum_R;				//ADC采样的反向功率和	/* 旧协议	浮点型数据(电压) */
	float ADC_Sum_V;				//ADC采样的电压和		/* 旧协议	浮点型数据(电压) */		//电压电流转换公式详见	update_System_Voltage_Electricity();
	float ADC_Sum_A;				//ADC采样的电流和		/* 旧协议	浮点型数据(电压) */		(待定)停止发射 BJ2000_Stop(); 扫频 BJ2000_Stop(); ADC检测无效(挂起任务)
	
	float Forward_Power;			//正向功率				/* 旧协议	浮点型数据(电压) */
	float Reverse_Power;			//反向功率				/* 旧协议	浮点型数据(电压) */
	float Voltage;					//45V下的电压	单位:V		//电压电流清零，不显示时的系统状态 -- 关机、正在开机、正在调频、正在扫频
	float Electricity;				//45V下的电流	单位:A		//电压电流显示的系统状态 -- 待机、发射(工作)、功率调节、报警
	float Standing_wave_ratio;		//驻波比		(新协议的字符数组swr[4]会转换成浮点型Standing_wave_ratio，便于比较)
	
	uint16_t Bj_Electricity_1[4];		//存储北京机器的功放一的电流
	uint16_t Bj_Electricity_2[4];		//存储北京机器的功放二的电流
	uint16_t Bj_Electricity_3[4];		//存储北京机器的功放三的电流
	uint16_t Bj_Electricity_4[4];		//存储北京机器的功放四的电流
	
	uint8_t Ip[4];
	uint8_t Port[2];
	uint8_t Init_Mark;		//初始化标记
	uint8_t Open_Close;		//0代表发射机关机，1代表开机
	uint8_t Power_Adjustment;			//增减功率执行标记
}Sys_Status_t;

typedef struct
{
    volatile uint8_t start[10][10];
    volatile uint8_t end[10][10];
	
	volatile uint8_t usage_diagram_count;	//记录是第几次查询到可用运行图
	
	volatile uint8_t no_respond_count;		//没有接收PC命令计数
    volatile uint8_t hard_control;			//硬件接管
	
    volatile uint8_t need_open;				//需要开机
	volatile uint8_t need_close;			//需要关机
	volatile uint8_t need_emit;				//查询运行图时，模式、频率或功率等级不一致：需要发射
}Monitor_t;

typedef struct Temperature_humidity
{
    float Temperature;//温度
    float Humidity;//湿度
}Temperature_Humidity_t;

typedef struct
{	
    volatile uint8_t alarm_history;			//历史报警记录
	
	volatile uint8_t emission;				//激励器自身报警
	volatile uint8_t emission_time_flag;
	
    volatile uint8_t no_respond;			//激励器3次操作不响应
	volatile uint8_t no_respond_time_flag;
	
	volatile uint8_t no_respond_count;		//激励器无响应计数器
    volatile uint8_t no_respond_locate;		//用于定位无响应的地方
	
    volatile uint8_t temperature_alarm;		//温度报警	温湿度传感器
	volatile uint8_t temperature_alarm_time_flag;
    volatile uint8_t humidity_alarm;		//湿度报警
	volatile uint8_t humidity_alarm_time_flag;
	
    volatile uint8_t over_Electric;			//过流报警	AD
	volatile uint8_t over_Electric_time_flag;
    volatile uint8_t over_Voltage;			//过压报警
	volatile uint8_t over_Voltage_time_flag;
    volatile uint8_t low_Voltage;			//欠压报警
	volatile uint8_t low_Voltage_time_flag;
	
	volatile uint8_t swr_alarm;				//驻波比报警	AD或者激励器返回的数据
	volatile uint8_t swr_alarm_time_flag;
    volatile uint8_t no_power;				//无功率输出
	volatile uint8_t no_power_time_flag;
	volatile uint8_t power_cataclysm;		//功率骤变  ±300W
}Alarm_t;	//当前有报警，则置位标志位，并记录报警的时间和报警值到Alarm_historyBack_t报警查询应答包

typedef struct
{
	uint8_t mode;				//固频、双频、三频		(新协议)'1','2','3'	(旧协议)0x01,0x02,0x03
//	uint8_t Channel[3];			//需要工作信道			新协议固定'1'，其实旧协议也可以取消(可以根据模式，发射相应频率，自然就是123信道了)
	uint8_t Freq[12];			//需要发射的频率		hex发射前再转换为ascii
	uint8_t power[3];			//需要工作的功率等级	直接存新协议的'1','2','3'
}Run_Diagram_data_t;			//获取到的运行图数据

typedef struct
{
    uint8_t Start_Time1[2];		//开始时间
    uint8_t End_Time1[2];		//结束时间
    uint8_t Power1[1];			//功率
    uint8_t Frq1[4];			//频率
	uint8_t channel1[1];		//信道			//新版本不用，但是保留

    uint8_t Start_Time2[2];		//开始时间
    uint8_t End_Time2[2];		//结束时间
    uint8_t Power2[1];			//功率
    uint8_t Frq2[4];			//频率
	uint8_t channel2[1];		//信道

    uint8_t Start_Time3[2];		//开始时间
    uint8_t End_Time3[2];		//结束时间
    uint8_t Power3[1];			//功率
    uint8_t Frq3[4];			//频率
	uint8_t channel3[1];		//信道

    uint8_t Start_Time4[2];		//开始时间
    uint8_t End_Time4[2];		//结束时间
    uint8_t Power4[1];			//功率
    uint8_t Frq4[4];			//频率
	uint8_t channel4[1];		//信道

    uint8_t Start_Time5[2];		//开始时间
    uint8_t End_Time5[2];		//结束时间
    uint8_t Power5[1];			//功率
    uint8_t Frq5[4];			//频率
	uint8_t channel5[1];		//信道

    uint8_t Start_Time6[2];		//开始时间
    uint8_t End_Time6[2];		//结束时间
    uint8_t Power6[1];			//功率
    uint8_t Frq6[4];			//频率
	uint8_t channel6[1];		//信道

    uint8_t Start_Time7[2];		//开始时间
    uint8_t End_Time7[2];		//结束时间
    uint8_t Power7[1];			//功率
    uint8_t Frq7[4];			//频率
	uint8_t channel7[1];		//信道

    uint8_t Start_Time8[2];		//开始时间
    uint8_t End_Time8[2];		//结束时间
    uint8_t Power8[1];			//功率
    uint8_t Frq8[4];			//频率
	uint8_t channel8[1];		//信道

    uint8_t Start_Time9[2];		//开始时间
    uint8_t End_Time9[2];		//结束时间
    uint8_t Power9[1];			//功率
    uint8_t Frq9[4];			//频率
	uint8_t channel9[1];		//信道

    uint8_t Start_Time10[2];	//开始时间
    uint8_t End_Time10[2];		//结束时间
    uint8_t Power10[1];			//功率
    uint8_t Frq10[4];			//频率
	uint8_t channel10[1];		//信道
}Run_Diagram_buf_t;				//运行图缓存(保存100个)
/*-----------------------------------------接收命令包---------------------------------------------------*/
typedef struct
{
    float 	Low_temp_limit[1];		//温度下限
    float 	Upp_temp_limit[1];		//温度上限
    float 	Low_humidity_limit[1];	//湿度下限
    float 	Upp_humidity_limit[1];	//湿度上限
    float 	Low_45I_limit[1];		//电流下限
    float 	Upp_45I_limit[1];		//电流上限
    float 	Low_45V_limit[1];		//电压下限
    float 	Upp_45V_limit[1];		//电压上限
	uint8_t timer[6];				//北斗时间
    uint8_t Transmitte_id[1];		//发射机ID
}Alarm_threshold_t;					//PC下发报警阈值的结构体	0101

typedef struct
{
	uint8_t mode;		//1固频、2双频、3三频
	uint8_t method;		//工作方式，FM
	uint8_t power[3];	//功率等级 		0：全功率	1：1/2功率	2：1/4功率
	uint8_t channel[3];	//信道
	
	uint8_t freq[12];	//频率
	uint8_t timer[6];	//北斗时间
}PC_Cmd_t;				//PC开启发射机的参数		0301

typedef struct
{
    uint8_t Transmitte_id[1];	//发射机ID
    uint8_t Disalarm_type[2];	//解除报警类型
    uint8_t Berdou_time[6];		//北斗时间
}Disalarm_t;					//PC解除报警状态	0501

typedef struct
{
    uint8_t	Transmitte_id[1];	//发射机ID
    uint8_t	state[1];			//状态
    float	Fre_Band[2];		//频段
}Scan_Frq_t;					//PC扫描最佳工作频率0901

typedef struct
{
    uint8_t Transmitte_id[1];	//发射机ID
    uint8_t state[1];			//状态
}Sacn_stop_t;					//PC停止扫描		0A01

typedef struct
{
    uint8_t Transmitte_id[1];	//发射机ID
    uint8_t Power_UP[2];		//需要增加的功率
}Add_Power_t;					//功率微调增加		0B01

typedef struct
{
    uint8_t Transmitte_id[1];	//发射机ID
    uint8_t Power_DOWN[2];		//需要减少的功率
}Sub_Power_t;					//功率微调减少		0C01

/* 运行图 */
typedef struct
{
	uint8_t Transmitte_id[1];	//发射机ID
    uint8_t count[1];			//有几段运行图
    uint8_t Continue[2];		//是否还有运行图
	
    uint8_t Start_Time1[2];		//开始时间
    uint8_t End_Time1[2];		//结束时间
    uint8_t Power1[1];			//功率
    uint8_t Frq1[4];			//频率
	uint8_t channel1[1];		//信道			//新版本不用，但是保留

    uint8_t Start_Time2[2];		//开始时间
    uint8_t End_Time2[2];		//结束时间
    uint8_t Power2[1];			//功率
    uint8_t Frq2[4];			//频率
	uint8_t channel2[1];		//信道

    uint8_t Start_Time3[2];		//开始时间
    uint8_t End_Time3[2];		//结束时间
    uint8_t Power3[1];			//功率
    uint8_t Frq3[4];			//频率
	uint8_t channel3[1];		//信道

    uint8_t Start_Time4[2];		//开始时间
    uint8_t End_Time4[2];		//结束时间
    uint8_t Power4[1];			//功率
    uint8_t Frq4[4];			//频率
	uint8_t channel4[1];		//信道

    uint8_t Start_Time5[2];		//开始时间
    uint8_t End_Time5[2];		//结束时间
    uint8_t Power5[1];			//功率
    uint8_t Frq5[4];			//频率
	uint8_t channel5[1];		//信道

    uint8_t Start_Time6[2];		//开始时间
    uint8_t End_Time6[2];		//结束时间
    uint8_t Power6[1];			//功率
    uint8_t Frq6[4];			//频率
	uint8_t channel6[1];		//信道

    uint8_t Start_Time7[2];		//开始时间
    uint8_t End_Time7[2];		//结束时间
    uint8_t Power7[1];			//功率
    uint8_t Frq7[4];			//频率
	uint8_t channel7[1];		//信道

    uint8_t Start_Time8[2];		//开始时间
    uint8_t End_Time8[2];		//结束时间
    uint8_t Power8[1];			//功率
    uint8_t Frq8[4];			//频率
	uint8_t channel8[1];		//信道

    uint8_t Start_Time9[2];		//开始时间
    uint8_t End_Time9[2];		//结束时间
    uint8_t Power9[1];			//功率
    uint8_t Frq9[4];			//频率
	uint8_t channel9[1];		//信道

    uint8_t Start_Time10[2];	//开始时间
    uint8_t End_Time10[2];		//结束时间
    uint8_t Power10[1];			//功率
    uint8_t Frq10[4];			//频率
	uint8_t channel10[1];		//信道
}Run_Diagram_t;					//0D01

/*----------------------------------------------------------------------*/
typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t	Alarm_paramet[2];		//报警参数
}Alarm_backPC_t;					//报警参数初始化应答包		0102

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t Trans_current_state[1];	//发射机当前的状态
    uint8_t Mode[1];				//当前工作的模式	//固频、双频、三频
    uint8_t Channel[3];				//信道
    uint8_t Freq[12];				//当前工作的频率
    uint8_t Power_grade[3];			//当前工作的功率等级
    float Forward_power[1];			//发射机正向功率
    float Reverse_power[1];			//发射机反向功率
    float Power_45_intensity[1];	//发射机45V供电电流取样
    float Power_45_voltage[1];		//发射机45V供电电压取样
    float temp[1];					//环境温度
    float humidity[1];				//湿度
    uint8_t emission[1];			//bit0-- 0 没有发射 1正在发射，bit1--0 关机 1开机
		uint8_t current_alarm_state[2];	//当前报警状态
		
		/*V2版本*/
		uint8_t	Alarm_Reservation[2];		//报警预留
		uint8_t	System_Version[20];			//系统版本，信息预留,其中-4为发射机类型
		uint8_t Divider[1];								//分割线，赋值为0xFF，便于观看协议
		uint8_t Control_Model[1];			//北京机器控制模式：自动/手动
		uint8_t Frequency_band_value[1];	//频段值
		uint8_t	Attenuation[1];					//衰减值
		uint8_t	Reservation[2];					//预留两个byte
		uint16_t Bj_Electricity_1[4];		//存储北京机器的功放一的电流
		uint16_t Amplifier_Temperature_1[1];	//功放一温度
		uint16_t Bj_Electricity_2[4];		//存储北京机器的功放二的电流
		uint16_t Amplifier_Temperature_2[1];	//功放二温度
		uint16_t Bj_Electricity_3[4];		//存储北京机器的功放三的电流
		uint16_t Amplifier_Temperature_3[1];	//功放三温度
		uint16_t Bj_Electricity_4[4];		//存储北京机器的功放四的电流
		uint16_t Amplifier_Temperature_4[1];	//功放四温度	
}Work_paraBack_t;					//查询应答包				0202

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t Trans_state[2];			//发射机返回PC此时的状态
}Trans_openBack_t;					//发射应答包				0302

typedef struct
{
	uint8_t Transmitte_id[1];		//发射机ID
	uint8_t Trans_state[1];			//返回发射机执行关闭的结果
}Trans_stopBack_t;					//停止发射应答包			0402

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t Disalarm_result[2];		//解除报警状态结果
    uint8_t current_alarm_state[2];	//当前报警状态
}DisalarmBack_t;					//解除报警状态的应答包		0502

typedef struct
{
	uint8_t Transmitte_id[1];		//发射机ID
    uint8_t alarm_history[2];		//历史存在的报警参数

    uint8_t no_power_time[6];		//无功率输出的时间
	
    float 	temp_history[1];		//报警温度时候的值
    uint8_t temp_time[6];			//温度报警的时间
	
    float	humidity_history[1];	//湿度报警时候的值
    uint8_t humidity_time[6];		//湿度报警的时间
	
    float 	power_45A_history[1];	//发射机45V供电电流时候的值(过流)
    uint8_t power_45A_time[6];		//发射机45V供电电流报警时间
	
    float	power_45V_history[1];	//发射机45V供电电压报警时候的值(过压欠压共用)
    uint8_t power_45V_time[6];		//发射机45V供电电压报警的时间
	
    uint8_t no_response_time[6];	//发射机无响应时间
	
    uint8_t	emission_time[6];		//发射机自身报警时间
	
    float 	swr_alarm[1];			//驻波比
    uint8_t swr_time[6];			//驻波比报警时间
	
		uint8_t	emission_alarm_content[40];	//激励器报警具体内容
		
		uint8_t Alarm_reservation[2];				//报警保留位
		uint8_t Alarm_Switch_Power[1];			//开关电源报警
		uint8_t Alarm_Abnormal_Output[1];		//输出异常报警
		uint8_t Alarm_Module_Electricity[1];//模块电流报警
		uint8_t Alarm_Module_Power[1];			//功率状态报警
		uint8_t Alarm_Module_Tem[1];				//模块温度报警
		uint8_t Alarm_Absorber_State_1[1];	//滤波器报警
		uint8_t Alarm_Reverse[1];						//反射报警
		uint8_t Alarm_Re[3];								//报警保留位
	
	
}Alarm_historyBack_t;				//报警查询应答包			0602

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果
}Power_onBack_t;					//开机应答包				0702

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果
}Power_offBack_t;					//关机应答包				0802

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果
    float	Fre_Band[2];			//频段
    float	swr[20];				//驻波比
    uint8_t Power_UP[20];			//最大上限功率
}Scan_FrqBack_t;					//扫频应答包				0902

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t result[1];				//状态
}Sacn_stopBack_t;					//停止扫频					0A02

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果

}Add_PowerBack_t;					//功率微调增加应答包		0B02

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果
}Sub_PowerBack_t;					//功率微调减少应答包		0C02

typedef struct
{
    uint8_t Transmitte_id[1];		//发射机ID
    uint8_t results[1];				//执行结果
}Run_DiagramBack_t;					//运行图应答包				0D02

/*----------------------------------------------------------------------*/
extern uint8_t g_fuc_cod[2];
extern uint8_t g_fuc_codToPC[2];

extern uint32_t				g_inquire_stamp;
extern uint32_t				g_stamp_distance;

extern Sys_Status_t 		System;				//系统	
extern Monitor_t			Monitor;			//硬件监控
extern Run_Diagram_data_t	Run_Diagram_data;	//获取到的运行图数据
extern Alarm_t				Alarm;				//报警
extern Temperature_Humidity_t Temperature_Humidity;		//温湿度

extern Alarm_threshold_t	Alarm_threshold;	//报警阈值				0101
extern PC_Cmd_t				PC_Cmd;				//PC指令				0301
extern Disalarm_t			Disalarm;			//PC解除报警			0501
extern Scan_Frq_t			Scan_Frq;			//扫描最佳工作频率		0901
extern Sacn_stop_t			Sacn_stop;			//停止扫频				0A01
extern Add_Power_t			Add_Power;			//功率增加				0B01
extern Sub_Power_t			Sub_Power;			//功率减小				0C01
extern Run_Diagram_t		Run_Diagram;		//运行图				0D01
extern Run_Diagram_buf_t	Run_Diagram_buf[10];//运行图缓存(100个)

extern Alarm_backPC_t		Alarm_backPC;		//报警参数初始化应答包	0102
extern Work_paraBack_t		Work_paraBack;		//查询应答包			0202
extern Trans_openBack_t		Trans_openBack;		//发射应答包			0302
extern Trans_stopBack_t		Trans_stopBack;		//停止发射应答包		0402
extern DisalarmBack_t		DisalarmBack;		//解除报警状态应答包	0502
extern Alarm_historyBack_t	Alarm_historyBack;	//历史报警应答包		0602
extern Power_onBack_t		Power_onBack;		//发射机上电应答包		0702
extern Power_offBack_t		Power_offBack;		//关机应答包			0802
extern Scan_FrqBack_t		Scan_FrqBack;		//扫频应答包			0902
extern Sacn_stopBack_t		Sacn_stopBack;		//停止扫频				0A02
extern Add_PowerBack_t		Add_PowerBack;		//功率微调增加应答包	0B02
extern Sub_PowerBack_t		Sub_PowerBack;		//功率微调减少应答包	0C02
extern Run_DiagramBack_t	Run_DiagramBack;	//运行图应答包			0D02



extern Scan_FrqBack_t		Scan_FrqBack1;		//PC扫描最佳工作频率	(查询)0902	//3.2-4.9
extern Scan_FrqBack_t		Scan_FrqBack2;		//PC扫描最佳工作频率	(查询)0902	//5.0-6.9
extern Scan_FrqBack_t		Scan_FrqBack3;		//PC扫描最佳工作频率	(查询)0902	//7.0-8.9
extern Scan_FrqBack_t		Scan_FrqBack4;		//PC扫描最佳工作频率	(查询)0902	//9.0-10.9
extern Scan_FrqBack_t		Scan_FrqBack5;		//PC扫描最佳工作频率	(查询)0902	//11.0-12.9
extern Scan_FrqBack_t		Scan_FrqBack6;		//PC扫描最佳工作频率	(查询)0902	//13.0-14.9
extern Scan_FrqBack_t		Scan_FrqBack7;		//PC扫描最佳工作频率	(查询)0902	//15.0-16.9
extern Scan_FrqBack_t		Scan_FrqBack8;		//PC扫描最佳工作频率	(查询)0902	//17.0-18.9
extern Scan_FrqBack_t		Scan_FrqBack9;		//PC扫描最佳工作频率	(查询)0902	//19.0-20.9
extern Scan_FrqBack_t		Scan_FrqBack10;		//PC扫描最佳工作频率	(查询)0902	//21.0-22.9
extern Scan_FrqBack_t		Scan_FrqBack11;		//PC扫描最佳工作频率	(查询)0902	//23.0-24.9
extern Scan_FrqBack_t		Scan_FrqBack12;		//PC扫描最佳工作频率	(查询)0902	//25.0-26.1

extern uint8_t adc1_zero_count;				//反向功率很容易等于0
extern uint8_t adc3_zero_count;				//电流
extern float ADC0_buff[AD_SAMPLE_NUM];
extern float ADC1_buff[AD_SAMPLE_NUM];
extern float ADC2_buff[AD_SAMPLE_NUM];
extern float ADC3_buff[AD_SAMPLE_NUM];

extern float SWR_array[256];				//驻波比缓存
extern float Forward_Power_array[256];		//正向功率
extern float Reverse_Power_array[256];		//反向功率

extern uint32_t	freq_band_time_stamp[10];	//存储的频段时间戳

/*--------------------------------------------------------------------*/
extern void System_Status_Clean(void);
extern void System_Status_Update(void);

#endif
