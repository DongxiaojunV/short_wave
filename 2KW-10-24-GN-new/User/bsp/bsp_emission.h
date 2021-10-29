#ifndef _BSP_EMISSION_H_
#define _BSP_EMISSION_H_
#include <stdint.h>

//#define Emis_Size	64

//typedef struct Temperature_humidity
//{
//    float Temperature;//温度
//    float Humidity;//湿度
//} Temperature_Humidity_t;

//typedef struct best_band
//{
//    float best_band_max;//最佳频段上限
//    float best_band_min;//最佳频段下限
//} Best_band_t;

//typedef struct show_parameter
//{
//    float Forward_Power;//正向功率，实际为电压
//    float Reverse_Power;//反向功率，实际为电压
//    float M_45V;//45V电压
//    float M_45I;//45V下的电流
//    float Standing_wave_ratio;//驻波比
//} Show_Parameter_t;

//typedef struct state
//{
//    volatile uint8_t Sys_gain;//系统增加功率
//    volatile uint8_t Sys_reduction;//系统减少功率
//    volatile uint8_t Sys_open;//系统开机
//    volatile uint8_t Sys_close;//系统关机
//    volatile uint8_t Sys_opening;//系统开机中
//    volatile uint8_t FM_working;//正在操作调频
//    volatile uint8_t Emis_working;//正在发射中
//    volatile uint8_t Emis_readay;//发射准备

//    volatile uint8_t History_alarm;//历史报警记录
//    volatile uint8_t Already_init;//已经初始化标志
//    volatile uint8_t Sweeping;//正在扫频状态
//    volatile uint8_t Already_Swept;//已经扫频完成标志，该标志需保存在flash上
//    volatile uint8_t Cancel_Sweeping;//取消扫频

//    volatile uint8_t SWR_alarm;//驻波比报警
//    volatile uint8_t No_power_level;//无功率输出
//    volatile uint8_t alarm;//激励器自身报警
//    volatile uint8_t no_respond;//激励器3次操作不响应
//    volatile uint8_t temperature_alarm;//温度报警
//    volatile uint8_t humidity_alarm;//湿度报警
//    volatile uint8_t over_Electric;//过流报警
//    volatile uint8_t over_Voltage;//过压报警
//    volatile uint8_t low_Voltage;//欠压报警
//} State_t;
//typedef union cmd
//{
//    volatile uint8_t inquire;//查询状态  CE
//    volatile uint8_t frequency;//工作频率 54
//    volatile uint8_t species;//工作种类   4D
//    volatile uint8_t power_level;//功率等级 50
//    volatile uint8_t channel;//信道  43
//    volatile uint8_t emission;//发射  0D
//    volatile uint8_t stop;//停止  18
//    volatile uint8_t gain_plus;//增益加 2B
//    volatile uint8_t gain_reduction;//增益减 2D
//    volatile uint8_t mode;//工作模式 44
//} cmd_t;

//typedef struct emission
//{
//    volatile uint8_t head;//帧头
//    cmd_t 	cmd;//指令码
//    volatile uint8_t data[4];//数据
//    volatile uint8_t tail;//帧尾
//} EmissionCmd_t;

//typedef struct respond
//{
//    uint8_t head[2];//固定帧头
//    volatile uint8_t channel;//信道
//    uint8_t frequency[4];//工作频率
//    volatile uint8_t species;//工作种类			//FM
//    volatile uint8_t status;//工作状态
//    volatile uint8_t mode;//工作模式			//固频、双频、三频
//    volatile uint8_t whethre_work;//是否工作
//    volatile uint8_t unused;//保留位，没有使用
//    volatile uint8_t power_level;//功率等级
//    volatile uint8_t check;//异或校验
//    uint8_t tail;//固定帧尾 因为是固定的，所以不加volatile修饰了
//} Respond_t;

///*----------------------------------------------------------------------------------------*/
//typedef struct monitor
//{
//    volatile uint8_t no_resp_count;//没有响应计数,PC心跳异常计数
//    volatile uint8_t hard_control;//硬件接管
//    volatile uint8_t rec_run_diagram;//接收到运行图
//    volatile uint8_t diagram_count;//一个过程中接收运行图次数
//    volatile uint8_t need_open;//需要开机
//} Monitor_t;

//typedef struct hard_control
//{
//    volatile uint8_t usage_diagram_count;//记录是第几次查询到可用运行图
//} Hard_control_t;

//typedef struct state_monitor
//{
//    volatile uint8_t start[10];
//    volatile uint8_t end[10];
//} State_monitor_t;
////查询状态  CE
//#define     INQUIRE      	0xCE
////停止
//#define     STOP         	0x18
////调用信道
//#define		CHANNEL			0x43
////频率
//#define 	FREQUENCY		0x54
////工作种类
//#define 	SPECIES			0x4D
////功率等级
//#define 	POWER_LEVEL		0x50
////发射
//#define 	EMISSION		0x0D
////工作模式
//#define		MODE			0x44
////调谐
//#define		TUNING			0x74
////增益加
//#define		Gain_plus		0x2B
//#define		POWER_ADD		0x2B		//兼容
////增益减
//#define 	Gain_reduction	0x2D
//#define 	POWER_SUB		0x2D		//兼容
////	uint8_t mode;//工作模式 44

//extern volatile uint8_t clean_sweeping;
//extern volatile uint8_t set_open;
//extern volatile uint8_t clean_open;
//extern volatile uint8_t clean_gain;
//extern volatile uint8_t clean_reduction;
//extern volatile uint8_t clean_FM;
////extern TaskHandle_t xHandleTaskEmis_Send;
////extern TaskHandle_t xHandleTask_Receive ;

//extern volatile int lock_flag1;
//extern volatile int lock_flag2;
//extern volatile int lock_flag3;

//extern Respond_t Respond_Cmd;	//激励器返回结构体
//extern Respond_t Respond_Cmd2;	//激励器返回结构体，双频
//extern Respond_t Respond_Cmd3;	//激励器返回结构体，三频

//void vTaskTaskTaskEmis_Receive(void *pvParameters);
//void vTaskTaskTaskEmis_Send(void *pvParameters);
//void vTaskTaskTask_hard_control(void *pvParameters);
//void vTaskTask_hard_monitor(void *pvParameters);
//void Pre_EmissionSend(EmissionCmd_t * item,uint8_t cmd,...);
//int get_history_alm(void);
//void clean_all_flags(void);
//void clean_alarm_bit_flags(void);
//int get_current_alm(void);
//float get_Standing_wave_ratio(float Forward_Power,float Reverse_Power);
//int find_hard_control(uint8_t Save_count);
//void juge_isOpen(uint8_t Save_count);
//int Advance_run_command(TickType_t xTicksToWait);

//extern int MT2000_Wait_Ack(void);

#endif /*bsp_emission.h*/

