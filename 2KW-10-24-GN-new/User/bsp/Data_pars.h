#ifndef __DATA_PARS_H__
#define __DATA_PARS_H__
#include <stdint.h>		/* uint8_t ... */

//#define Only_Fixed_Frequency//只能工作在固频状态宏定义


//#pragma pack(1)

/*----------------------------------------------------------------------------------------*/
//typedef struct
//{
//    float 	Low_temp_limit[1];//温度下线
//    float 	Upp_temp_limit[1];//温度上线
//    float 	Low_humidity_limit[1];//湿度下线
//    float 	Upp_humidity_limit[1];//湿度上线
//    float 	Low_45I_limit[1];//电流下线
//    float 	Upp_45I_limit[1];//电流上线
//    float 	Low_45V_limit[1];//电压下线
//    float 	Upp_45V_limit[1];//电压上线
//    uint8_t timer[6];//北斗时间
//    uint8_t Transmitte_id[1];//发射机ID
//} Alarm_threshold_t; //PC下发报警阈值的结构体

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t	Alarm_paramet[2];//报警参数
//} Alarm_backPC_t; //PC设置报警的返回包(由发射机返回PC)

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Retain[2];//这个是保留位
//} Working_paramet_t; //PC查询发射机工作状态以及环境的参数的指令

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Trans_current_state[1];//发射机当前的状态
//    uint8_t Type[1];//当前工作的种类						//固频、双频、三频
//    uint8_t Channel[3];//信道
//    uint8_t Freq[12];//当前工作的频率
//    uint8_t Power_grade[3];//当前工作的功率等级
//    float Forward_power[1];//发射机正向功率
//    float Reverse_power[1];//发射机反向功率
//    float Power_45_intensity[1];//发射机45V供电电流取样
//    float Power_45_voltage[1];//发射机45V供电电压取样
//    float temp[1];//环境温度
//    float humidity[1];//湿度
//    uint8_t Hist_alarm[1];//查看有无历史报警数据
//} Work_paraBack_t; //返回PC发射机工作参数的数据包

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Type[1];//需要工作的种类
//    uint8_t Channel[3];//需要工作信道
//    uint8_t Freq[12];//需要发射的频率
//    uint8_t Power_grade[3];//需要工作的功率等级
//    uint8_t Berdou_time[6];//北斗时间
//} Trans_open_t; //开启发射机

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Trans_state[2];//发射机返回PC此时的状态
//} Trans_openBack_t; //开启发射机向PC返回的数据包

//typedef struct
//{
//    uint8_t Type[1];//需要工作的种类
//    uint8_t Channel[3];//需要工作信道
//    uint8_t Freq[12];//需要发射的频率
//    uint8_t Power_grade[3];//需要工作的功率等级
//} Trans_open_Copy_t; //开启发射机副本

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Berdou_time[6];//北斗时间
//} Trans_stop_t; //停止发射机

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Trans_state[1];//返回发射机执行关闭的结果
//} Trans_stopBack_t; //停止发射机向PC返回的数据包

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Disalarm_type[2];//解除报警类型
//    uint8_t Berdou_time[6];//北斗时间
//} Disalarm_t; //PC解除报警状态

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Disalarm_result[2];//解除报警状态结果
//    uint8_t current_alarm_state[2];//当前报警状态
//} DisalarmBack_t; //解除报警状态的应答包

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Berdou_time[6];//北斗时间
//} Alarm_history_t; //PC查询历史报警


//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t alarm_history[2];//历史存在的报警参数
//    uint8_t Power_free_time[6];//无功率输出的时间
//    float 	Temp_history[1];//报警温度时候的值
//    uint8_t Temp_time[6];//温度报警的时间
//    float		humidity_history[1];//湿度报警时候的值
//    uint8_t Humidity_time[6];//湿度报警的时间
//    float 	Power_45A_history[1];//发射机45V供电电流时候的值（报警）
//    uint8_t Power_45A_time[6];//发射机45V供电电流报警时间
//    float		Power_45V_history[1];//发射机45V供电电压报警时候的值
//    uint8_t Power_45V_time[6];//发射机45V供电电压报警的时间
//    uint8_t No_response_time[6];//发射机无响应时间
//    uint8_t	Exciter_alarm[6];//发射机报警时间
//    float 	Bobbi[1];//驻波比
//    uint8_t Bobbi_alarm[6];//驻波比报警时间
//} Alarm_historyBack_t;

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Retain[1];//保留
//    uint8_t Berdou_time[6];//北斗时间
//} Power_on_t; //PC开启发射机（电源）

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果
//} Power_onBack_t; //返回PC开启发射机（电源）

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Retain[1];//保留
//} Power_off_t; //PC关闭电源

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果
//} Power_offBack_t; //返回PC关闭电源

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t state[1];//状态
//    float		Fre_Band[2];//频段
//} Scan_Frq_t; //PC扫描最佳工作频率

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果
//    float		Fre_Band[2];//频段
//    float		bobibi[20];//驻波比
//    uint8_t Power_UP[20];//最大上线功率
//} Scan_FrqBack_t; //PC扫描最佳工作频率返回

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t state[1];//状态
//} Sacn_stop_t; //PC停止扫描

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t result[1];//状态
//} Sacn_stopBack_t; //返回PC停止扫描

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Power_UP[2];//需要增加的功率

//} Add_Power_t; //功率微调增加
//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t Power_DOWN[2];//需要减少的功率

//} Sub_Power_t; //功率微调减少


//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果

//} Add_PowerBack_t; //功率微调增加回包
//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果

//} Sub_PowerBack_t; //功率微调减少回包


//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t count[1];//有几段运行图
//    uint8_t Continue[2];//是否还有运行图

//    uint8_t Start_Time1[2];//开始时间
//    uint8_t End_Timep1[2];//结束时间
//    uint8_t Power1[1];//功率
//    uint8_t Frq1[4];//频率
//    uint8_t channel1[1];//信道

//    uint8_t Start_Time2[2];//开始时间
//    uint8_t End_Timep2[2];//结束时间
//    uint8_t Power2[1];//功率
//    uint8_t Frq2[4];//频率
//    uint8_t channel2[1];//信道

//    uint8_t Start_Time3[2];//开始时间
//    uint8_t End_Timep3[2];//结束时间
//    uint8_t Power3[1];//功率
//    uint8_t Frq3[4];//频率
//    uint8_t channel3[1];//信道

//    uint8_t Start_Time4[2];//开始时间
//    uint8_t End_Timep4[2];//结束时间
//    uint8_t Power4[1];//功率
//    uint8_t Frq4[4];//频率
//    uint8_t channel4[1];//信道

//    uint8_t Start_Time5[2];//开始时间
//    uint8_t End_Timep5[2];//结束时间
//    uint8_t Power5[1];//功率
//    uint8_t Frq5[4];//频率
//    uint8_t channel5[1];//信道

//    uint8_t Start_Time6[2];//开始时间
//    uint8_t End_Timep6[2];//结束时间
//    uint8_t Power6[1];//功率
//    uint8_t Frq6[4];//频率
//    uint8_t channel6[1];//信道

//    uint8_t Start_Time7[2];//开始时间
//    uint8_t End_Timep7[2];//结束时间
//    uint8_t Power7[1];//功率
//    uint8_t Frq7[4];//频率
//    uint8_t channel7[1];//信道

//    uint8_t Start_Time8[2];//开始时间
//    uint8_t End_Timep8[2];//结束时间
//    uint8_t Power8[1];//功率
//    uint8_t Frq8[4];//频率
//    uint8_t channel8[1];//信道

//    uint8_t Start_Time9[2];//开始时间
//    uint8_t End_Timep9[2];//结束时间
//    uint8_t Power9[1];//功率
//    uint8_t Frq9[4];//频率
//    uint8_t channel9[1];//信道

//    uint8_t Start_Time10[2];//开始时间
//    uint8_t End_Timep10[2];//结束时间
//    uint8_t Power10[1];//功率
//    uint8_t Frq10[4];//频率
//    uint8_t channel10[1];//信道
//} Run_Diagram_t;

//typedef struct
//{
//    uint8_t Transmitte_id[1];//发射机ID
//    uint8_t results[1];//执行结果

//} Run_DiagramBack_t; //运行图返回包

//typedef struct
//{
//    uint8_t Start_Time1[2];//开始时间
//    uint8_t End_Timep1[2];//结束时间
//    uint8_t Power1[1];//功率
//    uint8_t Frq1[4];//频率
//    uint8_t channel1[1];//信道

//    uint8_t Start_Time2[2];//开始时间
//    uint8_t End_Timep2[2];//结束时间
//    uint8_t Power2[1];//功率
//    uint8_t Frq2[4];//频率
//    uint8_t channel2[1];//信道

//    uint8_t Start_Time3[2];//开始时间
//    uint8_t End_Timep3[2];//结束时间
//    uint8_t Power3[1];//功率
//    uint8_t Frq3[4];//频率
//    uint8_t channel3[1];//信道

//    uint8_t Start_Time4[2];//开始时间
//    uint8_t End_Timep4[2];//结束时间
//    uint8_t Power4[1];//功率
//    uint8_t Frq4[4];//频率
//    uint8_t channel4[1];//信道

//    uint8_t Start_Time5[2];//开始时间
//    uint8_t End_Timep5[2];//结束时间
//    uint8_t Power5[1];//功率
//    uint8_t Frq5[4];//频率
//    uint8_t channel5[1];//信道

//    uint8_t Start_Time6[2];//开始时间
//    uint8_t End_Timep6[2];//结束时间
//    uint8_t Power6[1];//功率
//    uint8_t Frq6[4];//频率
//    uint8_t channel6[1];//信道

//    uint8_t Start_Time7[2];//开始时间
//    uint8_t End_Timep7[2];//结束时间
//    uint8_t Power7[1];//功率
//    uint8_t Frq7[4];//频率
//    uint8_t channel7[1];//信道

//    uint8_t Start_Time8[2];//开始时间
//    uint8_t End_Timep8[2];//结束时间
//    uint8_t Power8[1];//功率
//    uint8_t Frq8[4];//频率
//    uint8_t channel8[1];//信道

//    uint8_t Start_Time9[2];//开始时间
//    uint8_t End_Timep9[2];//结束时间
//    uint8_t Power9[1];//功率
//    uint8_t Frq9[4];//频率
//    uint8_t channel9[1];//信道

//    uint8_t Start_Time10[2];//开始时间
//    uint8_t End_Timep10[2];//结束时间
//    uint8_t Power10[1];//功率
//    uint8_t Frq10[4];//频率
//    uint8_t channel10[1];//信道
//} Run_Diagram_buff_t;



//typedef struct
//{
//    uint8_t Alarm_PC_code[2];//PC下发报警参数的结构体
//    uint8_t Alarm_Back_code[2];//发射机返回PC报警
//    uint8_t Work_PC_code[2];//PC下发查询状态
//    uint8_t Work_PCBack_code[2];//PC下发查询发射机返回
//    uint8_t Topen_PC_code[2];//PC下发开启发射机
//    uint8_t Topen_PCBack_code[2];//发射机返回PC开启关闭的状态
//    uint8_t Tstop_PC_code[2];//PC下发开启发射机
//    uint8_t Tstop_PCBack_code[2];//发射机返回PC开启关闭的状态
//    uint8_t Disalarm_PC_code[2];//解除报警状态
//    uint8_t Disalarm_PCBack_code[2];//解除报警状态返回PC
//    uint8_t Histalarm_PC_code[2];//pc下发查询历史报警包
//    uint8_t Histalarm_PCBack_code[2];//PC下发查询历史报警包的回包
//    uint8_t PowerON_PC_code[2];//PC下发上电
//    uint8_t PowerON_PCBack_code[2];//PC下发上电的回包
//    uint8_t PowerOFF_PC_code[2];//PC下发断电
//    uint8_t PowerOFF_PCBack_code[2];//PC下发断电的回包
//    uint8_t ScanFrq_PC_code[2];//PC下发最佳工作频率
//    uint8_t ScanFrq_PCBack_code[2];//PC下发最佳工作频率的回包
//    uint8_t ScanStop_PC_code[2];//PC下发停止扫描
//    uint8_t ScanStop_PCBack_code[2];//PC下发停止扫描的回包
//    uint8_t AddPower_PC_code[2];//增加功率微调
//    uint8_t AddPower_PCBack_code[2];//增加功率微调的回包
//    uint8_t SubPower_PC_code[2];//减少功率微调
//    uint8_t SubPower_PCBack_code[2];//减少功率微调
//} Func_code_t;

//typedef struct
//{
//    uint8_t uart_pc[256];
//    volatile uint8_t len;
//} pc_data_t; //收到串口的原始数据

/*************
****************
******************
********************
**********************
************************
**************************
****************************
**************************
************************
**********************
********************
******************
****************
**************/

//typedef struct
//{
//    uint8_t Spe_data[256];//解析出来数据具体数据(应该是按照协议，里面的结构体长度一致)
//    uint8_t Spe_len;//具体数据的长度
//} Spe_data_buf_t;

//typedef struct
//{
//    uint8_t eff_sign;//标志这里面的数否有效
//    uint8_t CAN_ID_buf;//CANID
//    uint8_t Func_code[2];//功能码
//    Spe_data_buf_t p_Spe_data;//具体数据
//} CAN_data_t;

//typedef struct
//{
//    uint8_t Data_buffer[256];
//    uint8_t Len_buffer;
//} Assemble_buffer_t; //存放组包之后的数据(已经是完整的数据包了)

#pragma pack()

//extern volatile uint8_t CAN_ID;
//extern Scan_FrqBack_t				Scan_FrqBack1;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack2;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack3;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack4;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack5;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack6;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack7;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack8;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack9;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack10;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack11;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack12;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack13;//PC扫描最佳工作频率
//extern Alarm_threshold_t			Alarm_threshold;//PC下发报警阈值的结构体
//extern Alarm_backPC_t				Alarm_backPC;//PC设置报警的返回包(由发射机返回PC)
//extern Working_paramet_t			Working_paramet;//PC查询发射机工作状态以及环境的参数的指令
//extern Work_paraBack_t				Work_paraBack;//返回PC发射机工作参数的数据包
//extern Trans_open_t					Trans_open;//开启发射机
//extern Trans_openBack_t				Trans_openBack;//开启发射机向PC返回的数据包
//extern Trans_open_Copy_t     		Trans_open_Copy;//开启发射机副本
//extern Trans_stop_t					Trans_stop;//停止发射机
//extern Trans_stopBack_t				Trans_stopBack;//停止发射机向PC返回的数据包
//extern Disalarm_t					Disalarm;//PC解除报警状态
//extern DisalarmBack_t				DisalarmBack;//解除报警状态的应答包
//extern Alarm_history_t 				Alarm_history;//具体报警状态的查询
//extern Alarm_historyBack_t 			Alarm_historyBack;//具体的报警状态包返回
//extern Power_on_t 					Power_on;//PC开启发射机（电源）
//extern Power_onBack_t 				Power_onBack;//返回PC开启发射机（电源）
//extern Power_off_t					Power_off;//PC关闭电源
//extern Power_offBack_t				Power_offBack;//返回PC关闭电源
//extern Scan_Frq_t					Scan_Frq;//PC扫描最佳工作频率
//extern Scan_FrqBack_t				Scan_FrqBack;//PC扫描最佳工作频率
//extern Sacn_stop_t					Sacn_stop;//PC停止扫描
//extern Sacn_stopBack_t				Sacn_stopBack;//返回PC停止扫描
//extern Sub_PowerBack_t 				Sub_PowerBack;//减少功率返回的包
//extern Sub_Power_t 					Sub_Power;//减少功率
//extern Add_PowerBack_t 				Add_PowerBack;//增加功率的回包
//extern Add_Power_t 					Add_Power;//增加功率
//extern Run_DiagramBack_t 	  		Run_DiagramBack;//运行图返回
//extern Run_Diagram_t 				Run_Diagram;//运行图
//extern Run_Diagram_buff_t   		Run_Diagram_buff[10];//100个运行图缓存
//extern volatile uint8_t				Scan_Freq_flag;
//extern pc_data_t PC_data;

//uint8_t CAN_data_func(uint8_t *p_func_code_buf,pc_data_t *p_PC_dat);
//uint8_t Send_PC(uint8_t *Func_code);

//static CAN_data_t Accpet_CAN_data(pc_data_t *PC_dat);//这个是用在发射机那个主控的解析函数
//static uint8_t Data_Storage(CAN_data_t *CAN_data_cmp);
//static uint8_t Data_Assemble(uint8_t *Func_code,uint8_t *Specific_data,uint8_t Can_ID_buf,uint8_t Len,Assemble_buffer_t *Buffer);

//static uint8_t Check_Parameters_01(uint8_t *check_code);
//static uint8_t Check_Parameters_02(uint8_t *check_code);
//static uint8_t Check_Parameters_03(uint8_t *check_code);
//static uint8_t Check_Parameters_04(uint8_t *check_code);
//static uint8_t Check_Parameters_05(uint8_t *check_code);
//static uint8_t Check_Parameters_06(uint8_t *check_code);
#endif /*   */
