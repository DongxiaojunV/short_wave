#ifndef	_MT2000_PROTOCOL_H_
#define	_MT2000_PROTOCOL_H_

#include	"main.h"
#include	<stdint.h>
#include "W5500.h"	
#define 	Emis_Size	160   //读取串口缓冲区大小


#define		MT2000_ACK_NO_VALID			(-3)	//无效数据
#define		MT2000_ACK_ALARM			(-2)	//激励器报警
#define		MT2000_NO_ACK				(-1)	//无应答
#define		MT2000_ACK_NK				0		//设置错误
#define		MT2000_ACK_OK				1

#define		MT2000_WAIT_ACK_OK			0x01	//等待应答
#define		MT2000_CMD_POWER_ON			0x02	//开机
#define		MT2000_CMD_POWER_OFF		0x03	//关机

/* 旧机器协议指令 */
#define		MT2000_CMD_INQUIRE			0xCE	//查询状态
#define		MT2000_CMD_STOP				0x18	//停止
#define		MT2000_CMD_CHANNEL			0x43	//调用信道
#define		MT2000_CMD_FREQUENCY		0x54	//频率
#define		MT2000_CMD_SPECIES			0x4D	//工作种类
#define		MT2000_CMD_POWER_LEVEL		0x50	//功率等级
#define		MT2000_CMD_EMISSION			0x0D	//发射
#define		MT2000_CMD_MODE				0x44	//工作模式
#define		MT2000_CMD_TUNING			0x74	//调谐
#define		MT2000_CMD_POWER_ADD		0x2B	//增益加
#define		MT2000_CMD_POWER_SUB		0x2D	//增益减

typedef struct
{
	uint16_t Head_flag;	  	       //头标志位  //0~1  固定为0x42
	uint16_t backups1;	  	       //备份1  //2~3
	uint8_t  Can_ID[1];				//发射机ID	
	
	uint8_t  Gt2000_mode;         //模式   //30       //1固频、2双频、3三频、4AM                   代替MT2000_Tx.mode
	uint8_t  method;         //工作模式，新发射机没用到
	uint8_t  channel[3];     //信道，固定为01

	uint8_t forward_power[6];		       //发射机入射功率  //6~7
	uint8_t reverse_power[6];		       //发射机反射功率  //8~9
	uint8_t  swr[4];	    	             //驻波比  //10~13
	uint16_t Fault_attenuation;	      	//故障衰减  //14~15
	float    Working_current;		        //工作电流  //16~19
	uint8_t  system_status1;	         	//系统状态1  //20
	uint8_t  sys_open;		              //发射机状态2  //0关机，1开机
	uint8_t  sys_emit_open;		          //发射机状态2  //0关发射 1开发射
	uint8_t  sys_control_mode;          //发射机状态2  //本控遥控  1遥控  0本控
	uint8_t  Gt2000_freq1[10];		        //当前频率1  //22~23
	uint8_t  Gt2000_freq2[10];		        //当前频率2    //24~25
	uint8_t  Gt2000_freq3[10];		        //当前频率3    //26~27
  uint8_t  Frequency_power[2];          //功率   //28~29
	uint16_t  Now_all_power;            //当前总功率
	
	uint8_t  Frequency_band;          //频段   //31
	float Module1_current;      //模块1电流  //41~44
	float Module1_Temperature;    //模块1温度  //45~48
	float Module2_current;      //模块2电流  //49~52
	float Module2_Temperature;    //模块2温度  //53~56
	float Module3_current;      //模块3电流  //57~60
	float Module3_Temperature;    //模块3温度  //61~64
	float Module4_current;      //模块4电流  //65~68
	float Module4_Temperature;    //模块4温度  //69~72

	uint16_t Filter_Terminator [2];       //结束符   //158~159  固定为0x41 0x41
	
}GT2000_t;					//2KW发射机状态数据返回包	

extern GT2000_t  GT2000_Rx;  //接收发射机返回的状态数据
extern GT2000_t  GT2000_Tx;  //发送到发射机的数据

/*-----------------------------------------------------------------*/
extern void GT2000_Init(void);
extern void GT2000_Open(void);
extern void GT2000_Close(void);
extern int  GT2000_Emit(void);
extern void GT2000_Stop(void);							//发射机紧急停止(功放也会关)

extern void GT2000_Buffer_Clean(void);						/* 清空缓冲区(上电和解除报警) */

extern uint8_t BCC_Check_Sum(uint8_t *pdata, uint8_t len);	/* 异或和 */

/* 开机 */
extern void Gt2000_Open(void);
/* 关机 */
extern void Gt2000_Close(void);
/*开发射*/
extern void Gt_RF_Open(void);
/*关发射*/
extern void Gt_RF_Close(void);
/*功放复位*/
extern void Gt2000_Reset(void);
/*查询所有状态*/
extern void Gt_Inquire_All(void);
/*设置频率1、2、3*/
extern void Gt_Set_Freq(GT2000_t Set_CMD);
/* 设置功率 */
extern void Gt_Set_Power(GT2000_t Cmd_Power);
/* 设置工作模式1~4 */
extern void Gt_Set_Mode(uint8_t num);
/*信号源工作参数设置*/
extern void GT_Set_Value(GT2000_t Send_CMD);						

extern void GT2000_Alarm_Stop(void);					/* 激励器驻波比报警停止 */



extern	uint8_t Emission_Tx[Emis_Size];		//发送缓冲区
extern	uint8_t Emission_Rx[Emis_Size];		//接收缓冲区

extern void power_add(uint8_t *data); //功率微调+65计算
extern void power_sub(uint8_t *data);  //功率微减-65计算
extern void Range_Power(GT2000_t *Value); /* 功率转化为系统幅度值 */
extern void Power_Range(GT2000_t *Vlaue,uint8_t *Data);		//北京短波发射机幅度值转换函数

extern int GT2000_Wait_Ack(uint8_t cmd);								//北京短波发射机应答处理函数
extern int GT2000_Cmd_Analyze(uint8_t *msg, uint8_t cmd);//北京短波发射机数据解析函数
extern uint8_t Bj_BCC_Check_Sum(uint8_t *pdata, uint8_t len);//北京短波发射机通讯异或和算法函数

#endif
