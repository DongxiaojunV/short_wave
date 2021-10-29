#ifndef	_MY_PROTOCOL_H_
#define	_MY_PROTOCOL_H_

#include "includes.h"
#include <stdint.h>
#define		ADD_CAN_LEN						1		//返回给PC的数据中，保留位是否加入CAN长度
#define		CEIL_DIV(A, B)	(((A) + (B) -1) / (B))	//向上取整函数	/* 返回给PC的CAN数据长度使用了向上取整 */

typedef struct
{
    uint8_t eff_sign;			//标志这里面的数否有效
    uint8_t CAN_ID;				//CANID
    uint8_t Func_code[2];		//功能码
    uint8_t data_buf[256];		//具体数据
	uint8_t len;				//数据长度
}W5500_data_t;					//接收的CAN数据

typedef struct
{
    uint8_t data[256];
    uint8_t len;
}Buffer_t; 						//存放组包完成的数据


/*---------------------------------更新状态-----------------------------------*/
void update_status_without_inquire(void);

/*-------------------------------CAN-----------------------------------*/
/* 下面三个函数沿用旧程序，改了函数名而已。重写的在该各种函数下方，可以执行查询，解析其他会出错，不用。 */
/* 数据解析 */
extern uint8_t CAN_data_analyze(uint8_t *p_func_code_buf, Buffer_t *p_CAN_dat);		//CAN数据分析
extern uint8_t judge_is_valid_can_data(Buffer_t *buffer);					//判断数据正确性
extern W5500_data_t Get_CAN_Data(Buffer_t *p_PC_data);								//去帧尾

extern uint8_t Data_Storage(W5500_data_t *CAN_data_cmp);								//数据赋值更新

//装载协议包及发送
extern uint8_t Data_Assemble(uint8_t *Func_code, uint8_t *p_data, uint8_t Can_ID, uint8_t len, Buffer_t *Buffer);
extern uint8_t Send_PC(uint8_t *Func_code);

/*---------------------------------------------------------------------*/
//读取扫频断点，并返回下一个频点
extern uint16_t Scan_Breakpoint_Read(void);

//保存扫频断点
extern void Scan_Breakpoint_Save(uint8_t current_point_int, uint8_t current_point_dec);

/* 扫频 */
extern void Band_scan(uint8_t freq_begin, uint8_t freq_end, uint8_t index_dec);
extern void Optional_Band_scan(uint8_t Now_Frq,uint8_t Scan_end);
/*---------------------------------------------------------------------*/
extern void func_code_printf(void);

/* 频率 */
extern void freq_hex_to_str(uint8_t *freq_hex, uint8_t *freq_str);					//hex的频率转换为str频率
extern void freq_str_to_hex(uint8_t *freq_str, uint8_t *freq_hex);					//str的频率转换为hex频率
extern void freq_PChex_to_GT2000(uint8_t *freq_PC_str,uint8_t *freq_Trans_hex);  //PC下发的频率赋值给GT2000，低位在前
extern void freq_GT2000Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex);  //GT2000的频率赋值给GT2000，低位在前
extern void GT2000_Tx_freq_Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex);  //供查询扫频函数使用
extern unsigned char *mystrncpy( unsigned char *string, int n);   //要求截取的字符串不可以改变，但指向字符串的指针可以改变
extern void StrToHex(uint8_t *pbDest, uint8_t *pbSrc, int nLen); // remarks : 将字符串转化为16进制数
extern void freq_range_judge(uint8_t *freq);
extern int str_to_10D(uint8_t s[]);/*将字符串s转换成相应的整数*/
//find_hard_control()从flash获取运行图之后，通过set_run_diagram_power_level()赋值给Run_Diagram_data，
//在Task_Hardware_Monitor()判断是否需要发射，最后赋值给MT2000_Tx，然后发送
extern void		set_run_diagram_new_power_level(uint8_t channel, uint8_t power_level);	//设置运行图的功率等级	Run_Diagram_data

#endif
