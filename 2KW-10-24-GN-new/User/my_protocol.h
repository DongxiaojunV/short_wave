#ifndef	_MY_PROTOCOL_H_
#define	_MY_PROTOCOL_H_

#include "includes.h"
#include <stdint.h>
#define		ADD_CAN_LEN						1		//���ظ�PC�������У�����λ�Ƿ����CAN����
#define		CEIL_DIV(A, B)	(((A) + (B) -1) / (B))	//����ȡ������	/* ���ظ�PC��CAN���ݳ���ʹ��������ȡ�� */

typedef struct
{
    uint8_t eff_sign;			//��־�������������Ч
    uint8_t CAN_ID;				//CANID
    uint8_t Func_code[2];		//������
    uint8_t data_buf[256];		//��������
	uint8_t len;				//���ݳ���
}W5500_data_t;					//���յ�CAN����

typedef struct
{
    uint8_t data[256];
    uint8_t len;
}Buffer_t; 						//��������ɵ�����


/*---------------------------------����״̬-----------------------------------*/
void update_status_without_inquire(void);

/*-------------------------------CAN-----------------------------------*/
/* ���������������þɳ��򣬸��˺��������ѡ���д���ڸø��ֺ����·�������ִ�в�ѯ������������������á� */
/* ���ݽ��� */
extern uint8_t CAN_data_analyze(uint8_t *p_func_code_buf, Buffer_t *p_CAN_dat);		//CAN���ݷ���
extern uint8_t judge_is_valid_can_data(Buffer_t *buffer);					//�ж�������ȷ��
extern W5500_data_t Get_CAN_Data(Buffer_t *p_PC_data);								//ȥ֡β

extern uint8_t Data_Storage(W5500_data_t *CAN_data_cmp);								//���ݸ�ֵ����

//װ��Э���������
extern uint8_t Data_Assemble(uint8_t *Func_code, uint8_t *p_data, uint8_t Can_ID, uint8_t len, Buffer_t *Buffer);
extern uint8_t Send_PC(uint8_t *Func_code);

/*---------------------------------------------------------------------*/
//��ȡɨƵ�ϵ㣬��������һ��Ƶ��
extern uint16_t Scan_Breakpoint_Read(void);

//����ɨƵ�ϵ�
extern void Scan_Breakpoint_Save(uint8_t current_point_int, uint8_t current_point_dec);

/* ɨƵ */
extern void Band_scan(uint8_t freq_begin, uint8_t freq_end, uint8_t index_dec);
extern void Optional_Band_scan(uint8_t Now_Frq,uint8_t Scan_end);
/*---------------------------------------------------------------------*/
extern void func_code_printf(void);

/* Ƶ�� */
extern void freq_hex_to_str(uint8_t *freq_hex, uint8_t *freq_str);					//hex��Ƶ��ת��ΪstrƵ��
extern void freq_str_to_hex(uint8_t *freq_str, uint8_t *freq_hex);					//str��Ƶ��ת��ΪhexƵ��
extern void freq_PChex_to_GT2000(uint8_t *freq_PC_str,uint8_t *freq_Trans_hex);  //PC�·���Ƶ�ʸ�ֵ��GT2000����λ��ǰ
extern void freq_GT2000Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex);  //GT2000��Ƶ�ʸ�ֵ��GT2000����λ��ǰ
extern void GT2000_Tx_freq_Hex_to_PChex(unsigned char *freq_Trans_hex, unsigned char *freq_PC_Hex);  //����ѯɨƵ����ʹ��
extern unsigned char *mystrncpy( unsigned char *string, int n);   //Ҫ���ȡ���ַ��������Ըı䣬��ָ���ַ�����ָ����Ըı�
extern void StrToHex(uint8_t *pbDest, uint8_t *pbSrc, int nLen); // remarks : ���ַ���ת��Ϊ16������
extern void freq_range_judge(uint8_t *freq);
extern int str_to_10D(uint8_t s[]);/*���ַ���sת������Ӧ������*/
//find_hard_control()��flash��ȡ����ͼ֮��ͨ��set_run_diagram_power_level()��ֵ��Run_Diagram_data��
//��Task_Hardware_Monitor()�ж��Ƿ���Ҫ���䣬���ֵ��MT2000_Tx��Ȼ����
extern void		set_run_diagram_new_power_level(uint8_t channel, uint8_t power_level);	//��������ͼ�Ĺ��ʵȼ�	Run_Diagram_data

#endif
