#ifndef	_MT2000_PROTOCOL_H_
#define	_MT2000_PROTOCOL_H_

#include	"main.h"
#include	<stdint.h>
#include "W5500.h"	
#define 	Emis_Size	160   //��ȡ���ڻ�������С


#define		MT2000_ACK_NO_VALID			(-3)	//��Ч����
#define		MT2000_ACK_ALARM			(-2)	//����������
#define		MT2000_NO_ACK				(-1)	//��Ӧ��
#define		MT2000_ACK_NK				0		//���ô���
#define		MT2000_ACK_OK				1

#define		MT2000_WAIT_ACK_OK			0x01	//�ȴ�Ӧ��
#define		MT2000_CMD_POWER_ON			0x02	//����
#define		MT2000_CMD_POWER_OFF		0x03	//�ػ�

/* �ɻ���Э��ָ�� */
#define		MT2000_CMD_INQUIRE			0xCE	//��ѯ״̬
#define		MT2000_CMD_STOP				0x18	//ֹͣ
#define		MT2000_CMD_CHANNEL			0x43	//�����ŵ�
#define		MT2000_CMD_FREQUENCY		0x54	//Ƶ��
#define		MT2000_CMD_SPECIES			0x4D	//��������
#define		MT2000_CMD_POWER_LEVEL		0x50	//���ʵȼ�
#define		MT2000_CMD_EMISSION			0x0D	//����
#define		MT2000_CMD_MODE				0x44	//����ģʽ
#define		MT2000_CMD_TUNING			0x74	//��г
#define		MT2000_CMD_POWER_ADD		0x2B	//�����
#define		MT2000_CMD_POWER_SUB		0x2D	//�����

typedef struct
{
	uint16_t Head_flag;	  	       //ͷ��־λ  //0~1  �̶�Ϊ0x42
	uint16_t backups1;	  	       //����1  //2~3
	uint8_t  Can_ID[1];				//�����ID	
	
	uint8_t  Gt2000_mode;         //ģʽ   //30       //1��Ƶ��2˫Ƶ��3��Ƶ��4AM                   ����MT2000_Tx.mode
	uint8_t  method;         //����ģʽ���·����û�õ�
	uint8_t  channel[3];     //�ŵ����̶�Ϊ01

	uint8_t forward_power[6];		       //��������书��  //6~7
	uint8_t reverse_power[6];		       //��������书��  //8~9
	uint8_t  swr[4];	    	             //פ����  //10~13
	uint16_t Fault_attenuation;	      	//����˥��  //14~15
	float    Working_current;		        //��������  //16~19
	uint8_t  system_status1;	         	//ϵͳ״̬1  //20
	uint8_t  sys_open;		              //�����״̬2  //0�ػ���1����
	uint8_t  sys_emit_open;		          //�����״̬2  //0�ط��� 1������
	uint8_t  sys_control_mode;          //�����״̬2  //����ң��  1ң��  0����
	uint8_t  Gt2000_freq1[10];		        //��ǰƵ��1  //22~23
	uint8_t  Gt2000_freq2[10];		        //��ǰƵ��2    //24~25
	uint8_t  Gt2000_freq3[10];		        //��ǰƵ��3    //26~27
  uint8_t  Frequency_power[2];          //����   //28~29
	uint16_t  Now_all_power;            //��ǰ�ܹ���
	
	uint8_t  Frequency_band;          //Ƶ��   //31
	float Module1_current;      //ģ��1����  //41~44
	float Module1_Temperature;    //ģ��1�¶�  //45~48
	float Module2_current;      //ģ��2����  //49~52
	float Module2_Temperature;    //ģ��2�¶�  //53~56
	float Module3_current;      //ģ��3����  //57~60
	float Module3_Temperature;    //ģ��3�¶�  //61~64
	float Module4_current;      //ģ��4����  //65~68
	float Module4_Temperature;    //ģ��4�¶�  //69~72

	uint16_t Filter_Terminator [2];       //������   //158~159  �̶�Ϊ0x41 0x41
	
}GT2000_t;					//2KW�����״̬���ݷ��ذ�	

extern GT2000_t  GT2000_Rx;  //���շ�������ص�״̬����
extern GT2000_t  GT2000_Tx;  //���͵������������

/*-----------------------------------------------------------------*/
extern void GT2000_Init(void);
extern void GT2000_Open(void);
extern void GT2000_Close(void);
extern int  GT2000_Emit(void);
extern void GT2000_Stop(void);							//���������ֹͣ(����Ҳ���)

extern void GT2000_Buffer_Clean(void);						/* ��ջ�����(�ϵ�ͽ������) */

extern uint8_t BCC_Check_Sum(uint8_t *pdata, uint8_t len);	/* ���� */

/* ���� */
extern void Gt2000_Open(void);
/* �ػ� */
extern void Gt2000_Close(void);
/*������*/
extern void Gt_RF_Open(void);
/*�ط���*/
extern void Gt_RF_Close(void);
/*���Ÿ�λ*/
extern void Gt2000_Reset(void);
/*��ѯ����״̬*/
extern void Gt_Inquire_All(void);
/*����Ƶ��1��2��3*/
extern void Gt_Set_Freq(GT2000_t Set_CMD);
/* ���ù��� */
extern void Gt_Set_Power(GT2000_t Cmd_Power);
/* ���ù���ģʽ1~4 */
extern void Gt_Set_Mode(uint8_t num);
/*�ź�Դ������������*/
extern void GT_Set_Value(GT2000_t Send_CMD);						

extern void GT2000_Alarm_Stop(void);					/* ������פ���ȱ���ֹͣ */



extern	uint8_t Emission_Tx[Emis_Size];		//���ͻ�����
extern	uint8_t Emission_Rx[Emis_Size];		//���ջ�����

extern void power_add(uint8_t *data); //����΢��+65����
extern void power_sub(uint8_t *data);  //����΢��-65����
extern void Range_Power(GT2000_t *Value); /* ����ת��Ϊϵͳ����ֵ */
extern void Power_Range(GT2000_t *Vlaue,uint8_t *Data);		//�����̲����������ֵת������

extern int GT2000_Wait_Ack(uint8_t cmd);								//�����̲������Ӧ������
extern int GT2000_Cmd_Analyze(uint8_t *msg, uint8_t cmd);//�����̲���������ݽ�������
extern uint8_t Bj_BCC_Check_Sum(uint8_t *pdata, uint8_t len);//�����̲������ͨѶ�����㷨����

#endif
