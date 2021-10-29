#ifndef	_MAIN_H_
#define	_MAIN_H_

#include <stdint.h>

#pragma		pack(1)

/*-----------------------------------��/�ɻ���Э��------------------------------------------*/
#define		NEW_PROCOTOL			1			//��Э��

/*-----------------------------------�����汾����ѡ��------------------------------------------*/
#define		Device_Version		0xB1		//�����Ǳ����̲������,A1������������

/*---------------------------------------���Ź�---------------------------------------------*/
#define		IWDOG_EN				1			//ʹ�ܿ��Ź�	��Task_TH()��ι��

/*-------------------------------------��ӡ������Ϣ-----------------------------------------*/
#define		PRINTF_EN				0			//�Ƿ��ӡ������Ϣ

/*-------------------------------------��ӡ������Ϣ-----------------------------------------*/
#define		printf_EN				1			//�Ƿ��ӡ2KW������Ϣ

/*----------------------------------------W5500-----------------------------------------------*/
#define		STM32_W5500_EN			1			//ʹ������W5500

/*----------------------------------Firmware_Updare-----------------------------------------*/
#define		FIRMWARE_UPDATE_EN		1			//ʹ�ܹ̼�����

/*----------------------------------------TH------------------------------------------------*/
#define		NEW_TH_EN				1			//�Ƿ�̽ͷʽ��ʪ�ȱ�����
#define   New_Th          1
/*--------------------------------�豸����Ӧ�Զ��ָ�----------------------------------------*/
#define		NO_RESPOND_RECOVER		1			//1:�Զ��ָ�

/*--------------------------------�豸����Ӧ������----------------------------------------*/
#define		NO_RESPOND_MAX			3

/*------------------------------------��������----------------------------------------------*/
#define		PROTECT_ADJUST			0.95

/*------------------------------------AD�ɼ�����--------------------------------------------*/
#define		SWR_ALARM_THRESHOLD 24		//פ���ȱ����ٽ�ֵ,����10�����ڱȽ�
#define 	Debug_M_45I				(41.0)		//����	(37.5)
#define 	Debug_M_45V				(10.0)		//��ѹ	(45/4.76)
#define		AD_SAMPLE_NUM			50			//AD�ɼ�����(��ƽ��ֵ)	�����������ػ�����Ƿѹ��������100Ϊ10

/*------------------------------------Ӳ���ӹܿ��ػ�ʱ��--------------------------------------------*/
#define		BEFOREHAND_OPEN_TIME	5			//Ӳ���ӹܣ���ǰ����ʱ��				(��λ:����)
#define 	Hardware_Time 12		//Ӳ���ӹ�ʱ��ǰ��NTPѯ�ʵ�ʱ�䣬1��5S��12��1����
/*------------------------------------ɨƵ����ʱ���ɨƵ����--------------------------------------------*/
#define		SCAN_BREAKPOINT_ADDRESS		0x08039000	//ɨƵ�ϵ��ַ������һ��ɨƵƵ��ĵ�ַ
#define		SCAN_BREAKPOINT_LEN			0x02		//ɨƵ�ϵ�����ݳ���

#define		SCAN_ALARM_SWR_THRESHOLD	4		//ɨƵʱ�����ر�����פ����Ϊ4

#define		POWER_MAX				1200		//���������
#define   POWER_CHANGE_MAX 350
/*--------------------------------ɨƵ������Чʱ�䣬������Чʱ�䣬������ɨƵ-----------------------------*/
#define		FREQ_BAND_UPDATE_TIME	(7*24*3600)	//С��Ƶ�θ���ʱ�䣬��ɨƵʱֱ�ӷ�����ɨƵ������		��λ:s



/* ϵͳ״̬ */
#define		SYSTEM_EMISSING			0x01		//����״̬
#define		SYSTEM_STANDBY			0x02		//����״̬	�ѿ���������û��ִ���κβ���
#define		SYSTEM_FREQUENCY		0x03		//��Ƶ״̬	��г״̬
#define		SYSTEM_SHUTDOWN			0x04		//�ػ�״̬
#define		SYSTEM_SCAN				0x05		//ɨƵ״̬
#define		SYSTEM_OPENING			0x06		//���ڿ���

#define		SYSTEM_UNINITIALIZE		0x10		//δ��ʼ��
#define		SYSTEM_ALARM			0x11		//����״̬
#define		SYSTEM_HARDWARE			0x12		//Ӳ���ӹ�	PCռ���ˣ���ʹ��

/* �¾�Э�鹦�ʵȼ� */
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
	/* ϵͳ��Ϣ */
	uint8_t CAN_ID[1];				//�����ID	
	uint8_t Model[1];						//���������
	/* ��־λ */
	uint8_t already_init;			//��ʼ����ɱ�־
	uint8_t already_swept;			//ɨƵ��ɱ�־		//�ϵ�ɨ��֮�󣬸ñ�־λ�Ѿ�������
	uint8_t time[6];				//����ʱ��(��PC�õ����ݣ����µ�ϵͳ)
	uint8_t time_update_flag;		//����ʱ���־λ
	
	/* ϵͳ״̬ */
	uint8_t status;					//0x01����״̬	0x02����״̬	0x03��Ƶ״̬	0x04�ػ�	0x05ɨƵ	0x10δ��ʼ��	0x11����״̬
	uint8_t open;					//������־λ	0:�ػ�	1:���ڿ���	2:�Ѿ�����
	uint8_t close;					//�ػ���־λ	1:���ڹػ������ܼ��AD�������͵�ѹ�������ػ�֮��������
	
	uint8_t sweep;					//ɨƵ��־λ	1:����ɨƵ��2:ɨƵ��ɣ�����status��־λ������sweep����		��󣬱�����flash
	uint8_t cancel_sweeping;		//ȡ��ɨƵ		1:ȡ��ɨƵ
	uint8_t	Scan_Freq_flag;			//ɨƵƵ��
	
	uint8_t achieve_add_sub;			//����/��С������ɱ�־λ	1:��ɹ��ʵ���
	uint8_t modify_power;			//1:������������
	uint8_t cancel_add_sub;			//1:ȡ����������
	uint8_t protect_adjust;			//1:���ڽ��б�������		0:����Ҫ���ڻ���ڽ���(��ɻ�ʧ��)		(����/פ���� > 80%������ֵ)
	
	uint8_t stop;					//����ֹͣ��־λ	1:����ֹͣ(�����������ʣ������Ѿ����˹���ʱ�䣬��Ҫ����ֹͣ)	֪ͨTask_MT2000(),BJ2000_Stop();MT2000_Cmd_Stop();����λ
	
	/* ��Э��--MT2000_Rx������Ϣ�����֮����µ�ϵͳ״̬ */
	uint8_t emission;				//����/ֹͣ				(��Э��)'T'/'P'		(��Э��)0x01����/0x00ֹͣ
	uint8_t mode;					//��Ƶ��˫Ƶ����Ƶ		(��Э��)'1','2','3'	(��Э��)0x01,0x02,0x03
	uint8_t INQUIRE_Mode;  //�ӻ����ϲ�ѯ����ģʽ
	uint8_t method;					//������ʽ��FM			(��Э��)'F'			(��Э��)0x03
	uint8_t channel;				//�ŵ�					(��Э��)'1'			(��Э��)0x01,0x02,0x03	(��Э��3->��Э��1��������Э��̶�Ϊ'1'����ֻ��������ر���)
	uint8_t power[3];				//���ʵȼ�				(��Э��)'1':1/4		'2':1/2		'3':1	(��Э��)0x00:ȫ����	0x01:1/2����	0x02:1/4����
									//��Э�����������ʵȼ�����Э�鹲��һ�����ʵȼ�(����update_system_power_level()������ѡ����ԣ����ع��ʵȼ�)
									//(��Э��3->��Э��ʵ����Ϊ1��������Э����Ҫ0602���ظ�PC�����������������ù���ѡ����Ժ���������ֵΪ��Ҫ����Ĺ���)
								
	//��PC�յ�����ָ�ֱ�Ӹ�ֵ��MT2000_Tx��
	//���ߴ�flash��ȡ֮��ͨ��set_run_diagram_power_level()��ֵ��Run_Diagram_data����Task_Hardware_Monitor()�ж��Ƿ���Ҫ���䣬���ֵ��MT2000_Tx��Ȼ����
	uint8_t freq1[10];				//��ƵƵ��		����ɹ���MT2000_Rx => System
	uint8_t freq2[10];				//˫ƵƵ��		����ɹ���MT2000_Rx => System
	uint8_t freq3[10];				//��ƵƵ��		����ɹ���MT2000_Rx => System
	
	uint8_t forward[6];				//������		//��MT2000_Rx�������ݺ󣬻���µ�0602�����ظ�PC
	uint8_t reverse[6];				//������		//��MT2000_Rx�������ݺ󣬻���µ�0602�����ظ�PC
	uint8_t swr[4];					  //פ����		//��MT2000_Rx�������ݺ󣬻���µ�0602�����ظ�PC
	
	uint8_t Control_Model;			//������������ģʽ���Զ�/�ֶ�
	uint8_t Reverse_State;			//���������״̬
	uint8_t Absorber_State_1;			//�˲���״̬����ǰʹ�õ��˲���
	uint8_t Absorber_State_2;			//�˲���״̬
	uint16_t Amplifier_Temperature[4];		//�����¶ȼ�⣬ÿ��������һ����һ���ĸ�
	uint8_t	Launch_Switch_state;
	uint8_t	Attenuation;					//˥��ֵ
	uint8_t Frequency_band_value;	//Ƶ��ֵ
	uint8_t fbv_c;								//Ƶ��ֵ�˶Ա���
	/* ��Э�� */
	float ADC_Sum_F;				//ADC�����������ʺ�	/* ��Э��	����������(��ѹ) */
	float ADC_Sum_R;				//ADC�����ķ����ʺ�	/* ��Э��	����������(��ѹ) */
	float ADC_Sum_V;				//ADC�����ĵ�ѹ��		/* ��Э��	����������(��ѹ) */		//��ѹ����ת����ʽ���	update_System_Voltage_Electricity();
	float ADC_Sum_A;				//ADC�����ĵ�����		/* ��Э��	����������(��ѹ) */		(����)ֹͣ���� BJ2000_Stop(); ɨƵ BJ2000_Stop(); ADC�����Ч(��������)
	
	float Forward_Power;			//������				/* ��Э��	����������(��ѹ) */
	float Reverse_Power;			//������				/* ��Э��	����������(��ѹ) */
	float Voltage;					//45V�µĵ�ѹ	��λ:V		//��ѹ�������㣬����ʾʱ��ϵͳ״̬ -- �ػ������ڿ��������ڵ�Ƶ������ɨƵ
	float Electricity;				//45V�µĵ���	��λ:A		//��ѹ������ʾ��ϵͳ״̬ -- ����������(����)�����ʵ��ڡ�����
	float Standing_wave_ratio;		//פ����		(��Э����ַ�����swr[4]��ת���ɸ�����Standing_wave_ratio�����ڱȽ�)
	
	uint16_t Bj_Electricity_1[4];		//�洢���������Ĺ���һ�ĵ���
	uint16_t Bj_Electricity_2[4];		//�洢���������Ĺ��Ŷ��ĵ���
	uint16_t Bj_Electricity_3[4];		//�洢���������Ĺ������ĵ���
	uint16_t Bj_Electricity_4[4];		//�洢���������Ĺ����ĵĵ���
	
	uint8_t Ip[4];
	uint8_t Port[2];
	uint8_t Init_Mark;		//��ʼ�����
	uint8_t Open_Close;		//0����������ػ���1��������
	uint8_t Power_Adjustment;			//��������ִ�б��
}Sys_Status_t;

typedef struct
{
    volatile uint8_t start[10][10];
    volatile uint8_t end[10][10];
	
	volatile uint8_t usage_diagram_count;	//��¼�ǵڼ��β�ѯ����������ͼ
	
	volatile uint8_t no_respond_count;		//û�н���PC�������
    volatile uint8_t hard_control;			//Ӳ���ӹ�
	
    volatile uint8_t need_open;				//��Ҫ����
	volatile uint8_t need_close;			//��Ҫ�ػ�
	volatile uint8_t need_emit;				//��ѯ����ͼʱ��ģʽ��Ƶ�ʻ��ʵȼ���һ�£���Ҫ����
}Monitor_t;

typedef struct Temperature_humidity
{
    float Temperature;//�¶�
    float Humidity;//ʪ��
}Temperature_Humidity_t;

typedef struct
{	
    volatile uint8_t alarm_history;			//��ʷ������¼
	
	volatile uint8_t emission;				//��������������
	volatile uint8_t emission_time_flag;
	
    volatile uint8_t no_respond;			//������3�β�������Ӧ
	volatile uint8_t no_respond_time_flag;
	
	volatile uint8_t no_respond_count;		//����������Ӧ������
    volatile uint8_t no_respond_locate;		//���ڶ�λ����Ӧ�ĵط�
	
    volatile uint8_t temperature_alarm;		//�¶ȱ���	��ʪ�ȴ�����
	volatile uint8_t temperature_alarm_time_flag;
    volatile uint8_t humidity_alarm;		//ʪ�ȱ���
	volatile uint8_t humidity_alarm_time_flag;
	
    volatile uint8_t over_Electric;			//��������	AD
	volatile uint8_t over_Electric_time_flag;
    volatile uint8_t over_Voltage;			//��ѹ����
	volatile uint8_t over_Voltage_time_flag;
    volatile uint8_t low_Voltage;			//Ƿѹ����
	volatile uint8_t low_Voltage_time_flag;
	
	volatile uint8_t swr_alarm;				//פ���ȱ���	AD���߼��������ص�����
	volatile uint8_t swr_alarm_time_flag;
    volatile uint8_t no_power;				//�޹������
	volatile uint8_t no_power_time_flag;
	volatile uint8_t power_cataclysm;		//�������  ��300W
}Alarm_t;	//��ǰ�б���������λ��־λ������¼������ʱ��ͱ���ֵ��Alarm_historyBack_t������ѯӦ���

typedef struct
{
	uint8_t mode;				//��Ƶ��˫Ƶ����Ƶ		(��Э��)'1','2','3'	(��Э��)0x01,0x02,0x03
//	uint8_t Channel[3];			//��Ҫ�����ŵ�			��Э��̶�'1'����ʵ��Э��Ҳ����ȡ��(���Ը���ģʽ��������ӦƵ�ʣ���Ȼ����123�ŵ���)
	uint8_t Freq[12];			//��Ҫ�����Ƶ��		hex����ǰ��ת��Ϊascii
	uint8_t power[3];			//��Ҫ�����Ĺ��ʵȼ�	ֱ�Ӵ���Э���'1','2','3'
}Run_Diagram_data_t;			//��ȡ��������ͼ����

typedef struct
{
    uint8_t Start_Time1[2];		//��ʼʱ��
    uint8_t End_Time1[2];		//����ʱ��
    uint8_t Power1[1];			//����
    uint8_t Frq1[4];			//Ƶ��
	uint8_t channel1[1];		//�ŵ�			//�°汾���ã����Ǳ���

    uint8_t Start_Time2[2];		//��ʼʱ��
    uint8_t End_Time2[2];		//����ʱ��
    uint8_t Power2[1];			//����
    uint8_t Frq2[4];			//Ƶ��
	uint8_t channel2[1];		//�ŵ�

    uint8_t Start_Time3[2];		//��ʼʱ��
    uint8_t End_Time3[2];		//����ʱ��
    uint8_t Power3[1];			//����
    uint8_t Frq3[4];			//Ƶ��
	uint8_t channel3[1];		//�ŵ�

    uint8_t Start_Time4[2];		//��ʼʱ��
    uint8_t End_Time4[2];		//����ʱ��
    uint8_t Power4[1];			//����
    uint8_t Frq4[4];			//Ƶ��
	uint8_t channel4[1];		//�ŵ�

    uint8_t Start_Time5[2];		//��ʼʱ��
    uint8_t End_Time5[2];		//����ʱ��
    uint8_t Power5[1];			//����
    uint8_t Frq5[4];			//Ƶ��
	uint8_t channel5[1];		//�ŵ�

    uint8_t Start_Time6[2];		//��ʼʱ��
    uint8_t End_Time6[2];		//����ʱ��
    uint8_t Power6[1];			//����
    uint8_t Frq6[4];			//Ƶ��
	uint8_t channel6[1];		//�ŵ�

    uint8_t Start_Time7[2];		//��ʼʱ��
    uint8_t End_Time7[2];		//����ʱ��
    uint8_t Power7[1];			//����
    uint8_t Frq7[4];			//Ƶ��
	uint8_t channel7[1];		//�ŵ�

    uint8_t Start_Time8[2];		//��ʼʱ��
    uint8_t End_Time8[2];		//����ʱ��
    uint8_t Power8[1];			//����
    uint8_t Frq8[4];			//Ƶ��
	uint8_t channel8[1];		//�ŵ�

    uint8_t Start_Time9[2];		//��ʼʱ��
    uint8_t End_Time9[2];		//����ʱ��
    uint8_t Power9[1];			//����
    uint8_t Frq9[4];			//Ƶ��
	uint8_t channel9[1];		//�ŵ�

    uint8_t Start_Time10[2];	//��ʼʱ��
    uint8_t End_Time10[2];		//����ʱ��
    uint8_t Power10[1];			//����
    uint8_t Frq10[4];			//Ƶ��
	uint8_t channel10[1];		//�ŵ�
}Run_Diagram_buf_t;				//����ͼ����(����100��)
/*-----------------------------------------���������---------------------------------------------------*/
typedef struct
{
    float 	Low_temp_limit[1];		//�¶�����
    float 	Upp_temp_limit[1];		//�¶�����
    float 	Low_humidity_limit[1];	//ʪ������
    float 	Upp_humidity_limit[1];	//ʪ������
    float 	Low_45I_limit[1];		//��������
    float 	Upp_45I_limit[1];		//��������
    float 	Low_45V_limit[1];		//��ѹ����
    float 	Upp_45V_limit[1];		//��ѹ����
	uint8_t timer[6];				//����ʱ��
    uint8_t Transmitte_id[1];		//�����ID
}Alarm_threshold_t;					//PC�·�������ֵ�Ľṹ��	0101

typedef struct
{
	uint8_t mode;		//1��Ƶ��2˫Ƶ��3��Ƶ
	uint8_t method;		//������ʽ��FM
	uint8_t power[3];	//���ʵȼ� 		0��ȫ����	1��1/2����	2��1/4����
	uint8_t channel[3];	//�ŵ�
	
	uint8_t freq[12];	//Ƶ��
	uint8_t timer[6];	//����ʱ��
}PC_Cmd_t;				//PC����������Ĳ���		0301

typedef struct
{
    uint8_t Transmitte_id[1];	//�����ID
    uint8_t Disalarm_type[2];	//�����������
    uint8_t Berdou_time[6];		//����ʱ��
}Disalarm_t;					//PC�������״̬	0501

typedef struct
{
    uint8_t	Transmitte_id[1];	//�����ID
    uint8_t	state[1];			//״̬
    float	Fre_Band[2];		//Ƶ��
}Scan_Frq_t;					//PCɨ����ѹ���Ƶ��0901

typedef struct
{
    uint8_t Transmitte_id[1];	//�����ID
    uint8_t state[1];			//״̬
}Sacn_stop_t;					//PCֹͣɨ��		0A01

typedef struct
{
    uint8_t Transmitte_id[1];	//�����ID
    uint8_t Power_UP[2];		//��Ҫ���ӵĹ���
}Add_Power_t;					//����΢������		0B01

typedef struct
{
    uint8_t Transmitte_id[1];	//�����ID
    uint8_t Power_DOWN[2];		//��Ҫ���ٵĹ���
}Sub_Power_t;					//����΢������		0C01

/* ����ͼ */
typedef struct
{
	uint8_t Transmitte_id[1];	//�����ID
    uint8_t count[1];			//�м�������ͼ
    uint8_t Continue[2];		//�Ƿ�������ͼ
	
    uint8_t Start_Time1[2];		//��ʼʱ��
    uint8_t End_Time1[2];		//����ʱ��
    uint8_t Power1[1];			//����
    uint8_t Frq1[4];			//Ƶ��
	uint8_t channel1[1];		//�ŵ�			//�°汾���ã����Ǳ���

    uint8_t Start_Time2[2];		//��ʼʱ��
    uint8_t End_Time2[2];		//����ʱ��
    uint8_t Power2[1];			//����
    uint8_t Frq2[4];			//Ƶ��
	uint8_t channel2[1];		//�ŵ�

    uint8_t Start_Time3[2];		//��ʼʱ��
    uint8_t End_Time3[2];		//����ʱ��
    uint8_t Power3[1];			//����
    uint8_t Frq3[4];			//Ƶ��
	uint8_t channel3[1];		//�ŵ�

    uint8_t Start_Time4[2];		//��ʼʱ��
    uint8_t End_Time4[2];		//����ʱ��
    uint8_t Power4[1];			//����
    uint8_t Frq4[4];			//Ƶ��
	uint8_t channel4[1];		//�ŵ�

    uint8_t Start_Time5[2];		//��ʼʱ��
    uint8_t End_Time5[2];		//����ʱ��
    uint8_t Power5[1];			//����
    uint8_t Frq5[4];			//Ƶ��
	uint8_t channel5[1];		//�ŵ�

    uint8_t Start_Time6[2];		//��ʼʱ��
    uint8_t End_Time6[2];		//����ʱ��
    uint8_t Power6[1];			//����
    uint8_t Frq6[4];			//Ƶ��
	uint8_t channel6[1];		//�ŵ�

    uint8_t Start_Time7[2];		//��ʼʱ��
    uint8_t End_Time7[2];		//����ʱ��
    uint8_t Power7[1];			//����
    uint8_t Frq7[4];			//Ƶ��
	uint8_t channel7[1];		//�ŵ�

    uint8_t Start_Time8[2];		//��ʼʱ��
    uint8_t End_Time8[2];		//����ʱ��
    uint8_t Power8[1];			//����
    uint8_t Frq8[4];			//Ƶ��
	uint8_t channel8[1];		//�ŵ�

    uint8_t Start_Time9[2];		//��ʼʱ��
    uint8_t End_Time9[2];		//����ʱ��
    uint8_t Power9[1];			//����
    uint8_t Frq9[4];			//Ƶ��
	uint8_t channel9[1];		//�ŵ�

    uint8_t Start_Time10[2];	//��ʼʱ��
    uint8_t End_Time10[2];		//����ʱ��
    uint8_t Power10[1];			//����
    uint8_t Frq10[4];			//Ƶ��
	uint8_t channel10[1];		//�ŵ�
}Run_Diagram_t;					//0D01

/*----------------------------------------------------------------------*/
typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t	Alarm_paramet[2];		//��������
}Alarm_backPC_t;					//����������ʼ��Ӧ���		0102

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t Trans_current_state[1];	//�������ǰ��״̬
    uint8_t Mode[1];				//��ǰ������ģʽ	//��Ƶ��˫Ƶ����Ƶ
    uint8_t Channel[3];				//�ŵ�
    uint8_t Freq[12];				//��ǰ������Ƶ��
    uint8_t Power_grade[3];			//��ǰ�����Ĺ��ʵȼ�
    float Forward_power[1];			//�����������
    float Reverse_power[1];			//�����������
    float Power_45_intensity[1];	//�����45V�������ȡ��
    float Power_45_voltage[1];		//�����45V�����ѹȡ��
    float temp[1];					//�����¶�
    float humidity[1];				//ʪ��
    uint8_t emission[1];			//bit0-- 0 û�з��� 1���ڷ��䣬bit1--0 �ػ� 1����
		uint8_t current_alarm_state[2];	//��ǰ����״̬
		
		/*V2�汾*/
		uint8_t	Alarm_Reservation[2];		//����Ԥ��
		uint8_t	System_Version[20];			//ϵͳ�汾����ϢԤ��,����-4Ϊ���������
		uint8_t Divider[1];								//�ָ��ߣ���ֵΪ0xFF�����ڹۿ�Э��
		uint8_t Control_Model[1];			//������������ģʽ���Զ�/�ֶ�
		uint8_t Frequency_band_value[1];	//Ƶ��ֵ
		uint8_t	Attenuation[1];					//˥��ֵ
		uint8_t	Reservation[2];					//Ԥ������byte
		uint16_t Bj_Electricity_1[4];		//�洢���������Ĺ���һ�ĵ���
		uint16_t Amplifier_Temperature_1[1];	//����һ�¶�
		uint16_t Bj_Electricity_2[4];		//�洢���������Ĺ��Ŷ��ĵ���
		uint16_t Amplifier_Temperature_2[1];	//���Ŷ��¶�
		uint16_t Bj_Electricity_3[4];		//�洢���������Ĺ������ĵ���
		uint16_t Amplifier_Temperature_3[1];	//�������¶�
		uint16_t Bj_Electricity_4[4];		//�洢���������Ĺ����ĵĵ���
		uint16_t Amplifier_Temperature_4[1];	//�������¶�	
}Work_paraBack_t;					//��ѯӦ���				0202

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t Trans_state[2];			//���������PC��ʱ��״̬
}Trans_openBack_t;					//����Ӧ���				0302

typedef struct
{
	uint8_t Transmitte_id[1];		//�����ID
	uint8_t Trans_state[1];			//���ط����ִ�йرյĽ��
}Trans_stopBack_t;					//ֹͣ����Ӧ���			0402

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t Disalarm_result[2];		//�������״̬���
    uint8_t current_alarm_state[2];	//��ǰ����״̬
}DisalarmBack_t;					//�������״̬��Ӧ���		0502

typedef struct
{
	uint8_t Transmitte_id[1];		//�����ID
    uint8_t alarm_history[2];		//��ʷ���ڵı�������

    uint8_t no_power_time[6];		//�޹��������ʱ��
	
    float 	temp_history[1];		//�����¶�ʱ���ֵ
    uint8_t temp_time[6];			//�¶ȱ�����ʱ��
	
    float	humidity_history[1];	//ʪ�ȱ���ʱ���ֵ
    uint8_t humidity_time[6];		//ʪ�ȱ�����ʱ��
	
    float 	power_45A_history[1];	//�����45V�������ʱ���ֵ(����)
    uint8_t power_45A_time[6];		//�����45V�����������ʱ��
	
    float	power_45V_history[1];	//�����45V�����ѹ����ʱ���ֵ(��ѹǷѹ����)
    uint8_t power_45V_time[6];		//�����45V�����ѹ������ʱ��
	
    uint8_t no_response_time[6];	//���������Ӧʱ��
	
    uint8_t	emission_time[6];		//�������������ʱ��
	
    float 	swr_alarm[1];			//פ����
    uint8_t swr_time[6];			//פ���ȱ���ʱ��
	
		uint8_t	emission_alarm_content[40];	//������������������
		
		uint8_t Alarm_reservation[2];				//��������λ
		uint8_t Alarm_Switch_Power[1];			//���ص�Դ����
		uint8_t Alarm_Abnormal_Output[1];		//����쳣����
		uint8_t Alarm_Module_Electricity[1];//ģ���������
		uint8_t Alarm_Module_Power[1];			//����״̬����
		uint8_t Alarm_Module_Tem[1];				//ģ���¶ȱ���
		uint8_t Alarm_Absorber_State_1[1];	//�˲�������
		uint8_t Alarm_Reverse[1];						//���䱨��
		uint8_t Alarm_Re[3];								//��������λ
	
	
}Alarm_historyBack_t;				//������ѯӦ���			0602

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��
}Power_onBack_t;					//����Ӧ���				0702

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��
}Power_offBack_t;					//�ػ�Ӧ���				0802

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��
    float	Fre_Band[2];			//Ƶ��
    float	swr[20];				//פ����
    uint8_t Power_UP[20];			//������޹���
}Scan_FrqBack_t;					//ɨƵӦ���				0902

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t result[1];				//״̬
}Sacn_stopBack_t;					//ֹͣɨƵ					0A02

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��

}Add_PowerBack_t;					//����΢������Ӧ���		0B02

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��
}Sub_PowerBack_t;					//����΢������Ӧ���		0C02

typedef struct
{
    uint8_t Transmitte_id[1];		//�����ID
    uint8_t results[1];				//ִ�н��
}Run_DiagramBack_t;					//����ͼӦ���				0D02

/*----------------------------------------------------------------------*/
extern uint8_t g_fuc_cod[2];
extern uint8_t g_fuc_codToPC[2];

extern uint32_t				g_inquire_stamp;
extern uint32_t				g_stamp_distance;

extern Sys_Status_t 		System;				//ϵͳ	
extern Monitor_t			Monitor;			//Ӳ�����
extern Run_Diagram_data_t	Run_Diagram_data;	//��ȡ��������ͼ����
extern Alarm_t				Alarm;				//����
extern Temperature_Humidity_t Temperature_Humidity;		//��ʪ��

extern Alarm_threshold_t	Alarm_threshold;	//������ֵ				0101
extern PC_Cmd_t				PC_Cmd;				//PCָ��				0301
extern Disalarm_t			Disalarm;			//PC�������			0501
extern Scan_Frq_t			Scan_Frq;			//ɨ����ѹ���Ƶ��		0901
extern Sacn_stop_t			Sacn_stop;			//ֹͣɨƵ				0A01
extern Add_Power_t			Add_Power;			//��������				0B01
extern Sub_Power_t			Sub_Power;			//���ʼ�С				0C01
extern Run_Diagram_t		Run_Diagram;		//����ͼ				0D01
extern Run_Diagram_buf_t	Run_Diagram_buf[10];//����ͼ����(100��)

extern Alarm_backPC_t		Alarm_backPC;		//����������ʼ��Ӧ���	0102
extern Work_paraBack_t		Work_paraBack;		//��ѯӦ���			0202
extern Trans_openBack_t		Trans_openBack;		//����Ӧ���			0302
extern Trans_stopBack_t		Trans_stopBack;		//ֹͣ����Ӧ���		0402
extern DisalarmBack_t		DisalarmBack;		//�������״̬Ӧ���	0502
extern Alarm_historyBack_t	Alarm_historyBack;	//��ʷ����Ӧ���		0602
extern Power_onBack_t		Power_onBack;		//������ϵ�Ӧ���		0702
extern Power_offBack_t		Power_offBack;		//�ػ�Ӧ���			0802
extern Scan_FrqBack_t		Scan_FrqBack;		//ɨƵӦ���			0902
extern Sacn_stopBack_t		Sacn_stopBack;		//ֹͣɨƵ				0A02
extern Add_PowerBack_t		Add_PowerBack;		//����΢������Ӧ���	0B02
extern Sub_PowerBack_t		Sub_PowerBack;		//����΢������Ӧ���	0C02
extern Run_DiagramBack_t	Run_DiagramBack;	//����ͼӦ���			0D02



extern Scan_FrqBack_t		Scan_FrqBack1;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//3.2-4.9
extern Scan_FrqBack_t		Scan_FrqBack2;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//5.0-6.9
extern Scan_FrqBack_t		Scan_FrqBack3;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//7.0-8.9
extern Scan_FrqBack_t		Scan_FrqBack4;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//9.0-10.9
extern Scan_FrqBack_t		Scan_FrqBack5;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//11.0-12.9
extern Scan_FrqBack_t		Scan_FrqBack6;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//13.0-14.9
extern Scan_FrqBack_t		Scan_FrqBack7;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//15.0-16.9
extern Scan_FrqBack_t		Scan_FrqBack8;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//17.0-18.9
extern Scan_FrqBack_t		Scan_FrqBack9;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//19.0-20.9
extern Scan_FrqBack_t		Scan_FrqBack10;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//21.0-22.9
extern Scan_FrqBack_t		Scan_FrqBack11;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//23.0-24.9
extern Scan_FrqBack_t		Scan_FrqBack12;		//PCɨ����ѹ���Ƶ��	(��ѯ)0902	//25.0-26.1

extern uint8_t adc1_zero_count;				//�����ʺ����׵���0
extern uint8_t adc3_zero_count;				//����
extern float ADC0_buff[AD_SAMPLE_NUM];
extern float ADC1_buff[AD_SAMPLE_NUM];
extern float ADC2_buff[AD_SAMPLE_NUM];
extern float ADC3_buff[AD_SAMPLE_NUM];

extern float SWR_array[256];				//פ���Ȼ���
extern float Forward_Power_array[256];		//������
extern float Reverse_Power_array[256];		//������

extern uint32_t	freq_band_time_stamp[10];	//�洢��Ƶ��ʱ���

/*--------------------------------------------------------------------*/
extern void System_Status_Clean(void);
extern void System_Status_Update(void);

#endif