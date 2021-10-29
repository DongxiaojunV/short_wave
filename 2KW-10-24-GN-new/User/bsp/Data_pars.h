#ifndef __DATA_PARS_H__
#define __DATA_PARS_H__
#include <stdint.h>		/* uint8_t ... */

//#define Only_Fixed_Frequency//ֻ�ܹ����ڹ�Ƶ״̬�궨��


//#pragma pack(1)

/*----------------------------------------------------------------------------------------*/
//typedef struct
//{
//    float 	Low_temp_limit[1];//�¶�����
//    float 	Upp_temp_limit[1];//�¶�����
//    float 	Low_humidity_limit[1];//ʪ������
//    float 	Upp_humidity_limit[1];//ʪ������
//    float 	Low_45I_limit[1];//��������
//    float 	Upp_45I_limit[1];//��������
//    float 	Low_45V_limit[1];//��ѹ����
//    float 	Upp_45V_limit[1];//��ѹ����
//    uint8_t timer[6];//����ʱ��
//    uint8_t Transmitte_id[1];//�����ID
//} Alarm_threshold_t; //PC�·�������ֵ�Ľṹ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t	Alarm_paramet[2];//��������
//} Alarm_backPC_t; //PC���ñ����ķ��ذ�(�ɷ��������PC)

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Retain[2];//����Ǳ���λ
//} Working_paramet_t; //PC��ѯ���������״̬�Լ������Ĳ�����ָ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Trans_current_state[1];//�������ǰ��״̬
//    uint8_t Type[1];//��ǰ����������						//��Ƶ��˫Ƶ����Ƶ
//    uint8_t Channel[3];//�ŵ�
//    uint8_t Freq[12];//��ǰ������Ƶ��
//    uint8_t Power_grade[3];//��ǰ�����Ĺ��ʵȼ�
//    float Forward_power[1];//�����������
//    float Reverse_power[1];//�����������
//    float Power_45_intensity[1];//�����45V�������ȡ��
//    float Power_45_voltage[1];//�����45V�����ѹȡ��
//    float temp[1];//�����¶�
//    float humidity[1];//ʪ��
//    uint8_t Hist_alarm[1];//�鿴������ʷ��������
//} Work_paraBack_t; //����PC������������������ݰ�

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Type[1];//��Ҫ����������
//    uint8_t Channel[3];//��Ҫ�����ŵ�
//    uint8_t Freq[12];//��Ҫ�����Ƶ��
//    uint8_t Power_grade[3];//��Ҫ�����Ĺ��ʵȼ�
//    uint8_t Berdou_time[6];//����ʱ��
//} Trans_open_t; //���������

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Trans_state[2];//���������PC��ʱ��״̬
//} Trans_openBack_t; //�����������PC���ص����ݰ�

//typedef struct
//{
//    uint8_t Type[1];//��Ҫ����������
//    uint8_t Channel[3];//��Ҫ�����ŵ�
//    uint8_t Freq[12];//��Ҫ�����Ƶ��
//    uint8_t Power_grade[3];//��Ҫ�����Ĺ��ʵȼ�
//} Trans_open_Copy_t; //�������������

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Berdou_time[6];//����ʱ��
//} Trans_stop_t; //ֹͣ�����

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Trans_state[1];//���ط����ִ�йرյĽ��
//} Trans_stopBack_t; //ֹͣ�������PC���ص����ݰ�

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Disalarm_type[2];//�����������
//    uint8_t Berdou_time[6];//����ʱ��
//} Disalarm_t; //PC�������״̬

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Disalarm_result[2];//�������״̬���
//    uint8_t current_alarm_state[2];//��ǰ����״̬
//} DisalarmBack_t; //�������״̬��Ӧ���

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Berdou_time[6];//����ʱ��
//} Alarm_history_t; //PC��ѯ��ʷ����


//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t alarm_history[2];//��ʷ���ڵı�������
//    uint8_t Power_free_time[6];//�޹��������ʱ��
//    float 	Temp_history[1];//�����¶�ʱ���ֵ
//    uint8_t Temp_time[6];//�¶ȱ�����ʱ��
//    float		humidity_history[1];//ʪ�ȱ���ʱ���ֵ
//    uint8_t Humidity_time[6];//ʪ�ȱ�����ʱ��
//    float 	Power_45A_history[1];//�����45V�������ʱ���ֵ��������
//    uint8_t Power_45A_time[6];//�����45V�����������ʱ��
//    float		Power_45V_history[1];//�����45V�����ѹ����ʱ���ֵ
//    uint8_t Power_45V_time[6];//�����45V�����ѹ������ʱ��
//    uint8_t No_response_time[6];//���������Ӧʱ��
//    uint8_t	Exciter_alarm[6];//���������ʱ��
//    float 	Bobbi[1];//פ����
//    uint8_t Bobbi_alarm[6];//פ���ȱ���ʱ��
//} Alarm_historyBack_t;

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Retain[1];//����
//    uint8_t Berdou_time[6];//����ʱ��
//} Power_on_t; //PC�������������Դ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��
//} Power_onBack_t; //����PC�������������Դ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Retain[1];//����
//} Power_off_t; //PC�رյ�Դ

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��
//} Power_offBack_t; //����PC�رյ�Դ

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t state[1];//״̬
//    float		Fre_Band[2];//Ƶ��
//} Scan_Frq_t; //PCɨ����ѹ���Ƶ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��
//    float		Fre_Band[2];//Ƶ��
//    float		bobibi[20];//פ����
//    uint8_t Power_UP[20];//������߹���
//} Scan_FrqBack_t; //PCɨ����ѹ���Ƶ�ʷ���

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t state[1];//״̬
//} Sacn_stop_t; //PCֹͣɨ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t result[1];//״̬
//} Sacn_stopBack_t; //����PCֹͣɨ��

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Power_UP[2];//��Ҫ���ӵĹ���

//} Add_Power_t; //����΢������
//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t Power_DOWN[2];//��Ҫ���ٵĹ���

//} Sub_Power_t; //����΢������


//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��

//} Add_PowerBack_t; //����΢�����ӻذ�
//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��

//} Sub_PowerBack_t; //����΢�����ٻذ�


//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t count[1];//�м�������ͼ
//    uint8_t Continue[2];//�Ƿ�������ͼ

//    uint8_t Start_Time1[2];//��ʼʱ��
//    uint8_t End_Timep1[2];//����ʱ��
//    uint8_t Power1[1];//����
//    uint8_t Frq1[4];//Ƶ��
//    uint8_t channel1[1];//�ŵ�

//    uint8_t Start_Time2[2];//��ʼʱ��
//    uint8_t End_Timep2[2];//����ʱ��
//    uint8_t Power2[1];//����
//    uint8_t Frq2[4];//Ƶ��
//    uint8_t channel2[1];//�ŵ�

//    uint8_t Start_Time3[2];//��ʼʱ��
//    uint8_t End_Timep3[2];//����ʱ��
//    uint8_t Power3[1];//����
//    uint8_t Frq3[4];//Ƶ��
//    uint8_t channel3[1];//�ŵ�

//    uint8_t Start_Time4[2];//��ʼʱ��
//    uint8_t End_Timep4[2];//����ʱ��
//    uint8_t Power4[1];//����
//    uint8_t Frq4[4];//Ƶ��
//    uint8_t channel4[1];//�ŵ�

//    uint8_t Start_Time5[2];//��ʼʱ��
//    uint8_t End_Timep5[2];//����ʱ��
//    uint8_t Power5[1];//����
//    uint8_t Frq5[4];//Ƶ��
//    uint8_t channel5[1];//�ŵ�

//    uint8_t Start_Time6[2];//��ʼʱ��
//    uint8_t End_Timep6[2];//����ʱ��
//    uint8_t Power6[1];//����
//    uint8_t Frq6[4];//Ƶ��
//    uint8_t channel6[1];//�ŵ�

//    uint8_t Start_Time7[2];//��ʼʱ��
//    uint8_t End_Timep7[2];//����ʱ��
//    uint8_t Power7[1];//����
//    uint8_t Frq7[4];//Ƶ��
//    uint8_t channel7[1];//�ŵ�

//    uint8_t Start_Time8[2];//��ʼʱ��
//    uint8_t End_Timep8[2];//����ʱ��
//    uint8_t Power8[1];//����
//    uint8_t Frq8[4];//Ƶ��
//    uint8_t channel8[1];//�ŵ�

//    uint8_t Start_Time9[2];//��ʼʱ��
//    uint8_t End_Timep9[2];//����ʱ��
//    uint8_t Power9[1];//����
//    uint8_t Frq9[4];//Ƶ��
//    uint8_t channel9[1];//�ŵ�

//    uint8_t Start_Time10[2];//��ʼʱ��
//    uint8_t End_Timep10[2];//����ʱ��
//    uint8_t Power10[1];//����
//    uint8_t Frq10[4];//Ƶ��
//    uint8_t channel10[1];//�ŵ�
//} Run_Diagram_t;

//typedef struct
//{
//    uint8_t Transmitte_id[1];//�����ID
//    uint8_t results[1];//ִ�н��

//} Run_DiagramBack_t; //����ͼ���ذ�

//typedef struct
//{
//    uint8_t Start_Time1[2];//��ʼʱ��
//    uint8_t End_Timep1[2];//����ʱ��
//    uint8_t Power1[1];//����
//    uint8_t Frq1[4];//Ƶ��
//    uint8_t channel1[1];//�ŵ�

//    uint8_t Start_Time2[2];//��ʼʱ��
//    uint8_t End_Timep2[2];//����ʱ��
//    uint8_t Power2[1];//����
//    uint8_t Frq2[4];//Ƶ��
//    uint8_t channel2[1];//�ŵ�

//    uint8_t Start_Time3[2];//��ʼʱ��
//    uint8_t End_Timep3[2];//����ʱ��
//    uint8_t Power3[1];//����
//    uint8_t Frq3[4];//Ƶ��
//    uint8_t channel3[1];//�ŵ�

//    uint8_t Start_Time4[2];//��ʼʱ��
//    uint8_t End_Timep4[2];//����ʱ��
//    uint8_t Power4[1];//����
//    uint8_t Frq4[4];//Ƶ��
//    uint8_t channel4[1];//�ŵ�

//    uint8_t Start_Time5[2];//��ʼʱ��
//    uint8_t End_Timep5[2];//����ʱ��
//    uint8_t Power5[1];//����
//    uint8_t Frq5[4];//Ƶ��
//    uint8_t channel5[1];//�ŵ�

//    uint8_t Start_Time6[2];//��ʼʱ��
//    uint8_t End_Timep6[2];//����ʱ��
//    uint8_t Power6[1];//����
//    uint8_t Frq6[4];//Ƶ��
//    uint8_t channel6[1];//�ŵ�

//    uint8_t Start_Time7[2];//��ʼʱ��
//    uint8_t End_Timep7[2];//����ʱ��
//    uint8_t Power7[1];//����
//    uint8_t Frq7[4];//Ƶ��
//    uint8_t channel7[1];//�ŵ�

//    uint8_t Start_Time8[2];//��ʼʱ��
//    uint8_t End_Timep8[2];//����ʱ��
//    uint8_t Power8[1];//����
//    uint8_t Frq8[4];//Ƶ��
//    uint8_t channel8[1];//�ŵ�

//    uint8_t Start_Time9[2];//��ʼʱ��
//    uint8_t End_Timep9[2];//����ʱ��
//    uint8_t Power9[1];//����
//    uint8_t Frq9[4];//Ƶ��
//    uint8_t channel9[1];//�ŵ�

//    uint8_t Start_Time10[2];//��ʼʱ��
//    uint8_t End_Timep10[2];//����ʱ��
//    uint8_t Power10[1];//����
//    uint8_t Frq10[4];//Ƶ��
//    uint8_t channel10[1];//�ŵ�
//} Run_Diagram_buff_t;



//typedef struct
//{
//    uint8_t Alarm_PC_code[2];//PC�·����������Ľṹ��
//    uint8_t Alarm_Back_code[2];//���������PC����
//    uint8_t Work_PC_code[2];//PC�·���ѯ״̬
//    uint8_t Work_PCBack_code[2];//PC�·���ѯ���������
//    uint8_t Topen_PC_code[2];//PC�·����������
//    uint8_t Topen_PCBack_code[2];//���������PC�����رյ�״̬
//    uint8_t Tstop_PC_code[2];//PC�·����������
//    uint8_t Tstop_PCBack_code[2];//���������PC�����رյ�״̬
//    uint8_t Disalarm_PC_code[2];//�������״̬
//    uint8_t Disalarm_PCBack_code[2];//�������״̬����PC
//    uint8_t Histalarm_PC_code[2];//pc�·���ѯ��ʷ������
//    uint8_t Histalarm_PCBack_code[2];//PC�·���ѯ��ʷ�������Ļذ�
//    uint8_t PowerON_PC_code[2];//PC�·��ϵ�
//    uint8_t PowerON_PCBack_code[2];//PC�·��ϵ�Ļذ�
//    uint8_t PowerOFF_PC_code[2];//PC�·��ϵ�
//    uint8_t PowerOFF_PCBack_code[2];//PC�·��ϵ�Ļذ�
//    uint8_t ScanFrq_PC_code[2];//PC�·���ѹ���Ƶ��
//    uint8_t ScanFrq_PCBack_code[2];//PC�·���ѹ���Ƶ�ʵĻذ�
//    uint8_t ScanStop_PC_code[2];//PC�·�ֹͣɨ��
//    uint8_t ScanStop_PCBack_code[2];//PC�·�ֹͣɨ��Ļذ�
//    uint8_t AddPower_PC_code[2];//���ӹ���΢��
//    uint8_t AddPower_PCBack_code[2];//���ӹ���΢���Ļذ�
//    uint8_t SubPower_PC_code[2];//���ٹ���΢��
//    uint8_t SubPower_PCBack_code[2];//���ٹ���΢��
//} Func_code_t;

//typedef struct
//{
//    uint8_t uart_pc[256];
//    volatile uint8_t len;
//} pc_data_t; //�յ����ڵ�ԭʼ����

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
//    uint8_t Spe_data[256];//�����������ݾ�������(Ӧ���ǰ���Э�飬����Ľṹ�峤��һ��)
//    uint8_t Spe_len;//�������ݵĳ���
//} Spe_data_buf_t;

//typedef struct
//{
//    uint8_t eff_sign;//��־�������������Ч
//    uint8_t CAN_ID_buf;//CANID
//    uint8_t Func_code[2];//������
//    Spe_data_buf_t p_Spe_data;//��������
//} CAN_data_t;

//typedef struct
//{
//    uint8_t Data_buffer[256];
//    uint8_t Len_buffer;
//} Assemble_buffer_t; //������֮�������(�Ѿ������������ݰ���)

#pragma pack()

//extern volatile uint8_t CAN_ID;
//extern Scan_FrqBack_t				Scan_FrqBack1;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack2;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack3;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack4;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack5;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack6;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack7;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack8;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack9;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack10;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack11;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack12;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack13;//PCɨ����ѹ���Ƶ��
//extern Alarm_threshold_t			Alarm_threshold;//PC�·�������ֵ�Ľṹ��
//extern Alarm_backPC_t				Alarm_backPC;//PC���ñ����ķ��ذ�(�ɷ��������PC)
//extern Working_paramet_t			Working_paramet;//PC��ѯ���������״̬�Լ������Ĳ�����ָ��
//extern Work_paraBack_t				Work_paraBack;//����PC������������������ݰ�
//extern Trans_open_t					Trans_open;//���������
//extern Trans_openBack_t				Trans_openBack;//�����������PC���ص����ݰ�
//extern Trans_open_Copy_t     		Trans_open_Copy;//�������������
//extern Trans_stop_t					Trans_stop;//ֹͣ�����
//extern Trans_stopBack_t				Trans_stopBack;//ֹͣ�������PC���ص����ݰ�
//extern Disalarm_t					Disalarm;//PC�������״̬
//extern DisalarmBack_t				DisalarmBack;//�������״̬��Ӧ���
//extern Alarm_history_t 				Alarm_history;//���屨��״̬�Ĳ�ѯ
//extern Alarm_historyBack_t 			Alarm_historyBack;//����ı���״̬������
//extern Power_on_t 					Power_on;//PC�������������Դ��
//extern Power_onBack_t 				Power_onBack;//����PC�������������Դ��
//extern Power_off_t					Power_off;//PC�رյ�Դ
//extern Power_offBack_t				Power_offBack;//����PC�رյ�Դ
//extern Scan_Frq_t					Scan_Frq;//PCɨ����ѹ���Ƶ��
//extern Scan_FrqBack_t				Scan_FrqBack;//PCɨ����ѹ���Ƶ��
//extern Sacn_stop_t					Sacn_stop;//PCֹͣɨ��
//extern Sacn_stopBack_t				Sacn_stopBack;//����PCֹͣɨ��
//extern Sub_PowerBack_t 				Sub_PowerBack;//���ٹ��ʷ��صİ�
//extern Sub_Power_t 					Sub_Power;//���ٹ���
//extern Add_PowerBack_t 				Add_PowerBack;//���ӹ��ʵĻذ�
//extern Add_Power_t 					Add_Power;//���ӹ���
//extern Run_DiagramBack_t 	  		Run_DiagramBack;//����ͼ����
//extern Run_Diagram_t 				Run_Diagram;//����ͼ
//extern Run_Diagram_buff_t   		Run_Diagram_buff[10];//100������ͼ����
//extern volatile uint8_t				Scan_Freq_flag;
//extern pc_data_t PC_data;

//uint8_t CAN_data_func(uint8_t *p_func_code_buf,pc_data_t *p_PC_dat);
//uint8_t Send_PC(uint8_t *Func_code);

//static CAN_data_t Accpet_CAN_data(pc_data_t *PC_dat);//��������ڷ�����Ǹ����صĽ�������
//static uint8_t Data_Storage(CAN_data_t *CAN_data_cmp);
//static uint8_t Data_Assemble(uint8_t *Func_code,uint8_t *Specific_data,uint8_t Can_ID_buf,uint8_t Len,Assemble_buffer_t *Buffer);

//static uint8_t Check_Parameters_01(uint8_t *check_code);
//static uint8_t Check_Parameters_02(uint8_t *check_code);
//static uint8_t Check_Parameters_03(uint8_t *check_code);
//static uint8_t Check_Parameters_04(uint8_t *check_code);
//static uint8_t Check_Parameters_05(uint8_t *check_code);
//static uint8_t Check_Parameters_06(uint8_t *check_code);
#endif /*   */
