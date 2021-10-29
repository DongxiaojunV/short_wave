#ifndef _BSP_EMISSION_H_
#define _BSP_EMISSION_H_
#include <stdint.h>

//#define Emis_Size	64

//typedef struct Temperature_humidity
//{
//    float Temperature;//�¶�
//    float Humidity;//ʪ��
//} Temperature_Humidity_t;

//typedef struct best_band
//{
//    float best_band_max;//���Ƶ������
//    float best_band_min;//���Ƶ������
//} Best_band_t;

//typedef struct show_parameter
//{
//    float Forward_Power;//�����ʣ�ʵ��Ϊ��ѹ
//    float Reverse_Power;//�����ʣ�ʵ��Ϊ��ѹ
//    float M_45V;//45V��ѹ
//    float M_45I;//45V�µĵ���
//    float Standing_wave_ratio;//פ����
//} Show_Parameter_t;

//typedef struct state
//{
//    volatile uint8_t Sys_gain;//ϵͳ���ӹ���
//    volatile uint8_t Sys_reduction;//ϵͳ���ٹ���
//    volatile uint8_t Sys_open;//ϵͳ����
//    volatile uint8_t Sys_close;//ϵͳ�ػ�
//    volatile uint8_t Sys_opening;//ϵͳ������
//    volatile uint8_t FM_working;//���ڲ�����Ƶ
//    volatile uint8_t Emis_working;//���ڷ�����
//    volatile uint8_t Emis_readay;//����׼��

//    volatile uint8_t History_alarm;//��ʷ������¼
//    volatile uint8_t Already_init;//�Ѿ���ʼ����־
//    volatile uint8_t Sweeping;//����ɨƵ״̬
//    volatile uint8_t Already_Swept;//�Ѿ�ɨƵ��ɱ�־���ñ�־�豣����flash��
//    volatile uint8_t Cancel_Sweeping;//ȡ��ɨƵ

//    volatile uint8_t SWR_alarm;//פ���ȱ���
//    volatile uint8_t No_power_level;//�޹������
//    volatile uint8_t alarm;//������������
//    volatile uint8_t no_respond;//������3�β�������Ӧ
//    volatile uint8_t temperature_alarm;//�¶ȱ���
//    volatile uint8_t humidity_alarm;//ʪ�ȱ���
//    volatile uint8_t over_Electric;//��������
//    volatile uint8_t over_Voltage;//��ѹ����
//    volatile uint8_t low_Voltage;//Ƿѹ����
//} State_t;
//typedef union cmd
//{
//    volatile uint8_t inquire;//��ѯ״̬  CE
//    volatile uint8_t frequency;//����Ƶ�� 54
//    volatile uint8_t species;//��������   4D
//    volatile uint8_t power_level;//���ʵȼ� 50
//    volatile uint8_t channel;//�ŵ�  43
//    volatile uint8_t emission;//����  0D
//    volatile uint8_t stop;//ֹͣ  18
//    volatile uint8_t gain_plus;//����� 2B
//    volatile uint8_t gain_reduction;//����� 2D
//    volatile uint8_t mode;//����ģʽ 44
//} cmd_t;

//typedef struct emission
//{
//    volatile uint8_t head;//֡ͷ
//    cmd_t 	cmd;//ָ����
//    volatile uint8_t data[4];//����
//    volatile uint8_t tail;//֡β
//} EmissionCmd_t;

//typedef struct respond
//{
//    uint8_t head[2];//�̶�֡ͷ
//    volatile uint8_t channel;//�ŵ�
//    uint8_t frequency[4];//����Ƶ��
//    volatile uint8_t species;//��������			//FM
//    volatile uint8_t status;//����״̬
//    volatile uint8_t mode;//����ģʽ			//��Ƶ��˫Ƶ����Ƶ
//    volatile uint8_t whethre_work;//�Ƿ���
//    volatile uint8_t unused;//����λ��û��ʹ��
//    volatile uint8_t power_level;//���ʵȼ�
//    volatile uint8_t check;//���У��
//    uint8_t tail;//�̶�֡β ��Ϊ�ǹ̶��ģ����Բ���volatile������
//} Respond_t;

///*----------------------------------------------------------------------------------------*/
//typedef struct monitor
//{
//    volatile uint8_t no_resp_count;//û����Ӧ����,PC�����쳣����
//    volatile uint8_t hard_control;//Ӳ���ӹ�
//    volatile uint8_t rec_run_diagram;//���յ�����ͼ
//    volatile uint8_t diagram_count;//һ�������н�������ͼ����
//    volatile uint8_t need_open;//��Ҫ����
//} Monitor_t;

//typedef struct hard_control
//{
//    volatile uint8_t usage_diagram_count;//��¼�ǵڼ��β�ѯ����������ͼ
//} Hard_control_t;

//typedef struct state_monitor
//{
//    volatile uint8_t start[10];
//    volatile uint8_t end[10];
//} State_monitor_t;
////��ѯ״̬  CE
//#define     INQUIRE      	0xCE
////ֹͣ
//#define     STOP         	0x18
////�����ŵ�
//#define		CHANNEL			0x43
////Ƶ��
//#define 	FREQUENCY		0x54
////��������
//#define 	SPECIES			0x4D
////���ʵȼ�
//#define 	POWER_LEVEL		0x50
////����
//#define 	EMISSION		0x0D
////����ģʽ
//#define		MODE			0x44
////��г
//#define		TUNING			0x74
////�����
//#define		Gain_plus		0x2B
//#define		POWER_ADD		0x2B		//����
////�����
//#define 	Gain_reduction	0x2D
//#define 	POWER_SUB		0x2D		//����
////	uint8_t mode;//����ģʽ 44

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

//extern Respond_t Respond_Cmd;	//���������ؽṹ��
//extern Respond_t Respond_Cmd2;	//���������ؽṹ�壬˫Ƶ
//extern Respond_t Respond_Cmd3;	//���������ؽṹ�壬��Ƶ

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

