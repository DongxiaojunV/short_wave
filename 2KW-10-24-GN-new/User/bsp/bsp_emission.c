#include "includes.h"

#include "new_protocol.h"

float z[256];//��
uint8_t int_z[256];//�����ʵĳ���10intֵ
float f[256];//��
float begain;
float end;
/*
 *�ⲿ����
 */
extern EventGroupHandle_t xCreatedEventGroup;
extern TaskHandle_t xHandleTaskReceiveFormPC;
extern TaskHandle_t xHandleTaskSendToPC;
extern TaskHandle_t xHandleTaskUserIF;
extern uint8_t g_fuc_cod[2];//ȫ�ֹ�����
extern uint8_t g_fuc_codToPC[2];//���ظ�PC�Ĺ�����
// ADC1ת���ĵ�ѹֵͨ��MDA��ʽ����SRAM
extern __IO uint16_t ADC_ConvertedValue[NOFCHANEL];
void set_alarm_bit(void);
/*
 * ���ļ���������
 */
static int Is_available_data(const uint8_t * Msg);
static int MT2000_sendCommand(const uint8_t *SendCommand,const int Len,TickType_t xTicksToWait);
static int run_command(TickType_t xTicksToWait);
static int Band_scan(TickType_t xTicksToWait,int index);
static int get_better_SWR(float *array,int index);
//static int judge_fre_segment(void);

volatile uint8_t clean_sweeping;
volatile uint8_t clean_gain;
volatile uint8_t clean_reduction;
volatile uint8_t clean_FM;

uint8_t Emission_Rx[Emis_Size];//���ջ�����
uint8_t Emission_Tx[Emis_Size];//���ͻ�����

TaskHandle_t xHandleTaskEmis_Send = NULL;//�����߳�
TaskHandle_t xHandleTask_Receive = NULL;//�����߳�
TaskHandle_t xHandleTask_hard_control = NULL;//Ӳ������
TaskHandle_t xHandleTask_hard_monitor = NULL;//Ӳ��������

EmissionCmd_t EmissionCmd;//���͸����������ݽṹ��
Respond_t Respond_Cmd;//���������ؽṹ��
Respond_t Respond_Cmd2;//���������ؽṹ�壬˫Ƶ
Respond_t Respond_Cmd3;//���������ؽṹ�壬��Ƶ

State_t Sys_State;//ϵͳ״̬�ṹ��
Show_Parameter_t Show_Parameter;//��ʾ����
Best_band_t Best_band;//���Ƶ��
Temperature_Humidity_t Temperature_Humidity;//��ʪ��
Monitor_t Monitor;//���ϵͳ����Ȩ
Hard_control_t Hard_control;
State_monitor_t State_monitor[10];
float eADC0_buff[100];
float eADC1_buff[100];
float eADC2_buff[100];
float eADC3_buff[100];

float SWR_array[256];//פ���Ȼ���
//static int SWR_index[256];//פ�����±�Ѱַ

/* ����ָ��������(ͨ�����Ƽ��������Ӷ����Ʒ����״̬)
 * ���ȼ�5
 */
void vTaskTaskTaskEmis_Send(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* ����ӳ�500ms */
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;

    int ret;
    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* ��������ӳ�ʱ�� */
        if( xResult == pdPASS )
        {
            if((ulValue & BIT_0) != 0)			//����ʼ����ɣ����ѯ����״̬����������
            {
                //	Work_paraBack.Transmitte_id[0]=CAN_ID;//���can id �ȷ���� id
                if(	Sys_State.Already_init==1)   //�Ѿ���ʼ���˵�
                {
                    if(get_history_alm()==1)   //��ʷ�б���״̬
                    {
                        App_printf("NO1. history state was alarm!\r\n");
                        Work_paraBack.Trans_current_state[0]=0x11;//��ǰ�Ǳ���״̬
                        if(Sys_State.No_power_level==1&&Sys_State.Emis_working==1)   //�޹��ʱ���
                        {
                            Work_paraBack.Hist_alarm[0]=0x01;//���ڷ���
                        }
                        if(Sys_State.alarm==1)   //����������
                        {
                            Sys_State.Emis_working=0;
                            Work_paraBack.Hist_alarm[0]=0x00;//û�з���
                        }

                        if(Sys_State.Emis_working==0)
                        {
                            memset(Work_paraBack.Channel,0,3);//�ŵ�����
                            memset(Work_paraBack.Freq,0,12);//Ƶ������
                            memset(Work_paraBack.Power_grade,0,3);//���ʵȼ�����
                            Work_paraBack.Type[0]=0;//������������
                            Work_paraBack.Forward_power[0]=0;
                            Work_paraBack.Reverse_power[0]=0;
                            Work_paraBack.Power_45_intensity[0]=0;
                            Work_paraBack.Hist_alarm[0]=0x00;//û�з���
                            //��ѹ��ʾ
                            Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                        }
                        else
                        {
                            Work_paraBack.Hist_alarm[0]=0x01;//���ڷ���
                            Work_paraBack.Type[0]=Trans_open.Type[0];//�������࣬��Ƶ��˫Ƶ������Ƶ,�ѷ�����ĸ�״̬����ѯ��
                            Work_paraBack.Channel[0]=Trans_open.Channel[0];//Respond_Cmd.channel;//��ֵ�ŵ�
                            memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//��ֵƵ��
                            Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//Respond_Cmd.power_level;//���ʵȼ�

                            if(Work_paraBack.Type[0]!=0x01)   //���ǹ�Ƶ
                            {
                                Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//��ֵ�ŵ�
                                memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//��ֵƵ��
                                Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//���ʵȼ�
                                if(Work_paraBack.Type[0]!=0x02)   //Ҳ����˫Ƶ����ôֻ������Ƶ
                                {
                                    Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//��ֵ�ŵ�
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//��ֵƵ��
                                    Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//���ʵȼ�
                                    //App_printf("Respond_Cmd3.power_level=%02x\r\n",Respond_Cmd3.power_level);
                                }
                            }

                            //������ʾ
                            if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                            {
                                Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                            }
                            else     //С�� 1.5V����ϵ��Ϊ40
                            {
                                Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                            }
                            //��ѹ��ʾ
                            Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                            App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                            App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                            App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                            App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                            App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                        }
                    }
                    else     //ֻ��AD�ɼ�û�е�ǰ����������������м������ĵ�ǰ����
                    {
                        Work_paraBack.Hist_alarm[0]=0x01;//����״̬
                        //��ѯ
                        Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                        ret=run_command(xTicksToWait);//�ڲ�ѭ����3��
                        if(ret==1)   //�ɹ�״̬
                        {
                            if(Respond_Cmd.status==1)   //����״̬
                            {
                                App_printf("working state...\r\n");
                                Sys_State.Emis_working=1;
                                Work_paraBack.Trans_current_state[0]=0x01;//��������״̬
                                Work_paraBack.Type[0]=Trans_open.Type[0];//�������࣬��Ƶ��˫Ƶ������Ƶ,�ѷ�����ĸ�״̬����ѯ��
                                Work_paraBack.Channel[0]=Trans_open.Channel[0];//��ֵ�ŵ�
                                memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//��ֵƵ��
                                Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//���ʵȼ�
                                if(Work_paraBack.Type[0]!=0x01)   //���ǹ�Ƶ
                                {
                                    Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//��ֵ�ŵ�
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//��ֵƵ��
                                    Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//���ʵȼ�
                                    if(Work_paraBack.Type[0]!=0x02)   //Ҳ����˫Ƶ����ôֻ������Ƶ
                                    {
                                        Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//��ֵ�ŵ�
                                        memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//��ֵƵ��
                                        Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//���ʵȼ�
                                    }
                                }
                                //������ʾ
                                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                                {
                                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                }
                                else     //С�� 1.5V����ϵ��Ϊ40
                                {
                                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                }
                                //��ѹ��ʾ
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                                App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                                App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                                App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                                App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                                App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                            }
                            else     //����״̬,Respond_Cmd.status ��Ϊ0����Ϊ�������ֻ������״̬
                            {
                                App_printf("waiting state...\r\n");
                                Sys_State.Emis_working=0;
                                Work_paraBack.Trans_current_state[0]=0x02;//ֹͣ״̬���ȴ���״̬
                                memset(Work_paraBack.Channel,0,3);//�ŵ�����
                                memset(Work_paraBack.Freq,0,12);//Ƶ������
                                memset(Work_paraBack.Power_grade,0,3);//���ʵȼ�����
                                Work_paraBack.Type[0]=0;//������������
                                Work_paraBack.Forward_power[0]=0;
                                Work_paraBack.Reverse_power[0]=0;
                                Work_paraBack.Power_45_intensity[0]=0;
                                //��ѹ��ʾ
                                Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                            }
                        }
                        else     //����״̬
                        {
                            App_printf("NO2.current state is alarm!\r\n");
                            Work_paraBack.Trans_current_state[0]=0x11;//��ǰ�Ǳ���״̬
                            if(Sys_State.No_power_level==1&&Sys_State.Emis_working==1)   //�޹��ʱ���
                            {
                                Work_paraBack.Hist_alarm[0]=0x01;//���ڷ���
                            }
                            if(Sys_State.alarm==1)   //����������
                            {
                                Sys_State.Emis_working=0;
                                Work_paraBack.Hist_alarm[0]=0x00;//û�з���
                            }
                            if(Sys_State.Emis_working==0)
                            {
                                memset(Work_paraBack.Channel,0,3);//�ŵ�����
                                memset(Work_paraBack.Freq,0,12);//Ƶ������
                                memset(Work_paraBack.Power_grade,0,3);//���ʵȼ�����
                                Work_paraBack.Type[0]=0;//������������
                                Work_paraBack.Forward_power[0]=0;
                                Work_paraBack.Reverse_power[0]=0;
                                Work_paraBack.Power_45_intensity[0]=0;
                                //��ѹ��ʾ
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;
                                Work_paraBack.Hist_alarm[0]=0x00;//û�з���
                            }
                            else
                            {
                                Work_paraBack.Hist_alarm[0]=0x01;//���ڷ���
                                Work_paraBack.Type[0]=Trans_open.Type[0];//�������࣬��Ƶ��˫Ƶ������Ƶ,�ѷ�����ĸ�״̬����ѯ��
                                Work_paraBack.Channel[0]=Trans_open.Channel[0];//Respond_Cmd.channel;//��ֵ�ŵ�
                                memcpy(Work_paraBack.Freq,Trans_open.Freq,4);//��ֵƵ��
                                Work_paraBack.Power_grade[0]=Trans_open.Power_grade[0];//Respond_Cmd.power_level;//���ʵȼ�
                                if(Work_paraBack.Type[0]!=0x01)   //���ǹ�Ƶ
                                {
                                    Work_paraBack.Channel[1]=Trans_open.Channel[1];//Respond_Cmd2.channel;//��ֵ�ŵ�
                                    memcpy((uint8_t *)(Work_paraBack.Freq)+4,(uint8_t*)(Trans_open.Freq)+4,4);//��ֵƵ��
                                    Work_paraBack.Power_grade[1]=Trans_open.Power_grade[1];//���ʵȼ�
                                    if(Work_paraBack.Type[0]!=0x02)   //Ҳ����˫Ƶ����ôֻ������Ƶ
                                    {
                                        Work_paraBack.Channel[2]=Trans_open.Channel[2];//Respond_Cmd3.channel;//��ֵ�ŵ�
                                        memcpy((uint8_t *)(Work_paraBack.Freq)+8,(uint8_t*)(Trans_open.Freq)+8,4);//��ֵƵ��
                                        Work_paraBack.Power_grade[2]=Trans_open.Power_grade[2];//���ʵȼ�
                                        //App_printf("Respond_Cmd3.power_level=%02x\r\n",Respond_Cmd3.power_level);
                                    }
                                }
                                //������ʾ
                                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                                {
                                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                }
                                else     //С�� 1.5V����ϵ��Ϊ40
                                {
                                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                }
                                //��ѹ��ʾ
                                Work_paraBack.Power_45_voltage[0]=Show_Parameter.M_45V*Debug_M_45V;

                                App_printf("Forward_power=%f \r\n",Work_paraBack.Forward_power[0]);
                                App_printf("Reverse_power=%f \r\n",Work_paraBack.Reverse_power[0]);
                                App_printf("Power_45_voltage=%f \r\n",Work_paraBack.Power_45_voltage[0]);
                                App_printf("Power_45_intensity=%f \r\n",Work_paraBack.Power_45_intensity[0]);
                                App_printf("Standing_wave_ratio=%f \r\n",Show_Parameter.Standing_wave_ratio);
                            }
                        }

                    }
                }
                else     //û�г�ʼ��
                {
                    Work_paraBack.Trans_current_state[0]=0x10;//û�о�����ʼ��
                    memset(Work_paraBack.Channel,0,3);//�ŵ�����
                    memset(Work_paraBack.Freq,0,12);//Ƶ������
                    memset(Work_paraBack.Power_grade,0,3);//���ʵȼ�����
                    Work_paraBack.Type[0]=0;//������������
                    Work_paraBack.Forward_power[0]=0;
                    Work_paraBack.Reverse_power[0]=0;
                    Work_paraBack.Power_45_intensity[0]=0;
                    //��ѹ��ʾ
                    Work_paraBack.Power_45_voltage[0]=(float) ADC_ConvertedValue[2]/4096*3.3*2*Debug_M_45V;
                    Work_paraBack.Hist_alarm[0]=0;//û�з���
                }
                //��Ҫ����send to pc�߳�
                xTaskNotify(xHandleTaskSendToPC, /* Ŀ������ */
                            BIT_0, /* ����Ŀ�������¼���־λ bit0 */
                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_0 ���л�������������ֵ���¼���־λ��*/
            }
            else if((ulValue & BIT_1) != 0)		//��ѯ
            {
                /*����main�����е�ADC�ɼ�*/
                if(xHandleTaskUserIF!=NULL)
                {
                    vTaskSuspend(xHandleTaskUserIF);
                    App_printf("stop task Suspend main->ADC task!!!\r\n");
                }
                //ֹͣ����,��ֹͣ��ʱ���ٷ���ָֹͣ�Ҳ������Ӧ
                Trans_stopBack.Transmitte_id[0]=CAN_ID;
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                ret=run_command(xTicksToWait);
                if(ret==1)   //�ɹ���ѯ�ˣ��ż�������ģ�������Ǳ���״̬
                {
                    if(Respond_Cmd.status==0x00)   //�����ֹͣ״̬
                    {
                        Trans_stopBack.Trans_state[0]=0xFC;//�Ѿ���ֹͣ״̬
                        Sys_State.Emis_working=0;
                    }
                    else     //�������ֹͣ״̬������ָֹͣ��
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);
                        if(run_command(xTicksToWait)==1)   //ֹͣ�ɹ�
                        {
                            Trans_stopBack.Trans_state[0]=0xFE;
                            Sys_State.Emis_working=0;
                        }
                        else     //���ɹ�������ֹͣʧ�ܣ��Ҵ��ڱ���״̬
                        {
                            Trans_stopBack.Trans_state[0]=0x02;//ֹͣʧ��
                        }
                    }
                }
                else if(ret==2)     //���������
                {
                    Trans_stopBack.Trans_state[0]=0xFC;//�з��������ʱֹͣ�����
                }
                else     //����ѯ�����ɹ�������ֹͣʧ�ܣ��Ҵ��ڱ���״̬
                {
                    Trans_stopBack.Trans_state[0]=0x02;//PC�·����ŵ�ֹͣʧ��
                }
                if(xHandleTaskUserIF!=NULL)
                {
                    App_printf("stop task Resume main->ADC task!!!\r\n");
                    vTaskResume(xHandleTaskUserIF);
                }
                //Ȼ�󴥷�send to PC�Ĳ�ѯ�ذ�
                xTaskNotify(xHandleTaskSendToPC, /* Ŀ������ */
                            BIT_1, /* ����Ŀ�������¼���־λ bit1 */
                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_1 ���л�������������ֵ���¼���־λ��*/
            }

            else 	if((ulValue & BIT_2) != 0)	//����
            {
//						//�������
//						Sys_State.alarm=0;//�����������㣬֮���д��һ������
//						App_printf("//�������\r\n");
            }
            else 	if((ulValue & BIT_3) != 0)	//����
            {
                App_printf("Emis//ִ�в���\r\n");

#if	NEW_PROTOCOL_EN	//�»���Э��
                MT2000_Cmd_Channel();			//�����ŵ�������ģʽ��������ʽ��Ƶ�ʡ���Ƶʱ��
                ret = MT2000_Wait_Ack();
                if( ret == 1 )
                {
                    MT2000_Cmd_Tune();			//��г
                    for(i=0; i<10; i++)
                    {
                        vTaskDelay(500);
                        ret = MT2000_Wait_Ack();
                    }

                    if( ret == 1 )
                    {
                        MT2000_Cmd_Emit();		//����
                        ret = MT2000_Wait_Ack();
                    }
                }
#else
                Pre_EmissionSend(&EmissionCmd,CHANNEL,1);//�����ŵ�
                if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                {
                    //	App_printf("Respond_Cmd.species=%d\r\n",Respond_Cmd.species);
                    if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                    {
                        Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                        if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                        {
                            goto finish;
                        }
                    }

                    //�����ŵ��ɹ���ִ�������:
                    Pre_EmissionSend(&EmissionCmd,FREQUENCY,1);//��Ƶ����Ƶ��
                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                    {
                        if(Trans_open.Power_grade[0]==0x02)   //�ķ�֮һ����
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                        }
                        else if(Trans_open.Power_grade[0]==0x00)     //ȫ����
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊȫ����
                        }
                        //else if(Trans_open.Power_grade[0]==0x01)//����֮һ����
                        else   //Ԥ���������⣬�������ķ�֮һҲ����ȫ���ʵ�ʱ��͸�����֮һ
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ����֮һ
                        }
                        if(run_command(xTicksToWait)==1)
                        {
                            Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                            if(run_command(xTicksToWait*14)==1)   //��г��7s
                            {
                                Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                if(run_command(xTicksToWait)!=1)
                                {
                                    goto finish;
                                }
                                else
                                {
                                    Sys_State.Emis_working=1;//ϵͳ���ڷ���״̬
                                }
                                if(Trans_open.Type[0]!=0x01)   //���ǹ�Ƶ
                                {
                                    Sys_State.Emis_working=0;//ϵͳ���ڷ���״̬����
                                    vTaskDelay(500);
                                    Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                    if(run_command(xTicksToWait)==1)   //���ֹͣ�ɹ���ִ�еڶ����ŵ�
                                    {
                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,2);//�����ŵ�2
                                        if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                        {
                                            if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                                            {
                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                                                if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                                                {
                                                    goto finish;
                                                }
                                            }
                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,2);//˫Ƶ����Ƶ��
                                            if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                            {
                                                if(Trans_open.Power_grade[1]==0x02)   //�ķ�֮һ����
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                }
                                                else if(Trans_open.Power_grade[1]==0x00)     //ȫ����
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊȫ����
                                                }
                                                //else if(Trans_open.Power_grade[0]==0x01)//����֮һ����
                                                else   //Ԥ���������⣬�������ķ�֮һҲ����ȫ���ʵ�ʱ��͸�����֮һ
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ����֮һ
                                                }
                                                if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                                                    if(run_command(xTicksToWait*14)==1)   //��г��7s
                                                    {
                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                                        if(run_command(xTicksToWait)==1)
                                                        {
                                                            vTaskDelay(500);
                                                            Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                                            if(run_command(xTicksToWait)==1)
                                                            {
                                                                if(Trans_open.Type[0]==0x02)   //��˫Ƶ
                                                                {
                                                                    Pre_EmissionSend(&EmissionCmd,MODE,2);//����˫Ƶ
                                                                    if(run_command(xTicksToWait)==1)   //��������ɹ�
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//����˫Ƶ
                                                                        if(run_command(xTicksToWait)!=1)
                                                                        {
                                                                            goto finish;
                                                                        }
                                                                        else
                                                                        {
                                                                            Sys_State.Emis_working=1;//ϵͳ���ڷ���
                                                                        }
                                                                    }
                                                                }
                                                                else if(Trans_open.Type[0]==0x03)     //����Ƶ
                                                                {
                                                                    Pre_EmissionSend(&EmissionCmd,CHANNEL,3);//�����ŵ�3
                                                                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                                    {
                                                                        if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                                                                        {
                                                                            Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                                                                            if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                                                                            {
                                                                                goto finish;
                                                                            }
                                                                        }
                                                                        //�����ŵ��ɹ���ִ�������:
                                                                        Pre_EmissionSend(&EmissionCmd,FREQUENCY,3);//��Ƶ����Ƶ��
                                                                        if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                                        {
                                                                            if(Trans_open.Power_grade[2]==0x02)   //�ķ�֮һ����
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                            }
                                                                            else if(Trans_open.Power_grade[2]==0x01)     //����֮һ����
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                            }
                                                                            else if(Trans_open.Power_grade[2]==0x00)     //ȫ����
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                            }
                                                                            if(run_command(xTicksToWait)==1)
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                                                                                if(run_command(xTicksToWait*14)==1)   //��г��7s
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                                                                    if(run_command(xTicksToWait)==1)
                                                                                    {
                                                                                        vTaskDelay(500);
                                                                                        Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                                                                        if(run_command(xTicksToWait)==1)
                                                                                        {
                                                                                            Pre_EmissionSend(&EmissionCmd,MODE,3);//������Ƶ
                                                                                            if(run_command(xTicksToWait)==1)   //��������ɹ�
                                                                                            {
                                                                                                Pre_EmissionSend(&EmissionCmd,EMISSION);//������Ƶ
                                                                                                if(run_command(xTicksToWait)!=1)
                                                                                                {
                                                                                                    goto finish;
                                                                                                }
                                                                                                else
                                                                                                {
                                                                                                    Sys_State.Emis_working=1;//ϵͳ���ڷ���
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
finish:
#endif
                Sys_State.FM_working=0;//���ڹ�����־���㣬��������ɹ�������ѹ�ƹ�����������
            }
            else 	if((ulValue & BIT_4) != 0)	//����׼���ж�
            {
                App_printf("�Ȳ�ѯһ���ǲ���ֹͣ״̬\r\n");
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                {
                    if(Respond_Cmd.status==0x01 || get_history_alm()==1)   //����ǹ���״̬�����б�������û�н���ֹͣ״̬������ֽ��յ�����ָ�����
                    {
                        App_printf("waring: working or alarm state!\r\n");
                        Trans_openBack.Trans_state[0]=0xFC;//��PC����
                    }
                    else
                    {
                        Trans_openBack.Trans_state[0]=0xFD;//��ʾ���յ���ָ�������Ҫʱ��ȥ��Ƶ
                        Sys_State.Emis_readay=1;//����׼��	��־
                    }
                }
                else
                {
                    Trans_openBack.Trans_state[0]=0xFC;//��PC����
                }
                App_printf("Emiss:�ָ�ToPC�߳�\r\n");
                vTaskResume(xHandleTaskSendToPC);
            }
            else 	if((ulValue & BIT_5) != 0)	//ɨƵ
            {
                /*����main�����е�ADC�ɼ�*/
                if(xHandleTaskUserIF!=NULL)
                {
                    vTaskSuspend(xHandleTaskUserIF);
                    App_printf("Band_scan task Suspend main->ADC task!!!\r\n");
                }
                App_printf("Band scaning...\r\n");
                for(int i=0; i<10&&Sys_State.Cancel_Sweeping!=1; i++)
                {
                    switch(i)
                    {
                    case 0:
                    {
                        begain=5;
                        end=26;
                        break;
                    }
                    case 1:
                    {
                        begain=5.1;
                        end=26.1;
                        break;
                    }
                    case 2:
                    {
                        begain=5.2;
                        end=25.2;
                        break;
                    }
                    case 3:
                    {
                        begain=5.3;
                        end=25.3;
                        break;
                    }
                    case 4:
                    {
                        begain=5.4;
                        end=25.4;
                        break;
                    }
                    case 5:
                    {
                        begain=5.5;
                        end=25.5;
                        break;
                    }
                    case 6:
                    {
                        begain=5.6;
                        end=25.6;
                        break;
                    }
                    case 7:
                    {
                        begain=5.7;
                        end=25.7;
                        break;
                    }
                    case 8:
                    {
                        begain=5.8;
                        end=25.8;
                        break;
                    }
                    case 9:
                    {
                        begain=5.9;
                        end=25.9;
                        break;
                    }
                    }
                    Band_scan(xTicksToWait,i);
                    if(get_history_alm()==1)
                    {
                        break;
                    }
//							if(i==1)
//							{
//								Sys_State.Cancel_Sweeping=1;//����
//								break;
//							}
                    if(Sys_State.Cancel_Sweeping==1)
                    {
                        break;
                    }
                    for(unsigned int j=0; j<180&&Sys_State.Cancel_Sweeping!=1&&i!=9; j++)
                    {
                        vTaskDelay(1000);
                    }

                }
                if(Sys_State.Cancel_Sweeping==1)   //��������Band_scan������ȡ��ɨƵ�˳���
                {
                    if(Sys_State.Sweeping!=0)   //�����ɨƵ����ǿ��ֹͣ��ֹ�ˣ���Ҫ���������ֹͣ�Ա��ϰ�ȫ
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                        if(run_command(xTicksToWait)==1)   //���ֹͣ�ɹ�
                        {
                            Sys_State.Cancel_Sweeping=0;//ȡ��ɨƵ��־����
                            clean_sweeping=1;
                        }
                    }
                    else     //�Ѿ�ɨƵ��ɣ������־
                    {
                        Sys_State.Cancel_Sweeping=0;//ȡ��ɨƵ��־����
                        clean_sweeping=1;
                    }
                }
//						Sys_State.Sweeping=0;//ɨƵ������־����
                if(xHandleTaskUserIF!=NULL)
                {
                    App_printf("Band_scan task Resume main->ADC task!!!\r\n");
                    vTaskResume(xHandleTaskUserIF);
                }

            }
            else 	if((ulValue & BIT_6) != 0)	//��������
            {
                App_printf("Gain_plus...\r\n");
                Sys_State.Sys_gain=1;
                //���ӹ���
                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                {
                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                }
                else     //С�� 1.5V����ϵ��Ϊ40
                {
                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                }
                if(Work_paraBack.Forward_power[0]>=1800)
                {
                    Add_PowerBack.results [0]=0xFB;
                }
                if(Work_paraBack.Forward_power[0]>=(float)Add_Power.Power_UP[0]*10.0)   //�������з��ֵ�ǰ�Ĺ��ʴ�����Ҫ���ӵ��Ĺ���
                {
                    Add_PowerBack.results [0]=0xFB;//ֱ�ӷ��ز�������
                }
                else if(Work_paraBack.Forward_power[0]<=1800)
                {
                    int Back = 0;
                    float cmp_buffer = 0.0;
                    volatile uint8_t count = 0;
                    Pre_EmissionSend(&EmissionCmd,Gain_plus);//�����ָ���������������
                    for(int i = 0; i<50&&get_history_alm()==0&&Work_paraBack.Power_45_intensity[0]<100; i++)
                    {
                        cmp_buffer = Work_paraBack.Forward_power[0];//��ÿ�η����ӹ��ʵ�����֮ǰ���ȱ��浱ǰ��ֵ
                        Back = run_command(xTicksToWait*2);//����ӳɹ�������-2���������������Ҫͣ��������-1����0���Ǽ���������Ӧ
                        if(Back == -1 || Back == 0)   //������ؼ���������Ӧ������ִ��һ��
                        {
                            Back = run_command(xTicksToWait);//����ӳɹ�������-2���������������Ҫͣ��������-1����0���Ǽ���������Ӧ
                        }
                        //����Ҫ����ʱ����Լ200ms
                        vTaskDelay(1000);
                        if(fabs((float)Add_Power.Power_UP[0]*10.0 - Work_paraBack.Forward_power[0])<=30)   //�ӳɹ����Ҽӵ���Ԥ����Ƶ��
                        {
                            //Add_PowerBack.results [0]=0xFE;//�������ӳɹ�
                            break;
                        }
                        if(Back != 1)   //�����������ʧ��
                        {
                            //Add_PowerBack.results [0]=0xFC;//��������ʧ��
                            break;
                        }
                        if(Work_paraBack.Forward_power[0]>=1800.0)   //����֮�������1800W
                        {
                            //Add_PowerBack.results [0]=0xFE;//�������ӳɹ�
                            break;
                        }
                        else     //������Ҫ�ж��Ƿ����ӳɹ���
                        {
                            if(fabs(cmp_buffer-Work_paraBack.Forward_power[0])<=5)//˵������ǰ�����Ӻ�û�б仯
                                count++;
                            else
                            {
                                count = 0;//ֻҪ�м�ɹ�һ��֮�󣬾ͰѼ�����0.
                            }
                            if(count>=2)   //�����2��û�����ӹ��ʳɹ����ͷ�������ʧ��
                            {
                                //Add_PowerBack.results [0]=0xFC;//ֱ�ӷ�������ʧ��
                                count = 0;
                                break;
                            }
                        }
                    }
                }
                clean_gain=1;
//						//Ȼ�󴥷�send to PC�Ĳ�ѯ�ذ�
//						xTaskNotify(xHandleTaskSendToPC, /* Ŀ������ */
//												BIT_10, /* ����Ŀ�������¼���־λ bit10 */
//												eSetBits); /* ��Ŀ��������¼���־λ�� BIT_10 ���л�������������ֵ���¼���־λ��*/
            }
            else 	if((ulValue & BIT_7) != 0)	//���ʼ�С
            {
                App_printf("Gain_reduction...\r\n");
                Sys_State.Sys_reduction=1;
                //���ٹ���
                if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                {
                    Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                }
                else     //С�� 1.5V����ϵ��Ϊ40
                {
                    Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                }
                if(Work_paraBack.Forward_power[0]<=300)
                {
                    Sub_PowerBack.results [0]=0xFB;//���ܼ���
                }
                else if(Work_paraBack.Forward_power[0]<=(float)Sub_Power.Power_DOWN[0]*10.0)     //�ڼ����з��ֵ�ǰ�Ĺ���С����Ҫ���ٵ��Ĺ���
                {
                    Sub_PowerBack.results [0]=0xFB;//���ܼ���
                }
                else if(Work_paraBack.Forward_power[0]>=300)
                {
                    int Back = 0;
                    float cmp_buffer = 0.0;
                    volatile uint8_t count = 0;
                    //cmp_buffer = Work_paraBack.Forward_power[0];//���ȱ���û�����ӹ���֮ǰ�Ĺ���
                    Pre_EmissionSend(&EmissionCmd,Gain_reduction);//�����ָ��
                    for(int i = 0; i<50&&get_history_alm()==0&&Work_paraBack.Power_45_intensity[0]<100; i++)
                    {
                        cmp_buffer = Work_paraBack.Forward_power[0];//��ÿ�η����ٹ��ʵ�����֮ǰ���ȱ��浱ǰ��ֵ
                        Back = run_command(xTicksToWait*2);//����ӳɹ�������-2���������������Ҫͣ��������-1����0���Ǽ���������Ӧ
                        if(Back == -1 || Back == 0)   //������ؼ���������Ӧ������ִ��һ��
                        {
                            Back = run_command(xTicksToWait);//����ӳɹ�������-2���������������Ҫͣ��������-1����0���Ǽ���������Ӧ
                        }
                        //����Ҫ����ʱ����Լ200ms
                        vTaskDelay(1000);
                        if(fabs((float)Sub_Power.Power_DOWN[0]*10.0 - Work_paraBack.Forward_power[0])<=30)   //���ɹ����Ҽ�����Ԥ���Ĺ���
                        {
                            //Sub_PowerBack.results [0]=0xFE;//���ؼ��ٳɹ�
                            break;
                        }
                        if(Back != 1)   //�����������ʧ��
                        {
                            //Sub_PowerBack.results [0]=0xFC;//���ؼ���ʧ��
                            break;
                        }
                        if(Work_paraBack.Forward_power[0]<=300.0)   //����֮��С��300W
                        {
                            //	Sub_PowerBack.results [0]=0xFE;//���ؼ��ٳɹ�
                            break;
                        }
                        else     //������Ҫ�ж��Ƿ���ٳɹ���
                        {
                            if(fabs(cmp_buffer-Work_paraBack.Forward_power[0])<=5)//˵������ǰ�����Ӻ�û�б仯
                                count++;
                            else
                            {
                                count = 0;//ֻҪ�м�ɹ�һ��֮�󣬾ͰѼ�����0.
                            }
                            if(count>=2)   //�����2��û�����ӹ��ʳɹ����ͷ��ؼ���ʧ��
                            {
                                //Sub_PowerBack.results [0]=0xFC;//ֱ�ӷ�������ʧ��
                                count = 0;
                                break;
                            }
                        }
                    }
                }
                clean_reduction=1;
//						//Ȼ�󴥷�send to PC�Ĳ�ѯ�ذ�
//						xTaskNotify(xHandleTaskSendToPC, /* Ŀ������ */
//												BIT_11, /* ����Ŀ�������¼���־λ bit11 */
//												eSetBits); /* ��Ŀ��������¼���־λ�� BIT_11 ���л�������������ֵ���¼���־λ��*/
            }
            else 	if((ulValue & BIT_8) != 0)	//����
            {
                MT2000_Cmd_Tune();				//��г
                for(i=0; i<10; i++)
                {
                    MT2000_Cmd_Power_ON();
                    vTaskDelay(100);
                    ret = MT2000_Wait_Ack();
                    if( ret == 1 )
                    {
                        Sys_State.Sys_open = 1;
                    }
                }
            }
            else 	if((ulValue & BIT_9) != 0)	//����
            {
//						g_fuc_codToPC[0]=0x06;
//						g_fuc_codToPC[1]=0x02;
//						Alarm_historyBack.Transmitte_id[0]=	CAN_ID;
//						Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
//						if(run_command(xTicksToWait)==1)//�ڲ�ѭ����3��
//						{
//							if(Respond_Cmd.status==0x01)
//							{
//								Sys_State.Emis_working=1;
//							}
//							else
//							{
//								Sys_State.Emis_working=0;
//							}
//						}
//						/*�ȴ�������д*/
//						set_alarm_bit();//��Ҫ�����������bitλ
//						Send_PC(g_fuc_codToPC);//���ͳ�ȥ�����������ڱ�������Ӧ���������Ҫ���͵İ�������
            }
            else								//û�ж�Ӧ�¼�
            {
                App_printf("û�ж�Ӧ���¼�!!!\r\n");
            }

        }
        else									//�ȴ���ʱ
        {
            App_printf("vTaskTaskTaskEmis_Send TIME OUT!!!\r\n");
        }
    }
}
volatile int lock_flag1;
volatile int lock_flag2;
volatile int lock_flag3;

/* Ӳ���������
 * ���ȼ�8
 */
void vTaskTask_hard_monitor(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* ����ӳ�500ms */
    volatile uint8_t watch_count=0;

    while(1)
    {
        if(Monitor.hard_control==1&&flash_3_once_flag==1)   //Ӳ���ӹ�
        {
            /*debug*/
//			App_printf("\r\n%d %d %d %d\r\n",Trans_open_Copy.Type[0],lock_flag1,lock_flag2,lock_flag3);
//			vTaskDelay(1000);
            if(Monitor.need_open==1)
            {
                xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                            BIT_1, /* ����Ŀ�������¼���־λ BIT_1 */
                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_1 ���л�������������ֵ���¼���־λ��*/
            }
            if(Trans_open_Copy.Type[0]==0)   //��ֹͣ��
            {
                if(Sys_State.Emis_working==1)
                {
                    //ֹͣ
                    xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                BIT_3, /* ����Ŀ�������¼���־λ BIT_3 */
                                eSetBits); /* ��Ŀ��������¼���־λ�� BIT_3 ���л�������������ֵ���¼���־λ��*/
                }
            }
            else if(Trans_open_Copy.Type[0]==1&&lock_flag1==0)     //��Ƶ  �Ӹ���־���������������ɹ�֮��Ͳ���ִ���ˣ�ֻ��ֹͣ���ˣ������
            {
                if(Sys_State.FM_working!=1)   //���ǵ�Ƶ�У����ɲ�ѯ״̬
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                    {
                        if(Respond_Cmd.status==0)   //ֹͣ״̬
                        {
                            //��������Ҫ������
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//���ڹ�����־��λ
                            xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                        BIT_0, /* ����Ŀ�������¼���־λ bit0 */
                                        eSetBits); /* ��Ŀ��������¼���־λ�� BIT_0 ���л�������������ֵ���¼���־λ��*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            if(memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0||Respond_Cmd.mode!=1)   //NOTE:����ʹ��Trans_open������PC���Ʒ���֮����������Բ���ʹ��Trans_open_Copy
                            {
                                //ֹͣ
                                xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                            BIT_3, /* ����Ŀ�������¼���־λ BIT_3 */
                                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_3 ���л�������������ֵ���¼���־λ��*/
                            }
                            else
                            {
                                lock_flag1=1;
                            }
                        }
                    }
                }
            }
            else if(Trans_open_Copy.Type[0]==2&&lock_flag2==0)     //˫Ƶ
            {
                if(Sys_State.FM_working!=1)   //���ǵ�Ƶ�У����ɲ�ѯ״̬
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                    {
                        if(Respond_Cmd.status==0)   //ֹͣ״̬
                        {
                            //��������Ҫ������
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//���ڹ�����־��λ
                            xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                        BIT_0, /* ����Ŀ�������¼���־λ BIT_0 */
                                        eSetBits); /* ��Ŀ��������¼���־λ�� BIT_0 ���л�������������ֵ���¼���־λ��*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            /*NOTE ����������Ϊ1 2 3,3���߳�֮��������жϲ���ִ��ֹͣ��bug*/
                            /*NOTE ����������Ϊ1 2 3,3���߳�֮��������жϲ���ִ��ֹͣ��bug*/
                            /*NOTE ����������Ϊ1 2 3,3���߳�֮��������жϲ���ִ��ֹͣ��bug*/
                            if((memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+4,Respond_Cmd2.frequency,3)!=0)||Respond_Cmd.mode!=2)   //NOTE:����ʹ��Trans_open������PC���Ʒ���֮����������Բ���ʹ��Trans_open_Copy
                            {
                                //ֹͣ
                                xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                            BIT_3, /* ����Ŀ�������¼���־λ BIT_3 */
                                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_3 ���л�������������ֵ���¼���־λ��*/
                            }
                            else
                            {
                                lock_flag2=1;
                            }
                        }
                    }
                }
            }
            else if(Trans_open_Copy.Type[0]==3&&lock_flag3==0)     //��Ƶ
            {
                if(Sys_State.FM_working!=1)   //���ǵ�Ƶ�У����ɲ�ѯ״̬
                {
                    Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                    {
                        if(Respond_Cmd.status==0)   //ֹͣ״̬
                        {
                            //��������Ҫ������
                            memcpy(Trans_open.Type,Trans_open_Copy.Type,19);
                            Sys_State.FM_working=1;//���ڹ�����־��λ
                            xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                        BIT_0, /* ����Ŀ�������¼���־λ BIT_0 */
                                        eSetBits); /* ��Ŀ��������¼���־λ�� BIT_0 ���л�������������ֵ���¼���־λ��*/
                            Sys_State.FM_working=0;
                        }
                        else
                        {
                            if((memcmp(Trans_open.Freq,Respond_Cmd.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+4,Respond_Cmd2.frequency,3)!=0\
                                    ||memcmp((uint8_t*)Trans_open.Freq+8,Respond_Cmd3.frequency,3)!=0)||Respond_Cmd.mode!=3)   //NOTE:����ʹ��Trans_open������PC���Ʒ���֮����������Բ���ʹ��Trans_open_Copy
                            {
                                //ֹͣ
                                xTaskNotify(xHandleTask_hard_control, /* Ŀ����������ȼ����ͣ�����ִ���걾�����ȥִ���������� */
                                            BIT_3, /* ����Ŀ�������¼���־λ BIT_3 */
                                            eSetBits); /* ��Ŀ��������¼���־λ�� BIT_3 ���л�������������ֵ���¼���־λ��*/
                            }
                            else
                            {
                                lock_flag3=1;
                            }
                        }
                    }
                }
            }
        }
        vTaskDelay(100);
        //	if(watch_count++>=2)
        //	{
        //	watch_count=0;
        //	IWDG_Feed();
        //}

    }
}



/* Ӳ������
 * �������ȼ�4
 */
void vTaskTaskTask_hard_control(void *pvParameters)
{
    const TickType_t xTicksToWait = 500; /* ����ӳ�500ms */
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;
    uint8_t ret = 0;

    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* ��������ӳ�ʱ�� */
        if( xResult == pdPASS && Monitor.hard_control==1)   //Ӳ���ӹ�
        {
            if(xHandleTask_hard_monitor!=NULL)
            {
                vTaskSuspend(xHandleTask_hard_monitor);
                App_printf("hard_control task Suspend hard_monitor task!!!\r\n");
            }

            if((ulValue & BIT_0) != 0)   		//��Ƶ ˫Ƶ ��Ƶ
            {
                App_printf("Hard//ִ�в���\r\n");

#if	NEW_PROTOCOL_EN	//�»���Э��
                MT2000_Cmd_Channel();			//�����ŵ�������ģʽ��������ʽ��Ƶ�ʡ���Ƶʱ��
                ret = MT2000_Wait_Ack();
                if( ret == 1 )
                {
                    MT2000_Cmd_Tune();			//��г
                    for(i=0; i<10; i++)
                    {
                        vTaskDelay(500);
                        ret = MT2000_Wait_Ack();
                    }
                    if( ret == 1 )
                    {
                        MT2000_Cmd_Emit();		//����
                        ret = MT2000_Wait_Ack();
                    }
                }
#else
                Pre_EmissionSend(&EmissionCmd,CHANNEL,1);//�����ŵ�
                if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                {
                    if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                    {
                        Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                        if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                        {
                            goto option_fish1;
                        }
                    }
                    Pre_EmissionSend(&EmissionCmd,FREQUENCY,1);//��Ƶ����Ƶ��
                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                    {
                        if(Trans_open.Power_grade[0]==0x02)   //�ķ�֮һ����
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                        }
                        else if(Trans_open.Power_grade[0]==0x00)     //ȫ����
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊȫ����
                        }
                        //else if(Trans_open.Power_grade[0]==0x01)//����֮һ����
                        else   //Ԥ���������⣬�������ķ�֮һҲ����ȫ���ʵ�ʱ��͸�����֮һ
                        {
                            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ����֮һ
                        }
                        if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                        {
                            Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                            if(run_command(xTicksToWait*14)==1)   //��г��7s
                            {
                                Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                if(run_command(xTicksToWait)!=1)
                                {
                                    goto option_fish1;
                                }
                                else
                                {
                                    Sys_State.Emis_working=1;//ϵͳ���ڷ���״̬
                                }
                                if(Trans_open.Type[0]!=0x01)   //���ǹ�Ƶ
                                {
                                    Sys_State.Emis_working=0;//ϵͳ���ڷ���״̬����
                                    vTaskDelay(500);
                                    Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                    if(run_command(xTicksToWait)==1)   //���ֹͣ�ɹ���ִ�еڶ����ŵ�
                                    {
                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,2);//�����ŵ�2
                                        if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                        {
                                            if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                                            {
                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                                                if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                                                {
                                                    goto option_fish1;
                                                }
                                            }
                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,2);//˫Ƶ����Ƶ��
                                            if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                            {
                                                if(Trans_open.Power_grade[1]==0x02)   //�ķ�֮һ����
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                }
                                                else if(Trans_open.Power_grade[1]==0x00)     //ȫ����
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊȫ����
                                                }
                                                //else if(Trans_open.Power_grade[0]==0x01)//����֮һ����
                                                else   //Ԥ���������⣬�������ķ�֮һҲ����ȫ���ʵ�ʱ��͸�����֮һ
                                                {
                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ����֮һ
                                                }
                                                if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                {
                                                    if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                    {
                                                        Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                                                        if(run_command(xTicksToWait*14)==1)   //��г��7s
                                                        {
                                                            Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                                            if(run_command(xTicksToWait)==1)
                                                            {
                                                                vTaskDelay(500);
                                                                Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                                                if(run_command(xTicksToWait)==1)
                                                                {
                                                                    if(Trans_open.Type[0]==0x02)   //��˫Ƶ
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,MODE,2);//����˫Ƶ
                                                                        if(run_command(xTicksToWait)==1)   //��������ɹ�
                                                                        {
                                                                            Pre_EmissionSend(&EmissionCmd,EMISSION);//����˫Ƶ
                                                                            if(run_command(xTicksToWait)!=1)
                                                                            {
                                                                                goto option_fish1;
                                                                            }
                                                                            else
                                                                            {
                                                                                Sys_State.Emis_working=1;//ϵͳ���ڷ���
                                                                            }
                                                                        }
                                                                    }
                                                                    else if(Trans_open.Type[0]==0x03)     //����Ƶ
                                                                    {
                                                                        Pre_EmissionSend(&EmissionCmd,CHANNEL,3);//�����ŵ�3
                                                                        if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                                        {
                                                                            if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
                                                                            {
                                                                                Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
                                                                                if(run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
                                                                                {
                                                                                    goto option_fish1;
                                                                                }
                                                                            }
                                                                            //�����ŵ��ɹ���ִ�������:
                                                                            Pre_EmissionSend(&EmissionCmd,FREQUENCY,3);//��Ƶ����Ƶ��
                                                                            if(run_command(xTicksToWait)==1)   //�ڲ�ѭ����3��
                                                                            {
                                                                                if(Trans_open.Power_grade[2]==0x02)   //�ķ�֮һ����
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,4);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                                }
                                                                                else if(Trans_open.Power_grade[2]==0x01)     //����֮һ����
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                                }
                                                                                else if(Trans_open.Power_grade[2]==0x00)     //ȫ����
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,1);//�����ù��ʵȼ�Ϊ�ķ�֮һ
                                                                                }
                                                                                if(run_command(xTicksToWait)==1)
                                                                                {
                                                                                    Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                                                                                    if(run_command(xTicksToWait*14)==1)   //��г��7s
                                                                                    {
                                                                                        Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                                                                                        if(run_command(xTicksToWait)==1)
                                                                                        {
                                                                                            vTaskDelay(500);
                                                                                            Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                                                                                            if(run_command(xTicksToWait)==1)
                                                                                            {
                                                                                                Pre_EmissionSend(&EmissionCmd,MODE,3);//������Ƶ
                                                                                                if(run_command(xTicksToWait)==1)   //��������ɹ�
                                                                                                {
                                                                                                    Pre_EmissionSend(&EmissionCmd,EMISSION);//������Ƶ
                                                                                                    if(run_command(xTicksToWait)!=1)
                                                                                                    {
                                                                                                        goto option_fish1;
                                                                                                    }
                                                                                                    else
                                                                                                    {
                                                                                                        Sys_State.Emis_working=1;//ϵͳ���ڷ���
                                                                                                    }
                                                                                                }
                                                                                            }
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
option_fish1:
#endif
                Sys_State.FM_working=0;
            }
            else if((ulValue & BIT_1) != 0)     //����
            {
                if((float)ADC_ConvertedValue[2]/4096*3.3*2>4)		//45V��ѹ�е磬֤���Ѿ��ǿ���״̬
                {
                    Monitor.need_open=0;
                    Sys_State.Sys_open=1;
                    App_printf("system already open !\n");
                }
                else
                {

                    if((float)ADC_ConvertedValue[2]/4096*3.3*2>4)   //45V��ѹ�е磬֤���Ѿ��ǿ���״̬
                    {
                        App_printf("system open succeed!\n");
                        Monitor.need_open=0;
                        Sys_State.Sys_open=1;
                        Sys_State.Sys_opening = 0;					//�޸�7sǱ��BUG
                    }
                    else
                    {
                        Sys_State.Sys_close=1;
                        Sys_State.Sys_open=0;
                        Sys_State.Sys_opening = 0;					//�޸�7sǱ��BUG
                        App_printf("system open failure!\n");
                    }
                }
            }
            else if((ulValue & BIT_2) != 0)     //����
            {

            }
            else if((ulValue & BIT_3) != 0)     //ֹͣ
            {
                Pre_EmissionSend(&EmissionCmd,INQUIRE);//��ѯ���
                if(run_command(xTicksToWait))
                {
                    lock_flag1=0;
                    lock_flag2=0;
                    lock_flag3=0;
                    if(Respond_Cmd.status==0x00)   //�����ֹͣ״̬
                    {
                        //�Ѿ���ֹͣ״̬
                        Sys_State.Emis_working=0;
                    }
                    else     //�������ֹͣ״̬������ָֹͣ��
                    {
                        Pre_EmissionSend(&EmissionCmd,STOP);
                        if(run_command(xTicksToWait)==1)   //ֹͣ�ɹ�
                        {
//							lock_flag1=0;
//							lock_flag2=0;
//							lock_flag3=0;
                            Sys_State.Emis_working=0;
                        }
                        else     //���ɹ�������ֹͣʧ�ܣ��Ҵ��ڱ���״̬
                        {
                            App_printf("Stop failure!\r\n");
                        }
                    }
                }
            }
        }
        else
        {
            App_printf("vTaskTaskTask_hard_control TIME OUT!\r\n");
        }
        if(xHandleTask_hard_monitor!=NULL)
        {
            App_printf("hard_control task Resume hard_monitor task!!!\r\n");
            vTaskResume(xHandleTask_hard_monitor);
        }
    }
}




/* ���շ����Ӧ��
 * �������ȼ�3
 */
void vTaskTaskTaskEmis_Receive(void *pvParameters)
{
    uint8_t read;

    int count =0;
    int ret=0;

//	MT2000_Cmd_Emit();
//	MT2000_Cmd_Stop();
//	MT2000_Cmd_Channel();
//	MT2000_Cmd_Power_ON();
//	MT2000_Cmd_Power_OFF();
//	MT2000_Cmd_Power_Add();
//	MT2000_Cmd_Power_Sub();
//	MT2000_Cmd_Inquire();
//	MT2000_Cmd_Tune();

    while(1)
    {
        while(comGetChar(COM4,&read))
        {
            if(count<Emis_Size)
            {
                Emission_Rx[count++]=read;
            }

            App_printf("%02x ",Emission_Rx[count-1]);
            vTaskDelay(3);
        }

        if(count>2)
        {
#if	NEW_PROTOCOL_EN
//			Emission_Rx[count] = '\0';

//			/* �����Ƿ��յ��س����У������������־λ */
//			if( (Emission_Rx[count-2]=='\r') && (Emission_Rx[count-1]=='\n') )		//��֧��������������յ�����ָ��ᵱ��һ��ָ�����
//			{
            ret = MT2000_Cmd_Analyze(Emission_Rx);		//���������������ݲ����ؽ��
            count=0;
//			}
#else
            count=0;
            App_printf("\r\n");
            ret=Is_available_data(Emission_Rx);		//���������������ݲ����ؽ��
#endif

            if(ret==1)			/* �������Ƽ����� */
            {
                xEventGroupSetBits(xCreatedEventGroup, BIT_0);
                //	uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_0);
//						if((uxBits & BIT_0) == 0)//��if�ж����ڱ��������ȼ����ڷ����������ȼ�ʱ��Ч
//						{
//							App_printf("BIT_0 notice successed!\r\n");
//						}
            }
            else if(ret==-1)	/* ָ��ִ��ʧ�� */
            {
                /*ָ��ִ��ʧ��*/
                App_printf("cmd failure!!!\r\n");
                xEventGroupSetBits(xCreatedEventGroup, BIT_1);
//						uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_1);
//						if((uxBits & BIT_1) == 0)//��if�ж����ڱ��������ȼ����ڷ����������ȼ�ʱ��Ч
//						{
//							App_printf("BIT_1 cmd not respond!\r\n");
//						}
            }
            else if(ret==-2)	/* ������������Զ�ֹͣ���� */
            {
                /*������������Զ�ֹͣ����*/
                App_printf("emission alm!!!\r\n");
                xEventGroupSetBits(xCreatedEventGroup, BIT_2);
//						uxBits = xEventGroupSetBits(xCreatedEventGroup, BIT_2);
//						if((uxBits & BIT_2) == 0)//��if�ж����ڱ��������ȼ����ڷ����������ȼ�ʱ��Ч
//						{
//							App_printf("BIT_2 Warning Stop Emit!\r\n");
//						}
            }
            else				/* ����0����ʾ������������Ч */
            {
                App_printf("\r\n������������Ч!\r\n");
                memset(Emission_Rx,0,Emis_Size);
            }

        }
        else
        {
            count=0;
            vTaskDelay(10);
        }

    }
}
/*
	���͸����������������������������Ҫ���͵�������ϳ�һ�����ݰ�
	item����Ҫ���ϵĽṹ��
	cmd��ָ����,
	uint8_t inquire;//��ѯ״̬  CE
	uint8_t frequency;//����Ƶ�� 54
	uint8_t species;//��������   4D
	uint8_t power_level;//���ʵȼ� 50
	uint8_t channel;//�ŵ�  43
	uint8_t emission;//����  0D
	uint8_t stop;//ֹͣ  18
	uint8_t gain_plus;//����� 2B
	uint8_t gain_reduction;//����� 2D
	uint8_t mode;//����ģʽ 44
�������У�ֻ��һ�����ԣ������������һ�β���Ϊ���Եģ��������⸳ֵһ����Ա�����Ըı��������ֵ
*/
void Pre_EmissionSend(EmissionCmd_t * item,uint8_t cmd,...)
{
    char p[10];
    uint8_t Hex_buf[6];
    uint8_t temp;
    va_list arg_ptr;
    va_start(arg_ptr,cmd);
    item->head=0x7F; //֡ͷ

#if	NEW_PROTOCOL_EN
    MT2000_Tx.cmd = cmd;		//��ȡ����(Ϊ�˼��ݣ�Ψһʹ�õĵط�����run_command����)
#endif

    switch(cmd)
    {
    case TUNING:
    {
        item->cmd.inquire=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//��г\r\n");
        break;
    }
    case INQUIRE:
    {
        item->cmd.inquire=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//��ѯ\r\n");
        break;
    }
    case FREQUENCY:
    {
        item->cmd.frequency=cmd;
        if(Sys_State.Sweeping==1)   //����ɨƵ
        {
            double d_temp=va_arg(arg_ptr, double);

            sprintf(p,"%.6f",d_temp);//��С��ת�����ַ�
            //				for(int i=0;i<9;i++)
            //				{
            //					App_printf("%d ",p[i]);
            //				}
            //				App_printf("\r\n");
            uint8_t Integer;
            sscanf(p,"%02x",(unsigned int *)&Integer);//�õ�Ƶ�ʵ���������
            App_printf("%02x \r\n",Integer);
            if(Integer>=16)   //��λ��
            {
                for(int i=0; i<2; i++)
                {
                    sscanf(&p[3]+i*2,"%02x",(unsigned int *)(Hex_buf+i));
                    App_printf("%02x ",Hex_buf[i]);//�õ�С������ת����HEX16���ƣ�����ֱ�ӷ�����
                }
            }
            else     //1λ��
            {
                for(int i=0; i<2; i++)
                {
                    sscanf(&p[2]+i*2,"%02x",(unsigned int *)(Hex_buf+i));
                    App_printf("%02x ",Hex_buf[i]);//�õ�С������ת����HEX16���ƣ�����ֱ�ӷ�����
                }
            }
            App_printf("\r\n");
            item->data[0]=Integer;//������������
            memcpy((void *)&item->data[1],Hex_buf,2);//С�����ָ�ֵ�����������ͻ�����
            item->data[3]=0;
        }
        else
        {
            temp = va_arg(arg_ptr, int);
            if(temp==1)   //��Ƶ
            {
                memcpy((void *)item->data,Trans_open.Freq,4);
            }
            else if(temp==2)     //˫Ƶ
            {
                memcpy((void *)item->data,Trans_open.Freq+4,4);
            }
            else if(temp==3)     //��Ƶ
            {
                memcpy((void *)item->data,Trans_open.Freq+8,4);
            }
        }
        App_printf("Emis//����Ƶ��\r\n");
        break;
    }
    case SPECIES:
    {
        item->cmd.species=cmd;
        item->data[0]=0x03;//��Ƶ
        memset((void *)&item->data[1],0,3);
        MT2000_Tx.method = 'F';		//FM
        break;
    }
    case POWER_LEVEL:
    {
        item->cmd.power_level=cmd;
        temp = va_arg(arg_ptr, int);
        if(temp==4)   //4��֮1
        {
            item->data[0]=0x02;
        }
        else if(temp==2)     //2��֮1
        {
            item->data[0]=0x01;
        }
        else if(temp==1)     //ȫ����
        {
            item->data[0]=0x00;
        }
        memset((void *)&item->data[1],0,3);
        App_printf("Emis//���ù���Ϊ��\r\n");
        break;
    }
    case CHANNEL:
    {
        item->cmd.channel=cmd;
        temp = va_arg(arg_ptr, int);
        //ʹ���Ǹ�ȫ�ֱ����е��ŵ�
        //	if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)//��ѯʱ�Ĺ�����
        //	{
        //		item->data[0]=Working_paramet.Channel[0];//��ѯʱ���ŵ�
        //		}
        //		else if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)//����ʱ�Ĺ�����
        //		{
        if(temp==1)   //��Ƶ
        {
            item->data[0]=Trans_open.Channel[0];//��ѯʱ���ŵ�
        }
        else if(temp==2)     //˫Ƶ
        {
            item->data[0]=Trans_open.Channel[1];//��ѯʱ���ŵ�
        }
        else if(temp==3)     //��Ƶ
        {
            item->data[0]=Trans_open.Channel[2];//��ѯʱ���ŵ�
        }
        else     //ɨƵ�ŵ���1
        {
            item->data[0]=0x01;//ɨƵʱ���ŵ���Ĭ��1�ŵ�
        }
        //	}
        memset((void *)&item->data[1],0,3);
        App_printf("Emis//�ŵ�����\r\n");

        MT2000_Tx.channel[0] = '0';	//�ŵ�01
        MT2000_Tx.channel[1] = '1';
        MT2000_Tx.channel[2] = '\0';
        break;
    }
    case EMISSION:
    {
        item->cmd.emission=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//����\r\n");
        break;
    }
    case STOP:
    {
        item->cmd.stop=cmd;
        memset((void *)item->data,0,4);
        App_printf("Emis//ֹͣ����\r\n");
        break;
    }
    case Gain_plus:
    {
        item->cmd.gain_plus=cmd;
        memset((void *)item->data,0,4);
        break;
    }
    case Gain_reduction:
    {
        item->cmd.gain_reduction=cmd;
        memset((void *)item->data,0,4);
        break;
    }
    case MODE:
    {
        item->cmd.mode=cmd;
        temp = va_arg(arg_ptr, int);
        if(temp==1)
        {
            item->data[0]=0x01;
            MT2000_Tx.mode = '1';
        }
        else if(temp==2)
        {
            item->data[0]=0x02;
            MT2000_Tx.mode = '2';
        }
        else if(temp==3)
        {
            item->data[0]=0x03;
            MT2000_Tx.mode = '3';
        }
        memset((void *)&item->data[1],0,3);
        break;
    }
    default :
    {
        item->cmd.inquire=0;//����
        App_printf("��������!\r\n");
        break;
    }
    }
    //֡β  ��ȡ���ֵ
    item->tail=item->head^item->cmd.inquire^item->data[0]^item->data[1]^item->data[2]^item->data[3];
    memcpy(Emission_Tx,&EmissionCmd,sizeof(EmissionCmd));
    for(int i=0; i<15; i++)   //������Ϣ
    {
        App_printf("%02x ",Emission_Tx[i]);
    }
    App_printf("\r\n");
    va_end(arg_ptr);
}

/*
 *����4 ����
 *SendCommand:��Ҫ���͵�ָ��  Len:���ͳ���  xTicksToWait����ʱ�ȴ�ʱ��
 *const uint8_t *SendCommand�������ڵ��øú�����ʱ���  &EmissionCmd ǿ��ת����uint8_t *��Ҳ���ԣ����ǻ���ʹ���������memcopy��ʹ��Emission_Tx
 *return��1 �ɹ���-1 ���������ش��� -2 ���������
 */
static int MT2000_sendCommand(const uint8_t *SendCommand,const int Len,TickType_t xTicksToWait)
{
    EventBits_t uxBits;
    memset(Emission_Rx,0,60);
    comSendBuf(COM4,(uint8_t *)SendCommand,Len);
    uxBits = xEventGroupWaitBits(
                 xCreatedEventGroup,   /* �¼���־���� */
                 BIT_0|BIT_1|BIT_2,   /* �ȴ�bit0��bit1��bit2������ */
                 pdTRUE,             /* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                 pdFALSE ,          /* ����ΪpdFALSE ��ʾ�ȴ���һ�����ö�����*/
                 xTicksToWait); 	 /* �ȴ��ӳ�ʱ�� */

    if((uxBits & BIT_0) == BIT_0)
    {
        return 1;
    }
    else if((uxBits & BIT_1) == BIT_1)
    {
        return -1;
    }
    else if((uxBits & BIT_2) == BIT_2)
    {
        return -2;
    }
    else
    {
        App_printf("\r\n TIME OUT \r\n");
        return 0;
    }
}

/* MT2000_sendCommand	���Ͳ��ҵȴ�Ӧ��
 * MT2000_Wait_Ack		ֻ�ȴ�ʵ�ʱ�־�飬MT2000_sendCommand�ļ��ݰ�
 */
int MT2000_Wait_Ack(void)
{
    EventBits_t uxBits;
    const TickType_t delay_time = 500; /* ����ӳ�500ms */

    uxBits = xEventGroupWaitBits(
                 xCreatedEventGroup,	/* �¼���־���� */
                 BIT_0|BIT_1|BIT_2,		/* �ȴ�bit0��bit1��bit2������ */
                 pdTRUE,				/* �˳�ǰbit0��bit1�������������bit0��bit1�������òű�ʾ���˳���*/
                 pdFALSE ,				/* ����ΪpdFALSE ��ʾ�ȴ���һ�����ö�����*/
                 delay_time);			/* �ȴ��ӳ�ʱ�� */

    if((uxBits & BIT_0) == BIT_0)
    {
        return 1;
    }
    else if((uxBits & BIT_1) == BIT_1)
    {
        return -1;
    }
    else if((uxBits & BIT_2) == BIT_2)
    {
        return -2;
    }
    else
    {
        App_printf("\r\n TIME OUT \r\n");
        return 0;
    }
}

/*
  * return: 0��ʾ���ݲ����ã�����1 ��ʾ�������Ƽ����� ���� -1��ʾ����������ʧ�ܣ����� -2��ʾ������������Զ�ֹͣ����
 */
static int Is_available_data(const uint8_t * Msg)
{
    volatile unsigned int j = 0;
    uint8_t check;
    for( j=0; j<46; j++)   //j+1 ����j<31
    {
        if(Msg[j]==0x7F && Msg[j+1]==0xFE)
        {
            break;
        }
    }
    if(j>=30)   //19
    {
        //�����7F FE����ô���滹����13���ֽڣ�����j���ܴ��ڵ���19�������Ļ���������ݿ϶��ǲ�ȫ���߲��Ե�
        //����ֱ�ӷ���0 ����ʧ��
        //���������һ���������ݣ�Ϊ�˷�ֹ��ʱ��ƽ�����������ɿ�ԭ��ǰ����ֵ�\0,����Ҳ����������
        //�����������ݼ�ʹ����һ��ʼ������ʵ��֡ͷ��ֻҪ�ڻ����������ڳ���Ϳ�����������
        return 0;
    }
    /*���е�����֤���ҵ���֡ͷ*/
    /*��ʱ j�±�= 0xFE*///���ˣ������±���0X7F
    if(Msg[j+14]==0xFD)
    {
        /*���е�����֤���ҵ���֡β*/

        //�������У��
        check=Msg[j];
        for(int i=j+1; i<j+13; i++)
        {
            check ^=Msg[i];
        }
        if(check==Msg[j+13])
        {
            /*���е������Ѿ�У��ͨ����*/
            j=j+2;//�ƶ��������ֽڵ�֡ͷ����,��ʱ����Ҫ������λ��
            memcpy((char *)&Respond_Cmd+2,Msg+j,11);
            memcpy((char *)&Respond_Cmd2,Msg+j+11+2,13);
            memcpy((char *)&Respond_Cmd3,Msg+j+11+2+13+2,13);
            return 1;
        }
    }
    if(Msg[j+4]==0xFD)
    {
        if(Msg[j+3]==0x6F && Msg[j+2]==0xEE)
        {
            //���ó���
            return -1;
        }
    }
    if(Msg[j+5]==0xFD)
    {
        if(Msg[j+4]==0xEF && Msg[j+3]==0xEE && Msg[j+2]==0xEE)
        {
            //������������Զ�ֹͣ����
            return -2;
        }
    }
    return 0;
}

/*
 *ɨƵ����
 */
static int Band_scan(TickType_t xTicksToWait,int index)
{
//	//�˶Σ�17�������
//	float swr_compare[17]={3.2,3.6999,4.2,4.8499,5.5,6.3499,7.2,8.3499,9.5,10.9999,12.5,14.4999,16.5,\
//	19.1499,21.8,23.95,26.1};
    //�˶�
    //float swr_compare[8]={3.6999,4.8499,6.3499,8.3499,10.9999,14.4999,19.1499,23.95};

#if defined(Eight_seg)
    float swr_compare[8]= {3.6999,4.8499,6.3499,8.3499,10.9999,14.4999,19.1499,23.95};
    int SIZE =8;
#else
    int SIZE = ceil((end-begain))+1;
    float temp_begain=begain;
#endif
    float temp_Forward_power;
    float temp_Reverse_power;
    float temp_Standing_wave_ratio;

    Pre_EmissionSend(&EmissionCmd,CHANNEL,0);//����ɨƵר���ŵ�
    if(Advance_run_command(xTicksToWait)==1)
    {
        if(Respond_Cmd.species!=0x03)   //������ǵ�Ƶ״̬�����л�����Ƶ״̬
        {
            Pre_EmissionSend(&EmissionCmd,SPECIES);//��Ƶ״̬
            if(Advance_run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
            {
                goto finish;
            }
        }
        if(Respond_Cmd.power_level!=0x01)   //������Ƕ���֮һ����
        {
            /*�Ȳ�ȡȫ������֮һ���ʵ���*/
            Pre_EmissionSend(&EmissionCmd,POWER_LEVEL,2);//�����ù��ʵȼ�Ϊ����֮һ
            if(Advance_run_command(xTicksToWait)!=1)   //ʧ�ܣ�goto finish
            {
                goto finish;
            }
        }

        for(int i=0; i<SIZE; i++)
        {
            if(temp_begain<3.2)
            {
                temp_begain=3.2;
            }
            else if(temp_begain>26.1)
            {
                temp_begain=26.1;
            }
#if defined(Eight_seg)
            Pre_EmissionSend(&EmissionCmd,FREQUENCY,swr_compare[i]);//ѡ���һƵ�ε����ֵ
#else
            Pre_EmissionSend(&EmissionCmd,FREQUENCY,temp_begain);//ѡ���һƵ�ε����ֵ
            temp_begain +=1;
#endif
            if(Advance_run_command(xTicksToWait)==1)
            {
                Pre_EmissionSend(&EmissionCmd,TUNING);//��г
                if(Advance_run_command(xTicksToWait*12)==1)   //������6s
                {
                    Pre_EmissionSend(&EmissionCmd,EMISSION);//����ָ��
                    if(Advance_run_command(xTicksToWait)!=1)
                    {
                        goto finish;
                    }
                    else     //ADC��ѹ�ɼ����ж�פ����
                    {
                        vTaskDelay(3000);//����3s֮��ʼ�ɼ�
                        for(int i=0; i<100; i++)
                        {
                            eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                            eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                            //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                            eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                            vTaskDelay(1);
                        }
                        for(int i=1; i<100; i++)   //100���������
                        {
                            eADC0_buff[0]+=eADC0_buff[i];
                            eADC1_buff[0]+=eADC1_buff[i];
                            //	eADC2_buff[0]+=eADC2_buff[i];
                            eADC3_buff[0]+=eADC3_buff[i];
                        }
                        //ȡ100��ƽ��ֵ
                        Show_Parameter.Forward_Power=eADC0_buff[0]/100;//������
                        Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//������
                        //Show_Parameter.M_45V=eADC2_buff[0]/100;//��ѹ
                        Show_Parameter.M_45I=eADC3_buff[0]/100;//����

                        /*����Ӧ��ʹ�ù��ʶ�����ʹ�õ�ѹ�ı�ֵ����Ϊ�������ʱ�ֵ�������Եģ����¿ɼ���ͬ��λפ���Ȳ�ͬ*/
                        /*����ֱ��ʹ�õ�ѹ��ֵ�ƺ�����ȷ��*/
                        if(Show_Parameter.Forward_Power>5)   //�����ʵĵ�ѹ����5Vʱ��400�Ĳ���
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//�����ʱ������ֳ�����
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //С��5Vʱ��200���Ĳ���
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//�����ʱ������ֳ�����
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        else     //С��3.2v
                        {
                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//�����ʱ������ֳ�����
                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                        }
                        Show_Parameter.Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);

                        if(GPIO_ReadInputDataBit(In_GPIO_PORT,In_GPIO_PIN)==1)   //PF0
                        {
                            App_printf("Sys_State.alarm\r\n");
                            Sys_State.alarm=1;
                            App_printf("the first alarm,needing save RTC time!!!\r\n");
                            Time_Display( RTC_GetCounter(),&set_time);
                            Alarm_historyBack.Exciter_alarm[0]=set_time.tm_year-2000;
                            Alarm_historyBack.Exciter_alarm[1]=set_time.tm_mon;
                            Alarm_historyBack.Exciter_alarm[2]=set_time.tm_mday;
                            Alarm_historyBack.Exciter_alarm[3]=set_time.tm_hour;
                            Alarm_historyBack.Exciter_alarm[4]=set_time.tm_min;
                            Alarm_historyBack.Exciter_alarm[5]=set_time.tm_sec;
                        }
                        else
                        {
                            if(Show_Parameter.Standing_wave_ratio>=2.0||Show_Parameter.Standing_wave_ratio<1)
                            {
                                //��Ҫ����ֹͣ
                                //SWR_array[i]=Show_Parameter.Standing_wave_ratio;
                            }
                            else
                            {
                                for(int count=0; count<7; count++)
                                {
                                    //��ʱ10s�ȴ������ȶ��ٲɼ�
                                    vTaskDelay(900);//����1s֮���ٴο�ʼ�ɼ�
                                    for(int i=0; i<100; i++)
                                    {
                                        eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                                        eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                                        //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                                        eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                                        vTaskDelay(1);
                                    }
                                    for(int i=1; i<100; i++)   //100���������
                                    {
                                        eADC0_buff[0]+=eADC0_buff[i];
                                        eADC1_buff[0]+=eADC1_buff[i];
                                        //	eADC2_buff[0]+=eADC2_buff[i];
                                        eADC3_buff[0]+=eADC3_buff[i];
                                    }
                                    //ȡ100��ƽ��ֵ
                                    Show_Parameter.Forward_Power=eADC0_buff[0]/100;//������
                                    Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//������
                                    //Show_Parameter.M_45V=eADC2_buff[0]/100;//��ѹ
                                    Show_Parameter.M_45I=eADC3_buff[0]/100;//����
//							Show_Parameter.M_45V=(float) ADC_ConvertedValue[2]/4096*3.3;
//							Show_Parameter.M_45I =(float) ADC_ConvertedValue[3]/4096*3.3;
                                    /*����Ӧ��ʹ�ù��ʶ�����ʹ�õ�ѹ�ı�ֵ����Ϊ�������ʱ�ֵ�������Եģ����¿ɼ���ͬ��λפ���Ȳ�ͬ*/
                                    /*����ֱ��ʹ�õ�ѹ��ֵ�ƺ�����ȷ��*/
                                    if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                                    {
                                        Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                    }
                                    else     //С�� 1.5V����ϵ��Ϊ40
                                    {
                                        Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                    }
                                    if(Show_Parameter.Forward_Power>5)   //�����ʵĵ�ѹ����5Vʱ��400�Ĳ���
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//�����ʱ������ֳ�����
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //С��5Vʱ��200���Ĳ���
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//�����ʱ������ֳ�����
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    else     //С��3.2v
                                    {
                                        Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                                        //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//�����ʱ������ֳ�����
                                        Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                    }
                                    Show_Parameter.Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);
                                    if(Show_Parameter.Standing_wave_ratio>=1.8||Show_Parameter.Standing_wave_ratio<1 || Work_paraBack.Power_45_intensity[0]>=100)
                                    {
//								z[i]=temp_Forward_power;
//								f[i]=temp_Reverse_power;
                                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                                        goto s_finish;
                                    }
                                    else
                                    {
//							 	z[i]=temp_Forward_power;
//								f[i]=temp_Reverse_power;
                                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                                    }
                                }
                                for(int count=0; count<30; count++)   //����30��
                                {
                                    Pre_EmissionSend(&EmissionCmd,Gain_plus);//�����ָ��
                                    if(run_command(xTicksToWait*2)==1)   //����ӳɹ�
                                    {
                                        //��ʱ10s�ȴ������ȶ��ٲɼ�
                                        vTaskDelay(300);//����300ms֮���ٴο�ʼ�ɼ�
                                        for(int i=0; i<100; i++)
                                        {
                                            eADC0_buff[i] =(float) ADC_ConvertedValue[0]/4096*3.3*3;
                                            eADC1_buff[i] =(float) ADC_ConvertedValue[1]/4096*3.3;
                                            //	eADC2_buff[i] =(float) ADC_ConvertedValue[2]/4096*3.3*2;
                                            eADC3_buff[i] =(float) ADC_ConvertedValue[3]/4096*3.3*2;
                                            vTaskDelay(1);
                                        }
                                        for(int i=1; i<100; i++)   //100���������
                                        {
                                            eADC0_buff[0]+=eADC0_buff[i];
                                            eADC1_buff[0]+=eADC1_buff[i];
                                            //	eADC2_buff[0]+=eADC2_buff[i];
                                            eADC3_buff[0]+=eADC3_buff[i];
                                        }
                                        //ȡ100��ƽ��ֵ
                                        Show_Parameter.Forward_Power=eADC0_buff[0]/100;//������
                                        Show_Parameter.Reverse_Power=eADC1_buff[0]/100;//������
                                        //Show_Parameter.M_45V=eADC2_buff[0]/100;//��ѹ
                                        Show_Parameter.M_45I=eADC3_buff[0]/100;//����
                                        //������ʾ
                                        if(Show_Parameter.M_45I>1.5)   //2.3-1.5V=0.8V =>��Ӧ 30A������37.5
                                        {
                                            Work_paraBack.Power_45_intensity[0]=(Show_Parameter.M_45I-1.5)*Debug_M_45I+60;
                                        }
                                        else     //С�� 1.5V����ϵ��Ϊ40
                                        {
                                            Work_paraBack.Power_45_intensity[0]=Show_Parameter.M_45I*40;
                                        }
                                        if(Show_Parameter.Forward_Power>5)   //�����ʵĵ�ѹ����5Vʱ��400�Ĳ���
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-5)*Forward_P+1000+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Forward_P;//�����ʱ������ֳ�����
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
                                        else if(Show_Parameter.Forward_Power<=5&&Show_Parameter.Forward_Power>3.2)     //С��5Vʱ��200���Ĳ���
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power-3.2)*277.8+500+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*277.8;//�����ʱ������ֳ�����
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
                                        else     //С��3.2v
                                        {
                                            temp_Forward_power=Work_paraBack.Forward_power[0];
                                            temp_Reverse_power=Work_paraBack.Reverse_power[0];
                                            Work_paraBack.Forward_power[0]=(Show_Parameter.Forward_Power)*500/3.2+20;
                                            //Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*500/3.2;//�����ʱ������ֳ�����
                                            Work_paraBack.Reverse_power[0]=Show_Parameter.Reverse_Power*Work_paraBack.Forward_power[0]/Show_Parameter.Forward_Power;
                                        }
//							temp_Standing_wave_ratio=Show_Parameter.Standing_wave_ratio ;
                                        temp_Standing_wave_ratio =get_Standing_wave_ratio(Work_paraBack.Forward_power[0],Work_paraBack.Reverse_power[0]);

                                        //	if(Work_paraBack.Forward_power[0]>1500||(fabs(temp_Forward_power-Work_paraBack.Forward_power[0])<=5)\//
                                        if(Work_paraBack.Forward_power[0]>1500 ||temp_Standing_wave_ratio>=1.8||temp_Standing_wave_ratio<1 || Work_paraBack.Power_45_intensity[0]>=100)
                                        {
                                            z[i*10+index]=temp_Forward_power;
                                            f[i*10+index]=temp_Reverse_power;
//								SWR_array[i]=temp_Standing_wave_ratio;
                                            goto s_finish;
                                        }
                                        else
                                        {
                                            z[i*10+index]=Work_paraBack.Forward_power[0];
                                            f[i*10+index]=Work_paraBack.Reverse_power[0];
//								SWR_array[i]=Show_Parameter.Standing_wave_ratio;
                                        }
                                    }
                                }
                            }
                        }
s_finish:
                        z[i*10+index]=Work_paraBack.Forward_power[0];//212
                        f[i*10+index]=Work_paraBack.Reverse_power[0];
                        SWR_array[i*10+index]=Show_Parameter.Standing_wave_ratio;
                        App_printf("Forward_power=%f \r\n",z[i*10+index]);
                        App_printf("Reverse_power=%f \r\n",f[i*10+index]);
                        App_printf("Standing_wave_ratio=%f \r\n",SWR_array[i*10+index]);
                        Pre_EmissionSend(&EmissionCmd,STOP);//ָֹͣ��
                        if(Advance_run_command(xTicksToWait)!=1)
                        {
                            goto finish;
                        }
                        else
                        {
                            if(i!=SIZE-1)
                            {
                                vTaskDelay(10000);//��ʱ10s֮�󿪻�
                            }
                            if(i<SIZE-1)
                            {
                                Sys_State.Sweeping=1;//����ɨƵ
                            }
                            else if(i==SIZE-1)
                            {
                                get_better_SWR(SWR_array,index);//��ȡ����פ���ȣ���λ������פ���Ȱ����ֶ�
                                if(index==9)   //2
                                {
                                    //judge_fre_segment();//��λפ���Ȱ����ֶ�
                                    if(InternalFlash_SaveData_2()==1)
                                    {
                                        Flash2_to_AcceptAPP();
                                        App_printf("flash save successed!\r\n");
                                        clean_sweeping=1;
                                    }
                                    else
                                    {
                                        App_printf("flash save error!\r\n");
                                        clean_sweeping=1;
                                    }
//									Sys_State.Sweeping=0;
//									Sys_State.Already_Swept=1;//ɨƵ����
                                }
                            }
                        }
                    }
                }
                else
                {
                    goto finish;
                }
            }
            else
            {
                goto finish;
            }
        }
    }
finish:


    return 0;
}


/*
 * return: 1 ������-2 ��������� ������������ڼ���������Ӧ
 */
static int run_command(TickType_t xTicksToWait)
{
    int ret=0;

    //ѭ����������Ӧ���ͱ���������
    for(int i=0; i<2; i++)
    {
#if	NEW_PROTOCOL_EN
        switch( MT2000_Tx.cmd )
        {
        case EMISSION:		//����
            MT2000_Cmd_Emit();
            ret = MT2000_Wait_Ack();
            break;

        case STOP:			//ֹͣ
            MT2000_Cmd_Stop();
            ret = MT2000_Wait_Ack();
            break;

        case CHANNEL:		//�����ŵ�
            MT2000_Cmd_Channel();
            ret = MT2000_Wait_Ack();
            break;

        case POWER_ADD:		//��������(΢��)
            MT2000_Cmd_Power_Add();
            ret = MT2000_Wait_Ack();
            break;

        case POWER_SUB:		//���ʼ�С(΢��)
            MT2000_Cmd_Power_Sub();
            ret = MT2000_Wait_Ack();
            break;

        case INQUIRE:		//��ѯ
            MT2000_Cmd_Inquire();
            ret = MT2000_Wait_Ack();
            break;

        case TUNING:		//��г
            MT2000_Cmd_Tune();
            ret = MT2000_Wait_Ack();
            break;

            /* ���¼�����������		û����Ӧ���ֻ�������ŵ���ʱ��������� */
        case FREQUENCY:		//Ƶ��-------------------------------------------
            ret = 1;
            break;

        case SPECIES:		//��������---------------------------------------
            ret = 1;
            break;

        case POWER_LEVEL:	//���ʵȼ�---------------------------------------
            ret = 1;
            break;

        case MODE:			//����ģʽ---------------------------------------
            ret = 1;
            break;
        }
#else
        ret=MT2000_sendCommand(Emission_Tx,sizeof(EmissionCmd),xTicksToWait);
#endif

        if(ret==1)
        {
            //����ʱ��Ҫȡ����ʷ�ı�����־
            Sys_State.no_respond=0;
            Alarm_historyBack.alarm_history[0] &=~(1<<7);
            //Sys_State.alarm=0;���������Ӧ����ȡ�������Ǽ���������״̬������PCȡ��
            /*���Ӧ�ò���ӡ*/
            //App_printf("//��ѯ goto finish;\r\n");
            return ret;
        }
        if(ret==-2)
        {
            if(Sys_State.alarm==0)
            {
                Sys_State.alarm=1;
                App_printf("the first alarm,needing save RTC time!!!\r\n");
                Time_Display( RTC_GetCounter(),&set_time);
                Alarm_historyBack.Exciter_alarm[0]=set_time.tm_year-2000;
                Alarm_historyBack.Exciter_alarm[1]=set_time.tm_mon;
                Alarm_historyBack.Exciter_alarm[2]=set_time.tm_mday;
                Alarm_historyBack.Exciter_alarm[3]=set_time.tm_hour;
                Alarm_historyBack.Exciter_alarm[4]=set_time.tm_min;
                Alarm_historyBack.Exciter_alarm[5]=set_time.tm_sec;
                /*������Ҫ��ȡ���Ӧ��RTCʱ�䣬Ȼ��ֵ��0602�������Ӧ���ֶ�*/
                /*������Ҫ��ȡ���Ӧ��RTCʱ�䣬Ȼ��ֵ��0602�������Ӧ���ֶ�*/
                /*������Ҫ��ȡ���Ӧ��RTCʱ�䣬Ȼ��ֵ��0602�������Ӧ���ֶ�*/
            }
            App_printf("alarm alarm alarm!!!\r\n");
            return ret;
        }
        if(ret==-1 || ret==0)
        {
            if(i==2)
            {
//					Sys_State.no_respond=1;
//					App_printf("Sys_State.no_respond =%d\r\n",Sys_State.no_respond);
//					return ret;
                break;
            }
        }
    }

    if(Sys_State.no_respond==0)
    {
        Sys_State.no_respond=1;
        App_printf("the first no_respond,needing save RTC time!!!\r\n");
        Time_Display( RTC_GetCounter(),&set_time);
        Alarm_historyBack.No_response_time[0]=set_time.tm_year-2000;
        Alarm_historyBack.No_response_time[1]=set_time.tm_mon;
        Alarm_historyBack.No_response_time[2]=set_time.tm_mday;
        Alarm_historyBack.No_response_time[3]=set_time.tm_hour;
        Alarm_historyBack.No_response_time[4]=set_time.tm_min;
        Alarm_historyBack.No_response_time[5]=set_time.tm_sec;
    }
//	Sys_State.no_respond=1;
//	App_printf("Sys_State.no_respond =%d\r\n",Sys_State.no_respond);
    return ret;		//Ϊ�˲�ʹ��������������˵�ģʽ����ǰ�������return 0��û�������
//	return 0;	//��������£�������ִ�е�����
}

/*
 *run_command�����ļ�ǿ�棬��Ҫ��Ϊ���ܹ���ɨƵ��ʱ���һ���жϣ���Ѹ���Ƴ�ɨ��
 */
int Advance_run_command(TickType_t xTicksToWait)
{
    int ret=0;
    if(Sys_State.Cancel_Sweeping!=1)   //PCû��ǿ��ȡ��ɨƵ
    {
        ret=run_command(xTicksToWait);
    }
    return ret;
}
/*����1��ʾ����ʷ������Ϣ*/
int get_history_alm(void)
{
    volatile uint8_t* check=&Sys_State.SWR_alarm;
    for(int i=0; i<9; i++)
    {
        if(*check++==1)
        {
            Sys_State.History_alarm=1;//���ֻ����PC���
            return 1;
        }

    }
    return 0;
//	if(Sys_State.alarm==1 || Sys_State.no_respond==1)
//	{
//		//����Ҫ����ADC�ɼ������ݽ����жϣ�������ʱ�Ȳ�д
//		//����Ҫ����ADC�ɼ������ݽ����жϣ�������ʱ�Ȳ�д
//		//����Ҫ����ADC�ɼ������ݽ����жϣ�������ʱ�Ȳ�д
//		Sys_State.History_alarm=1;//���ֻ����PC���
//		return 1;
//	}
//	return 0;
}

/*����1��ʾ��ǰ�б�����Ϣ*/
int get_current_alm(void)
{
    if(Sys_State.alarm==1 || Sys_State.no_respond==1)
    {
        /*�о������if�жϼ�������ص�Ӧ�ò�ʹ�ã���ֻʹ��AD�ɼ���*/
        /*�о������if�жϼ�������ص�Ӧ�ò�ʹ�ã���ֻʹ��AD�ɼ���*/
        /*�о������if�жϼ�������ص�Ӧ�ò�ʹ�ã���ֻʹ��AD�ɼ���*/
        return 1;
    }
    return 0;
}

void clean_all_flags(void)
{
    Sys_State.History_alarm=0;//��ʷ������־

    Sys_State.alarm=0;//�����������

    Sys_State.no_respond=0;//����������Ӧ����

    Sys_State.SWR_alarm=0;//פ���ȱ���

    Sys_State.low_Voltage=0;//Ƿѹ����

    Sys_State.over_Voltage=0;//��ѹ����

    Sys_State.over_Electric=0;//��������

    Sys_State.humidity_alarm=0;//ʪ�ȱ���

    Sys_State.temperature_alarm=0;//�¶ȱ���

    Sys_State.No_power_level=0;//�޹����������

//	Alarm_historyBack.alarm_history[0]=0;//����
//	Alarm_historyBack.alarm_history[1]=0;
    //��ʷ������ȫ����������
    memset(&Alarm_historyBack,0,sizeof(Alarm_historyBack));

}

void clean_alarm_bit_flags(void)
{
    Alarm_historyBack.alarm_history[0]=DisalarmBack.current_alarm_state[0];
    Alarm_historyBack.alarm_history[1]=DisalarmBack.current_alarm_state[1];
    if((Alarm_historyBack.alarm_history[0]&(1<<0))==0)
    {
        Sys_State.No_power_level=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<1))==0)
    {
        Sys_State.temperature_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<2))==0)
    {
        Sys_State.humidity_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<3))==0)
    {
        Sys_State.over_Electric=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<4))==0)
    {
        Sys_State.over_Voltage=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<5))==0)
    {
        Sys_State.low_Voltage=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<6))==0)
    {
        Sys_State.SWR_alarm=0;
    }
    if((Alarm_historyBack.alarm_history[0]&(1<<7))==0)
    {
        Sys_State.no_respond=0;
    }
    if((Alarm_historyBack.alarm_history[1]&(1<<0))==0)
    {
        Sys_State.alarm=0;
    }
}
/*��ȡפ���Ⱥ���������פ����
 *Forward_Power:������(��ʵ�ǵ�ѹֵ)   Reverse_Power��������
 */
float get_Standing_wave_ratio(float Forward_Power,float Reverse_Power)
{
    float temp;
    if(Forward_Power==0 || Forward_Power==Reverse_Power)
    {
        return -1;
    }
    temp=(float)sqrt(Reverse_Power/Forward_Power);
    return (1+temp)/(1-temp);
}
/*
 *��ȡ���פ����,��λ�������פ����Ƶ�Σ����������Ƶ�θ���
 */
static int get_better_SWR(float *array,int index)
{
    //float *temp=array;
    //float min=0;
    int min_count=0;
#if defined(Eight_seg)
    int SIZE =8;
#else
    int SIZE = ceil((end-begain))+1;
#endif
    App_printf("SIZE:%d\r\n",SIZE);
    App_printf("swr:");
    for(int i=0; i<SIZE; i++)
    {
        App_printf("%f ",array[i*10+index]);
    }
    App_printf("\r\n\r\n");

    App_printf("Forward_Power:");
    for(int i=0; i<SIZE; i++)
    {
        App_printf("%f ",z[i*10+index]);
        if(z[i*10+index]<1100)
        {
            z[i*10+index]=0;
        }
        else if(z[i*10+index]>=1550)
        {
            z[i*10+index]=1550;
        }

    }
    App_printf("\r\n\r\n");

//App_printf("Reverse_Power:");
//	for(int i=0;i<SIZE;i++)
//	{
//		App_printf("%f ",f[i]);
//	}
//	App_printf("\r\n\r\n");

    return min_count;
}
/*
 *�ж����Ƶ���ǵڼ���
 */
//static int judge_fre_segment(void)
//{
//	return 0;
//}

/*
 *Save_count:ʵ��Ϊflash3_Save��������ͼ�洢��flash����
 */
int find_hard_control(uint8_t Save_count)
{
    //Hard_control.usage_diagram_count=0;//��������
    for(int i=0; i<Save_count; i++)
    {
        //for(int j=0;j<10;j++)
        //{
        //��һ��
        if(State_monitor[i].start[0]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time1[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time1[0]*60+Run_Diagram_buff[i].Start_Time1[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep1[0]*60+Run_Diagram_buff[i].End_Timep1[1])))
            {
                //	if(set_time.tm_min== Run_Diagram_buff[i].Start_Time1[1]||(set_time.tm_hour*60+set_time.tm_min)\
                //										<(Run_Diagram_buff[i].End_Timep1[0]*60+Run_Diagram_buff[i].End_Timep1[1]))
                //	{
                App_printf("\r\nthe 1 start");
                State_monitor[i].start[0]=1;
                State_monitor[i].end[0]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power1[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power1[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq1,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power1[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq1,4);
                }
                //}
            }

        }

        if(State_monitor[i].end[0]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep1[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep1[1])
                {
                    App_printf("\r\nthe 1  end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[0]=1;
                    State_monitor[i].start[0]=0;//�����־
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq1,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq1,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }
        //�ڶ���
        if(State_monitor[i].start[1]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time2[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time2[0]*60+Run_Diagram_buff[i].Start_Time2[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep2[0]*60+Run_Diagram_buff[i].End_Timep2[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time2[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep2[0]*60+Run_Diagram_buff[i].End_Timep2[1]))
//					{
                App_printf("\r\nthe 2 start");
                State_monitor[i].start[1]=1;
                State_monitor[i].end[1]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power2[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power2[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq2,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power2[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq2,4);
                }
                //	}
            }

        }

        if(State_monitor[i].end[1]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep2[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep2[1])
                {
                    App_printf("\r\nthe 2 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[1]=1;
                    State_monitor[i].start[1]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq2,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq2,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }
        //������
        if(State_monitor[i].start[2]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time3[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time3[0]*60+Run_Diagram_buff[i].Start_Time3[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep3[0]*60+Run_Diagram_buff[i].End_Timep3[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time3[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep3[0]*60+Run_Diagram_buff[i].End_Timep3[1]))
//					{
                App_printf("\r\nthe 3");
                State_monitor[i].start[2]=1;
                State_monitor[i].end[2]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power3[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power3[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq3,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power3[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq3,4);
                }
                //	}
            }

        }
        if(State_monitor[i].end[2]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep3[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep3[1])
                {
                    App_printf("\r\nthe 3 end\r\n");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[2]=1;
                    State_monitor[i].start[2]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq3,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq3,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }


        }
        //���ĸ�
        if(State_monitor[i].start[3]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time4[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time4[0]*60+Run_Diagram_buff[i].Start_Time4[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep4[0]*60+Run_Diagram_buff[i].End_Timep4[1])))
            {
//				if(set_time.tm_min== Run_Diagram_buff[i].Start_Time4[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep4[0]*60+Run_Diagram_buff[i].End_Timep4[1]))
//				{
                App_printf("\r\nthe 4");
                State_monitor[i].start[3]=1;
                State_monitor[i].end[3]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power4[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power4[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq4,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power4[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq4,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[3]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep4[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep4[1])
                {
                    App_printf("\r\nthe 4 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[3]=1;
                    State_monitor[i].start[3]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq4,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq4,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }
        //�����
        if(State_monitor[i].start[4]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time5[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time5[0]*60+Run_Diagram_buff[i].Start_Time5[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep5[0]*60+Run_Diagram_buff[i].End_Timep5[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time5[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep5[0]*60+Run_Diagram_buff[i].End_Timep5[1]))
//					{
                App_printf("\r\nthe 5");
                State_monitor[i].start[4]=1;
                State_monitor[i].end[4]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power5[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power5[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq5,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power5[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq5,4);
                }
                //	}
            }
        }
        if(State_monitor[i].end[4]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].Start_Time5[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep5[1])
                {
                    App_printf("\r\nthe 5 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[4]=1;
                    State_monitor[i].start[4]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq5,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq5,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }

        //������
        if(State_monitor[i].start[5]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time6[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time6[0]*60+Run_Diagram_buff[i].Start_Time6[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep6[0]*60+Run_Diagram_buff[i].End_Timep6[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time6[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep6[0]*60+Run_Diagram_buff[i].End_Timep6[1]))
//					{
                App_printf("\r\nthe 6");
                State_monitor[i].start[5]=1;
                State_monitor[i].end[5]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power6[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power6[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq6,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power6[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq6,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[5]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep6[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep6[1])
                {
                    App_printf("\r\nthe 6 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[5]=1;
                    State_monitor[i].start[5]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq6,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq6,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }

        //���߸�
        if(State_monitor[i].start[6]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time7[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time7[0]*60+Run_Diagram_buff[i].Start_Time7[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep7[0]*60+Run_Diagram_buff[i].End_Timep7[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time7[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep7[0]*60+Run_Diagram_buff[i].End_Timep7[1]))
//					{
                App_printf("\r\nthe 7");
                State_monitor[i].start[6]=1;
                State_monitor[i].end[6]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power7[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power7[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq7,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power7[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq7,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[6]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep7[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep7[1])
                {
                    App_printf("\r\nthe 7 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[6]=1;
                    State_monitor[i].start[6]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq7,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq7,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }
        //�ڰ˸�
        if(State_monitor[i].start[7]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time8[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time8[0]*60+Run_Diagram_buff[i].Start_Time8[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep8[0]*60+Run_Diagram_buff[i].End_Timep8[1])))
            {
//						if(set_time.tm_min== Run_Diagram_buff[i].Start_Time8[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep8[0]*60+Run_Diagram_buff[i].End_Timep8[1]))
//						{
                App_printf("\r\nthe 8");
                State_monitor[i].start[7]=1;
                State_monitor[i].end[7]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power8[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power8[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq8,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power8[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq8,4);
                }
                //}
            }
        }
        if(State_monitor[i].end[7]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep8[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep8[1])
                {
                    App_printf("\r\nthe 8 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[7]=1;
                    State_monitor[i].start[7]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq8,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq8,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }

        //�ھŸ�
        if(State_monitor[i].start[8]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time9[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time9[0]*60+Run_Diagram_buff[i].Start_Time9[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep9[0]*60+Run_Diagram_buff[i].End_Timep9[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time9[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep9[0]*60+Run_Diagram_buff[i].End_Timep9[1]))
//					{
                App_printf("\r\nthe 9");
                State_monitor[i].start[8]=1;
                State_monitor[i].end[8]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power9[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power9[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq9,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power9[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq9,4);
                }
                //	}
            }
        }

        if(State_monitor[i].end[8]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep9[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep9[1])
                {
                    App_printf("\r\nthe 9 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[8]=1;
                    State_monitor[i].start[8]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq9,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq9,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }

        //��ʮ��
        if(State_monitor[i].start[9]!=1)
        {
            if(Run_Diagram_buff[i].Start_Time10[0]!=0xFF&&((set_time.tm_hour*60+set_time.tm_min)>=(Run_Diagram_buff[i].Start_Time10[0]*60+Run_Diagram_buff[i].Start_Time10[1]))\
                    &&((set_time.tm_hour*60+set_time.tm_min)<(Run_Diagram_buff[i].End_Timep10[0]*60+Run_Diagram_buff[i].End_Timep10[1])))
            {
//					if(set_time.tm_min== Run_Diagram_buff[i].Start_Time10[1]||(set_time.tm_hour*60+set_time.tm_min)\
//														<(Run_Diagram_buff[i].End_Timep10[0]*60+Run_Diagram_buff[i].End_Timep10[1]))
//					{
                App_printf("\r\nthe 10 ");
                State_monitor[i].start[9]=1;
                State_monitor[i].end[9]=0;
                Hard_control.usage_diagram_count++;
                if(Hard_control.usage_diagram_count==1)
                {
                    Trans_open_Copy.Type[0]=1;
                    Trans_open_Copy.Channel[0]=1;
                    Trans_open_Copy.Power_grade[0]=Run_Diagram_buff[i].Power10[0];
                    memcpy(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4);
                }
                else if(Hard_control.usage_diagram_count==2)
                {
                    Trans_open_Copy.Type[0]=2;
                    Trans_open_Copy.Channel[1]=2;
                    Trans_open_Copy.Power_grade[1]=Run_Diagram_buff[i].Power10[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq10,4);
                }
                else if(Hard_control.usage_diagram_count==3)
                {
                    Trans_open_Copy.Type[0]=3;
                    Trans_open_Copy.Channel[2]=3;
                    Trans_open_Copy.Power_grade[2]=Run_Diagram_buff[i].Power10[0];
                    memcpy((uint8_t *)(Trans_open_Copy.Freq)+8,Run_Diagram_buff[i].Frq10,4);
                }
                //	}
            }
        }
        if(State_monitor[i].end[9]!=1)   //����
        {
            if(set_time.tm_hour== Run_Diagram_buff[i].End_Timep10[0])
            {
                if(set_time.tm_min== Run_Diagram_buff[i].End_Timep10[1])
                {
                    App_printf("\r\nthe 10 end");
                    Hard_control.usage_diagram_count--;
                    State_monitor[i].end[9]=1;
                    State_monitor[i].start[9]=0;
                    if(Trans_open_Copy.Type[0]==1)
                    {
                        Trans_open_Copy.Type[0]=0;
                        //��Ƶʱ��ʱ�䵽��ֹͣ����
                        memset(Trans_open_Copy.Freq,0,12);//����
                    }
                    if(Trans_open_Copy.Type[0]==2)
                    {
                        Trans_open_Copy.Type[0]=1;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                        }
                        //������߳��ڶ���������Ҫ��������
                    }
                    if(Trans_open_Copy.Type[0]==3)
                    {
                        Trans_open_Copy.Type[0]=2;
                        if(memcmp(Trans_open_Copy.Freq,Run_Diagram_buff[i].Frq10,4)==0)   //�߳���һ��
                        {
                            Trans_open_Copy.Freq[0]=Trans_open_Copy.Freq[4];
                            Trans_open_Copy.Freq[1]=Trans_open_Copy.Freq[5];
                            Trans_open_Copy.Freq[2]=Trans_open_Copy.Freq[6];
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        else if(memcmp((uint8_t *)(Trans_open_Copy.Freq)+4,Run_Diagram_buff[i].Frq10,4)==0)     //�߳��ڶ���
                        {
                            //���1byte��Ϊ0�����Բ�����
                            Trans_open_Copy.Freq[4]=Trans_open_Copy.Freq[8];
                            Trans_open_Copy.Freq[5]=Trans_open_Copy.Freq[9];
                            Trans_open_Copy.Freq[6]=Trans_open_Copy.Freq[10];
                        }
                        //������߳�������������Ҫ��������
                    }
                }
            }
        }
        //}

    }
    return 0;
}

/*�ж��Ƿ���Ҫ����*/
void juge_isOpen(uint8_t Save_count)
{
    uint32_t min;//����
    min=set_time.tm_hour*60+set_time.tm_min;
    for(int i=0; i<Save_count; i++)
    {
        for(int j=0; j<10; j++)
        {
            if(Sys_State.Sys_open==1)
            {
                Monitor.need_open=0;
                return ;
            }
            if(Run_Diagram_buff[i].Start_Time1[0]*60+Run_Diagram_buff[i].Start_Time1[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time2[0]*60+Run_Diagram_buff[i].Start_Time2[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }

            if(Run_Diagram_buff[i].Start_Time3[0]*60+Run_Diagram_buff[i].Start_Time3[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time4[0]*60+Run_Diagram_buff[i].Start_Time4[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time5[0]*60+Run_Diagram_buff[i].Start_Time5[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time6[0]*60+Run_Diagram_buff[i].Start_Time6[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time7[0]*60+Run_Diagram_buff[i].Start_Time7[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time8[0]*60+Run_Diagram_buff[i].Start_Time8[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time9[0]*60+Run_Diagram_buff[i].Start_Time9[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }
            if(Run_Diagram_buff[i].Start_Time10[0]*60+Run_Diagram_buff[i].Start_Time10[1]-min<=10)
            {
                /*��Ҫ����*/
                Monitor.need_open=1;
            }

        }
    }
}

