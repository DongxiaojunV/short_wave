#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "bsp_adc.h"
#include "alarm.h"
#include "power.h"
#include "W5500.h"
#include "main.h"

extern uint8_t g_fuc_cod[2];			//ȫ�ֹ�����

extern float Scan_Begin;			//��ΧɨƵ��ʼ��
extern float Scan_End;				//��ΧɨƵ������

Buffer_t RxBuf_FromPC;

TaskHandle_t xHandleTask_Upper_Computer;

/* ��ѯָ��0201��ֱ�ӷ��Ͳ����أ������Ķ���֪ͨTask_MT2000���ͣ��ȴ�PC��0201��ѯ��� */
void Task_Upper_Computer(void *pvParameters)
{
    uint8_t	  j = 0;				//ʧ������������ͼʱʹ��
    uint8_t		k = 0;					//��ͨforѭ��	(ֹͣ�͹ػ��ȴ�)

    int			count =0;
    int			ret=0;

    static		uint8_t task_upper_inquire_counter = 0;

    Alarm.no_respond_locate = 0;
    Alarm.no_respond_count = 0;

	
    //δ��ʼ�������������
    if( System.already_init == 0x00 )
    {
        System.status = SYSTEM_UNINITIALIZE;
    }
    else
    {		
			Gt_Inquire_All();
			ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);

        if( ret == MT2000_ACK_OK )
        {
            if( System.Voltage > 40.0 )
            {
                System.open = 0x02;

                if( System.emission == 0x01 )
                {
                    System.status = SYSTEM_EMISSING;
                }
                else
                {
                    System.status = SYSTEM_STANDBY;
                }			
            }
        }
        else
        {
            if( get_history_alarm() == 0x01 )
            {
                System.status = SYSTEM_ALARM;
            }

            Alarm.no_respond_locate = 15;
        }

        if( System.already_init == 0 )
        {
            System.status = SYSTEM_UNINITIALIZE;
        }
    }

    while(1)
    {
        update_status_without_inquire();		//ֻ����״̬������ѯ
        count = W5500_rx_buf_len;
        W5500_rx_buf_len = 0;

        if(count>2)
        {
            RxBuf_FromPC.len = count;//�������ݳ���
            ret=CAN_data_analyze(g_fuc_cod, &RxBuf_FromPC);
            count = 0;

            if(ret==1)
            {
                Monitor.no_respond_count=0;	//����ͨ��������
                Monitor.hard_control=0;
                Monitor.need_emit = 0;
                Monitor.need_open = 0;
                Monitor.need_close = 0;

                Run_Diagram_data.mode = 0;	//��ʼ��
                Run_Diagram_data.power[0] = 0;
                Run_Diagram_data.power[1] = 0;
                Run_Diagram_data.power[2] = 0;
                memset(Run_Diagram_data.Freq, 0, sizeof(Run_Diagram_data.Freq));

                if(Monitor.hard_control==1)
                {
                    Monitor.usage_diagram_count=0;
                    System.mode = 0;		//�����㣬�ٲ�ѯ��ֵ

                    for(int i=0; i<flash3_Save; i++)
                    {
                        for(j=0; j<10; j++)
                        {
                            Monitor.start[i][j] = 0;
                            Monitor.end[i][j] = 0;
                        }
                    }
                }

                func_code_printf();

                /* ���ݹ����룬ִ����Ӧ���� */
                if(g_fuc_cod[0]==0x01&&g_fuc_cod[1]==0x01)				//��ʼ��
                {
										System.Init_Mark=1;
                    System.CAN_ID[0] = Alarm_threshold.Transmitte_id[0];	//����flash

                    Alarm_backPC.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x01;
                    g_fuc_codToPC[1]=0x02;

                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    if(InternalFlash_SaveData_1(0x01)==0)		/* д��flash */
                    {
                        System.already_init = 0;
                        flash_1_once_flag = 0;

                        Alarm_backPC.Alarm_paramet[0] = 0xFC;		//��ʼ��ʧ��
                        Alarm_backPC.Alarm_paramet[1] = 0x00;
                    }
                    else
                    {
                        System.already_init = 1;
                        flash_1_once_flag = 1;

                        Alarm_backPC.Alarm_paramet[0] = 0xFE;	//�ɹ���ʼ����������flash
                        Alarm_backPC.Alarm_paramet[1] = 0x00;
                        Flash_to_AcceptAPP();

                        if( System.Voltage > 40.0 )
                        {
                            System.open = 0x02;
                        }
                        else
                        {
                            System.open = 0x00;
                        }
						
												if( get_history_alarm() == 0x01 )
												{
													System.status = SYSTEM_ALARM;
												}
												else	if( System.open == 0x02 )
												{
													System.status = SYSTEM_STANDBY;		//��ѯʱ�������ˣ���һ���Ǵ������п����Ƿ���/��������/ɨƵ�ȡ�
												}
												else
												{
													System.status = SYSTEM_SHUTDOWN;
												}
                    }
										
                    Send_PC(g_fuc_codToPC);
                }
                else if(g_fuc_cod[0]==0x02&&g_fuc_cod[1]==0x01)			//��ѯ
                {
                    g_fuc_codToPC[0]=0x02;
                    g_fuc_codToPC[1]=0x02;			
				 	          g_stamp_distance = g_inquire_stamp - RTC_GetCounter();	//��ȡ����ʱ���������ʱ��Ĳ�ֵ

                    System.time_update_flag = 0x01;//������ʱ���־λ

                    //Work_paraBack����
                    Work_paraBack.Transmitte_id[0]=System.CAN_ID[0];	//CAN_ID
                    Work_paraBack.Mode[0]=0;
                    memset(Work_paraBack.Freq,0,12);
                    Work_paraBack.Power_45_voltage[0]=0;
                    Work_paraBack.Power_45_intensity[0]=0;
                    memset(Work_paraBack.Freq,0,12);			//Ƶ�������㣬��ֵ
                    Work_paraBack.Forward_power[0] = 0.0;		//���������㣬��ֵ
                    Work_paraBack.Reverse_power[0] = 0.0;
                    Work_paraBack.emission[0]=0x00;
									
										memset(Work_paraBack.Alarm_Reservation,0,2);
										memset(Work_paraBack.System_Version,0,20);
										Work_paraBack.Divider[0]=0xFF;
									  Work_paraBack.Control_Model[0]=0;
									  Work_paraBack.Frequency_band_value[0]=0;
										Work_paraBack.Attenuation[0]=0;
										memset(Work_paraBack.Reservation,0,2);																
										memset(Work_paraBack.Bj_Electricity_1,0,4);
										memset(Work_paraBack.Bj_Electricity_2,0,4);
										memset(Work_paraBack.Bj_Electricity_3,0,4);
										memset(Work_paraBack.Bj_Electricity_4,0,4);
										Work_paraBack.Amplifier_Temperature_1[0]=0;
										Work_paraBack.Amplifier_Temperature_2[0]=0;
										Work_paraBack.Amplifier_Temperature_3[0]=0;
										Work_paraBack.Amplifier_Temperature_4[0]=0;
										Work_paraBack.System_Version[16]=System.Model[0];		//����Ĭ��ֵ�����ı�
									
                    Work_paraBack.Channel[0] = PC_Cmd.channel[0];	//������ŵ�		(�ɻ�����3�ŵ����»���ֻ��1���ŵ������ݣ�ֱ��ԭ������)
                    Work_paraBack.Channel[1] = PC_Cmd.channel[1];
                    Work_paraBack.Channel[2] = PC_Cmd.channel[2];

                    Work_paraBack.Power_grade[0] = PC_Cmd.power[0];	//����Ĺ��ʵȼ�	(�ɻ�����3���ʵȼ����»���ֻ��1�����ʵȼ������ݣ�ֱ��ԭ������)
                    Work_paraBack.Power_grade[1] = PC_Cmd.power[1];
                    Work_paraBack.Power_grade[2] = PC_Cmd.power[2];
										
										Work_paraBack.Control_Model[0]=System.Control_Model;   //0�Զ�/1�ֶ�
										Work_paraBack.Frequency_band_value[0]=System.Frequency_band_value;   //Ƶ��ֵ
										Work_paraBack.Attenuation[0]=System.Attenuation;  //˥��ֵ
										
										for(uint8_t n=0;n<4;n++)
										{
											Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);  
											Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
											Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
											Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
										}
										Work_paraBack.Amplifier_Temperature_1[0]=Convert_byte_order_16(System.Amplifier_Temperature[0]);
										Work_paraBack.Amplifier_Temperature_2[0]=Convert_byte_order_16(System.Amplifier_Temperature[1]);
										Work_paraBack.Amplifier_Temperature_3[0]=Convert_byte_order_16(System.Amplifier_Temperature[2]);
										Work_paraBack.Amplifier_Temperature_4[0]=Convert_byte_order_16(System.Amplifier_Temperature[3]);
										
                    set_alarm_bit();  //��λ������־λ 
                    Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];
                    Work_paraBack.current_alarm_state[1] = Alarm_historyBack.alarm_history[1];

                    Work_paraBack.temp[0]=Temperature_Humidity.Temperature;		//�¶�
                    Work_paraBack.humidity[0]=Temperature_Humidity.Humidity;	//ʪ��


                    if(System.status == SYSTEM_UNINITIALIZE )			//δ��ʼ��״̬
                    {
                        Work_paraBack.Trans_current_state[0] = SYSTEM_UNINITIALIZE;
                        Work_paraBack.Power_45_voltage[0]	 = System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	 = 0;
												Work_paraBack.Amplifier_Temperature_1[0]=Convert_byte_order_16(System.Amplifier_Temperature[0]);
												Work_paraBack.Amplifier_Temperature_2[0]=Convert_byte_order_16(System.Amplifier_Temperature[1]);
												Work_paraBack.Amplifier_Temperature_3[0]=Convert_byte_order_16(System.Amplifier_Temperature[2]);
												Work_paraBack.Amplifier_Temperature_4[0]=Convert_byte_order_16(System.Amplifier_Temperature[3]);
			
										    for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" ���������δ��ʼ��״̬ \n");
                    }
                    else	if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) ||(Alarm.power_cataclysm==1) )	//����������������Ӧ�𱨾����޹��ʱ���
                    {
                        //����������������Ӧ�𱨾����޹��ʱ�������������������ر�������ѯʱ���Ϸ��أ������������ȷ��ز���ʧ�ܣ��ص�����/����/����״̬���ȴ���ѯʱ�ٱ���

                        Work_paraBack.Trans_current_state[0]=SYSTEM_ALARM;			//��ǰ�Ǳ���״̬
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//ģʽ

//                        freq_str_to_hex(System.freq1,	Work_paraBack.Freq);		//Ƶ��
//											  freq_str_to_hex(System.freq2,	Work_paraBack.Freq+4);
//                        freq_str_to_hex(System.freq3,	Work_paraBack.Freq+8);
											  StrToHex(Work_paraBack.Freq, System.freq1, 4); //remarks : ���ַ���ת��Ϊ16������
                        StrToHex(Work_paraBack.Freq+4, System.freq2, 4); //remarks : ���ַ���ת��Ϊ16������
											  StrToHex(Work_paraBack.Freq+8, System.freq3, 4); //remarks : ���ַ���ת��Ϊ16������

                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
												for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
                        Work_paraBack.emission[0] = (System.emission&0x01)|(System.Open_Close&0x02<<1);		//������

                        if( System.emission == 0x01 )
                        {
                            System.stop = 1;
                            xTaskNotify(xHandleTask_MT2000,				//ֹͣ����
                                        BIT_0,
                                        eSetBits);
                        }
												Trans_printf(" ��������ڽ�������״̬��ֹͣ����  \n");
                    }
                    else	if( System.status == SYSTEM_OPENING )		//���ڿ���
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_OPENING;
											  Trans_printf(" ������������ڿ���  \n");
                    }
                    else	if( System.status == SYSTEM_SCAN )			//ɨƵ
                    {
												if( (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )		//ɨƵʱ����ѯ������פ���ȱ����͹�������(�ٴ�����)
												{
													Alarm.swr_alarm = 0x00;
													Alarm.over_Electric = 0x00;
												}
						
                        Work_paraBack.Trans_current_state[0]=SYSTEM_SCAN;
                        Work_paraBack.Mode[0]=0x01;
                        Work_paraBack.emission[0]=(System.emission&0x01)|(System.Open_Close&0x02<<1);
												
												
											GT2000_Tx_freq_Hex_to_PChex(GT2000_Tx.Gt2000_freq1, Work_paraBack.Freq);  //��ѯ����ʮ����������ת��Ϊ��λ����ʮ������
												
                        Trans_printf(" Task_upper��ѯɨƵ״̬������ GT2000_Tx.Gt2000_freq1= %02X %02X %02X %02X \n",\
												GT2000_Tx.Gt2000_freq1[0],GT2000_Tx.Gt2000_freq1[1],GT2000_Tx.Gt2000_freq1[2],GT2000_Tx.Gt2000_freq1[3]);
												Trans_printf(" Task_upper��ѯɨƵ״̬������ Work_paraBack.Freq[1-4]= %02X %02X %02X %02X \n",\
												Work_paraBack.Freq[0],Work_paraBack.Freq[1],Work_paraBack.Freq[2],Work_paraBack.Freq[3]);
												
                        Work_paraBack.Power_grade[0] = 180;		//ɨƵĬ�Ϲ��ʵȼ�-8����
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;
                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" ���������ɨƵ״̬  \n");
                    }
                    else	if( System.status == SYSTEM_SHUTDOWN )		//�ػ�
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_SHUTDOWN;
											  Trans_printf(" ��������ڹػ�״̬  \n");
                    }
   
                    else	if( System.status == SYSTEM_STANDBY )		//����״̬
                    {
                        Work_paraBack.Trans_current_state[0] = System.status;
                        Work_paraBack.Power_45_voltage[0]	 = System.Voltage;
                        Work_paraBack.Power_45_intensity[0]  = 0.0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" ��������ڴ���״̬  \n");
                    }
										
                    else	if( System.status == SYSTEM_EMISSING )		//����״̬
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_EMISSING;
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//ģʽ

											freq_GT2000Hex_to_PChex(System.freq1, Work_paraBack.Freq);  //��ѯ����ʮ����������ת��Ϊ��λ����ʮ������
											freq_GT2000Hex_to_PChex(System.freq2, Work_paraBack.Freq+4);
											freq_GT2000Hex_to_PChex(System.freq3, Work_paraBack.Freq+8);
//											memcpy(Work_paraBack.Freq, System.freq1, 4);   //��ֵ��ѯ����Ƶ�ʸ���λ����System.freq1��System_Status_Update�и��¡�
//											  memcpy(Work_paraBack.Freq+4, System.freq2, 4);
//											  memcpy(Work_paraBack.Freq+8, System.freq3, 4);
									    Trans_printf(" ��ѯ������״̬Work_paraBack.Mode[0]=  %d \nWork_paraBack.Freq=  ",Work_paraBack.Mode[0]);
											for(k=0;k<12;k++)
											{
									    Trans_printf("%02x ",Work_paraBack.Freq[k]);
                      }
											Trans_printf(" \n  System.freq1= ");
											for(k=0;k<4;k++)
											{
											Trans_printf("%02x ",System.freq1[k]);
											}
											Trans_printf(" \n");
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;
											Trans_printf("%02x ",System.freq1[k]);
											

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
												for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
												Trans_printf(" Work_paraBack.Bj_Electricity_1_4== ");
												for(uint8_t i=0;i<4;i++)
												{
													Trans_printf(" %x ",Work_paraBack.Bj_Electricity_1[i]);  //��ӡ������Ϣ
												}
												Trans_printf(" \n ");
                        Work_paraBack.emission[0]=(System.emission&0x01)|(System.Open_Close&0x02<<1);
												Trans_printf(" ��������ڷ���״̬  \n");
                    }
                    else	if( System.status == SYSTEM_ALARM )			//����״̬
                    {
                        Work_paraBack.Trans_current_state[0]=SYSTEM_ALARM;			//��ǰ�Ǳ���״̬
                        Work_paraBack.Mode[0] = System.INQUIRE_Mode;					//ģʽ

                        freq_GT2000Hex_to_PChex(System.freq1, Work_paraBack.Freq);  //��ѯ����ʮ����������ת��Ϊ��λ����ʮ������
										  	freq_GT2000Hex_to_PChex(System.freq2, Work_paraBack.Freq+4);
										  	freq_GT2000Hex_to_PChex(System.freq3, Work_paraBack.Freq+8); 
//											  memcpy(Work_paraBack.Freq, System.freq1, 4);   //��ֵ��ѯ����Ƶ�ʸ���λ����System.freq1��System_Status_Update�и��¡�
//											  memcpy(Work_paraBack.Freq+4, System.freq2, 4);
//											  memcpy(Work_paraBack.Freq+8, System.freq3, 4);
											Trans_printf(" ��ѯ������״̬״̬Work_paraBack.Freq=  ");
											for(k=0;k<12;k++)
											{
									    Trans_printf(" %x  \n",Work_paraBack.Freq[k]);
                      }
											Trans_printf(" System.freq1= %x  ",System.freq1);
											
                        Work_paraBack.Forward_power[0] = (float)System.Forward_Power;
                        Work_paraBack.Reverse_power[0] = (float)System.Reverse_Power;

                        Work_paraBack.Power_45_voltage[0]	= System.Voltage;
                        Work_paraBack.Power_45_intensity[0]	= 0;
											  for(uint8_t n=0;n<4;n++)
												{
													Work_paraBack.Bj_Electricity_1[n]=Convert_byte_order_16(System.Bj_Electricity_1[n]);
													Work_paraBack.Bj_Electricity_2[n]=Convert_byte_order_16(System.Bj_Electricity_2[n]);
													Work_paraBack.Bj_Electricity_3[n]=Convert_byte_order_16(System.Bj_Electricity_3[n]);
													Work_paraBack.Bj_Electricity_4[n]=Convert_byte_order_16(System.Bj_Electricity_4[n]);
												}
                        Work_paraBack.emission[0] = (System.emission&0x01)|(System.Open_Close&0x02<<1);
												Trans_printf(" ��������ڱ���״̬  \n");
                    }
                    else												//����
                    {
                        //App_printf("PC inquire when System.status = ???\r\n");
											Trans_printf(" δ��ȡ�������״̬��������  \n");
                    }
										
                    Send_PC(g_fuc_codToPC);
										
                }
                else	if(g_fuc_cod[0]==0x03&&g_fuc_cod[1]==0x01)		//����
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );  //ʱ��� -> ����ʱ�䲢��ӡ

                    Trans_openBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x03;
                    g_fuc_codToPC[1]=0x02;

                    if( System.open != 0x02 )					//�Ƿ񿪻�//������־λ	0:�ػ�	1:���ڿ���	2:�Ѿ�����
                    {
											Trans_openBack.Trans_state[0]=0xFB;  //0xFB��ʾû�п���
                    }
                    else	if( System.emission == 0x01 )		//�Ƿ����ڷ���
										{
											Trans_openBack.Trans_state[0]=0xFC;  //0xFC��ʾ����ʧ��
										}
										else										//����
                    {
                        Trans_openBack.Trans_state[0]=0;	//����	����BJ2000_Open()�������յ������Ӧ�����Trans_openBack.Trans_state[0]��

                        /* �����жϣ�������ȷ��ֵ��MT2000_Tx��ͨ��MT2000_Cmd_Channel()��MT2000_Cmd_Tune()��ɵ�Ƶ��г�����MT2000_Cmd_Emit()���� */
                        switch( PC_Cmd.mode )
                        {
													case 0x01:	GT2000_Tx.Gt2000_mode = 1;					break;  //��ֵmode��GT2000_TxΪ��ȡ������λ���·�������
													case 0x02:	GT2000_Tx.Gt2000_mode = 2;					break;
													case 0x03:	GT2000_Tx.Gt2000_mode = 3;					break;
													default:	Trans_openBack.Trans_state[0]=0xFE;		break;	//��������
                        }
												
											  if(PC_Cmd.power[0]<100&&PC_Cmd.power[0]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//��������
												}
												if(PC_Cmd.power[1]<100&&PC_Cmd.power[1]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//��������
												}
												if(PC_Cmd.power[2]<100&&PC_Cmd.power[2]>=250)
												{
													Trans_openBack.Trans_state[0]=0xFE;		//��������
												}        
													
                        if( GT2000_Tx.Gt2000_mode > 0 )
                        {  //�ж��·�Ƶ�ʱ�����3.2~21.85
                            if( (PC_Cmd.freq[0]>0x21 && PC_Cmd.freq[1]>0x85) || (PC_Cmd.freq[0]>=0x22) ||
                                (PC_Cmd.freq[0]<0x03 && PC_Cmd.freq[1]<0x20) || (PC_Cmd.freq[0]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//��������
                        }

                        if( GT2000_Tx.Gt2000_mode > 1 )
                        {
                            if( (PC_Cmd.freq[4]>0x21 && PC_Cmd.freq[5]>0x85) || (PC_Cmd.freq[4]>=0x22) ||
                                (PC_Cmd.freq[4]<0x03 && PC_Cmd.freq[5]<0x20) || (PC_Cmd.freq[4]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//��������
                        }

                        if( GT2000_Tx.Gt2000_mode > 2 )
                        {
                            if( (PC_Cmd.freq[8]>0x21 && PC_Cmd.freq[9]>0x85) || (PC_Cmd.freq[8]>=0x22) ||
                                    (PC_Cmd.freq[8]<0x03 && PC_Cmd.freq[9]<0x20) || (PC_Cmd.freq[8]<=0x02) )
                                Trans_openBack.Trans_state[0]=0xFE;		//��������
                        }

                        if( (PC_Cmd.freq[3]!=0x00) || (PC_Cmd.freq[7]!=0x00) || (PC_Cmd.freq[11]!=0x00) )
                            Trans_openBack.Trans_state[0]=0xFE;		//��������

                        if(Trans_openBack.Trans_state[0]==0xFE)		//��������
                        {
                            Trans_printf(" PC�·��������� ");//�ᵽ��󣬷��ظ�PC
                        }
                        else	//������ȷ
                        {
                            /*-----------------------------��ȡPC���õ����ݣ��洢��MT2000_Tx-----------------------------------*/
                            switch( PC_Cmd.mode )	//�������࣬��Ƶ/˫Ƶ/��Ƶ
                            {
																case 0x01:	GT2000_Tx.Gt2000_mode = 1;	break;  //��PC�·��Ĺ���ģʽ�洢��GT2000_Tx��
																case 0x02:	GT2000_Tx.Gt2000_mode = 2;	break;
																case 0x03:	GT2000_Tx.Gt2000_mode = 3;	break;
																default:	break;
                            }

                            GT2000_Tx.method = 'F';		//F
														Trans_printf(" PC�·��Ĺ���PC_Cmd.power= %d %d %d \n",PC_Cmd.power[0],PC_Cmd.power[1],PC_Cmd.power[2]);
                            Power_Range(&GT2000_Tx,PC_Cmd.power);   //��PC�·��ķ���ֵת��Ϊ���ʣ����洢��GT2000_Tx��
                            Trans_printf(" ��ȡ����power= %x %x \n",GT2000_Tx.Frequency_power[0],GT2000_Tx.Frequency_power[1]);
                           
														/* ��ֵƵ�� */ 
                            freq_hex_to_str(PC_Cmd.freq, 	GT2000_Tx.Gt2000_freq1);   //�ڴ˴���PC�·���Ƶ�����ݸ�ֵ��Tx
                            freq_hex_to_str(PC_Cmd.freq+4,	GT2000_Tx.Gt2000_freq2);    //16����ת��Ϊ�ַ���
                            freq_hex_to_str(PC_Cmd.freq+8,	GT2000_Tx.Gt2000_freq3);
														
//														Trans_printf(" ת���ɵ�str1Ϊfreq1= %s \n",GT2000_Tx.Gt2000_freq1);
//														Trans_printf(" ת���ɵ�str2Ϊfreq2= %s \n",GT2000_Tx.Gt2000_freq2);
//														Trans_printf(" ת���ɵ�str3Ϊfreq3= %s \n",GT2000_Tx.Gt2000_freq3);
														freq_PChex_to_GT2000(PC_Cmd.freq,GT2000_Tx.Gt2000_freq1);  //���ַ�����ʽ������ת��Ϊʮ�����Ƹ�ֵ��Gt2000_freq1
														freq_PChex_to_GT2000(PC_Cmd.freq+4,GT2000_Tx.Gt2000_freq2);  //���ַ�����ʽ��ת��Ϊʮ������
														freq_PChex_to_GT2000(PC_Cmd.freq+8,GT2000_Tx.Gt2000_freq3);  //���ַ�����ʽ��ת��Ϊʮ������
											
														
														Trans_printf(" PCƵ��ת��Ϊ�����Ƶ�ʺ�GT2000_Tx.Gt2000_freq1= %02x %02x freq2=%02x %02x freq3=%02x %02x \n",GT2000_Tx.Gt2000_freq1[0],\
														GT2000_Tx.Gt2000_freq1[1],GT2000_Tx.Gt2000_freq2[0],GT2000_Tx.Gt2000_freq2[1],GT2000_Tx.Gt2000_freq3[0],GT2000_Tx.Gt2000_freq3[1]);

														
                            if( (System.emission == 0x01) || (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                            {  //���ڷ��䣬����Ӧ���޹��ʣ�פ���ȱ��������������������ط���ʧ��
																Trans_openBack.Trans_state[0]=0xFC;		//����ʧ��
															  Trans_printf(" ���ڷ��䣬����Ӧ���޹��ʣ�פ���ȱ��������������������ط���ʧ�� ");
                            }
                            else
                            {
                                System.stop = 0x00;
                                System.cancel_sweeping = 0x00;
                                Trans_openBack.Trans_state[0]=0xFD;		//��ʾ���յ���ָ�������Ҫʱ��ȥ��Ƶ���ȴ��ٴβ�ѯ
                                xTaskNotify(xHandleTask_MT2000,			//����
                                            BIT_3,             
                                            eSetBits);  //�����¼�3֪ͨTask_MT2000.c�����䡣
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x04&&g_fuc_cod[1]==0x01)		//ֹͣ����
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    Trans_stopBack.Transmitte_id[0] = System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x04;
                    g_fuc_codToPC[1]=0x02;

                    //û������ֹͣ��ָ����ԣ�������Ѿ�ֹͣ�ˣ���ֱ�ӷ��أ�����ȴ�ֹͣ����ٷ���
                    if( System.emission != 0x01 )
                    {
                        Trans_stopBack.Trans_state[0]=0xFC;	//�Ѿ���ֹͣ״̬

                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        System_Status_Clean();
											Trans_printf(" ֹͣ����__�Ѿ���ֹͣ״̬ ");
                    }
                    else
                    {
                        System.stop = 0x01;

                        //App_printf("Task_Upper_Computer Stop Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		//ֹͣ����
                                    BIT_0,
                                    eSetBits);

                        Trans_stopBack.Trans_state[0]=0x02;//�ȴ�ǰ����λΪֹͣʧ��
                        for(k=0; k<40; k++)
                        {
                            if( System.emission==0x00 )	//ֹͣ���䣬�˳��ȴ�
                                break;
                            else
                                vTaskDelay(50);
                        }
												Trans_printf(" ֹͣ����__����BIT_0 ");
                    }
										
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x05&&g_fuc_cod[1]==0x01)		//�������
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    //App_printf("PC DisAlarm\r\n");
                    DisalarmBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x05;
                    g_fuc_codToPC[1]=0x02;

										emission_alarm_content_clean();		//�����������������������
					
                    GT2000_Buffer_Clean();
                    memset(DisalarmBack.current_alarm_state, 0, 2);
                    if(get_history_alarm()==0)	//û�б���
                    {
                        DisalarmBack.Disalarm_result[0]=0xFD;
                    }
                    else     					//�б���
                    {
											DisalarmBack.Disalarm_result[0]=0xFE;	//��������ɹ�
											
											if(Alarm.emission==1)		//���й��ŷ��صı���		�������ԣ�
											{
												Gt2000_Reset();		//��λ����
												Trans_printf(" ���Ÿ�λ \n");
											}
											
											clean_all_alarm_flags();				//������б�����־λ

											GT2000_Buffer_Clean();					//������������bj2000��Buffer
                    }

                    if( System.already_init == 0x00 )			//���㱨������ˣ�Ҳ��һ���Ǵ������߹ػ�״̬���п�����δ��ʼ��ʱ���ľ�
                    {
                        System.status = SYSTEM_UNINITIALIZE;
                    }
                    else	if( System.Voltage > 40.0 )
                    {
                        System.open = 0x02;

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else	if( (System.emission == 0x01) && (System.Forward_Power>100.0) )
                        {
                            if( System.sweep == 0x01 )				//����ɨƵ(ɨƵʱ�󱨾������������ɨƵ��־λ)
                                System.status = SYSTEM_SCAN;
                            else
                                System.status = SYSTEM_EMISSING;	//�������ɨƵ�����书�ʻ���Ϊ0��ֻ���Ƿ���״̬
                        }
                        else	if( System.status == SYSTEM_EMISSING )
                        {

                        }
                        else
                        {
                            System.emission = 0x00;   //δ����
                            System.status = SYSTEM_STANDBY;			//��ѯʱ�������ˣ���һ���Ǵ������п����Ƿ���/��������/ɨƵ�ȡ�
                        }
                    }
                    else
                    {
                        System.open = 0x00;

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_SHUTDOWN;
                        }
                    }

                    Alarm.no_respond_locate = 0;
                    Alarm.no_respond_count = 0;
            
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x06&&g_fuc_cod[1]==0x01)		//������ѯ
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.Transmitte_id[0]=	System.CAN_ID[0];
										Alarm_historyBack.Alarm_reservation[0]=0xFF;
								 	  Alarm_historyBack.Alarm_reservation[1]=0xFF;//��ֵԤ��λλ0xff����ʱ���ָ�����
                    g_fuc_codToPC[0]=0x06;
                    g_fuc_codToPC[1]=0x02;
                    set_alarm_bit();
                    Send_PC(g_fuc_codToPC);

                    //ֱ�ӷ��أ�����ѯ������
                }
                else	if(g_fuc_cod[0]==0x07&&g_fuc_cod[1]==0x01)		//���������
                {
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );

                    Power_onBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x07;
                    g_fuc_codToPC[1]=0x02;

					          if( System.open == 0x02 )			//�Ѿ��������ٷ�����ָ��(�����˲�һ���Ǵ���״̬�����ǿ���ָ���Ѿ������ˣ�һ���Ǵ�������ֲ���Ҫ)
                    {
                        Power_onBack.results[0]=0xFC;
                        //App_printf("system open alreadly 1!\n");
                    }
                    else	if( System.open == 0x00 )	//�ػ��ˣ���Ҫִ�п���
                    {
                        //���������ʱ���жϿ������Power_onBack.results[0]������Ϊ�ػ�֮����Ҫ�����������Ͽ���
                        //(����Ӳ���ӹ�Task_Hardware_Monitor()��Task_MT2000()�Ĺػ�������ֻ��BJ2000_Open()��BJ2000_Close()�޸�Power_onBack.results[0]��ֵ)
                        if( Power_onBack.results[0] == 0xFD )
                        {
                            Power_onBack.results[0] = 0x00;						
														System.status = SYSTEM_SHUTDOWN;						
                        }
                        else	if( System.Voltage > 40.0 )
                        {
                            System.open = 0x02;
                            Power_onBack.results[0]=0xFC;
														System.status = SYSTEM_STANDBY;                          
                        }
                        else
                        {
                            System.open = 0x01;
                            System.status = SYSTEM_OPENING;	//����ϵͳ״̬Ϊ���ڿ���״̬(��ѯʱ���о�)
                            Power_onBack.results[0]=0xFE;	//���ڿ���(������Ӧ��֮�����£����ʧ����ֵΪ0xFD)
                            xTaskNotify(xHandleTask_MT2000,	//����
                                        BIT_8,
                                        eSetBits);
                        }
                    }
                    else	if( System.open == 0x01 )	//���ڿ���ʱ���ַ��Ϳ���ָ��
                    {
                        System.status = SYSTEM_OPENING;	//���ڿ���״̬(�϶��ǿ���״̬���ٸ�ֵ��³����)
                        Power_onBack.results[0]=0xFE;
                    }

										if( get_history_alarm() == 0x01 )				//����
										{
											System.status = SYSTEM_ALARM;
										}
						
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x08&&g_fuc_cod[1]==0x01)		//������ػ�
                {
                    Power_offBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x08;
                    g_fuc_codToPC[1]=0x02;

                    if( System.emission == 0x01 )
                    {
                        Power_offBack.results[0]=0xFD;				//��Ҫ�����ػ�ʧ��
                    }
                    else	if( System.status == SYSTEM_SHUTDOWN )	//�Ѿ��ػ�
                    {
                        System.open = 0x00;
												System.Voltage = 0.0;
												System.Electricity = 0.0;
						
                        Power_offBack.results[0]=0xFC;	//�Ѿ��ػ�
                        //App_printf("system already closed!\n");
                    }
                    else	//ִ�йػ�
                    {
                        System.close = 1;
												
                        //App_printf("Task_Upper_Computer Close Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,	//�ػ�
                                    BIT_9,
                                    eSetBits);

                        for(k=0; k<20; k++)
                        {
														 if( System.close == 0 )			//�ػ��ɹ�
														{
															System.status = SYSTEM_SHUTDOWN;
															System.sweep = 0x00;
															
															System.open = 0x00;
															System.Voltage = 0.0;
															System.Electricity = 0.0;
															
															System.achieve_add_sub = 0;
															System.modify_power = 0;
															System.cancel_add_sub = 0;
															System_Status_Clean();
																							break;
														}
                            else
                                vTaskDelay(50);
                        }
                    }
                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x09&&g_fuc_cod[1]==0x01)		//ɨƵ
                {
                    Scan_FrqBack.Transmitte_id[0]	= System.CAN_ID[0];
                    Scan_FrqBack1.Transmitte_id[0]	= System.CAN_ID[0];	//3.2-4.9		
                    Scan_FrqBack2.Transmitte_id[0]	= System.CAN_ID[0];	//5.0-6.9
                    Scan_FrqBack3.Transmitte_id[0]	= System.CAN_ID[0];	//7.0-8.9
                    Scan_FrqBack4.Transmitte_id[0]	= System.CAN_ID[0];	//9.0-10.9
                    Scan_FrqBack5.Transmitte_id[0]	= System.CAN_ID[0];	//11.0-12.9
                    Scan_FrqBack6.Transmitte_id[0]	= System.CAN_ID[0];	//13.0-14.9
                    Scan_FrqBack7.Transmitte_id[0]	= System.CAN_ID[0];	//15.0-16.9
                    Scan_FrqBack8.Transmitte_id[0]	= System.CAN_ID[0];	//17.0-18.9
                    Scan_FrqBack9.Transmitte_id[0]	= System.CAN_ID[0];	//19.0-20.9
                    Scan_FrqBack10.Transmitte_id[0]	= System.CAN_ID[0];	//21.0-22.9
                    Scan_FrqBack11.Transmitte_id[0]	= System.CAN_ID[0];	//23.0-24.9
                    Scan_FrqBack12.Transmitte_id[0]	= System.CAN_ID[0];	//25.0-26.1

                    g_fuc_codToPC[0]=0x09;
                    g_fuc_codToPC[1]=0x02;

                    if(Scan_Frq.state[0]==0xFE)					//��ʼɨƵ
                    {
												/*��ΧɨƵ��ֵ*/
												Scan_Begin=(Scan_Frq.Fre_Band[0])*10/10.0;
												Scan_End=(Scan_Frq.Fre_Band[1])*10/10.0;											
												/**************/
                        if( (System.emission == 0x01) || (System.status == SYSTEM_SHUTDOWN) || (System.status == SYSTEM_OPENING)|| \
							(Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                        {
                            //App_printf("Task_MT2000_Sweep waring: working or alarm state!\r\n");

                            Scan_FrqBack.results[0]=0xFD;			//ɨƵʧ��
                            System.Scan_Freq_flag=0;
                        }
                        else	//�������ڹ�����û�б���
                        {
                            //App_printf("Scan execute!\r\n");

                            if( System.open == 0x02 )				//����״̬
                            {
                                if( System.emission == 0x01 )		//����״̬
                                {
                                    Scan_FrqBack.results[0]=0xFC;	//��ǰ���ڷ���״̬
                                }
                                else	if( System.sweep == 0x01 )	//����ɨƵ�����·�ɨƵ
                                {
                                    Scan_FrqBack.results[0]=0xFE;	//����ɨƵ
                                    System.status = SYSTEM_SCAN;
                                }
                                else     //��һ��ɨƵ
                                {
                                    System.sweep = 0x01;			//����ɨƵ
                                    System.modify_power = 0x01;
                                    Scan_FrqBack.results[0]=0xFE;

                                    System.stop = 0x00;
                                    System.cancel_sweeping = 0x00;
                                    System.status = SYSTEM_SCAN;

                                    //App_printf("Task_Upper_Computer Sweep Notify!\r\n");
                                    xTaskNotify(xHandleTask_MT2000,	/* ɨƵ */
                                                BIT_5,
                                                eSetBits);
                                }

                                System.Scan_Freq_flag=0;
                            }
                            else	//�ػ�״̬����Ҫ����
                            {
                                Scan_FrqBack.results[0]=0xFF;
                                System.Scan_Freq_flag=0;
                            }
                        }
                    }
                    else if(Scan_Frq.state[0]==0xFD)			//��ѯ���Ƶ��
                    {
                        //App_printf("Scan inquire!\r\n");

                        if( System.sweep == 0x01 )				//����ɨƵ
                        {
                            System.status = SYSTEM_SCAN;
                            Scan_FrqBack.results[0]=0xFE;

                            System.Scan_Freq_flag=0;
                        }
                        else	/*	if( System.already_swept == 1 )	//����ʷ���Ƶ��	*/		//û�����ɨƵҲ����	--20181014 By Luonus
                        {
                            if( System.sweep == 0x01 )			//����ɨƵ��˵����Ҫ�������Ƶ��
                            {
                                System.status = SYSTEM_SCAN;
                                Scan_FrqBack.results[0]=0xFE;	//����ɨƵ

                                System.Scan_Freq_flag=0;
                            }
                            else	//ֱ�ӷ��ظ�PC
                            {
                                Scan_FrqBack.results[0]=0xFB;	//�Ѿ���ȡ���Ƶ��
                                if(Scan_Frq.Fre_Band[0]==5)
                                {
                                    System.Scan_Freq_flag=2;
                                    Scan_FrqBack2.results[0]=0xFB;
                                    memcpy(Scan_FrqBack2.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==7)
                                {
                                    System.Scan_Freq_flag=3;
                                    Scan_FrqBack3.results[0]=0xFB;
                                    memcpy(Scan_FrqBack3.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==9)
                                {
                                    System.Scan_Freq_flag=4;
                                    Scan_FrqBack4.results[0]=0xFB;
                                    memcpy(Scan_FrqBack4.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==11)
                                {
                                    System.Scan_Freq_flag=5;
                                    Scan_FrqBack5.results[0]=0xFB;
                                    memcpy(Scan_FrqBack5.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==13)
                                {
                                    System.Scan_Freq_flag=6;
                                    Scan_FrqBack6.results[0]=0xFB;
                                    memcpy(Scan_FrqBack6.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==15)
                                {
                                    System.Scan_Freq_flag=7;
                                    Scan_FrqBack7.results[0]=0xFB;
                                    memcpy(Scan_FrqBack7.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==17)
                                {
                                    System.Scan_Freq_flag=8;
                                    Scan_FrqBack8.results[0]=0xFB;
                                    memcpy(Scan_FrqBack8.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==19)
                                {
                                    System.Scan_Freq_flag=9;
                                    Scan_FrqBack9.results[0]=0xFB;
                                    memcpy(Scan_FrqBack9.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==21)
                                {
                                    System.Scan_Freq_flag=10;
                                    Scan_FrqBack10.results[0]=0xFB;
                                    memcpy(Scan_FrqBack10.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==23)
                                {
                                    System.Scan_Freq_flag=11;
                                    Scan_FrqBack11.results[0]=0xFB;
                                    memcpy(Scan_FrqBack11.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                                else	if(Scan_Frq.Fre_Band[0]==25)
                                {
                                    System.Scan_Freq_flag=12;
                                    Scan_FrqBack12.results[0]=0xFB;
                                    memcpy(Scan_FrqBack12.Fre_Band,Scan_Frq.Fre_Band,8);
                                }
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);		//���˲�ѯ���������Ƿ��� Scan_FrqBack������(����System.Scan_Freq_flag=0;)
                }
                else	if(g_fuc_cod[0]==0x0A&&g_fuc_cod[1]==0x01)		//ֹͣɨƵ
                {
                    Sacn_stopBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0A;
                    g_fuc_codToPC[1]=0x02;

                    if(Sacn_stop.state[0]==0XFE)   //ֹͣɨƵ
                    {
                        if(System.sweep == 0x01)   //����ɨƵ
                        {
                            System.stop = 0x01;

                            System.cancel_sweeping = 0x01;
                            Sacn_stopBack.result[0]=0xFB;	//����ֹͣ
                        }
                        else     //����ɨƵ״̬
                        {
                            Sacn_stopBack.result[0]=0xFA;	//û��ɨƵ������ֹͣ
                        }
                    }
                    else	if(Sacn_stop.state[0]==0XFD)	//��ѯֹͣɨƵ�Ƿ�ɹ�
                    {
                        if(System.sweep == 0x01)			//����ɨƵ
                        {
													System.stop = 0x01;
                            System.cancel_sweeping = 0x01;
                            Sacn_stopBack.result[0]=0xFB;	//����ֹͣ
                        }
                        else	if( System.sweep != 1 )		//û��ɨƵ
                        {
                            if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )   //����Ӧ������������
                            {
                                Sacn_stopBack.result[0]=0xFC;//ֹͣʧ��
                            }
                            else
                            {
                                System.sweep = 0;				//����
                                Sacn_stopBack.result[0]=0xFE;	//ֹͣ�ɹ�
                            }
                        }
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0B&&g_fuc_cod[1]==0x01)		//��������
                {
                    Add_PowerBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0B;
                    g_fuc_codToPC[1]=0x02;

                    if( (System.open != 0x02) || (System.emission != 0x01) )			//û�п������߲��Ƿ���״̬
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        Add_PowerBack.results[0]=0xFC;		//���ӹ���ʧ��
                    }
                    else	//ִ�����ӹ���
                    {
												System.Power_Adjustment=1;
                        if( System.modify_power == 0x01 )
                        {
                            System.cancel_add_sub = 1;
                        }
                        else
                        {
                            System.cancel_add_sub = 0;
                            System.modify_power = 0x01;
                        }

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_EMISSING;
                        }

                        System.stop = 0x00;
                        System.cancel_sweeping = 0x00;
                        Add_PowerBack.results[0]=0xFE;		//���ӳɹ�

                        //App_printf("Task_Upper_Computer Add Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		/* ���ӹ��� */
                                    BIT_1,
                                    eSetBits);
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0C&&g_fuc_cod[1]==0x01)		//���ʼ�С
                {
                    Sub_PowerBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0C;
                    g_fuc_codToPC[1]=0x02;

                    if( (System.open != 0x02) || (System.emission != 0x01) )			//û�п������߲��Ƿ���״̬
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        Sub_PowerBack.results[0]=0xFC;		//��С����ʧ��
                    }
                    else
                    {
												System.Power_Adjustment=1;
                        if( System.modify_power == 0x01 )
                        {
                            System.cancel_add_sub = 1;
                        }
                        else
                        {
                            System.cancel_add_sub = 0;
                            System.modify_power = 0x01;
                        }

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_EMISSING;
                        }

                        System.stop = 0x00;
                        System.cancel_sweeping = 0x00;
                        Sub_PowerBack.results[0]=0xFE;		//��С�ɹ�

                        //App_printf("Task_Upper_Computer Sub Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,	/* ��С���� */
                                    BIT_2,
                                    eSetBits);
                    }

                    Send_PC(g_fuc_codToPC);
                }
                else	if(g_fuc_cod[0]==0x0D && g_fuc_cod[1]==0x01)	//����ͼ
                {
                    Run_DiagramBack.Transmitte_id[0]=System.CAN_ID[0];
                    g_fuc_codToPC[0]=0x0D;
                    g_fuc_codToPC[1]=0x02;

                    if(Run_Diagram.Continue[1]==0xFC)   //����ͼ�������
                    {
                        if(InternalFlash_SaveData_3(Run_Diagram.Continue[0])==0)		//�����ʱ���ѵ�ַ����λ��λ����ȡʱ����ȡ����ֵΪflash_3_once_flag
                        {
                            flash_3_once_flag=0;
                            Run_DiagramBack.results[0]=0xFD;
                        }
                        else
                        {
                            Flash3_to_AcceptAPP();		//��ȡ����ͼ��־λ������
                        }
                    }
                    else
                    {
                        flash_3_once_flag=0;
                    }

                    switch(Run_Diagram.Continue[0])
                    {
						case 1:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_1,WRITE_END_ADDR_1,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 2:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_2,WRITE_END_ADDR_2,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 3:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_3,WRITE_END_ADDR_3,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 4:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_4,WRITE_END_ADDR_4,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 5:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_5,WRITE_END_ADDR_5,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 6:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_6,WRITE_END_ADDR_6,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 7:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_7,WRITE_END_ADDR_7,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 8:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_8,WRITE_END_ADDR_8,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 9:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_9,WRITE_END_ADDR_9,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						case 10:
							if(Flash_Save_Run_Diagram(WRITE_START_ADDR_10,WRITE_END_ADDR_10,Run_Diagram.count[0])==0)
							{
								Run_DiagramBack.results[0]=0xFD;
							}
							else
							{
								Run_DiagramBack.results[0]=0xFE;
							}
							break;
						default:
							Run_DiagramBack.results[0]=0xFD;
							break;
                    }
				
					if(flash_3_once_flag==1)
					{
						if(flash3_Save<=10&&flash3_Save>0)
						{
							judg_read_flash(flash3_Save);		//������ͼ������Run_Diagram_buf[n]������պ���δ��������ͼ��flash
							//App_printf("receive run diagram success!\r\n");
						}
					}
							
                    Send_PC(g_fuc_codToPC);
                }
            }
            else	if(ret==2)		//���ݲ�����������05 02��ͷ������03 04��β
            {
                //App_printf("It is not a valid CAN data!\r\n");
            }
            else	if(ret==3)		//�����ɹ������Ƿ����system.CAN_ID���󣬷���δ��ʼ��
            {
                //App_printf("PC//inquire\r\n");
                g_fuc_codToPC[0]=0x02;
                g_fuc_codToPC[1]=0x02;

                Work_paraBack.Trans_current_state[0] = SYSTEM_UNINITIALIZE;

                //Work_paraBack����
                Work_paraBack.Mode[0]=0;
                memset(Work_paraBack.Freq,0,12);
                Work_paraBack.Power_45_voltage[0]=0;
                Work_paraBack.Power_45_intensity[0]=0;
                memset(Work_paraBack.Freq,0,12);			//Ƶ�������㣬��ֵ
                Work_paraBack.Forward_power[0] = 0.0;		//���������㣬��ֵ
                Work_paraBack.Reverse_power[0] = 0.0;
                Work_paraBack.emission[0]=0x00;

                Work_paraBack.Channel[0] = PC_Cmd.channel[0];	//������ź�		(�ɻ�����3�ŵ����»���ֻ��1���ŵ������ݣ�ֱ��ԭ������)
                Work_paraBack.Channel[1] = PC_Cmd.channel[1];
                Work_paraBack.Channel[2] = PC_Cmd.channel[2];

                Work_paraBack.Power_grade[0] = PC_Cmd.power[0];	//����Ĺ��ʵȼ�	(�ɻ�����3���ʵȼ����»���ֻ��1�����ʵȼ������ݣ�ֱ��ԭ������)
                Work_paraBack.Power_grade[1] = PC_Cmd.power[1];
                Work_paraBack.Power_grade[2] = PC_Cmd.power[2];

                set_alarm_bit();
                Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];
                Work_paraBack.current_alarm_state[0] = Alarm_historyBack.alarm_history[0];

                Work_paraBack.temp[0]=Temperature_Humidity.Temperature;		//�¶�
                Work_paraBack.humidity[0]=Temperature_Humidity.Humidity;	//ʪ��

                Send_PC(g_fuc_codToPC);			//Data_Assemble() �����CAN_ID �޸�Ϊ Work_paraBack.Trans_current_state[0] --20181031 Luonus

                //App_printf("system.CAN_ID math faiule!\r\n");
            }
            else	if(ret==4)		//û�й�����
            {
                //App_printf("no have fuction code!\r\n!\r\n");
            }
            else	if(ret==5)		//CRCУ�����
            {
                //App_printf("CRC fault!\r\n");
            }
        }
        else
        {
            count=0;
        }

        if( Monitor.hard_control == 0x00 )	//Ӳ���ӹܵ����в�ѯ������֪ͨ
        {
            if( System.already_init == 0x00 )
            {
                System.status = SYSTEM_UNINITIALIZE;
            }
            else
            {
                task_upper_inquire_counter++;
                if( task_upper_inquire_counter > 10 )
                {
                    //���ڿ�������Ƶ��ɨƵ����������	����ѯ
                    if( (System.status==SYSTEM_OPENING) || (System.status==SYSTEM_SCAN) || (System.modify_power==0x01) )
                    {
                        task_upper_inquire_counter = 10;
                    }
                    else
                    {
                        task_upper_inquire_counter = 0;

                        //App_printf("Task_Upper_Computer Inquire(1s) Notify!\r\n");
                        xTaskNotify(xHandleTask_MT2000,		//��ѯ
                                    BIT_10,
                                    eSetBits);
                    }
                }
            }
        }

        vTaskDelay(50);
    }
}

/*ת��16bit���ֽ��� ��λ���λ����*/
uint16_t Convert_byte_order_16(uint16_t tni2)
{
	tni2=(((tni2>>8)&0xff) | ((tni2&0xff)<<8));
	return tni2;
}
