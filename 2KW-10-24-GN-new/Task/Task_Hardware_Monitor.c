#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "NTP.h"
#include "alarm.h"

TaskHandle_t xHandleTask_Hardware_Monitor;
uint8_t Updata_time_flag=0;		//�ϵ�ͬ��һ��ʱ���־
void power_check(void);

void Task_Hardware_Monitor(void *pvParameters)
{
    uint8_t i = 0;
    static uint8_t inquire_counter = 0;
    int ret;
    uint8_t temp_freq_str[10] = "00000000";

    Run_Diagram_data.mode = '0';	//��ʼ��
    Run_Diagram_data.power[0] = '0';
    Run_Diagram_data.power[1] = '0';
    Run_Diagram_data.power[2] = '0';
    memset(Run_Diagram_data.Freq, 0, sizeof(Run_Diagram_data.Freq));

    Monitor.hard_control = 0;
    Monitor.no_respond_count = 0;

    Monitor.need_open = 0;
    Monitor.need_close = 0;
    Monitor.need_emit = 0;

    while(1)
    {
#if	1
			if(Monitor.no_respond_count==60||Updata_time_flag==1)	//����λ��ͨ���쳣,����Ӳ���ӹ�
			{
				Updata_time_flag=2;		//�ϵ�ֻ��һ��
				if(Hardware_Time_Count>=Hardware_Time)		//1����ͬ��һ��ʱ��
				{
					NTP_Pack_Send();
					Hardware_Time_Count=0;
				}
			}
        if( (Monitor.hard_control==1) && (flash_3_once_flag==1) )	//Ӳ���ӹ�
        {
            PC_Cmd.mode = Run_Diagram_data.mode - '0';				//����ͼ����������µ�	PC_Cmd
            memcpy(PC_Cmd.freq, Run_Diagram_data.Freq, 12);
            PC_Cmd.power[0] = Run_Diagram_data.power[0];
            PC_Cmd.power[1] = Run_Diagram_data.power[1];
            PC_Cmd.power[2] = Run_Diagram_data.power[2];
					
						Power_Range(&GT2000_Tx,PC_Cmd.power);		//ת��Ϊ��������
					
            PC_Cmd.channel[0] = 0x01;
            PC_Cmd.channel[1] = 0x01;
            PC_Cmd.channel[2] = 0x01;

            if( Monitor.need_open == 0x01 )									//��Ҫ����
            {
                if( System.open == 0x02 )		//�Ѿ�����
                {               
                    Monitor.need_open = 0;
                }
                else	if( System.open == 0 )	//�ػ��ˣ���Ҫ����
                {
                    if( System.Voltage > 40.0 )
                    {
                        System.open = 0x02;
                        Monitor.need_open = 0;
												System.status = SYSTEM_STANDBY;
                        Power_onBack.results[0]=0xFC;
                    }
                    else
                    {
                        System.open = 0x01;
												Monitor.need_open = 0x01;
                        System.status = SYSTEM_OPENING;

                        xTaskNotify(xHandleTask_MT2000,	//����
                                    BIT_8,
                                    eSetBits);
                    }
                }
								else	if( System.open == 0x01 )		//���ڿ���
								{ 
										if( System.Voltage > 40.0 )
										{
												Monitor.need_open = 0;
												System.open = 0x02;
												System.status = SYSTEM_STANDBY;
										}
										else
										{
												System.open = 0x01;
												Monitor.need_open = 0x01;
												System.status = SYSTEM_OPENING;
										}
								}
								
								if( get_history_alarm() == 0x01 )
								{
									System.status = SYSTEM_ALARM;
								}
							}
            else	if( Monitor.need_close == 0x01 )						//��Ҫ�ػ�
            {
                if( (System.open == 0) && (System.Voltage < 40.0) )					//�Ѿ��ػ���
                {
                    System.open = 0x00;
                    Monitor.need_close = 0;

                    if( get_history_alarm() == 0x01 )
                    {
                        System.status = SYSTEM_ALARM;
                    }
                    else
                    {
                        System.status = SYSTEM_SHUTDOWN;
                    }
                }
                else	if( (System.open == 0x02) || ( System.status == SYSTEM_STANDBY ) )		//�����ˣ���Ҫ�ػ�
                {
                    System.close = 1;
                    Monitor.need_close = 0x00;
                    xTaskNotify(xHandleTask_MT2000,	//�ػ�
                                BIT_9,
                                eSetBits);

                    for(i=0; i<30; i++)				//�ȴ��ػ��ɹ�
                    {
                        if( (System.close == 0) || (Monitor.hard_control==0) )		//�ػ��ɹ������˳�Ӳ���ӹ�	���Task_MT2000()�ػ�����
                            break;
                        else
                            vTaskDelay(100);
                    }
                }
            }
            else	if( (PC_Cmd.mode==0x00) && (System.emission==0x01) )	//��Ҫֹͣ
            {

                if( System.stop == 0x00 )
                {
                    System.stop = 0x01;
                    xTaskNotify(xHandleTask_MT2000,		//ֹͣ����
                                BIT_0,
                                eSetBits);
                }
                else
                {
                    //App_printf("Task_Hardware_Monitor Stopp not execute!\r\n");
                }
            }
            else	if( Monitor.need_emit == 0x00 )							//�Ƿ���Ҫ����
            {
                if( System.emission==0x01 )							//����״̬
                {
                    if( (PC_Cmd.mode+'0') != System.mode )			//ģʽ��ͬ����Ҫ����
                    {
                        Monitor.need_emit = 0x01;
                    }
                    else											//ģʽ��ͬ
                    {
                        if( (Monitor.need_emit == 0x00) && (PC_Cmd.mode >= 0x01) )					//��Ƶ
                        {
                            freq_hex_to_str(PC_Cmd.freq, temp_freq_str);
                            ret = strcmp((const char *)temp_freq_str, (const char *)System.freq1);

                            if( ret == 0)														//Ƶ����ͬ
                            {
                                Monitor.need_emit = 0x00;
                            }
                            else																//Ƶ�ʲ�ͬ����Ҫ����
                            {
                                Monitor.need_emit = 0x01;
                            }
                        }

                        if( (Monitor.need_emit == 0x00) && (PC_Cmd.mode >= 0x02) )					//˫Ƶ
                        {
                            freq_hex_to_str(PC_Cmd.freq+4, temp_freq_str);
                            ret = strcmp((const char *)temp_freq_str, (const char *)System.freq2);

                            if( ret == 0)														//Ƶ����ͬ
                            {
                                Monitor.need_emit = 0x00;
                            }
                            else																//Ƶ�ʲ�ͬ����Ҫ����
                            {
                                Monitor.need_emit = 0x01;
                            }
                        }

                        if( (Monitor.need_emit == 0x00) && (PC_Cmd.mode >= 0x03) )					//��Ƶ
                        {
                            freq_hex_to_str(PC_Cmd.freq+8, temp_freq_str);
                            ret = strcmp((const char *)temp_freq_str, (const char *)System.freq3);

                            if( ret == 0)														//Ƶ����ͬ
                            {
                                Monitor.need_emit = 0x00;
                            }
                            else																//Ƶ�ʲ�ͬ����Ҫ����
                            {
                                Monitor.need_emit = 0x01;
                            }
                        }

                        if( Monitor.need_emit == 0x00 )
												{
                            power_check();
												}
                    }
                }
                else												//���Ƿ���״̬
                {
                    if( PC_Cmd.mode >= 0x01 )
                        Monitor.need_emit = 0x01;					//��Ҫ����
                    else
                        Monitor.need_emit = 0x00;					//����Ҫ����
                }

                if( Monitor.need_emit == 0x01 )						//��Ҫ����
                {
                    if( System.status == SYSTEM_OPENING )
                    {

                    }
                    else
                    {
                        /*----------------------------------���÷������-----------------------------------*/
                        switch( PC_Cmd.mode )
                        {
													case 0x01:		GT2000_Tx.Gt2000_mode = 1;					break;
													case 0x02:		GT2000_Tx.Gt2000_mode = 2;					break;
													case 0x03:		GT2000_Tx.Gt2000_mode = 3;					break;
													default:		Trans_openBack.Trans_state[0]=0xFE;		break;	//��������
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
												
											/*PC�·�������ת��Ϊ����ֵ [100~250],��Ӧ��0~1950W��,1*13*/
                        Power_Range(&GT2000_Tx,PC_Cmd.power);   //&GT2000_Tx  �˱���Ϊָ�����ͣ�Ӧ��ȡ��ַ���š�

                        freq_hex_to_str(PC_Cmd.freq,			System.freq1);
                        freq_hex_to_str(PC_Cmd.freq,			GT2000_Tx.Gt2000_freq1);	//Ƶ��
                        if( PC_Cmd.mode == 0x02 )
                        {
                            freq_hex_to_str(PC_Cmd.freq+4,		GT2000_Tx.Gt2000_freq2);	//Ƶ��

                            memset(GT2000_Tx.Gt2000_freq3, '0', 8);
                            GT2000_Tx.Gt2000_freq3[8] = '\0';
                        }
                        else	if( PC_Cmd.mode == 0x03 )
                        {
                            freq_hex_to_str(PC_Cmd.freq+4,	GT2000_Tx.Gt2000_freq2);		//Ƶ��
                            freq_hex_to_str(PC_Cmd.freq+8,	GT2000_Tx.Gt2000_freq3);
                        }
                        else
                        {
                            memset(GT2000_Tx.Gt2000_freq2, '0', 8);
                            GT2000_Tx.Gt2000_freq2[8] = '\0';
                            memset(GT2000_Tx.Gt2000_freq3, '0', 8);
                            GT2000_Tx.Gt2000_freq3[8] = '\0';

                            memset(System.freq2, '0', 8);
                            System.freq2[8] = '\0';
                            memset(System.freq3, '0', 8);
                            System.freq3[8] = '\0';
                        }

                        /*-------------------------------------����--------------------------------------*/
                        if( System.open != 0x02 )			//û�п���
                        {
                            if( System.Voltage > 40.0 )		//�ٴ��ж��Ƿ񿪻�
                            {
                                System.open = 0x02;
                                Monitor.need_open = 0;
                                Power_onBack.results[0]=0xFC;

                                if( get_history_alarm() == 0x01 )
                                {
                                    System.status = SYSTEM_ALARM;
                                }
                                else
                                {
                                    System.status = SYSTEM_STANDBY;
                                }
                            }
                            else							//�Ѿ���ǰ�����ˣ���ȷʵû�п����ɹ���ִ�п�������
                            {
                                Monitor.need_open = 0x01;
                            }
                        }

                        if( System.open == 0x02 )						//������
                        {
                            if( System.emission == 0x01 )
                            {
                                System.stop = 0x01;
                                xTaskNotify(xHandleTask_MT2000,			//ֹͣ����
                                            BIT_0,
                                            eSetBits);

                                for(i=0; i<200; i++)
                                {
                                    if( System.stop == 0x00 )
                                        break;
                                    else
                                        vTaskDelay(10);
                                }
                            }

                            if( System.emission == 0x00 )
                            {
																if( (System.emission == 0x01) || (Alarm.emission==1) || (Alarm.no_respond==1) || (Alarm.no_power==1) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                                {
                                    //App_printf("Task_Harware_Monitor waring: working or alarm state!\r\n");
                                }
                                else
                                {
                                    xTaskNotify(xHandleTask_MT2000,			//����
                                                BIT_3,
                                                eSetBits);
                                }
                            }
                            else
                            {

                            }
                        }
                    }
                }
            }

            if( (Monitor.need_open==0x00) && (Monitor.need_close==0x00) && (Monitor.need_emit==0x00) &&			//����Ҫ�������ػ�������
				(System.close==0x00) && (System.stop==0x00) && (System.status!=SYSTEM_OPENING))	//�������ڹػ���ֹͣ������
            {
                inquire_counter++;
                if( inquire_counter > 20 )
                {
                    inquire_counter = 0;
                    xTaskNotify(xHandleTask_MT2000,					//��ѯ(����״̬������Ƿ񱨾�)
                                BIT_10,
                                eSetBits);
                }
            }
            else
            {
                inquire_counter = 0;
            }
        }

#endif
        vTaskDelay(100);
    }
}

void power_check(void)
{	
		/***Ӳ���ӹ�ʱ���Ƚ��������ڷ���Ĺ��ʵȼ�������ͼ�еģ���ͬ����Ҫ���·���****/
		if(System.mode=='1')
		{
			if(System.power[0]==PC_Cmd.power[0])
			{
				Monitor.need_emit = 0x00;
			}
		  else
			{
					Monitor.need_emit = 0x01;
			}
		}
		
		else if(System.mode=='2')
		{
			if(System.power[0]==PC_Cmd.power[0]&&System.power[1]==PC_Cmd.power[1])
			{
				Monitor.need_emit = 0x00;
			}
			
			else
			{
					Monitor.need_emit = 0x01;
			}
		}
		
		else if(System.mode=='3')
		{
			if(System.power[0]==PC_Cmd.power[0]&&System.power[1]==PC_Cmd.power[1]&&System.power[2]==PC_Cmd.power[2])
			{
					Monitor.need_emit = 0x00;
			}
		  else
			{
					Monitor.need_emit = 0x01;
			}
		}



}
