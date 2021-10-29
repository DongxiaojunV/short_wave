#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "alarm.h"
#include "stand_wave_rate.h"
#include "power.h"
#include "math.h"

uint8_t	re_scan_flag = 0x00;

float Scan_Begin;			//��ΧɨƵ��ʼ��
float Scan_End;				//��ΧɨƵ������
float Old_Forward_Power=0;		//���ڱȽ��ϴεĹ��ʱ仯
uint8_t Power_cataclysm_count=0;
TaskHandle_t xHandleTask_MT2000;

void Task_MT2000(void *pvParameters)
{
    uint32_t ulValue;
    BaseType_t xResult;

    uint8_t i = 0;
    uint8_t j = 0;
    uint8_t repeat_send_counter = 0;		//ָ��ʧ�ܣ��ط����
    uint32_t current_time_stamp = 0;

		uint8_t		freq_begin;
		uint8_t		freq_end;
		
		uint8_t		next_point_int = 0;
		uint8_t		next_point_dec = 0;
		uint16_t	next_point = 0;
		
		GT2000_t Change_Power_Value;
	
    int ret;
    while(1)
    {
        xResult = xTaskNotifyWait(0x00000000,
                                  0xFFFFFFFF,
                                  &ulValue,
                                  portMAX_DELAY); /* ��������ӳ�ʱ�� */

        if( xResult == pdPASS )
        {
            if((ulValue & BIT_0) != 0)			//ֹͣ  Bit0��Task_Upper.c�еĲ�ѯ��ֹͣ���䡢������
            {
								System.Power_Adjustment=1;
							  Trans_printf(" BIT_0_to_GT2000_Stop ! ");
                GT2000_Stop();
							  
								vTaskDelay(2000);
								System.Power_Adjustment=0; //��������ִ�б�� 1����
            }
						
            else	if((ulValue & BIT_1) != 0)	//���ӹ���		//������
            {
							if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }

                System.modify_power = 0x01;
                if(System.mode==1)		//����ǵ�Ƶ
								{
									  power_add(GT2000_Rx.Frequency_power);  //����΢��������һ������+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];										
								}
								
								else if(System.mode==2)
								{							
									  power_add(GT2000_Rx.Frequency_power);  //����΢��������һ������+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];		
								}
								
								else if(System.mode==3)
								{
									  power_add(GT2000_Rx.Frequency_power);  //����΢��������һ������+65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];		
								}
								
								memcpy(Change_Power_Value.Gt2000_freq1,GT2000_Rx.Gt2000_freq1,2);
								memcpy(Change_Power_Value.Gt2000_freq2,GT2000_Rx.Gt2000_freq2,2);
								memcpy(Change_Power_Value.Gt2000_freq3,GT2000_Rx.Gt2000_freq3,2);
								
								Change_Power_Value.Gt2000_mode=System.INQUIRE_Mode;   //�ӻ����ϲ鵽�Ĺ���ģʽ
//								Power_Range(&Change_Power_Value,Change_Power_Value.Frequency_power);		//ת��Ϊ����ֵ����
								
				Gt_RF_Close();		//�ط���
				vTaskDelay(50);
								
								GT_Set_Value(Change_Power_Value);	 //���ù���������ǰ�渳ֵ
								Trans_printf(" 2�ɴ˽������ù���ģʽ��Ƶ��22 ");
								vTaskDelay(500);											
					Gt_RF_Open();
					Trans_printf(" �������������·�22 ");
								
								if(System.status!=SYSTEM_SCAN)		//�������ɨƵ��ֱ�Ӹ���״̬
								{
									System.status=SYSTEM_EMISSING;
								}
								vTaskDelay(100);
								Gt_Inquire_All();		//��ѯһ�Σ������ޱ���
								ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);							
								if( ret == MT2000_ACK_ALARM )
								{
										GT2000_Alarm_Stop();
								}
								else	if( ret == MT2000_ACK_OK )
								{
										GT2000_Buffer_Clean();
								}			

								if( ret != MT2000_ACK_OK)	
								{
										System.stop = 0x00;

										System.open = 0x02;
										System.achieve_add_sub = 0;
										System.modify_power = 0;
										System.cancel_add_sub = 0;
										System_Status_Clean();

										Alarm.no_respond_locate = 10;
								}
								//����ɹ�֮��System.emission=0x01; ��Щ��־λδ��λ������Task_Hardware_Monitor()���ٴ��ж�Ϊ��Ҫ���䣬���־���

								/* ����ɹ�֮�󣬲������ϸ������ݣ�������ȴ����ݸ���(���ʻ���µıȽ���) */
						for(i=0; i<100; i++)
						{
							Gt_Inquire_All();
							vTaskDelay(50);
							ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
							if( ret == MT2000_ACK_ALARM )
							{
									GT2000_Alarm_Stop();
							}
							if( System.Forward_Power >= 150.0 )
							{
									System.emission = 0x01;
									break;
							}
							else
							{
									System.emission = 0x00;		//�ȴ����صĹ��ʲ�Ϊ�㣬��������
							}

								vTaskDelay(100);
						}
	
						if( ret != MT2000_ACK_OK )
						{
								Alarm.no_respond_locate = 12;
						}
					
						if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//�޹��ʱ���
						{
						
								if( Alarm.no_power == 0 )
								{
									if( System.time_update_flag == 0x01 )
									{
										Alarm.no_power_time_flag = 0x01;
										Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
										Alarm_historyBack.no_power_time[0]=set_time.tm_year-2000;
										Alarm_historyBack.no_power_time[1]=set_time.tm_mon;
										Alarm_historyBack.no_power_time[2]=set_time.tm_mday;
										Alarm_historyBack.no_power_time[3]=set_time.tm_hour;
										Alarm_historyBack.no_power_time[4]=set_time.tm_min;
										Alarm_historyBack.no_power_time[5]=set_time.tm_sec;
									}

									Alarm.alarm_history = 1;
									Alarm.no_power = 1;
									Trans_printf(" ���ӹ��ʼ�⵽�޹��ʱ���2�� \n");
								}
						}
						//����ɹ�����鱨��
						swr_power_alarm_check();	//פ���ȼ��

						if( Alarm.emission == 0x01 )	//������������������ֹͣ
						{
								GT2000_Alarm_Stop();
						}
								
						if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )	//����������������Ӧ�𱨾����޹��ʱ���
						{
								System.status = SYSTEM_ALARM;
						}
						else	if( (System.cancel_sweeping==0x01) || (System.stop==0x01) )			//ȡ��ɨƵ��ֹͣ�󷵻�
						{
								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_STANDBY;
								}
						}

                System.modify_power = 0x00;
								
								vTaskDelay(3000);		//�ȴ������ȶ�
						
								System.Power_Adjustment=0;
            }
						
            else 	if((ulValue & BIT_2) != 0)	//��С����		
            {
                if( get_history_alarm() == 0x01 )
                {
                    System.status = SYSTEM_ALARM;
                }
                else
                {
                    System.status = SYSTEM_EMISSING;
                }

                System.modify_power = 0x01;
                if(System.mode==1)		//����ǵ�Ƶ
								{
									if(GT2000_Rx.Now_all_power>0x41)			//��ǰ��������65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //����΢��������һ�μ�С-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//ʧ��
									}
								}
								
								else if(System.mode==2)
								{
									if(GT2000_Rx.Now_all_power>0x41)			//��ǰ��������65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //����΢��������һ�μ�С-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//ʧ��
									}
								}
								
								else if(System.mode==3)
								{
									if(GT2000_Rx.Now_all_power>0x41)			//��ǰ��������65W
									{
										power_sub(GT2000_Rx.Frequency_power);  //����΢��������һ�μ�С-65W
										Change_Power_Value.Frequency_power[0]=GT2000_Rx.Frequency_power[0];												
								    Change_Power_Value.Frequency_power[1]=GT2000_Rx.Frequency_power[1];	
									}
									else
									{
										Sub_PowerBack.results[0]=0xFC;				//ʧ��
									}
								}
								
								memcpy(Change_Power_Value.Gt2000_freq1,GT2000_Rx.Gt2000_freq1,8);
								memcpy(Change_Power_Value.Gt2000_freq2,GT2000_Rx.Gt2000_freq2,8);
								memcpy(Change_Power_Value.Gt2000_freq3,GT2000_Rx.Gt2000_freq3,8);
								
								Change_Power_Value.Gt2000_mode=System.INQUIRE_Mode;   //�ӻ����ϲ鵽�Ĺ���ģʽ
//								Power_Range(&Change_Power_Value,Change_Power_Value.power);		//ת��Ϊ����ֵ����
								
								Gt_RF_Close();		//�ط���
								vTaskDelay(50);
								
								GT_Set_Value(Change_Power_Value);	//���÷��������
								Trans_printf(" 3�ɴ˽������ù���ģʽ��Ƶ������ ");
								vTaskDelay(50);
								Gt2000_Open(); //����
								Trans_printf(" ����--�ȴ�30s--�ٿ����� ");
								vTaskDelay(30000);				//�ȴ�30s									
								Gt_RF_Open();  //������Gt_RF_Open
								Trans_printf(" �������������·� ");
								
								
								if(System.status!=SYSTEM_SCAN)		//�������ɨƵ��ֱ�Ӹ���״̬
								{
									System.status=SYSTEM_EMISSING;
								}
								vTaskDelay(100);
								Gt_Inquire_All();		//��ѯһ�Σ������ޱ���
								ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);						
								if( ret == MT2000_ACK_ALARM )
								{
										GT2000_Alarm_Stop();
								}
								else	if( ret == MT2000_ACK_OK )
								{
										GT2000_Buffer_Clean();
								}			

								if( ret != MT2000_ACK_OK)	
								{
										System.stop = 0x00;

										System.open = 0x02;
										System.achieve_add_sub = 0;
										System.modify_power = 0;
										System.cancel_add_sub = 0;
										System_Status_Clean();

										Alarm.no_respond_locate = 10;
								}
								//����ɹ�֮��System.emission=0x01; ��Щ��־λδ��λ������Task_Hardware_Monitor()���ٴ��ж�Ϊ��Ҫ���䣬���־���

								/* ����ɹ�֮�󣬲������ϸ������ݣ�������ȴ����ݸ���(���ʻ���µıȽ���) */
						for(i=0; i<100; i++)
						{
							Gt_Inquire_All();
							vTaskDelay(50);
							ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
							if( ret == MT2000_ACK_ALARM )
							{
									GT2000_Alarm_Stop();
							}
							if( System.Forward_Power >= 150.0 )
							{
									System.emission = 0x01;
									break;
							}
							else
							{
									System.emission = 0x00;		//�ȴ����صĹ��ʲ�Ϊ�㣬��������
							}

								vTaskDelay(100);
						}
	
						if( ret != MT2000_ACK_OK )
						{
								Alarm.no_respond_locate = 12;
						}
					
						if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//�޹��ʱ���
						{
						
								if( Alarm.no_power == 0 )
								{
									if( System.time_update_flag == 0x01 )
									{
										Alarm.no_power_time_flag = 0x01;
										Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
										Alarm_historyBack.no_power_time[0]=set_time.tm_year-2000;
										Alarm_historyBack.no_power_time[1]=set_time.tm_mon;
										Alarm_historyBack.no_power_time[2]=set_time.tm_mday;
										Alarm_historyBack.no_power_time[3]=set_time.tm_hour;
										Alarm_historyBack.no_power_time[4]=set_time.tm_min;
										Alarm_historyBack.no_power_time[5]=set_time.tm_sec;
									}

									Alarm.alarm_history = 1;
									Alarm.no_power = 1;
									Trans_printf(" ��⵽�޹��ʱ���3�� \n");
								}
						}
						//����ɹ�����鱨��
						swr_power_alarm_check();	//פ���ȼ��

						if( Alarm.emission == 0x01 )	//������������������ֹͣ
						{
								GT2000_Alarm_Stop();
						}
								
						if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )	//����������������Ӧ�𱨾����޹��ʱ���
						{
								System.status = SYSTEM_ALARM;
						}
						else	if( (System.cancel_sweeping==0x01) || (System.stop==0x01) )			//ȡ��ɨƵ��ֹͣ�󷵻�
						{
								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_STANDBY;
								}
						}

                System.modify_power = 0x00;
						
								vTaskDelay(3000);		//�ȴ������ȶ�
						
								System.Power_Adjustment=0;
            }
						
            else 	if((ulValue & BIT_3) != 0)	//����
            {
                //App_printf("�Ȳ�ѯһ���ǲ���ֹͣ״̬\r\n");
                     //�ظ����ͼ�����
                for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
                {
									
											Gt_Inquire_All();		//�����״̬��ѯ								
											ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
											if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM) )
													break;

                }
								Trans_printf(" �ظ������Ȳ�ѯһ���ǲ���ֹͣ״̬ \n");
                if( ret == MT2000_ACK_OK )   //�ɹ���ѯ�ˣ��ż�������ģ�������Ǳ���״̬
                {
                    if( GT2000_Rx.sys_open == 0 )
                    {
                        System.open = 0x00;  //������־λ	0:�ػ�	1:���ڿ���	2:�Ѿ�����
											  Trans_printf(" �����δ�������ػ� \n");
                    }
                    else	if( (System.emission == 0x01) || (Alarm.emission==0x01) || (Alarm.no_respond==0x01) ||
                                (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) || (Alarm.over_Electric==0x01) )
                    {
                            //���ڷ��䣬����Ӧ���޹��ʣ�פ���ȱ�����������������ִ�в���
											 Trans_printf(" ���ڷ��䣬����Ӧ���޹��ʣ�פ���ȱ�����������������ִ�в��� \n");
                    }
                    else	//ִ�з���
                    {
											if(System.Launch_Switch_state!=1)		//��ѯ������Ƿ���״̬
											{
                        GT2000_Emit(); //������
												vTaskDelay(10000);   //�ȴ�
												Trans_printf(" 3_T0_GT2000_Emit() \n");
												Trans_printf(" ��ѯ������Ƿ���״̬�������� \n");

                        if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) || (Alarm.swr_alarm==0x01) )	//����������������Ӧ�𱨾����޹��ʱ���
                        {
                            GT2000_Stop();
													Trans_printf(" �ѷ��Ϳ�����������н���������ֹͣ�� \n");
                        }

                        if( System.emission == 0x01 )
                        {
                            if( Monitor.hard_control == 0x00 )
                            {
                                System.status = SYSTEM_EMISSING;		//����Ӳ���ӹܣ�����ɨƵ�������⣬����ֱ�ӷ��ط���״̬
															  Trans_printf(" ����Ӳ���ӹܣ�����ɨƵ�������⣬����ֱ�ӷ��ط���״̬ \n");
                            }                           
                                if( get_history_alarm() == 0x01 )
                                {
                                    System.status = SYSTEM_ALARM;
																	Trans_printf(" ������;�У�����ʷ���������ر���״̬ \n");
                                }
                                else
                                {
                                    System.status = SYSTEM_EMISSING;
																	Trans_printf(" ������ɹ������ط���״̬ \n");
                                }
                            							
														if( Monitor.hard_control == 0x01 )
														{
															Monitor.need_emit = 0x00; //��ѯ����ͼʱ��ģʽ��Ƶ�ʻ��ʵȼ���һ�£�1��Ҫ���� 0����Ҫ
															Trans_printf(" Monitor.hard_control=1������Ҫ���� \n");
														}
                        }
                        else
                        {
                            System.sweep = 0;
                            System.stop = 0;
                            System.achieve_add_sub = 0;
                            System.modify_power = 0;
                            System.cancel_add_sub = 0;
                            System_Status_Clean();

														System.status = SYSTEM_STANDBY;
													Trans_printf(" System.emission��=1������״̬ \n");
                 
                        }
						
											if( get_history_alarm() == 0x01 )
											{
												System.status = SYSTEM_ALARM;
												Trans_printf(" ִ�з������������ʷ������ \n");
											}
										}
                    }
                }
                else	//��ѯʧ��
                {
                    Alarm.no_respond_locate = 16;
                    //App_printf("Task_MT2000 Emit Inquire 16 no respond.\r\n");
									Trans_printf(" System.emission��=1������״̬ \n");
                }
							Trans_printf(" �����¼�BIT_3ִ����� \n");
            }
						
            else 	if((ulValue & BIT_4) != 0)	//����
            {

            }
            else 	if((ulValue & BIT_5) != 0)	//ɨƵ
            {
				
				/*--------------------------------ɨƵǰ�������������-----------------------------------*/
                for(i=0; (i<10)&&((System.cancel_sweeping == 0x00) && (System.stop == 0x00) ); i++)		
				{                          //ȡ��ɨƵ		1:ȡ��ɨƵ               //����ֹͣ��־λ	1:����ֹͣ(�����������ʣ������Ѿ����˹���ʱ�䣬��Ҫ����ֹͣ)	֪ͨTask_MT2000(),BJ2000_Stop();MT2000_Cmd_Stop();����λ
					current_time_stamp = RTC_GetCounter() + g_stamp_distance;
					
					next_point = Scan_Breakpoint_Read();
					next_point_int = next_point/10;    //��������
					next_point_dec = next_point%10;    //С������
					     //�洢ɨƵ������ʱ���
					if( (freq_band_time_stamp[i] != 0) && (freq_band_time_stamp[i] != 0xFFFFFFFF) &&	//ʱ�����������Ч����(��һ��û����)
						(current_time_stamp < (freq_band_time_stamp[i] + FREQ_BAND_UPDATE_TIME) ) )		//������Ч��δ����
          {
						if( (next_point == 50) && 						//next point is 5.0MHz
							(freq_band_time_stamp[9] != 0) && 			//.9 is finish
							(freq_band_time_stamp[9] != 0xFFFFFFFF) ) //�洢ɨƵ������ʱ���
						{						
							re_scan_flag = 0x01;  //1:������Ч��ɨ�����ݣ�ɨ�谴ť�ѱ�����
							break;
						}
						else	if( next_point_dec > i )		//�Ѿ�ɨ���ģ�����
						{
							re_scan_flag = 0x00;
							continue;
						}
						else	//�������ݹ��ڣ���Ҫ����ɨƵ��������(ֹͣɨƵ֮��ᱣ��ʱ��������ݲ�����ڣ���������)
						{
							re_scan_flag = 0x00;
							freq_band_time_stamp[i] = 0x00; //����洢ɨƵ������ʱ���

							if( (i==0) || (i==1) )
							{
								for( j=next_point_int; j<=26; j++)
								{
									Forward_Power_array[ (j-3)*10 + i ] = 0.0;  //��3Mhz��ʼ���棬����[0]~[23.9]
									Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
									SWR_array[ (j-3)*10 + i ] = 0.0;
								}
							}
							else
							{
								for( j=next_point_int; j<=25; j++)
								{
									Forward_Power_array[ (j-3)*10 + i ] = 0.0;  //��3.2Mhz��ʼ���棬����[0]~[22.9]
									Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
									SWR_array[ (j-3)*10 + i ] = 0.0;
								}
							}
						}
					}
					else	//�ö�����ȫ����Ч���߹��ڣ�ȫ������
					{
						freq_band_time_stamp[i] = 0x00;

						if( (i==0) || (i==1) )
						{
							for(j=5; j<=26; j++)
							{
								Forward_Power_array[ (j-3)*10 + i ] = 0.0;
								Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
								SWR_array[ (j-3)*10 + i ] = 0.0;
							}
						}
						else
						{
							for(j=5; j<=25; j++)
							{
								Forward_Power_array[ (j-3)*10 + i ] = 0.0;
								Reverse_Power_array[ (j-3)*10 + i ] = 0.0;
								SWR_array[ (j-3)*10 + i ] = 0.0;
							}
						}
					}
				}
				
				if(InternalFlash_SaveData_2()==1)   //�������ʺ�פ���ȱ��浽flash   1:�ɹ�
				{
					//App_printf("flash save successed!\r\n");
				}
				else
				{
					//App_printf("flash save error!\r\n");
				}
				
                Flash2_to_AcceptAPP();		//��ȡ  //��ȡflash�ڵ������ʺ�פ���ȵ�Scan_FrqBackx
				
				if(Scan_Begin==0||Scan_End==0)
				{
	/*---------------------------------------��ʼɨƵ----------------------------------------*/
					for(int num=0; (num<10)&&((System.cancel_sweeping == 0x00) && (System.stop == 0x00) ); num++)		
					{
							current_time_stamp = RTC_GetCounter() + g_stamp_distance;

							if( re_scan_flag == 0x01 )
							{
								
							}
							else	if( (freq_band_time_stamp[num] != 0) && (freq_band_time_stamp[num] != 0xFFFFFFFF) )		//��Ч����(��������֮ǰ�Ѿ�������)
							{
								continue;  //��������ѭ������������δִ�е���䣬����������һ�ε�ѭ�������ж�����������ѭ����
							}
							//����̲��豸���ʷ�Χ5.9-21.8
					switch(num)
					{
						case 0:		freq_begin=6;		freq_end=21;		break;
						case 1:		freq_begin=6;		freq_end=21;		break;
						case 2:		freq_begin=6;		freq_end=21;		break;
						case 3:		freq_begin=6;		freq_end=21;		break;
						case 4:		freq_begin=6;		freq_end=21;		break;
						case 5:		freq_begin=6;		freq_end=21;		break;
						case 6:		freq_begin=6;		freq_end=21;		break;
						case 7:		freq_begin=6;		freq_end=21;		break;
						case 8:		freq_begin=6;		freq_end=21;		break;
						case 9:		freq_begin=5;		freq_end=20;		break;
					}

                    /*------------------------------------------ɨƵ------------------------------------------*/

                    Band_scan(freq_begin, freq_end, num);		
					
					//�˳�ɨƵ֮��ͱ��棬�����˳�ԭ����ʲô
					freq_band_time_stamp[num] = RTC_GetCounter() + g_stamp_distance;
                    if(InternalFlash_SaveData_2()==1)   //�������ʺ�פ���ȱ��浽flash   1:�ɹ�
                    {
                        Flash2_to_AcceptAPP();   //��ȡflash�ڵ������ʺ�פ���ȵ�Scan_FrqBackx
                        //App_printf("flash save successed!\r\n");
                    }
                    else
                    {
                        //App_printf("flash save error!\r\n");
                        System.sweep = 0;
                    }
					
                    //��Ӧ�𱨾����޹��ʱ���(פ���Ⱥ͵���������������)		--20190415 Luonus
                    if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )
                    {
											Trans_printf(" scan1ɨƵ����Ӧ�𱨾����޹��ʱ������������˳�ѭ�� \n");
                        break;  //�˳�ѭ��
                    }
                    else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//ȡ��ɨƵ
                    {
											Trans_printf(" scan1ɨƵ��ȡ��ɨƵ�������� \n");
                        break;
                    }
                    else	if( System.Voltage < 40.0 )				//�ػ���
                    {
											Trans_printf(" scan1ɨƵSystem.Voltage < 40.0���ػ��� \n");
                        break;
                    }

					if( num == 9 )	
					{
						System.already_swept = 1;    //ɨƵ��ɱ�־		//�ϵ�ɨ��֮�󣬸ñ�־λ�Ѿ�������
                        System.sweep = 0x02;  //ɨƵ��־λ	1:����ɨƵ��2:ɨƵ��ɣ�����status��־λ������sweep����		��󣬱�����flash
						
						Scan_Breakpoint_Save(5, 9);					//ɨƵ���ˣ��´�ɨƵ������ɨƵ  //��ȡɨƵ�ϵ�5.9
						break;
					}

              }    //forѭ������
					}	  //Scan_Begin��=0 && Scan_End��=0
				else
				{
					uint8_t Scan_start,Scan_end;				//���ܳ���255
					uint8_t clean_int,clean_dec;
					Scan_start=Scan_Begin*10;			//����Ϊ����
					Scan_end=Scan_End*10;
				/*---------------------------------------��ʼɨƵ2----------------------------------------*/
												
				/*--------------------------------ɨƵǰ�������������-----------------------------------*/
        if((System.cancel_sweeping == 0x00) && (System.stop == 0x00))  //δȡ��ɨƵ��δ��ͣ		1:ȡ��ɨƵ	1:����ֹͣ
				{		
						for(j=Scan_start; j<=Scan_end; j++)
						{
							clean_int=j/10;
							clean_dec=j%10;
							Forward_Power_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
							Reverse_Power_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
							SWR_array[ (clean_int-3)*10 + clean_dec ] = 0.0;
						}
				}
                 for(int count=Scan_start;(count<=Scan_end)&&(System.cancel_sweeping == 0x00)&&(System.stop == 0x00);count++)
								 {									 
										Trans_printf(" ����ֶ�ɨƵ����2��count= %d \n",count);   //ΪʲôI��=0
									 Optional_Band_scan(count,Scan_end);  //�ֶ�ɨƵ����2
								    
									
										//�˳�ɨƵ֮��ͱ��棬�����˳�ԭ����ʲô
                    if(InternalFlash_SaveData_2()==1) //�������ʺ�פ���ȱ��浽flash   1:�ɹ�
                    {
                        Flash2_to_AcceptAPP();   //��ȡflash�ڵ������ʺ�פ���ȵ�Scan_FrqBackx
                        //App_printf("flash save successed!\r\n");
                    }
                    else
                    {
                        //App_printf("flash save error!\r\n");
                        System.sweep = 0;
                    }
					
                    //��Ӧ�𱨾����޹��ʱ���(פ���Ⱥ͵���������������)		--20190415 Luonus
                    if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )
                    {
											Trans_printf(" ɨƵ����Ӧ�𱨾����޹��ʱ����������� \n");
                        break;  //�˳�ѭ��
                    }
                    else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//ȡ��ɨƵ
                    {
											Trans_printf(" ɨƵ��ȡ��ɨƵ�������� \n");
                        break;
                    }
                    else	if( System.Voltage < 40.0 )				//�ػ���
                    {
											Trans_printf(" System.Voltage < 40.0���ػ��� \n");
                        break;
                    }


										if( i ==  Scan_end)	
										{
											System.already_swept = 1;   //ɨƵ��ɱ�־		//�ϵ�ɨ��֮�󣬸ñ�־λ�Ѿ�������
																	System.sweep = 0x02;  //ɨƵ��־λ	1:����ɨƵ��2:ɨƵ��ɣ�����status��־λ������sweep����		��󣬱�����flash
											
											Trans_printf(" �Ѿ�ɨƵ���ˣ��´�ɨƵ������ɨƵ \n");
											Scan_Breakpoint_Save(5, 9);					//ɨƵ���ˣ��´�ɨƵ������ɨƵ
											break;
										}
							}   //for����
						}	
				
						re_scan_flag = 0x00;   //��ɨƵ��־λ��ֹͣɨƵ
                if( (Alarm.emission==0x01) || (Alarm.no_respond==0x01) || (Alarm.no_power==0x01) )		//�����˳�		--20190415 Luonus
                {
									Trans_printf(" ɨƵ�����˳� \n");
                    System.sweep = 0x00;
                    Scan_FrqBack.results[0] = 0xFD;		//ɨƵʧ��
									
				            Gt_RF_Close();		
										vTaskDelay(3000);  //�ط���3s��ػ�
										Gt2000_Close();
										vTaskDelay(1500);   //�ػ�
										ret=MT2000_ACK_OK;


                    if( ret == MT2000_ACK_OK )	//ֹͣ�ɹ�
                    {
                        System.stop = 0x00;

                        System.achieve_add_sub = 0x00;
                        System.modify_power = 0x00;
                        System.cancel_add_sub = 0x00;
                        System_Status_Clean();

                        System.sweep = 0x00;
                        System.already_swept = 0x00;
                    }
                    else
                    {
                        Alarm.no_respond_locate = 17;
                        //App_printf("Task_MT2000 Sweep stop 17 no respond.\r\n");
                    }
                }
                else	if( (System.cancel_sweeping == 0x01) || (System.stop == 0x01)  )		//��������Band_scan������ȡ��ɨƵ�˳���
                {       //ɨƵ��־λ	1:����ɨƵ��2:ɨƵ���
                    Trans_printf(" ��������Band_scan������ȡ��ɨƵ�˳��ġ���ͣ�˳��� \n");
									if( System.sweep == 0x01 )			//�����ɨƵ����ǿ��ֹͣ��ֹ�ˣ���Ҫ���������ֹͣ�Ա��ϰ�ȫ
                    {
											
				            Gt_RF_Close();		
											vTaskDelay(3000);
											Gt2000_Close();
											vTaskDelay(1500);
											ret=MT2000_ACK_OK;

                        if( ret == MT2000_ACK_OK )	//ֹͣ�ɹ� ����Ĭ�ϳɹ�
                        {
                            System.sweep = 0;
                            System.stop = 0;
                            System.achieve_add_sub = 0;
                            System.modify_power = 0;
                            System.cancel_add_sub = 0;
                            System_Status_Clean();

                            if( get_history_alarm() == 0x01 )
                            {
                                System.status = SYSTEM_ALARM;
                            }
                            else
                            {
                                System.status = SYSTEM_STANDBY;
                            }

                            System.already_swept = 0;
                            System.cancel_sweeping = 0;
                            Trans_stopBack.Trans_state[0]=0xFE;		//ֹͣ�ɹ�
                        }
                        else     //���ɹ�������ֹͣʧ�ܣ��Ҵ��ڱ���״̬
                        {
                            Trans_stopBack.Trans_state[0]=0x02;//ֹͣʧ��
//							System.cancel_sweeping = 1;

                            Alarm.no_respond_locate = 19;
                            //App_printf("Task_MT2000 Sweep stop 19 no respond.\r\n");
                        }
                    }
                    else     //ȡ��ɨƵ��ʱ�������Ѿ����ɨƵ�������־
                    {
                        System.achieve_add_sub = 0;
                        System.modify_power = 0;
                        System.cancel_add_sub = 0;
                        System_Status_Clean();

                        if( get_history_alarm() == 0x01 )
                        {
                            System.status = SYSTEM_ALARM;
                        }
                        else
                        {
                            System.status = SYSTEM_STANDBY;
                        }

                        System.sweep = 0x02;
                        System.already_swept = 1;
                        System.cancel_sweeping = 0;				//ȡ��ɨƵ��־����
                        Trans_stopBack.Trans_state[0]=0xFE;		//ֹͣ�ɹ�
                    }

                    System.sweep = 0x00;
                }
                else	if( System.Voltage < 40.0 )   //�ػ�״̬
                {
                    System.achieve_add_sub = 0;
                    System.modify_power = 0;
                    System.cancel_add_sub = 0;
                    System_Status_Clean();

                    System.status = SYSTEM_SHUTDOWN;
                    System.open = 0x00;
                    System.sweep = 0x00;
                    System.already_swept = 0;
									Trans_printf(" ɨƵ���ڹػ�״̬ \n");
                }
            }
            else 	if((ulValue & BIT_6) != 0)	//����
            {

            }
            else 	if((ulValue & BIT_7) != 0)	//����
            {

            }
            else 	if((ulValue & BIT_8) != 0)	//����
            {
                GT2000_Open();
							  Trans_printf(" Task_MT2000���Ѵ��������¼� \n");
            }
            else 	if((ulValue & BIT_9) != 0)	//�ػ�
            {
								System.Power_Adjustment=1;
							
                GT2000_Close();
								Power_offBack.results[0]= 0xFE;	//�ػ��ɹ�
								Power_onBack.results[0]	= 0xFD;	//����ʧ��						

								System.close = 0;		//�ػ��ɹ�
								System_Status_Clean();
								System.Voltage = 0.0;
								System.Electricity = 0.0;
								System.achieve_add_sub = 0;
								System.modify_power = 0;
								System.cancel_add_sub = 0;
								System.sweep = 0;
								System.open = 0;

								if( get_history_alarm() == 0x01 )
								{
										System.status = SYSTEM_ALARM;
								}
								else
								{
										System.status = SYSTEM_SHUTDOWN;
								}
                                 

                //����ػ�֮�����Ͽ�������Ϊ��Ҫ�ȴ��ػ���ɣ��᷵�ؿ���ʧ�ܣ����� if(Power_onBack.results[0]==0xFD) ��ʾ���ڹػ����������ʾ�ػ����
                Power_onBack.results[0] = 0x00;			//�ػ���ɣ����Կ�����(���ܹػ���û�гɹ�)
					
								vTaskDelay(2000);
								System.Power_Adjustment=0;

            }
						
            else 	if((ulValue & BIT_10) != 0)	//��ѯ
            {						
								float power_change=0;
								for(repeat_send_counter=0; repeat_send_counter<NO_RESPOND_MAX; repeat_send_counter++)
								{
										Gt_Inquire_All();		//�����״̬��ѯ
										ret=GT2000_Wait_Ack(MT2000_CMD_INQUIRE);
										if( (ret == MT2000_ACK_OK) || (ret == MT2000_ACK_ALARM))
												break;
								}
                
                if( ret == MT2000_ACK_ALARM )	//����������
                {
                    GT2000_Alarm_Stop();
									Trans_printf(" ������������������ͣ�� ");
                }
                else	if( ret == MT2000_ACK_OK )
                {
										if(Old_Forward_Power!=0)															//���ӹ��������
										{
											if(System.status==SYSTEM_EMISSING&&System.Power_Adjustment!=1&&System.stop==0&&System.close==0)
											{	
												power_change=fabs(Old_Forward_Power-System.Forward_Power);
												if(System.mode==1)		//����ǵ�Ƶ			//������
												{
													if(power_change>=POWER_CHANGE_MAX)
													{
														Power_cataclysm_count++;
														if(Power_cataclysm_count>=3)
														{
															Alarm.power_cataclysm=1;
															Alarm.alarm_history = 1;
															GT2000_Alarm_Stop();
															System.status=SYSTEM_ALARM;	
															Power_cataclysm_count=0;		
															Trans_printf(" ��Ƶʱ����������350�����棡 ");
														}															
													}
													else
													{
														Power_cataclysm_count=0;
													}
												}
												else if(System.mode==2||System.mode==3)		//��Ƶʱ//������
												{
													if(System.Forward_Power<=10)
													{
														Alarm.power_cataclysm=1;
														Alarm.alarm_history = 1;
														GT2000_Alarm_Stop();
														System.status=SYSTEM_ALARM;
														Trans_printf(" ��Ƶʱ����������350�����棡 ");
													}
												}
											}
										}
										
										Old_Forward_Power=System.Forward_Power;
									
                    if( System.already_init == 0x00 )
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
													if(System.Power_Adjustment!=1)
													{
														System.emission = 0x00;
                            System.status = SYSTEM_STANDBY;			//��ѯʱ�������ˣ���һ���Ǵ������п����Ƿ���/��������/ɨƵ�ȡ�
													}
                        }
                    }
                    else
                    {
                        if( (System.status == SYSTEM_OPENING) || (System.open == 0x01) )
                        {

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
                    }
                }
								else if(ret==MT2000_NO_ACK)		//��Ӧ�𱨾�-------2021/4/9
								{
									if(System.status!=SYSTEM_UNINITIALIZE)		//δ��ʼ��ʱ��������
									{
										System.status=SYSTEM_ALARM;	
										Trans_printf(" Task_MT2000��ѯ�¼�����Ӧ�������� ");
									}										
								}

                if( System.emission == 0x01 )
                {
                    System.open = 0x02;
                }
            }
            else								//û�ж�Ӧ�¼�
            {
                //App_printf("û�ж�Ӧ���¼�!!!\r\n");
            }
        }
        else									//�ȴ���ʱ
        {
            //App_printf("Task_MT2000 TIME OUT!!!\r\n");
        }

        vTaskDelay(100);
    }
}
