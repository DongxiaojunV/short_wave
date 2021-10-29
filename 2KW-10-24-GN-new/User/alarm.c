#include "alarm.h"
#include "includes.h"
#include "my_protocol.h"
#include "bsp_uart_fifo.h"
#include "power.h"
#include "stand_wave_rate.h"

#include "main.h"

extern TaskHandle_t xHandleTask_MT2000;

/* ��AD��ȡƽ��ֵ */
void Read_Sum_AD(void)
{
    uint8_t i=0;
    float temp_adc_sum_f = 0;
    float temp_adc_sum_r = 0;
    float temp_adc_sum_v = 0;
    float temp_adc_sum_a = 0;

    for(i=0; i<AD_SAMPLE_NUM; i++)	//��ȡAD
    {
        ADC0_buff[i] = (float) ADC_ConvertedValue[0]/4096*3.3*3;
        ADC1_buff[i] = (float) ADC_ConvertedValue[1]/4096*3.3;
        ADC2_buff[i] = (float) ADC_ConvertedValue[2]/4096*3.3*2;
        ADC3_buff[i] = (float) ADC_ConvertedValue[3]/4096*3.3*2;
        vTaskDelay(1);
    }

    adc1_zero_count = 0;
    adc3_zero_count = 0;
    for(i=1; i<AD_SAMPLE_NUM; i++)
    {
        temp_adc_sum_f += ADC0_buff[i];		//������
        temp_adc_sum_r += ADC1_buff[i];		//������
        temp_adc_sum_v += ADC2_buff[i];		//��ѹ
        temp_adc_sum_a += ADC3_buff[i];		//����

        if(  ADC1_buff[i] < 0.001 )
            adc1_zero_count++;

        if(  ADC3_buff[i] < 0.001 )
            adc3_zero_count++;
    }

    System.ADC_Sum_F	= temp_adc_sum_f/AD_SAMPLE_NUM;						//������
    System.ADC_Sum_R	= temp_adc_sum_r/(AD_SAMPLE_NUM-adc1_zero_count);	//������
    System.ADC_Sum_V	= temp_adc_sum_v/AD_SAMPLE_NUM;						//��ѹ
    System.ADC_Sum_A	= temp_adc_sum_a/(AD_SAMPLE_NUM-adc3_zero_count);	//����
}

/* ��ѹ����������� */
void VA_alarm_check(void)
{
    if( (System.emission == 0x01) && (System.stop == 0x00) )	//���ڷ��䣬��û������ֹͣ
    {
        if( Alarm.over_Electric == 0 )												//�����������
        {
            if( System.Electricity >= Alarm_threshold.Upp_45I_limit[0] )
            {
                if( System.time_update_flag == 0x01 )
                {
                    Alarm.over_Electric_time_flag = 0x01;
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.power_45A_time[0]=set_time.tm_year-2000;
                    Alarm_historyBack.power_45A_time[1]=set_time.tm_mon;
                    Alarm_historyBack.power_45A_time[2]=set_time.tm_mday;
                    Alarm_historyBack.power_45A_time[3]=set_time.tm_hour;
                    Alarm_historyBack.power_45A_time[4]=set_time.tm_min;
                    Alarm_historyBack.power_45A_time[5]=set_time.tm_sec;
                }

                Alarm.alarm_history = 1;
                Alarm.over_Electric = 1;
                Alarm_historyBack.power_45A_history[0] = System.Electricity;
            }
			else	if( (System.Electricity >= (PROTECT_ADJUST*Alarm_threshold.Upp_45I_limit[0]) ) && (System.sweep != 0x01 ) )
			{
				if( System.Forward_Power < 500 )				//��С���ʵ�500W������������PROTECT_ADJUST��������������
				{
					System.protect_adjust = 0x00;
				}
				else	if( System.protect_adjust == 0x01 )		//���ڼ�С���ʣ��ȴ�����������
				{
					
				}
				else
				{
					System.protect_adjust = 0x01;
					Sub_Power.Power_DOWN[0] = 50;
					
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
	//				Sub_PowerBack.results[0]=0xFE;		//��С�ɹ�

	//				App_printf("Task_Upper_Computer Sub Notify!\r\n");
					xTaskNotify(xHandleTask_MT2000,	/* ��С���� */
								BIT_2,
								eSetBits);
				}
			}
			else	//����
			{
				if( System.Standing_wave_ratio > (PROTECT_ADJUST*(SWR_ALARM_THRESHOLD/10.0)) )	//��Ȼ��������������פ������Ҫ�������ڣ����ﲻ������
				{
					
				}
				else	if( System.protect_adjust == 0x01 )								//�������ڳɹ������ڽ�����ȡ����С����
				{
					System.protect_adjust = 0x00;
					System.cancel_add_sub = 0x01;
				}
				else																	//����������/����/ɨƵ
				{
					
				}
			}
        }
		
        if( System.Voltage >= Alarm_threshold.Upp_45V_limit[0] )					//��ѹ����(��ѹ)
        {
            if( Alarm.over_Voltage == 0 )
            {
                if( System.time_update_flag == 0x01 )
                {
                    Alarm.over_Voltage_time_flag = 0x01;
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.power_45V_time[0]=set_time.tm_year-2000;
                    Alarm_historyBack.power_45V_time[1]=set_time.tm_mon;
                    Alarm_historyBack.power_45V_time[2]=set_time.tm_mday;
                    Alarm_historyBack.power_45V_time[3]=set_time.tm_hour;
                    Alarm_historyBack.power_45V_time[4]=set_time.tm_min;
                    Alarm_historyBack.power_45V_time[5]=set_time.tm_sec;
                }

                Alarm.alarm_history = 1;
                Alarm.over_Voltage = 1;								
                Alarm_historyBack.power_45V_history[0] = System.Voltage;
								Trans_printf(" ��ѹ����1��");
            }
        }
        else	if( System.Voltage <= Alarm_threshold.Low_45V_limit[0] )			//��ѹ����(Ƿѹ)
        {
			if( Alarm.no_respond == 0x01 )		//�»����豸����Ӧʱ������Ƿѹ����
			{
				
			}
			else
	
				if( Alarm.low_Voltage == 0 )
				{
					if( System.time_update_flag == 0x01 )
					{
						Alarm.low_Voltage_time_flag = 0x01;
						Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
						Alarm_historyBack.power_45V_time[0]=set_time.tm_year-2000;
						Alarm_historyBack.power_45V_time[1]=set_time.tm_mon;
						Alarm_historyBack.power_45V_time[2]=set_time.tm_mday;
						Alarm_historyBack.power_45V_time[3]=set_time.tm_hour;
						Alarm_historyBack.power_45V_time[4]=set_time.tm_min;
						Alarm_historyBack.power_45V_time[5]=set_time.tm_sec;
					}

					Alarm.alarm_history = 1;
					Alarm.low_Voltage = 1;
					Alarm_historyBack.power_45V_history[0] = System.Voltage;
					Trans_printf(" Ƿѹ����1��");
				}
        }
    }
    else	if( (System.open==0x02) && (System.close!=1) )		//�Ѿ����������Ҳ������ڹػ��ż���ѹ����
    {
        if( System.Voltage >= Alarm_threshold.Upp_45V_limit[0] )					//��ѹ����(��ѹ)
        {
            if( Alarm.over_Voltage == 0 )
            {
                if( System.time_update_flag == 0x01 )
                {
                    Alarm.over_Voltage_time_flag = 0x01;
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.power_45V_time[0]=set_time.tm_year-2000;
                    Alarm_historyBack.power_45V_time[1]=set_time.tm_mon;
                    Alarm_historyBack.power_45V_time[2]=set_time.tm_mday;
                    Alarm_historyBack.power_45V_time[3]=set_time.tm_hour;
                    Alarm_historyBack.power_45V_time[4]=set_time.tm_min;
                    Alarm_historyBack.power_45V_time[5]=set_time.tm_sec;
                }

                Alarm.alarm_history = 1;
                Alarm.over_Voltage = 1;
                Alarm_historyBack.power_45V_history[0] = System.Voltage;
								Trans_printf(" ��ѹ����2��");
            }
        }
        else	if( System.Voltage <= Alarm_threshold.Low_45V_limit[0] )			//��ѹ����(Ƿѹ)
        {
            if( Alarm.low_Voltage == 0 )
            {
                if( System.time_update_flag == 0x01 )
                {
                    Alarm.low_Voltage_time_flag = 0x01;
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.power_45V_time[0]=set_time.tm_year-2000;
                    Alarm_historyBack.power_45V_time[1]=set_time.tm_mon;
                    Alarm_historyBack.power_45V_time[2]=set_time.tm_mday;
                    Alarm_historyBack.power_45V_time[3]=set_time.tm_hour;
                    Alarm_historyBack.power_45V_time[4]=set_time.tm_min;
                    Alarm_historyBack.power_45V_time[5]=set_time.tm_sec;
                }

                Alarm.alarm_history = 1;
                Alarm.low_Voltage = 1;
                Alarm_historyBack.power_45V_history[0] = System.Voltage;
								Trans_printf(" Ƿѹ����2��");
            }

        }
				
    }
}

/* פ���Ⱥ��޹��ʱ������ */
void swr_power_alarm_check(void)
{
		int wave_ratio=0;
		wave_ratio=System.Standing_wave_ratio*10;
    if( (System.emission == 0x01) && (System.stop == 0x00) )	//���ڷ��䣬��û������ֹͣ
    {
        if( wave_ratio >= SWR_ALARM_THRESHOLD && System.Reverse_Power>200)	//פ���ȱ���
        {
            if( Alarm.swr_alarm == 0 )
            {
                if( System.time_update_flag == 0x01 )
                {
                    Alarm.swr_alarm_time_flag = 0x01;
                    Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                    Alarm_historyBack.swr_time[0]=set_time.tm_year-2000;
                    Alarm_historyBack.swr_time[1]=set_time.tm_mon;
                    Alarm_historyBack.swr_time[2]=set_time.tm_mday;
                    Alarm_historyBack.swr_time[3]=set_time.tm_hour;
                    Alarm_historyBack.swr_time[4]=set_time.tm_min;
                    Alarm_historyBack.swr_time[5]=set_time.tm_sec;
                }
							
                Alarm.alarm_history = 1;
                Alarm.swr_alarm = 1;
								if(2.2<System.Standing_wave_ratio&&System.Standing_wave_ratio<2.4)		//�����ٽ�ֵС��2.4��Ч
								{
									System.Standing_wave_ratio=2.2;
								}
								else if(System.Standing_wave_ratio>=2.4)
								{
									System.Standing_wave_ratio=2.4;
								}
                Alarm_historyBack.swr_alarm[0] = System.Standing_wave_ratio;	//����ֵ
								Trans_printf(" פ���ȱ���1��");
            }
        }
				
//		if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//�޹��ʱ���
//		{
//		
//				if( Alarm.no_power == 0 )
//				{
//					if( System.time_update_flag == 0x01 )
//					{
//						Alarm.no_power_time_flag = 0x01;
//						Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
//						Alarm_historyBack.no_power_time[0]=set_time.tm_year-2000;
//						Alarm_historyBack.no_power_time[1]=set_time.tm_mon;
//						Alarm_historyBack.no_power_time[2]=set_time.tm_mday;
//						Alarm_historyBack.no_power_time[3]=set_time.tm_hour;
//						Alarm_historyBack.no_power_time[4]=set_time.tm_min;
//						Alarm_historyBack.no_power_time[5]=set_time.tm_sec;
//					}

//					Alarm.alarm_history = 1;
//					Alarm.no_power = 1;
//				}
//		}
		
		if( (System.Reverse_Power >= 150 ) && (System.sweep != 0x01 ) )			//�����ʴ���150W���Ҳ���ɨƵ״̬
		{
			if( System.Forward_Power < 500 )				//��С���ʵ�500W�������ʶ�����150W��������������
			{
				System.protect_adjust = 0x00;
			}
			else	if( System.protect_adjust == 0x01 )		//���ڼ�С���ʣ��ȴ�����������
			{
				
			}
			else											//��Ҫ���뱣��
			{
				System.protect_adjust = 0x01;
				Sub_Power.Power_DOWN[0] = 50;
				
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
//				Sub_PowerBack.results[0]=0xFE;		//��С�ɹ�

//				App_printf("Task_Upper_Computer Sub Notify!\r\n");
				xTaskNotify(xHandleTask_MT2000,	/* ��С���� */
							BIT_2,
							eSetBits);
			}
		}
		else	//����
		{
			if( System.Electricity >= (PROTECT_ADJUST*Alarm_threshold.Upp_45I_limit[0]) )	//��Ȼ���������������ǵ�����Ҫ�������ڣ����ﲻ������
			{
				
			}
			else	if( System.protect_adjust == 0x01 )								//�������ڳɹ������ڽ�����ȡ����С����
			{
				System.protect_adjust = 0x00;
				System.cancel_add_sub = 0x01;
			}
			else																	//����������/����/ɨƵ
			{
				
			}
		}
    }
}

/* ��ӡ������Ϣ */
void alarm_printf(void)
{
    if( Alarm.alarm_history == 1 )			//��ʷ������־
    {
        if( Alarm.emission == 1 )			//�����������
        {
            if( Alarm.emission_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.emission_time[0],
                           Alarm_historyBack.emission_time[1],
                           Alarm_historyBack.emission_time[2],
                           Alarm_historyBack.emission_time[3],
                           Alarm_historyBack.emission_time[4],
                           Alarm_historyBack.emission_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.emission_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.emission_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.emission_time[1]=set_time.tm_mon;
                Alarm_historyBack.emission_time[2]=set_time.tm_mday;
                Alarm_historyBack.emission_time[3]=set_time.tm_hour;
                Alarm_historyBack.emission_time[4]=set_time.tm_min;
                Alarm_historyBack.emission_time[5]=set_time.tm_sec;
            }

            App_printf("emission alarm alarm alarm!\r\n");
        }

        if( Alarm.no_respond == 1 )			//����������Ӧ����
        {
            if( Alarm.no_respond_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.no_response_time[0],
                           Alarm_historyBack.no_response_time[1],
                           Alarm_historyBack.no_response_time[2],
                           Alarm_historyBack.no_response_time[3],
                           Alarm_historyBack.no_response_time[4],
                           Alarm_historyBack.no_response_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.no_respond_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.no_response_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.no_response_time[1]=set_time.tm_mon;
                Alarm_historyBack.no_response_time[2]=set_time.tm_mday;
                Alarm_historyBack.no_response_time[3]=set_time.tm_hour;
                Alarm_historyBack.no_response_time[4]=set_time.tm_min;
                Alarm_historyBack.no_response_time[5]=set_time.tm_sec;
            }

            App_printf("no_respond alarm alarm alarm!\r\n");
        }

        if( Alarm.over_Electric == 1 )		//��������
        {
            if( Alarm.over_Electric_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.power_45A_time[0],
                           Alarm_historyBack.power_45A_time[1],
                           Alarm_historyBack.power_45A_time[2],
                           Alarm_historyBack.power_45A_time[3],
                           Alarm_historyBack.power_45A_time[4],
                           Alarm_historyBack.power_45A_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.over_Electric_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.power_45A_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.power_45A_time[1]=set_time.tm_mon;
                Alarm_historyBack.power_45A_time[2]=set_time.tm_mday;
                Alarm_historyBack.power_45A_time[3]=set_time.tm_hour;
                Alarm_historyBack.power_45A_time[4]=set_time.tm_min;
                Alarm_historyBack.power_45A_time[5]=set_time.tm_sec;
            }

            App_printf("Alarm_historyBack.power_45A_history[0] = %f\r\n",	Alarm_historyBack.power_45A_history[0]);
            App_printf("over_Electric alarm alarm alarm!\r\n");
        }

        if( Alarm.low_Voltage == 1 || Alarm.over_Voltage == 1 )		//Ƿѹ/��ѹ����
        {
            if( (Alarm.low_Voltage_time_flag == 0x01) || (Alarm.over_Voltage_time_flag == 0x01) )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.power_45V_time[0],
                           Alarm_historyBack.power_45V_time[1],
                           Alarm_historyBack.power_45V_time[2],
                           Alarm_historyBack.power_45V_time[3],
                           Alarm_historyBack.power_45V_time[4],
                           Alarm_historyBack.power_45V_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.over_Voltage_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.power_45V_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.power_45V_time[1]=set_time.tm_mon;
                Alarm_historyBack.power_45V_time[2]=set_time.tm_mday;
                Alarm_historyBack.power_45V_time[3]=set_time.tm_hour;
                Alarm_historyBack.power_45V_time[4]=set_time.tm_min;
                Alarm_historyBack.power_45V_time[5]=set_time.tm_sec;
            }

            if( Alarm.low_Voltage == 1 )
            {
                App_printf("Alarm_threshold.Low_45V_limit[0] = %f\r\n", Alarm_threshold.Low_45V_limit[0]);
                App_printf("Alarm_historyBack.power_45V_history = %f\r\n", Alarm_historyBack.power_45V_history[0]);
                App_printf("Alarm.low_Voltage alarm alarm alarm!\r\n");
            }
            if( Alarm.over_Voltage == 1 )
            {
                App_printf("Alarm_threshold.Upp_45V_limit[0] = %f\r\n", Alarm_threshold.Upp_45V_limit[0]);
                App_printf("Alarm_historyBack.power_45V_history = %f\r\n", Alarm_historyBack.power_45V_history[0]);
                App_printf("Alarm.over_Voltage alarm alarm alarm!\r\n");
            }
        }

        if( Alarm.swr_alarm == 1 )			//פ���ȱ���
        {
            if( Alarm.swr_alarm_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.swr_time[0],
                           Alarm_historyBack.swr_time[1],
                           Alarm_historyBack.swr_time[2],
                           Alarm_historyBack.swr_time[3],
                           Alarm_historyBack.swr_time[4],
                           Alarm_historyBack.swr_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.swr_alarm_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.swr_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.swr_time[1]=set_time.tm_mon;
                Alarm_historyBack.swr_time[2]=set_time.tm_mday;
                Alarm_historyBack.swr_time[3]=set_time.tm_hour;
                Alarm_historyBack.swr_time[4]=set_time.tm_min;
                Alarm_historyBack.swr_time[5]=set_time.tm_sec;
            }

            App_printf("Alarm_historyBack.swr_alarm[0] = %f\r\n", Alarm_historyBack.swr_alarm[0]);
            App_printf("swr_alarm alarm alarm alarm!\r\n");
        }

        if( Alarm.no_power == 1 )			//�޹����������
        {
            if( Alarm.no_power_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.no_power_time[0],
                           Alarm_historyBack.no_power_time[1],
                           Alarm_historyBack.no_power_time[2],
                           Alarm_historyBack.no_power_time[3],
                           Alarm_historyBack.no_power_time[4],
                           Alarm_historyBack.no_power_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
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

            App_printf("no_power alarm alarm alarm!\r\n");
        }

        if( Alarm.temperature_alarm == 1 )	//�¶ȱ���
        {
            if( Alarm.temperature_alarm_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.temp_time[0],
                           Alarm_historyBack.temp_time[1],
                           Alarm_historyBack.temp_time[2],
                           Alarm_historyBack.temp_time[3],
                           Alarm_historyBack.temp_time[4],
                           Alarm_historyBack.temp_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.temperature_alarm_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.temp_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.temp_time[1]=set_time.tm_mon;
                Alarm_historyBack.temp_time[2]=set_time.tm_mday;
                Alarm_historyBack.temp_time[3]=set_time.tm_hour;
                Alarm_historyBack.temp_time[4]=set_time.tm_min;
                Alarm_historyBack.temp_time[5]=set_time.tm_sec;
            }

            App_printf("Temperature=%f C\r\n", Alarm_historyBack.temp_history);
            App_printf("temperature_alarm alarm alarm alarm!\r\n");
        }

        if( Alarm.humidity_alarm == 1 )		//ʪ�ȱ���
        {
            if( Alarm.humidity_alarm_time_flag == 0x01 )
            {
                App_printf("\r\n20%d-%d-%d	%0.2d:%0.2d:%0.2d\r\n",
                           Alarm_historyBack.humidity_time[0],
                           Alarm_historyBack.humidity_time[1],
                           Alarm_historyBack.humidity_time[2],
                           Alarm_historyBack.humidity_time[3],
                           Alarm_historyBack.humidity_time[4],
                           Alarm_historyBack.humidity_time[5]);
            }
            else	if( System.time_update_flag == 0x01 )
            {
                Alarm.humidity_alarm_time_flag = 0x01;
                Time_Printf( (RTC_GetCounter()+g_stamp_distance), &set_time );
                Alarm_historyBack.humidity_time[0]=set_time.tm_year-2000;
                Alarm_historyBack.humidity_time[1]=set_time.tm_mon;
                Alarm_historyBack.humidity_time[2]=set_time.tm_mday;
                Alarm_historyBack.humidity_time[3]=set_time.tm_hour;
                Alarm_historyBack.humidity_time[4]=set_time.tm_min;
                Alarm_historyBack.humidity_time[5]=set_time.tm_sec;
            }

            App_printf("Humidity=%f%% \r\n", Alarm_historyBack.humidity_history);
            App_printf("humidity_alarm alarm alarm alarm!\r\n");
        }
    }


}

/*����1��ʾ����ʷ������Ϣ*/
int get_history_alarm(void)
{
    uint8_t ret = 0;
//	if( Alarm.alarm_history == 1 )		//��ʷ������־
//		ret = 1;

    set_alarm_bit();

    if( Alarm.emission == 1 )			//�����������
    {
        Alarm.alarm_history = 1;
        ret = 1;
			  Trans_printf(" ����������� �� \n");
        return ret;
			
    }

    if( Alarm.no_respond == 1 )			//����������Ӧ����
    {
        Alarm.alarm_history = 1;
        ret = 1;
			  Trans_printf(" ����������Ӧ���� �� \n");
        return ret;
    }

    if( Alarm.over_Electric == 1 )		//��������
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" �������� �� \n");
        return ret;
    }

    if( Alarm.low_Voltage == 1 )		//Ƿѹ����
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" Ƿѹ���� �� \n");
        return ret;
    }

    if( Alarm.over_Voltage == 1 )		//��ѹ����
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" ��ѹ���� �� \n");
        return ret;
    }

    if( Alarm.swr_alarm == 1 )			//פ���ȱ���
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" פ���ȱ��� �� \n");
        return ret;
    }

    if( Alarm.no_power == 1 )			//�޹����������
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" �޹���������� �� \n");
        return ret;
    }

    if( Alarm.temperature_alarm == 1 )	//�¶ȱ���
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" �¶ȱ��� �� \n");
        return ret;
    }

    if( Alarm.humidity_alarm == 1 )		//ʪ�ȱ���
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" ʪ�ȱ��� �� \n");
        return ret;
    }
//   Trans_printf(" ����ʷ����������0�� \n");
    Alarm.alarm_history = 0;
    ret = 0;
    return ret;
}

/* ������б�����־λ */
void clean_all_alarm_flags(void)
{
    Alarm.alarm_history=0;		//��ʷ������־
    Alarm.emission=0;			//�����������
    Alarm.no_respond=0;			//����������Ӧ����

    Alarm.over_Electric=0;		//��������
    Alarm.low_Voltage=0;		//Ƿѹ����
    Alarm.over_Voltage=0;		//��ѹ����
    Alarm.swr_alarm=0;			//פ���ȱ���
    Alarm.no_power=0;			//�޹����������

    Alarm.temperature_alarm=0;	//�¶ȱ���
    Alarm.humidity_alarm=0;		//ʪ�ȱ���
		Alarm.power_cataclysm=0;
    //��ʷ������ȫ����������
    memset(&Alarm_historyBack,0,sizeof(Alarm_historyBack));
}

/* ��λ������־λ */
void set_alarm_bit(void)
{
    Alarm_historyBack.alarm_history[0] = 0x00;
    Alarm_historyBack.alarm_history[1] = 0x00;

    if( Alarm.no_power == 1 )		//�޹��ʱ���		//������Task_Alarm()����ѹ���� �� MT2000_Rx��MT2000_Cmd_Analyze()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<0;
    }

    if(Alarm.temperature_alarm==1)	//�¶ȱ���			//Task_TH()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<1;
    }

    if(Alarm.humidity_alarm==1)		//ʪ�ȱ���			//Task_TH()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<2;
    }

    if(Alarm.over_Electric==1)		//��������			//Task_Alarm()����ѹ����
    {
        Alarm_historyBack.alarm_history[0] |= 1<<3;
    }

    if(Alarm.over_Voltage==1)		//��ѹ����			//Task_Alarm()����ѹ����
    {
        Alarm_historyBack.alarm_history[0] |= 1<<4;
    }

    if(Alarm.low_Voltage==1)		//Ƿѹ����			//Task_Alarm()����ѹ����
    {
        Alarm_historyBack.alarm_history[0] |= 1<<5;
    }

    if(Alarm.swr_alarm==1)			//פ���ȱ���		//MT2000_Rx��MT2000_Cmd_Analyze()	(�ݲ�ͨ����ȡAD��ù��ʣ�����פ���ȣ���ⱨ��)
    {
        Alarm_historyBack.alarm_history[0] |= 1<<6;
    }

    if(Alarm.no_respond==1)			//����������Ӧ����	//MT2000_Rx��MT2000_Wait_Ack()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<7;
    }

    if(Alarm.emission==1)			//�����������	//MT2000_Rx��MT2000_Cmd_Analyze()
    {
        Alarm_historyBack.alarm_history[1] |= 1<<0;
    }
		
		if(Alarm.power_cataclysm==1)	//������䱨��
		{
				Alarm_historyBack.alarm_history[1] |= 1<<2;		//��1λ����λ��ռ��,������һλ
		}
}

void emission_alarm_content_clean(void)
{
	uint8_t	i = 0;
	
	for(i=0; i<40; i++)
	{
		Alarm_historyBack.emission_alarm_content[i] = 0x00;
	}
}
