#include "alarm.h"
#include "includes.h"
#include "my_protocol.h"
#include "bsp_uart_fifo.h"
#include "power.h"
#include "stand_wave_rate.h"

#include "main.h"

extern TaskHandle_t xHandleTask_MT2000;

/* 读AD并取平均值 */
void Read_Sum_AD(void)
{
    uint8_t i=0;
    float temp_adc_sum_f = 0;
    float temp_adc_sum_r = 0;
    float temp_adc_sum_v = 0;
    float temp_adc_sum_a = 0;

    for(i=0; i<AD_SAMPLE_NUM; i++)	//读取AD
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
        temp_adc_sum_f += ADC0_buff[i];		//正向功率
        temp_adc_sum_r += ADC1_buff[i];		//反向功率
        temp_adc_sum_v += ADC2_buff[i];		//电压
        temp_adc_sum_a += ADC3_buff[i];		//电流

        if(  ADC1_buff[i] < 0.001 )
            adc1_zero_count++;

        if(  ADC3_buff[i] < 0.001 )
            adc3_zero_count++;
    }

    System.ADC_Sum_F	= temp_adc_sum_f/AD_SAMPLE_NUM;						//正向功率
    System.ADC_Sum_R	= temp_adc_sum_r/(AD_SAMPLE_NUM-adc1_zero_count);	//反向功率
    System.ADC_Sum_V	= temp_adc_sum_v/AD_SAMPLE_NUM;						//电压
    System.ADC_Sum_A	= temp_adc_sum_a/(AD_SAMPLE_NUM-adc3_zero_count);	//电流
}

/* 电压电流报警检测 */
void VA_alarm_check(void)
{
    if( (System.emission == 0x01) && (System.stop == 0x00) )	//正在发射，且没有正在停止
    {
        if( Alarm.over_Electric == 0 )												//过流报警检测
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
				if( System.Forward_Power < 500 )				//减小功率到500W，电流都大于PROTECT_ADJUST，不再做处理了
				{
					System.protect_adjust = 0x00;
				}
				else	if( System.protect_adjust == 0x01 )		//正在减小功率，等待，不做处理
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
	//				Sub_PowerBack.results[0]=0xFE;		//减小成功

	//				App_printf("Task_Upper_Computer Sub Notify!\r\n");
					xTaskNotify(xHandleTask_MT2000,	/* 减小功率 */
								BIT_2,
								eSetBits);
				}
			}
			else	//正常
			{
				if( System.Standing_wave_ratio > (PROTECT_ADJUST*(SWR_ALARM_THRESHOLD/10.0)) )	//虽然电流正常，但是驻波比需要保护调节，这里不做处理
				{
					
				}
				else	if( System.protect_adjust == 0x01 )								//保护调节成功，调节结束，取消减小功率
				{
					System.protect_adjust = 0x00;
					System.cancel_add_sub = 0x01;
				}
				else																	//正常的增减/发射/扫频
				{
					
				}
			}
        }
		
        if( System.Voltage >= Alarm_threshold.Upp_45V_limit[0] )					//电压报警(过压)
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
								Trans_printf(" 过压报警1！");
            }
        }
        else	if( System.Voltage <= Alarm_threshold.Low_45V_limit[0] )			//电压报警(欠压)
        {
			if( Alarm.no_respond == 0x01 )		//新机器设备无响应时，不报欠压报警
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
					Trans_printf(" 欠压报警1！");
				}
        }
    }
    else	if( (System.open==0x02) && (System.close!=1) )		//已经开机，并且不是正在关机才检测电压报警
    {
        if( System.Voltage >= Alarm_threshold.Upp_45V_limit[0] )					//电压报警(过压)
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
								Trans_printf(" 过压报警2！");
            }
        }
        else	if( System.Voltage <= Alarm_threshold.Low_45V_limit[0] )			//电压报警(欠压)
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
								Trans_printf(" 欠压报警2！");
            }

        }
				
    }
}

/* 驻波比和无功率报警检测 */
void swr_power_alarm_check(void)
{
		int wave_ratio=0;
		wave_ratio=System.Standing_wave_ratio*10;
    if( (System.emission == 0x01) && (System.stop == 0x00) )	//正在发射，且没有正在停止
    {
        if( wave_ratio >= SWR_ALARM_THRESHOLD && System.Reverse_Power>200)	//驻波比报警
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
								if(2.2<System.Standing_wave_ratio&&System.Standing_wave_ratio<2.4)		//报警临界值小于2.4有效
								{
									System.Standing_wave_ratio=2.2;
								}
								else if(System.Standing_wave_ratio>=2.4)
								{
									System.Standing_wave_ratio=2.4;
								}
                Alarm_historyBack.swr_alarm[0] = System.Standing_wave_ratio;	//报警值
								Trans_printf(" 驻波比报警1！");
            }
        }
				
//		if( (System.Forward_Power <= 10.0) && (System.Electricity >= 10.0) )	//无功率报警
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
		
		if( (System.Reverse_Power >= 150 ) && (System.sweep != 0x01 ) )			//反向功率大于150W，且不是扫频状态
		{
			if( System.Forward_Power < 500 )				//减小功率到500W，反向功率都大于150W，不再做处理了
			{
				System.protect_adjust = 0x00;
			}
			else	if( System.protect_adjust == 0x01 )		//正在减小功率，等待，不做处理
			{
				
			}
			else											//需要进入保护
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
//				Sub_PowerBack.results[0]=0xFE;		//减小成功

//				App_printf("Task_Upper_Computer Sub Notify!\r\n");
				xTaskNotify(xHandleTask_MT2000,	/* 减小功率 */
							BIT_2,
							eSetBits);
			}
		}
		else	//正常
		{
			if( System.Electricity >= (PROTECT_ADJUST*Alarm_threshold.Upp_45I_limit[0]) )	//虽然反向功率正常，但是电流需要保护调节，这里不做处理
			{
				
			}
			else	if( System.protect_adjust == 0x01 )								//保护调节成功，调节结束，取消减小功率
			{
				System.protect_adjust = 0x00;
				System.cancel_add_sub = 0x01;
			}
			else																	//正常的增减/发射/扫频
			{
				
			}
		}
    }
}

/* 打印报警信息 */
void alarm_printf(void)
{
    if( Alarm.alarm_history == 1 )			//历史报警标志
    {
        if( Alarm.emission == 1 )			//发射机自身报警
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

        if( Alarm.no_respond == 1 )			//激励器无响应报警
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

        if( Alarm.over_Electric == 1 )		//过流报警
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

        if( Alarm.low_Voltage == 1 || Alarm.over_Voltage == 1 )		//欠压/过压报警
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

        if( Alarm.swr_alarm == 1 )			//驻波比报警
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

        if( Alarm.no_power == 1 )			//无功率输出报警
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

        if( Alarm.temperature_alarm == 1 )	//温度报警
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

        if( Alarm.humidity_alarm == 1 )		//湿度报警
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

/*返回1表示有历史报警消息*/
int get_history_alarm(void)
{
    uint8_t ret = 0;
//	if( Alarm.alarm_history == 1 )		//历史报警标志
//		ret = 1;

    set_alarm_bit();

    if( Alarm.emission == 1 )			//发射机自身报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			  Trans_printf(" 发射机自身报警 ！ \n");
        return ret;
			
    }

    if( Alarm.no_respond == 1 )			//激励器无响应报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			  Trans_printf(" 激励器无响应报警 ！ \n");
        return ret;
    }

    if( Alarm.over_Electric == 1 )		//过流报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 过流报警 ！ \n");
        return ret;
    }

    if( Alarm.low_Voltage == 1 )		//欠压报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 欠压报警 ！ \n");
        return ret;
    }

    if( Alarm.over_Voltage == 1 )		//过压报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 过压报警 ！ \n");
        return ret;
    }

    if( Alarm.swr_alarm == 1 )			//驻波比报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 驻波比报警 ！ \n");
        return ret;
    }

    if( Alarm.no_power == 1 )			//无功率输出报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 无功率输出报警 ！ \n");
        return ret;
    }

    if( Alarm.temperature_alarm == 1 )	//温度报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 温度报警 ！ \n");
        return ret;
    }

    if( Alarm.humidity_alarm == 1 )		//湿度报警
    {
        Alarm.alarm_history = 1;
        ret = 1;
			Trans_printf(" 湿度报警 ！ \n");
        return ret;
    }
//   Trans_printf(" 无历史报警，返回0！ \n");
    Alarm.alarm_history = 0;
    ret = 0;
    return ret;
}

/* 清除所有报警标志位 */
void clean_all_alarm_flags(void)
{
    Alarm.alarm_history=0;		//历史报警标志
    Alarm.emission=0;			//发射机自身报警
    Alarm.no_respond=0;			//激励器无响应报警

    Alarm.over_Electric=0;		//过流报警
    Alarm.low_Voltage=0;		//欠压报警
    Alarm.over_Voltage=0;		//过压报警
    Alarm.swr_alarm=0;			//驻波比报警
    Alarm.no_power=0;			//无功率输出报警

    Alarm.temperature_alarm=0;	//温度报警
    Alarm.humidity_alarm=0;		//湿度报警
		Alarm.power_cataclysm=0;
    //历史报警包全部数据清零
    memset(&Alarm_historyBack,0,sizeof(Alarm_historyBack));
}

/* 置位报警标志位 */
void set_alarm_bit(void)
{
    Alarm_historyBack.alarm_history[0] = 0x00;
    Alarm_historyBack.alarm_history[1] = 0x00;

    if( Alarm.no_power == 1 )		//无功率报警		//两处：Task_Alarm()检测电压电流 和 MT2000_Rx的MT2000_Cmd_Analyze()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<0;
    }

    if(Alarm.temperature_alarm==1)	//温度报警			//Task_TH()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<1;
    }

    if(Alarm.humidity_alarm==1)		//湿度报警			//Task_TH()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<2;
    }

    if(Alarm.over_Electric==1)		//过流报警			//Task_Alarm()检测电压电流
    {
        Alarm_historyBack.alarm_history[0] |= 1<<3;
    }

    if(Alarm.over_Voltage==1)		//过压报警			//Task_Alarm()检测电压电流
    {
        Alarm_historyBack.alarm_history[0] |= 1<<4;
    }

    if(Alarm.low_Voltage==1)		//欠压报警			//Task_Alarm()检测电压电流
    {
        Alarm_historyBack.alarm_history[0] |= 1<<5;
    }

    if(Alarm.swr_alarm==1)			//驻波比报警		//MT2000_Rx的MT2000_Cmd_Analyze()	(暂不通过读取AD获得功率，计算驻波比，检测报警)
    {
        Alarm_historyBack.alarm_history[0] |= 1<<6;
    }

    if(Alarm.no_respond==1)			//激励器无响应报警	//MT2000_Rx的MT2000_Wait_Ack()
    {
        Alarm_historyBack.alarm_history[0] |= 1<<7;
    }

    if(Alarm.emission==1)			//发射机自身报警	//MT2000_Rx的MT2000_Cmd_Analyze()
    {
        Alarm_historyBack.alarm_history[1] |= 1<<0;
    }
		
		if(Alarm.power_cataclysm==1)	//功率骤变报警
		{
				Alarm_historyBack.alarm_history[1] |= 1<<2;		//第1位被上位机占用,往后延一位
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
