#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"

TaskHandle_t xHandleTask_TH;
Temperature_Humidity_t Temperature_Humidity;


int TH_Analyze(uint8_t *buff, int len);
uint8_t TH_Cheak(void);

/*
*********************************************************************************************************
*	����UART2����ʪ�����ݲ�����
*********************************************************************************************************
*/
void Task_TH(void *pvParameters)
{
    uint8_t fail_count = 0;
    uint8_t read;
    uint8_t buf[30];
    int count =0;

    while(1)
    {
        while(comGetChar(COM2,&read))
        {
            buf[count++]=read;
            vTaskDelay(1);
        }

        if(count>2)
        {
            if(TH_Analyze(buf,count))
            {
                fail_count = 0;
            }
            else
            {
                fail_count++;
                if( fail_count >= 100 )		//10s���ղ������ݣ���ʪ�ȵ��ߣ���ֵ����
                {
                    fail_count = 0;

                    comClearRxFifo(COM2);
                    Temperature_Humidity.Humidity = 0.0;
                    Temperature_Humidity.Temperature = 0.0;
                }
            }

            count=0;
        }
        else
        {
            fail_count++;
            if( fail_count >= 100 )		//10s���ղ������ݣ���ʪ�ȵ��ߣ���ֵ����
            {
                fail_count = 0;

                comClearRxFifo(COM2);
                Temperature_Humidity.Humidity = 0.0;
                Temperature_Humidity.Temperature = 0.0;
            }
        }

        TH_Cheak();
        vTaskDelay(100);
    }
}

/* ��ʪ�Ƚ��������� */
int TH_Analyze(uint8_t *buff, int len)
{
    uint16_t Temperature=0;
    uint16_t Humidity=0;
    uint8_t i = 0;

    if(len<=8)
    {
        return 0;
    }
    else
    {
        if( buff[i]==0x01 && buff[i+1]==0x03 && buff[i+2]==0x04 )	//������Ч	�޸ĵ�ַ
        {
			#if	NEW_TH_EN
					
					  #if New_Th
							//ʪ��
							Humidity = (buff[i+5]<<8) + buff[i+6];					//ʪ��
							Temperature_Humidity.Humidity = Humidity/10.0;

							if( Temperature_Humidity.Humidity < 0.0 )				//�޷�
							{
									Temperature_Humidity.Humidity = 0;
							}
							else	if( Temperature_Humidity.Humidity > 100.0 )
							{
									Temperature_Humidity.Humidity = 100;
							}
				
							//�¶�
							Temperature= (buff[i+3]<<8) + buff[i+4];				//�¶�
							Temperature_Humidity.Temperature = Temperature/10.0;

							if( Temperature_Humidity.Temperature < (-40.0) )		//���ﲻ��ȷ��������float��int�Ƚ�
							{
									Temperature_Humidity.Temperature = -40;
							}
							else	if( Temperature_Humidity.Temperature > 100.0)
							{
									Temperature_Humidity.Temperature = 100;
							}
						#else
				//ʪ��
							Humidity = (buff[i+3]<<8) + buff[i+4];					//ʪ��
							Temperature_Humidity.Humidity = Humidity/10.0;

							if( Temperature_Humidity.Humidity < 0.0 )				//�޷�
							{
									Temperature_Humidity.Humidity = 0;
							}
							else	if( Temperature_Humidity.Humidity > 100.0 )
							{
									Temperature_Humidity.Humidity = 100;
							}
				
							//�¶�
							Temperature= (buff[i+5]<<8) + buff[i+6];				//�¶�
							Temperature_Humidity.Temperature = Temperature/10.0;

							if( Temperature_Humidity.Temperature < (-40.0) )		//���ﲻ��ȷ��������float��int�Ƚ�
							{
									Temperature_Humidity.Temperature = -40;
							}
							else	if( Temperature_Humidity.Temperature > 100.0)
							{
									Temperature_Humidity.Temperature = 100;
							}
			#endif
			#else
			//�¶�
            Temperature = (buff[i+3]<<8) + buff[i+4];				//�¶�
            Temperature_Humidity.Temperature = Temperature/10.0 - 40;

            if( Temperature_Humidity.Temperature < (-40.0) )		//���ﲻ��ȷ��������float��int�Ƚ�
            {
                Temperature_Humidity.Temperature = -40;
            }
            else	if( Temperature_Humidity.Temperature > 100.0)
            {
                Temperature_Humidity.Temperature = 100;
            }

            //ʪ��
            Humidity=(buff[i+5]<<8) + buff[i+6];					//ʪ��
            Temperature_Humidity.Humidity = Humidity/10.0;

            if( Temperature_Humidity.Humidity < 0.0 )				//�޷�
            {
                Temperature_Humidity.Humidity = 0;
            }
            else if( Temperature_Humidity.Humidity > 100.0)
            {
                Temperature_Humidity.Humidity = 100;
            }
			#endif

            return 1;

        }
    }
    return 0;
}

/* ��ʪ�ȱ�����⺯�� */
uint8_t TH_Cheak(void)
{
    uint8_t ret = 0x00;
    static uint8_t TH_alarm_counter = 0x00;

    TH_alarm_counter++;
    if( TH_alarm_counter >= 50 )
    {
        TH_alarm_counter = 0x00;
        //App_printf("Temperature=%f C ,Humidity=%f%% \r\n",Temperature_Humidity.Temperature,Temperature_Humidity.Humidity);
    }

    if( System.already_init == 1 )   //�Ƿ��ʼ��
    {
        if( (Temperature_Humidity.Humidity>Alarm_threshold.Upp_humidity_limit[0]) || 
			(Temperature_Humidity.Humidity<Alarm_threshold.Low_humidity_limit[0]) )
        {
            ret = 0x01;

            if( Alarm.humidity_alarm == 0 )   //��PC��û���������ʱ��ֻ�����һ�εı���ֵ
            {
                if( System.time_update_flag == 0x01 )
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

                Alarm.alarm_history = 1;
                Alarm.humidity_alarm = 1;
                Alarm_historyBack.humidity_history[0]=Temperature_Humidity.Humidity;	//����ֵ

                //App_printf("Humidity alarm alarm alarm!\r\n");
            }
        }

        if(Temperature_Humidity.Temperature>Alarm_threshold.Upp_temp_limit[0] || Temperature_Humidity.Temperature<Alarm_threshold.Low_temp_limit[0])
        {
            ret = 0x01;

            if( Alarm.temperature_alarm == 0 )   //��PC��û���������ʱ��ֻ�����һ�εı���ֵ
            {
                if( System.time_update_flag == 0x01 )
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

                Alarm.alarm_history = 1;
                Alarm.temperature_alarm = 1;
                Alarm_historyBack.temp_history[0]=Temperature_Humidity.Temperature;	//����ֵ

                //App_printf("Temperature alarm alarm alarm!\r\n");
            }
        }
    }

    return ret;
}
