#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "stand_wave_rate.h"
#include "alarm.h"
#include "power.h"

/*
 * ����ִ��˳�� Alarm_t��¼��������(��λ)��Alarm_historyBack_t��¼����ʱ��(0602���ظ�PC)����ʱ��vTimerCallbackÿ5s��ӡһ�α�����Ϣ
 * alarm_printf():����Alarm�ı������ͣ���ӡ��Ӧ������ʾ
 * get_history_alarm():����Alarm�ı�����Ϣ����λϵͳ�������λSystem.alarm
 * clean_all_alarm_flags():���Alarm���б�����־λ��ͬʱ���ϵͳ�������λSystem.alarm
 * set_alarm_bit()������Alarm�ı������ͣ���λAlarm_historyBack�ı���bit(0602���ظ�PC)
 */

TaskHandle_t xHandleTask_Alarm;

void Task_Alarm(void *pvParameters)
{
	uint8_t n;
    while(1)
    {
        if( System.status == SYSTEM_EMISSING )		//����״̬�µ�ѹ�����������(������޹��ʱ���)
        {
           swr_power_alarm_check();	//פ���Ⱥ͹��ʱ������
					for(n=0;n>10;n++)
					{
					Trans_printf(" Task_Alarm���ڷ���״̬��פ���Ⱥ͹��ʱ�����⣡ \n");
					n=0;
					}
        }
				
				if(System.Control_Model==1)		//ң�ؿ��ر��ر���
				{
					if(System.status!=SYSTEM_UNINITIALIZE)
					{
						vTaskDelay(100);
						if(System.Control_Model==1)		//�ٴμ�⣬��ֹ����
						{
						System.emission = 0x00;	
						Alarm.no_respond = 0x01;
						Trans_printf(" Alarm.no_respond 2 ! ");
						Alarm.alarm_history = 0x01;
						Trans_printf(" Task_Alarmң�ؿ��ر��ر��ˣ� \n");
						}
					}					
				}
				
        vTaskDelay(100);
    }
}
