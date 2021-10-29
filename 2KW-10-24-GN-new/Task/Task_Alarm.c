#include "Task_Config.h"
#include "bsp_uart_fifo.h"
#include "my_protocol.h"
#include "stand_wave_rate.h"
#include "alarm.h"
#include "power.h"

/*
 * 报警执行顺序 Alarm_t记录报警类型(置位)，Alarm_historyBack_t记录报警时间(0602返回给PC)，定时器vTimerCallback每5s打印一次报警信息
 * alarm_printf():根据Alarm的报警类型，打印相应报警提示
 * get_history_alarm():根据Alarm的报警信息，置位系统报警标记位System.alarm
 * clean_all_alarm_flags():清除Alarm所有报警标志位，同时清除系统报警标记位System.alarm
 * set_alarm_bit()：根据Alarm的报警类型，置位Alarm_historyBack的报警bit(0602返回给PC)
 */

TaskHandle_t xHandleTask_Alarm;

void Task_Alarm(void *pvParameters)
{
	uint8_t n;
    while(1)
    {
        if( System.status == SYSTEM_EMISSING )		//发射状态下电压电流报警检测(并检测无功率报警)
        {
           swr_power_alarm_check();	//驻波比和功率报警检测
					for(n=0;n>10;n++)
					{
					Trans_printf(" Task_Alarm正在发射状态下驻波比和功率报警检测！ \n");
					n=0;
					}
        }
				
				if(System.Control_Model==1)		//遥控开关被关闭了
				{
					if(System.status!=SYSTEM_UNINITIALIZE)
					{
						vTaskDelay(100);
						if(System.Control_Model==1)		//再次检测，防止误判
						{
						System.emission = 0x00;	
						Alarm.no_respond = 0x01;
						Trans_printf(" Alarm.no_respond 2 ! ");
						Alarm.alarm_history = 0x01;
						Trans_printf(" Task_Alarm遥控开关被关闭了！ \n");
						}
					}					
				}
				
        vTaskDelay(100);
    }
}
