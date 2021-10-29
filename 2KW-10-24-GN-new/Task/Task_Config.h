#ifndef	_TASK_CONFIG_H_
#define	_TASK_CONFIG_H_

#include "includes.h"
#include "main.h"

typedef struct
{
	char *C_IP;		//字符型IP
	char *Socket0_Port;	
	char *Socket1_Port;
	char *Socket2_Port;
	char *Stm32_ID;
	char Buff_Pack[100];		//回复包
	uint8_t Pack_Creat_Mark;
}Socket2;

extern uint8_t Hardware_Time_Count;				//硬件接管授时定时计数,默认刚进入硬件接管的时候更新一次时间

extern TaskHandle_t xHandleTask_App;
extern TaskHandle_t xHandleTask_Upper_Computer;
extern TaskHandle_t xHandleTask_Hardware_Monitor;
extern TaskHandle_t xHandleTask_MT2000;
extern TaskHandle_t xHandleTask_TH;
extern TaskHandle_t	xHandleTask_Alarm;
extern TaskHandle_t xW5500Task_APP;

extern void Task_App(void * pvParameters);
extern void Task_Upper_Computer(void * pvParameters);
extern void Task_Hardware_Monitor(void *pvParameters);
extern void Task_MT2000(void *pvParameters);
extern void Task_TH(void *pvParameters);
extern void Task_Alarm(void *pvParameters);
extern void W5500_APP(void * pvParameters);
extern void Socket2_Pack_Creat(void);

/*转换16bit的字节序*/
uint16_t Convert_byte_order_16(uint16_t tni2);

#endif
