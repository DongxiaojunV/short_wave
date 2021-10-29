#include "bsp.h"
#include "includes.h"
#include "fifo.h"
#include "bsp_uart.h"

#if	PRINTF_EN
#define DEBUG_ENABLE 		1
#else
#define DEBUG_ENABLE 		0
#endif

#if  printf_EN
   #define Trans_printf_ENABLE 1
#else
   #define Trans_printf_ENABLE 0
#endif

extern SemaphoreHandle_t	xMutex;
/*
*********************************************************************************************************
*	函 数 名: bsp_Init
*	功能说明: 初始化硬件设备。只需要调用一次。该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量。
*			 全局变量。
*	形    参: 无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Init(void)
{
    /* 优先级分组设置为4，可配置0-15级抢占式优先级，0级子优先级，即不存在子优先级。*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    bsp_InitUart(); 		/* 初始化串口 */

    Startup_GPIO_Init();	/* 旧机器开关机和输入检测 */
    LED_GPIO_Config();		/* 初始LED指示灯端口 */
}

void  App_printf(char *format,...)
{
#if DEBUG_ENABLE

    char  buf_str[127 + 1];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (size_t      ) sizeof(buf_str),
                    (char const *) format,
                    v_args);
    va_end(v_args);


    xSemaphoreTake(xMutex, portMAX_DELAY);
    printf("%s", buf_str);
    xSemaphoreGive(xMutex);
#endif
}

void  Trans_printf(char *format,...)
{
#if Trans_printf_ENABLE

    char  buf_str[127 + 1];
    va_list   v_args;


    va_start(v_args, format);
    (void)vsnprintf((char       *)&buf_str[0],
                    (size_t      ) sizeof(buf_str),
                    (char const *) format,
                    v_args);
    va_end(v_args);


    xSemaphoreTake(xMutex, portMAX_DELAY);
    printf("%s", buf_str);
    xSemaphoreGive(xMutex);
#endif
}


/***************************** (END OF FILE) *********************************/
