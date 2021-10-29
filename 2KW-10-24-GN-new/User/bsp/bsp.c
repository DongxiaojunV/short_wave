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
*	�� �� ��: bsp_Init
*	����˵��: ��ʼ��Ӳ���豸��ֻ��Ҫ����һ�Ρ��ú�������CPU�Ĵ���������ļĴ�������ʼ��һЩȫ�ֱ�����
*			 ȫ�ֱ�����
*	��    ��: ��
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void bsp_Init(void)
{
    /* ���ȼ���������Ϊ4��������0-15����ռʽ���ȼ���0�������ȼ����������������ȼ���*/
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    bsp_InitUart(); 		/* ��ʼ������ */

    Startup_GPIO_Init();	/* �ɻ������ػ��������� */
    LED_GPIO_Config();		/* ��ʼLEDָʾ�ƶ˿� */
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
