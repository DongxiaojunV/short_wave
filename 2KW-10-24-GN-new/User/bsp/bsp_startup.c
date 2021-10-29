#include "bsp.h"
#include "bsp_startup.h"

void Startup_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

//	GPIO_Remap_SWJ_Disable;											//Full SWJ Disabled (JTAG-DP + SW-DP)
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);		/*使能SWD 禁用JTAG*/

    RCC_APB2PeriphClockCmd(Switch_GPIO_CLK , ENABLE);
    GPIO_InitStructure.GPIO_Pin = Open_GPIO_PIN|Close_GPIO_PIN;		//开关机
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(Switch_GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = State_GPIO_PIN;					//开机状态检测
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(In_GPIO_PORT, &GPIO_InitStructure);


    RCC_APB2PeriphClockCmd(In_GPIO_CLK , ENABLE);
    GPIO_InitStructure.GPIO_Pin = In_1_GPIO_PIN | In_2_GPIO_PIN | In_3_GPIO_PIN | In_4_GPIO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(In_GPIO_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(Switch_GPIO_PORT, Open_GPIO_PIN);	//先输出低
    GPIO_ResetBits(Switch_GPIO_PORT, Close_GPIO_PIN);	//先输出低
}
