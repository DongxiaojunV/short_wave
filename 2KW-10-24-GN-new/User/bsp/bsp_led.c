/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
  * @attention
  *
  * 实验平台:秉火 F103-霸道 STM32 开发板
  * 论坛    :http://www.firebbs.cn
  * 淘宝    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp.h"
#include "bsp_led.h"

/**
 * @brief  初始化控制LED的IO
 * @param  无
 * @retval 无
 */
void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( LED1_GPIO_CLK  , ENABLE);	/*开启LED相关的GPIO外设时钟*/
    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;		/*选择要控制的GPIO引脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/*设置引脚模式为通用推挽输出*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/*设置引脚速率为50MHz */
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);		/*调用库函数，初始化GPIO*/

    RCC_APB2PeriphClockCmd( LED2_GPIO_CLK  , ENABLE);	/*开启LED相关的GPIO外设时钟*/
    GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;		/*选择要控制的GPIO引脚*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/*设置引脚模式为通用推挽输出*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/*设置引脚速率为50MHz */
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);		/*调用库函数，初始化GPIO*/

    /* 关闭所有led灯	*/
    GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
    GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
}

/*********************************************END OF FILE**********************/
