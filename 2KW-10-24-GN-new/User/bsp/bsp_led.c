/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  fire
  * @version V1.0
  * @date    2013-xx-xx
  * @brief   ledӦ�ú����ӿ�
  ******************************************************************************
  * @attention
  *
  * ʵ��ƽ̨:���� F103-�Ե� STM32 ������
  * ��̳    :http://www.firebbs.cn
  * �Ա�    :https://fire-stm32.taobao.com
  *
  ******************************************************************************
  */

#include "bsp.h"
#include "bsp_led.h"

/**
 * @brief  ��ʼ������LED��IO
 * @param  ��
 * @retval ��
 */
void LED_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd( LED1_GPIO_CLK  , ENABLE);	/*����LED��ص�GPIO����ʱ��*/
    GPIO_InitStructure.GPIO_Pin = LED1_GPIO_PIN;		/*ѡ��Ҫ���Ƶ�GPIO����*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/*��������ģʽΪͨ���������*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/*������������Ϊ50MHz */
    GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);		/*���ÿ⺯������ʼ��GPIO*/

    RCC_APB2PeriphClockCmd( LED2_GPIO_CLK  , ENABLE);	/*����LED��ص�GPIO����ʱ��*/
    GPIO_InitStructure.GPIO_Pin = LED2_GPIO_PIN;		/*ѡ��Ҫ���Ƶ�GPIO����*/
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	/*��������ģʽΪͨ���������*/
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	/*������������Ϊ50MHz */
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);		/*���ÿ⺯������ʼ��GPIO*/

    /* �ر�����led��	*/
    GPIO_SetBits(LED1_GPIO_PORT, LED1_GPIO_PIN);
    GPIO_SetBits(LED2_GPIO_PORT, LED2_GPIO_PIN);
}

/*********************************************END OF FILE**********************/
