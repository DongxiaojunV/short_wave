#ifndef	_BSP_STARTUP_H_
#define	_BSP_STARTUP_H_

#include "stm32f10x.h"

#define GPIO_Remap_SWJ_JTAGDisable  ((uint32_t)0x00300200)  /*!< JTAG-DP Disabled and SW-DP Enabled */

/*---------------------------------------------------------------------------------------*/
#define In_GPIO_PORT    	GPIOA						/* GPIO端口 */
#define In_GPIO_CLK 	    RCC_APB2Periph_GPIOA		/* GPIO端口时钟 */
#define In_1_GPIO_PIN		GPIO_Pin_4					//电平输入检测1
#define In_2_GPIO_PIN		GPIO_Pin_5					//电平输入检测2
#define In_3_GPIO_PIN		GPIO_Pin_6					//电平输入检测3
#define In_4_GPIO_PIN		GPIO_Pin_7					//电平输入检测4

#define Switch_GPIO_PORT	GPIOB						/* GPIO端口 */
#define Switch_GPIO_CLK		RCC_APB2Periph_GPIOB		/* GPIO端口时钟 */
#define Open_GPIO_PIN		GPIO_Pin_5					//开机
#define Close_GPIO_PIN		GPIO_Pin_6					//关机
#define State_GPIO_PIN		GPIO_Pin_7					//开关机状态检测


/*---------------------------------------------------------------------------------------*/
#define	SWITCH_RELAY_NORMALLY_OPEN		0

#if		SWITCH_RELAY_NORMALLY_OPEN
#define	SWITCH_ON			1
#define	SWITCH_OFF			0
#else
#define	SWITCH_ON			0
#define	SWITCH_OFF			1
#endif

#define	READ_SWITCH_STATE		GPIO_ReadInputDataBit(Switch_GPIO_PORT, State_GPIO_PIN)
//#define	READ_ALARM_INFO_1		GPIO_ReadInputDataBit(In_GPIO_PORT, In_1_GPIO_PIN)
//#define	READ_ALARM_INFO_2		GPIO_ReadInputDataBit(In_GPIO_PORT, In_2_GPIO_PIN)
//#define	READ_ALARM_INFO_3		GPIO_ReadInputDataBit(In_GPIO_PORT, In_3_GPIO_PIN)
#define	READ_ALARM_INFO_4		GPIO_ReadInputDataBit(In_GPIO_PORT, In_4_GPIO_PIN)

extern void Startup_GPIO_Init(void);


#endif
