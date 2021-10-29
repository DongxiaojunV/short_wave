#include "bsp_uart.h"
#include "fifo.h"

#if	0
uint8_t uart5_buf[UART5_RX_LEN];

void uart_init(void)
{
	/* 串口5 TX = PC12   RX = PD2 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* 第1步： 开启GPIO和UART时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* 第2步：将USART Tx的GPIO配置为推挽复用模式 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* 第3步：将USART Rx的GPIO配置为浮空输入模式
	由于CPU复位后，GPIO缺省都是浮空输入模式，因此下面这个步骤不是必须的
	但是，我还是建议加上便于阅读，并且防止其它地方修改了这个口线的设置参数 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* 第4步： 配置串口硬件参数 */
    USART_InitStructure.USART_BaudRate = 115200;	/* 波特率 */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

	/* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* 使能接收中断 */
    USART_Cmd(UART5, ENABLE);						/* 使能串口 */
    USART_ClearFlag(UART5, USART_FLAG_TC);			/* 清发送完成标志，Transmission Complete flag */
}

void UART5_Put_Char(uint8_t ch)
{
    while( (UART5->SR&0x40) == 0x00 );
    UART5->DR = ch;
}

void UART5_Send_Data(uint8_t *pData, uint16_t Size)
{
    uint16_t i;

    for(i=0; i<Size; i++)
    {
        while( (UART5->SR & 0x40) == 0x00 );
        UART5->DR = *(pData + i);
    }
}

void UART5_IRQHandler(void)
{
	uint8_t read;
	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET)
	{	
		read = USART_ReceiveData(UART5);
		fifo_push(&fifo_uart5, read);
	}
}
#endif
