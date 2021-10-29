#include "bsp_uart.h"
#include "fifo.h"

#if	0
uint8_t uart5_buf[UART5_RX_LEN];

void uart_init(void)
{
	/* ����5 TX = PC12   RX = PD2 */
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* ��1���� ����GPIO��UARTʱ�� */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    /* ��2������USART Tx��GPIO����Ϊ���츴��ģʽ */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* ��3������USART Rx��GPIO����Ϊ��������ģʽ
	����CPU��λ��GPIOȱʡ���Ǹ�������ģʽ���������������費�Ǳ����
	���ǣ��һ��ǽ�����ϱ����Ķ������ҷ�ֹ�����ط��޸���������ߵ����ò��� */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* ��4���� ���ô���Ӳ������ */
    USART_InitStructure.USART_BaudRate = 115200;	/* ������ */
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No ;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(UART5, &USART_InitStructure);

	/* ����USARTΪ�ж�Դ */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);	/* ʹ�ܽ����ж� */
    USART_Cmd(UART5, ENABLE);						/* ʹ�ܴ��� */
    USART_ClearFlag(UART5, USART_FLAG_TC);			/* �巢����ɱ�־��Transmission Complete flag */
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
