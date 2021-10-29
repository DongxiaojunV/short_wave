#ifndef	_BSP_UART_H_

#include "stm32f10x.h"
#include "stm32f10x_usart.h"

#define	UART5_RX_LEN	2100

extern uint8_t uart5_buf[UART5_RX_LEN];

extern void uart_init(void);
extern void UART5_Put_Char(uint8_t ch);
extern void UART5_Send_Data(uint8_t *pData, uint16_t Size);

#endif
