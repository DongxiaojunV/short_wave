#ifndef	_FIFO_H_
#define	_FIFO_H_

#include <stdint.h>		/* uint8_t ... */

typedef struct
{
	uint8_t		*data;
	uint16_t	heart;
	uint16_t	tail;
	uint16_t	len;
}fifo_t;

//extern fifo_t fifo_uart1;
//extern fifo_t fifo_uart2;
extern fifo_t fifo_uart5;

extern void fifo_init(fifo_t *fifo, uint8_t *buf, uint16_t len);
extern void fifo_clean(fifo_t *fifo);
extern uint8_t is_fifo_empty(fifo_t *fifo);
extern uint8_t is_fifo_full(fifo_t *fifo);

extern uint8_t fifo_len(fifo_t *fifo);
extern uint8_t fifo_push(fifo_t *fifo, uint8_t buf);
extern uint8_t fifo_pull(fifo_t *fifo, uint8_t *buf);
extern uint8_t fifo_npull(fifo_t *fifo, uint8_t *buf, uint8_t num);


#endif
