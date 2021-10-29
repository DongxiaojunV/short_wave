#include "fifo.h"
#include "string.h"

/* ����ڴ�������Ƿ�֧�ָ��Ǿ����� */
#define		FIFO_NO_COVER_WRITE		1

/* �ȶ�д���ƣ���
 * ͷָ��ָ�����ݣ��ȶ�����
 * βָ��ָ��գ���д����
 */

//fifo_t fifo_uart1;
//fifo_t fifo_uart2;
fifo_t fifo_uart5;


void fifo_init(fifo_t *fifo, uint8_t *buf, uint16_t len)
{
    fifo->data	= buf;
    fifo->heart	= 0;
    fifo->tail	= 0;
    fifo->len	= len;
}

void fifo_clean(fifo_t *fifo)
{
    fifo->heart	= fifo->tail;
    fifo->len	= 0;
}

uint8_t is_fifo_empty(fifo_t *fifo)
{
    if( fifo->heart == fifo->tail )
        return 1;
    else
        return 0;
}

uint8_t is_fifo_full(fifo_t *fifo)
{
    if( ( (fifo->tail + 1) % fifo->len) == fifo->heart )
        return 1;
    else
        return 0;
}

uint8_t fifo_len(fifo_t *fifo)
{
    if( fifo->tail >= fifo->heart )
        return ( fifo->tail - fifo->heart );
    else
        return ( fifo->tail + fifo->len - fifo->heart );
}

uint8_t fifo_push(fifo_t *fifo, uint8_t buf)
{
#if	FIFO_NO_COVER_WRITE
    if( is_fifo_full(fifo) )
    {
        return 0;
    }
#endif

    fifo->data[fifo->tail] = buf;
    fifo->tail = ( fifo->tail + 1 ) % fifo->len;

    return 1;
}

uint8_t fifo_pull(fifo_t *fifo, uint8_t *buf)
{
    if( is_fifo_empty(fifo) )
    {
        return 0;
    }
    else
    {
        *buf = fifo->data[fifo->heart];
        fifo->heart = ( fifo->heart + 1 ) % fifo->len;

        return 1;
    }

}

uint8_t fifo_npull(fifo_t *fifo, uint8_t *buf, uint8_t num)
{
//    uint8_t i = 0;
    uint8_t len;

    len	= fifo_len(fifo);

    if( num > len )
    {
        num = len;
    }
	
//    for(i=0; i<num; i++)
//    {
//        *(buf+i) = fifo->data[fifo->heart];
//        fifo->heart = ( fifo->heart + 1 ) % fifo->len;
//    }

	memcpy(buf, fifo->data+fifo->heart, num);
	fifo->heart = ( fifo->heart + num ) % fifo->len;
    return len;
}
