#ifndef _USART_USART_IO_H_
#define _USART_USART_IO_H_
#include "stm32f1xx.h"
#include "ring_buf.h"
#include <stdatomic.h>
typedef struct
{
	USART_TypeDef * hUsart;
	RingBuf_t	  txRingBuf;
	RingBuf_t	  rxRingBuf;
	bool		  active;
	bool		  tx_busy;
}UsartIo_t;


void uart_io_active(UsartIo_t * me);
void uart_io_deactive(UsartIo_t * me);
bool uart_io_is_active(UsartIo_t * me);
uint32_t uart_io_read_byte(UsartIo_t* me, uint8_t* c);
uint32_t uart_io_read(UsartIo_t * me, uint8_t * Buffer, uint32_t ui32MaxLen);
uint32_t uart_io_write(UsartIo_t * me, const uint8_t * pui8Buffer, uint32_t ui32NumData);
uint32_t uart_io_write_char(UsartIo_t * me, const uint8_t ui8Val);
void uart_io_rx_callback(UsartIo_t * me);
void uart_io_tx_callback(UsartIo_t * me);

#endif /* DRIVER_COMPONENTS_USART_USART_IO_H_ */
