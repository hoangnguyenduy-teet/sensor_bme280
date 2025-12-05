
#include "bsp_usart.h"
#include "usart_io.h"
#include "ring_buf.h"
#include "stddef.h"
#include "board.h"
#include "board_configs.h"
#include "error_codes.h"

static uint8_t ui8ConsoleRxBuffer[CONFIG_CONSOLE_RX_BUFFER_SIZE];
static uint8_t ui8ConsoleTxBuffer[CONFIG_CONSOLE_TX_BUFFER_SIZE];


static UsartIo_t ConsoleUart = {
		.hUsart = CONSOLE_USART,
		.active = false,
		.txRingBuf = {
				.p_ui8Buf = ui8ConsoleTxBuffer,
				.ui32End = CONFIG_CONSOLE_TX_BUFFER_SIZE,
				.ui32Head = 0,
				.ui32Tail = 0
		},
		.rxRingBuf = {
				.p_ui8Buf = ui8ConsoleRxBuffer,
				.ui32End = CONFIG_CONSOLE_RX_BUFFER_SIZE,
				.ui32Head = 0,
				.ui32Tail = 0
				},
		.tx_busy = false
};

void bsp_uart_console_init()
{
	uart_io_active(&ConsoleUart);
    NVIC_SetPriority(USART1_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 14, 0));
    NVIC_EnableIRQ(USART1_IRQn);
}

uint32_t console_putchar(char c)
{
	return uart_io_write_char(&ConsoleUart, c);
}


uint32_t console_getchar(char *c)
{
	return uart_io_read_byte(&ConsoleUart, (uint8_t *)c);
}

void USART1_IRQHandler()
{
	uart_io_tx_callback(&ConsoleUart);
	uart_io_rx_callback(&ConsoleUart);
}
