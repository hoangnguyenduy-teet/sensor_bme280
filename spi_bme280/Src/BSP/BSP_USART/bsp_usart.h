#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#include "board.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void bsp_uart_console_init(void);

/**
 * @brief Transmit a character over the console USART.
 *
 * @param c The character to be transmitted.
 * @return ERROR_OK on success, ERROR_FAIL on failure.
 */
uint32_t console_putchar(char c);

/**
 * @brief Receive a character from the console USART.
 *
 * @param c Pointer to a char where the received character will be stored.
 * @return ERROR_OK on success, ERROR_FAIL if no data or invalid pointer.
 */
uint32_t console_getchar(char *c);

#ifdef __cplusplus
}
#endif

#endif // __BSP_USART_H__
