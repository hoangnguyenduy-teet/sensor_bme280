#ifndef CONSOLE_H_
#define CONSOLE_H_
#include "stdint.h"

uint32_t console_putchar(char c);
uint32_t console_getchar(char *c);
uint32_t console_get_available_char();

void console_printf(const char *pcString,...);

#endif // CONSOLE_H_
