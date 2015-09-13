/*
 * File:		uart.h
 * Purpose:     Provide common UART routines for polled serial IO
 *
 * Notes:
 */

#ifndef __UART_H__
#define __UART_H__
#define TERM_PORT UART0_BASE_PTR
/********************************************************************/

void uart_init (UART_MemMapPtr, int, int);
char uart_getchar (UART_MemMapPtr);
void uart_putchar (UART_MemMapPtr, char);
int uart_getchar_present (UART_MemMapPtr);
void send_string(char string[], UART_MemMapPtr channel);

/********************************************************************/

#endif /* __UART_H__ */
