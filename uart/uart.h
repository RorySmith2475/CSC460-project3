/*
 * uart.h
 *
 * Created: 2021-01-22 9:03:02 PM
 *  Author: aa_abdullah
 */ 


#ifndef UART_H_
#define UART_H_

#ifndef  F_CPU
#define F_CPU 16000000UL
#endif
#define BUAD	9600

void uart_init(int baud);
void uart_write_char(char c);
void uart_print(char* str, int size);
void uart_write_int(unsigned int n);
//void uart_send(unsigned char *buf);

#endif /* UART_H_ */