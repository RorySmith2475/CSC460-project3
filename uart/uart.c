/*
 * uart.c
 *
 * Created: 2021-01-22 9:04:59 PM
 *  Author: aa_abdullah
 */ 
#include <avr/io.h>
#include <stdio.h>
#include "uart.h"

void uart_init(int baud)
{
	/* Replace with your application code */
	
	int BRC = ((F_CPU/16/baud) -1);
	UBRR0H = (BRC >> 8);
	UBRR0L = BRC;
	
	UCSR0B = (1 << TXEN0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
	
	return;
}

void uart_write_char(char c)
{
	UDR0 = c;
}

void uart_write_int(unsigned int n)
{
	char str[12];
	sprintf(str, "%d", n);
	uart_print(str, 12);
}

void uart_print(char str[], int size)
{
	for (int i=0; i<size && str[i] !='\0'; i++)
	{
		UDR0 = str[i];	//put character into send buffer
		while (!(UCSR0A & (1<<UDRE0))); //wait until done
	}
}
