/*
 * lcd.h
 *
 * Created: 2021-01-23 3:08:17 PM
 *  Author: aa_abdullah
 */ 

#ifndef LCD_H_
#define LCD_H_

#include <avr/io.h>						/* Include AVR std. library file */
#include <util/delay.h>					/* Include inbuilt defined Delay header file */
#include <stdio.h>

#ifndef  F_CPU
#define F_CPU 16000000UL
#endif

#define LCD_Dir DDRB					/* Define LCD data port direction */
#define LCD_Port PORTB					/* Define LCD data port */
#define RS PG5							/* Define Register Select (data reg./command reg.) signal pin */
#define RS_Dir DDRG
#define RS_Port	PORTG
#define EN PH3 							/* Define Enable signal pin */
#define EN_Dir DDRH
#define EN_Port PORTH

void LCD_Command( unsigned char cmnd );
void LCD_Char( unsigned char data );
void LCD_Init(void);								/* LCD Initialize function */
void LCD_goto(char row, char col);					/* LCD goto row, col */
void LCD_String(char *str);							/* Send string to LCD function */
void LCD_String_at(char row, char pos, char *str);	/* Send string to LCD with xy position */
void LCD_Clear();
void LCD_write_int(unsigned int n);
#endif /* LCD_H_ */