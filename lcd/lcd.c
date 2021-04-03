/*
 * lcd.c
 *
 * Created: 2021-01-23 3:09:46 PM
 *  Author: aa_abdullah
 */ 

#include "lcd.h"

void LCD_Command( unsigned char cmnd )
{
	LCD_Port = (LCD_Port & 0x0F) | (cmnd & 0xF0); /* sending upper nibble */
	RS_Port &= (0<<RS);				/* RS=0, command reg. */
	EN_Port |= (1<<EN);				/* Enable pulse */
	_delay_us(1);
	EN_Port &= (0<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (cmnd << 4);  /* sending lower nibble */
	EN_Port |= (1<<EN);
	_delay_us(1);
	EN_Port &= (0<<EN);
	_delay_ms(2);
}


void LCD_Char( unsigned char data )
{
	LCD_Port = (LCD_Port & 0x0F) | (data & 0xF0); /* sending upper nibble */
	RS_Port |= (1<<RS);				/* RS=1, data reg. */
	EN_Port |= (1<<EN);
	_delay_us(1);
	EN_Port &= (0<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0x0F) | (data << 4); /* sending lower nibble */
	EN_Port |= (1<<EN);
	_delay_us(1);
	EN_Port &= (0<<EN);
	_delay_ms(2);
}

void LCD_Init(void)					/* LCD Initialize function */
{
	LCD_Dir = 0xFF;						/* Make LCD command port direction as o/p */
	RS_Dir |= (1<<RS);
	RS_Port |= (1<<RS);
	EN_Dir |= (1<<EN);
	EN_Port &= (0<<EN);
	
	
	_delay_ms(20);						/* LCD Power ON delay always >15ms */
	
	LCD_Command(0x33);
	LCD_Command(0x32);		    		/* send for 4 bit initialization of LCD  */
	LCD_Command(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	LCD_Command(0x0c);              	/* Display on cursor off*/
	LCD_Command(0x06);              	/* Increment cursor (shift cursor to right)*/
	LCD_Command(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
	_delay_ms(2);
}

void LCD_goto(char row, char col)		/* Go to row, col position */
{
	if (row == 0 && col<16) LCD_Command((col & 0x0F)|0x80);
	else if (row == 1 && col<16) LCD_Command((col & 0x0F)|0xC0);
}

void LCD_String(char *str)				/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)				/* Send each char of string till the NULL */
	{
		LCD_Char (str[i]);
	}
}

void LCD_String_at(char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	LCD_goto(row, pos);
	LCD_String(str);					/* Call LCD string function */
}

void LCD_Clear()
{
	LCD_Command (0x01);					/* Clear display */
	_delay_ms(2);
	LCD_Command (0x80);					/* Cursor 1st row 0th position */
}

void LCD_write_int(unsigned int n)
{
	char str[12];
	sprintf(str, "%d", n);
	LCD_String(str);
}