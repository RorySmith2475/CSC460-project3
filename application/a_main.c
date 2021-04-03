/*
 * a_main.c
 *
 * Created: 2021-03-30
 *  Author: rorysmith
 */ 

#include "../os.h"
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../blink/blink.h"
#include "../lcd/lcd.h"
#include "../uart/uart.h"
#include "util/delay.h"
#include "util/atomic.h"

typedef enum application_state
{
	CALIBRATING = 0,
	READY,
	OBJECT_DETECTED	
} APPLICATION_STATE;
static APPLICATION_STATE state = CALIBRATING;

#define CALIBRATION_READINGS	20
#define MAX_RTT					28000
#define MIN_RTT					200
#define POTENTIOMETER_MAX		1023

static uint8_t num_readings;
static uint64_t calibrated_RTT; // time measured during calibration
static uint32_t RTT;			// time measured during READY state
static uint8_t accuracy = 100;	// % RTT has to be diff from calibrated_RTT for an object to be detected

volatile uint8_t new_reading;
volatile uint32_t start_time;
volatile uint32_t time_taken;
volatile uint16_t timer4_overflow_count;

#define TURN_GREEN_LED_ON()		PORTB |= (1<<PB0)
#define TURN_GREEN_LED_OFF()	PORTB &= ~ (1<<PB0)
#define TURN_RED_LED_ON()		PORTB |= (1<<PB1)
#define TURN_RED_LED_OFF()		PORTB &= ~ (1<<PB1)

#define TURN_BUZZER_ON()	PORTB |= (1<<PB3)
#define TURN_BUZZER_OFF()	PORTB &= ~ (1<<PB3)

void Start_timer4()
{
	Disable_Interrupt();
	timer4_overflow_count = 0;
	TCCR4A = 0;
	TCCR4B = 0;
	TIFR4 = (1 << ICF4) | (1 << TOV4);  // Clear pending
	TCCR4B |= (1 << ICNC4);				// Disable noise canceling
	TCCR4B |= (1 << CS41);				// Set prescaller
	TCNT4 = 0;
	Enable_Interrupt();
}

void Stop_timer4()
{
	Disable_Interrupt();
	TCCR4A = 0;
	TCCR4B = 0;
	Enable_Interrupt();
}

void Task_Update_Displays()	
{
	LCD_Init();
	LCD_Clear();
	
	// Set buzzer + green and red LEDs to output
	DDRB |= (1<<DDB3);
	DDRB |= (1<<DDB1);
	DDRB |= (1<<DDB0);
	
	for(;;)
	{
		switch(state)
		{			
		case CALIBRATING:
			LCD_goto(0,0);
			LCD_String("CALIBRATING...");
			break;
			
		case READY:
			LCD_goto(0,5);
			LCD_String("READY!");
			LCD_goto(1,0);			
			LCD_String("Accuracy = ");
			LCD_goto(1,11);
			LCD_write_int(accuracy);
			
			TURN_GREEN_LED_ON();
			TURN_RED_LED_OFF();
			TURN_BUZZER_OFF();
			break;
			
		case OBJECT_DETECTED:
			LCD_goto(0,5);
			LCD_String("OBJECT");
			LCD_goto(1,4);
			LCD_String("DETECTED");
			
			TURN_RED_LED_ON();
			TURN_GREEN_LED_OFF();
			TURN_BUZZER_ON();
			break;
		}
		
		_delay_ms(10000);
		LCD_Clear();
		_delay_ms(20);
	}
}

void Task_Read_Potentiometer()
{
	ADCSRA |= ((1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));
	ADMUX |= (1<<REFS0); //Set Voltage reference to Avcc (5v)
	ADCSRA |= (1<<ADEN); //Turn on ADC
	ADCSRA |= (1<<ADSC); //Do an initial conversion
	
	for(;;)
	{
		ADMUX &= 0xE0; //Clear bits MUX0-4
		ADMUX |= PF0 & 0x07; //Defines the new ADC channel to be read by setting bits MUX0-2
		ADCSRB = PF0 & (1<<MUX5); //Set MUX5
		ADCSRA |= (1<<ADSC); //Starts a new conversion
		
		while(ADCSRA & (1<<ADSC)); //Wait until the conversion is done
		
		accuracy = (uint8_t)((float)ADCW / (float)POTENTIOMETER_MAX * 100);
	}
}

/**
 * Periodic task that sends a 10us pulse to the ultrasonic sensor.
 */
void Task_Send_Ultrasonic_Pulse()
{
	DDRB |= (1<<DDB2);	// Set pin 52 output
	
	for(;;)
	{		
		PORTB &= ~ (1<<PB2);
		_delay_us(5);
		PORTB |= (1<<PB2);
		_delay_us(10);
		PORTB &= ~ (1<<PB2);
		
		Task_Next();
	}
}

void Task_Evaluate_Reading()
{
	for(;;)
	{		
		Disable_Interrupt();
		RTT = time_taken;
		uint8_t evaluate = new_reading;
		new_reading = 0;
		Enable_Interrupt();
		
		
		
		if(!evaluate) continue; // No new reading
		uart_write_int(RTT);
		uart_write_char('\n');
		
		if(RTT < MIN_RTT || RTT > MAX_RTT) continue;
	
		switch(state)
		{			
		case CALIBRATING:
			num_readings++;
			calibrated_RTT += RTT;
							
			if(num_readings >= CALIBRATION_READINGS) {
				calibrated_RTT /= CALIBRATION_READINGS;
				
				state = READY;
				num_readings = 0;
				new_reading = 0;
				RTT = 0;
			}
			break;
			
		case READY:			
			if(abs(RTT - calibrated_RTT) >= (accuracy*calibrated_RTT/100)) {
				state = OBJECT_DETECTED;
			}
			break;
			
		case OBJECT_DETECTED:
			break;
		}
	}
}

/**
 * Interrupt handler triggered on both rising and falling edges
 * for pins in the INT2 vector. Should only receive interrupts
 * from PD2.
 */
ISR(INT2_vect)
{
	if(PIND & (1<<PIND2)) { // Rising edge	
		Stop_timer4();
		Start_timer4();
		start_time = TCNT4;	
	}
	else if(start_time > 0) {
		if(!timer4_overflow_count) { // looped over to 0. Skip this reading
			new_reading = 1;
			time_taken = TCNT4 - start_time;
			start_time = 0;
			//Stop_timer4();
		}
	}
}

/** 
 * Called when timer overflow happens. Used to tell the rest of the system to ignore 
 * the last RTT since it will be invalid.
 */
ISR(TIMER4_COMPA_vect)
{
	timer4_overflow_count++;
	Stop_timer4();
}

void Interrupt_init()
{
	Disable_Interrupt();
	
	DDRD &= ~ (1<<DDD2);
	EICRA |= (1 << ISC20); // Any change
	//EICRA |= (1 << ISC21 | 1 << ISC20); // Rising edge
	EIMSK |= (1<<INT2);	
	
	TIMSK4 |= (1<<OCIE4A); // Set timer4 interrupt
	
	Enable_Interrupt();
}

/**
 * Main function called by the os at startup.
 *
 * Creates all tasks needed for the application and then exits.
 */
void a_main()
{
	Interrupt_init();
	
	uart_init(9600);
	
	Task_Create_Period(Task_Send_Ultrasonic_Pulse, 0, 50, 10000, 2000);
	Task_Create_WRR(Task_Evaluate_Reading, 0, 1);
	Task_Create_WRR(Task_Update_Displays, 0, 5);	
	Task_Create_WRR(Task_Read_Potentiometer, 0, 1);
}
