/*
 * Range328.c
 *
 * Created: 4/25/2018 10:14:43 AM
 * Author : Mark
 
 HY-SO5 Ultrasonic Distance Sensor
 Trig -> PORTD7
 Echo -> PORTB0
 
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "USART0.h"

FILE *fio_0 = &usart0_Stream;

volatile unsigned char MIP;
volatile unsigned int ECHO;
volatile unsigned int Tick;


/******************************************************************************
This ISR is called when the overflow is detected after the start of the echo.  
******************************************************************************/
ISR (TIMER1_OVF_vect) {	// For long ECHO's
		TIMSK1 = 0;	// No further interrupts.
		TCCR1B = 0; // Stop Clock
		MIP = 0xFF;	// End Measurement (Time Out)
}


/******************************************************************************
This ISR is called when an event is detected on the input capture pin. 
******************************************************************************/
ISR (TIMER1_CAPT_vect) {	// Start and Stop ECHO measurement;
	if((TCCR1B & (1<<ICES1)) != 0) {  // this is the rising edge.  
		TCCR1B |= (1<<CS11);	// Start counting with ck/8;
		TCCR1B &= ~(1<<ICES1);  // Configure Negative Edge Capture for end of echo pulse.
	}
	
	else {				// this is the falling edge.  
		ECHO = TCNT1;
		TIMSK1 = (1<<OCIE1B);	// Enables the Compare B interrupt for POST Trigger Delay: Approx 10mS
		TCNT1 = 0;
	}
}

/******************************************************************************
This ISR is called when the counter reaches the count in compare register B. (10 msec)
We disable the interrupts, and go back to idle.  
******************************************************************************/
ISR (TIMER1_COMPB_vect) {	// Compare B: Post ECHO delay 10mS
	TIMSK1 = 0;	// No further interrupts.
	TCCR1B = 0; // Stop Clock
	MIP = 0;	// End Measurement
}


/******************************************************************************
This ISR is called when the 10 usec is elapsed. 
The trigger is returned to a low value. 
We configure the counter 1 to be configured in input capture mode.  
In input capture mode, you can trigger the counter from the input capture pion.  
******************************************************************************/
ISR (TIMER1_COMPA_vect) {	// Compare A : End of Trigger Pulse
	PORTD &= ~(1<<PORTD7);
	TIMSK1 = (1<<ICIE1)|(1<<TOIE1); // enables the T/C1 Overflow and Capture interrupt;
	TCCR1B = (1<<ICES1);			// Set Positive edge for capture but Don't count yet
}


/******************************************************************************
Function to raise the pulse of the trigger.  
Initialize the counter to generate an interrupt when the counter reaches the count 
// defined in Output Compare Register A (10 usec)
******************************************************************************/
void Trigger( void ) {		// Config Timer 1 for 10 to 15uS pulse.
	if(MIP == 0) {	// Don't allow re-trigger.
		MIP = 1;				// Set Measurement in progress FLAG
		DDRD |= (1<<DDD7);		// PD7 as Output for Trigger pulse.
		DDRB &= ~(1<<DDB0);		// PB0 as Input for Input Capture (ECHO).
		
		TCNT1 = 0;				// Clear last Echo times.
		
		OCR1B = 20000;			// 10 mS Post echo Delay
		OCR1A = 20;				// 10 us Trigger length.

		PORTD |= (1<<PORTD7);		// Start Pulse.

		TIFR1 = 0xFF;			//  Clear all timer interrupt flags
		TCCR1A = 0;   // Timer mode with Clear Output on Match
		TCCR1B = (1<<WGM12) | (1<<CS11);  // Counting with CKio/8 CTC Mode enabled
		TIMSK1 = (1<<OCIE1A);	// enables the T/C1 Overflow, Compare A, and Capture interrupt;
	}
}


/******************************************************************************
******************************************************************************/
int main(void){
	
	uint8_t key;
	
	init_uart0(103);

	sei();
	
	fprintf_P(fio_0,PSTR("Hello\n\n\r")	);

	while (1 == 1) {
		while(uart0_RxCount() == 0){};
		
		key = uart0_getc();
		
		switch(key) {
			case 13:
			fprintf_P(fio_0,PSTR("\n\r")	);
			break;

			case 'T':
			case 't':
			Trigger();
			while (MIP == 1) {};
			if(MIP != 0xFF)
				fprintf_P(fio_0,PSTR("Echo is %duS\n\n\r"), ECHO >>= 1);
			break;
			
		}
	}
}



