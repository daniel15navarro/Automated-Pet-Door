/* HC-SR04 Ultrasonic Distance Sensor.  
 Trig -> PORTD7
 Echo -> PORTB0
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <stdio.h>

#include "USART0.h"
#include "ldr_and_speaker_functions.h"

FILE *fio_0 = &usart0_Stream;

#define TRIG_PIN PIND7
#define ECHO_PIN PINB0
#define GREEN_LED PINB1
#define ORANGE_LED PINB2
#define RED_LED PINB3
#define SPEAKER_PIN PINB4
#define DISTANCE_THRESHOLD 1 // 1 cm
#define MAX_DISTANCE 100  // 1 meter (100 cm)
#define THRESHOLD 300          // Light level threshold

volatile unsigned char MIP;
volatile unsigned int ECHOHigh, ECHOLow, ECHOHighPipe;

/******************************************************************************
This ISR handles the overflow condition for long echoes.
******************************************************************************/
ISR (TIMER1_OVF_vect) {    // For long ECHO's
	if(ECHOHighPipe >= 2) {
		TIMSK1 = 0;   // No further interrupts.
		TCCR1B = 0; // Stop Clock
		MIP = 0xFF;   // End Measurement
	}
	ECHOHighPipe++;      // Add 1 to High byte.
}

/******************************************************************************
This ISR handles the echo signal detection.
******************************************************************************/
ISR(TIMER1_CAPT_vect) {
    if ((TCCR1B & (1 << ICES1)) != 0) {  // Rising edge detected
        TCCR1B |= (1 << CS11);    // Start timer with prescaler 8
        TCCR1B &= ~(1 << ICES1);  // Switch to falling edge detection
    } else {  // Falling edge detected
        ECHOLow = TCNT1;
        ECHOHigh = ECHOHighPipe;
		TIMSK1 = (1 << OCIE1B);
    }
}

/******************************************************************************
This ISR stops the trigger pulse after 10µs.
******************************************************************************/
ISR(TIMER1_COMPA_vect) {
    PORTD &= ~(1 << TRIG_PIN);  // End trigger pulse
    TIMSK1 = (1 << ICIE1) | (1 << TOIE1);  // Enable overflow & input capture interrupts
    TCCR1B = (1 << ICES1);  // Set positive edge detection (don't start counting yet)
}

/******************************************************************************
This ISR is called when the counter reaches the count in compare register B. (10 msec)
We disable the interrupts, and go back to idle.
******************************************************************************/
ISR (TIMER1_COMPB_vect) {  // Compare B: Post ECHO delay 10mS
	TIMSK1 = 0;   // No further interrupts.
	TCCR1B = 0; // Stop Clock
	MIP = 0;      // End Measurement
}

/******************************************************************************
Function to trigger the ultrasonic sensor.
******************************************************************************/
void Trigger(void) {
    if (MIP == 0) {
        MIP = 1;
        DDRD |= (1 << TRIG_PIN);
        DDRD &= ~(1 << ECHO_PIN);

        PORTD &= ~(1 << TRIG_PIN);  // Ensure LOW before starting
        PORTD |= (1 << TRIG_PIN);   // Start HIGH pulse

        // Use Timer1 for precise 10µs pulse
        TCNT1 = 0;
        OCR1A = 20;  // 10µs (assuming 8MHz clock and prescaler 8)
        TIFR1 = 0xFF;  // Clear all flags
        TCCR1A = 0;
        TCCR1B = (1 << WGM12) | (1 << CS11);  // CTC mode, prescaler 8
        TIMSK1 = (1 << OCIE1A);  // Enable compare match A interrupt
    }
}

/******************************************************************************
Function to calculate distance based on echo pulse duration.
******************************************************************************/
unsigned int CalculateDistance(void) {
    if (ECHOHigh == 0 && ECHOLow == 0) {
        return 999;
    }
    unsigned long time_us = ((ECHOHigh * 65536) + ECHOLow) * (0.5); // (8 / (16 / 1000000));
    return (unsigned int) (time_us / 58);
}

/********
Buzzer Code
****/

volatile uint8_t timerFlag = 0;

/*
void initADCSetup(void) {
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable ADC, prescaler 128
}
*/

uint16_t readADCValue(void) {
	ADCSRA |= (1 << ADSC); // Start conversion
	while (ADCSRA & (1 << ADSC)); // Wait for conversion to finish
	return ADC;
}

void setupPWMOutput(void) {
	DDRB |= (1 << SPEAKER_PIN); // Set speaker pin as output
	TCCR1A = (1 << COM1A0) | (1 << WGM12); // Toggle OC1A on compare match, CTC mode
	TCCR1B = (1 << CS10); // No prescaler
}


void playToneFrequency(uint16_t frequency) {
	uint16_t ocrValue = (F_CPU / (2 * frequency)) - 1;
	OCR1A = ocrValue;
}

void stopToneOutput(void) {
	TCCR1A &= ~(1 << COM1A0); // Disable toggle mode
}

void setupTimerInterrupt(void) {
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
	OCR0A = 156; // Approx 100ms interrupt at 16MHz clock
	TIMSK0 = (1 << OCIE0A); // Enable compare match interrupt
}

ISR(TIMER0_COMPA_vect) {
	timerFlag = 1;
}

/*
MAIN
*/

int main(void) {
    uint8_t key;

    init_uart0(103);
		
	sei();

	fprintf_P(fio_0, PSTR("Initiating system. Please wait...\n\n\r"));
	
	//PORTB &= ~(1 << GREEN_LED);
	_delay_ms(2000);
	fprintf_P(fio_0, PSTR("Testing green LED...\n\r"));
	DDRB |= (1 << GREEN_LED);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(2000);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(750);
	fprintf_P(fio_0, PSTR("Testing orange LED...\n\r"));
	DDRB |= (1 << ORANGE_LED);
	PORTB |= (1 << ORANGE_LED);
	_delay_ms(2000);
	PORTB &= ~(1 << ORANGE_LED);
	_delay_ms(750);
	fprintf_P(fio_0, PSTR("Testing red LED...\n\r"));
	DDRB |= (1 << RED_LED);
	PORTB |= (1 << RED_LED);
	_delay_ms(2000);
	PORTB &= ~(1 << RED_LED);
	_delay_ms(750);
	fprintf_P(fio_0, PSTR("Testing speaker...\n\r"));
	DDRB |= (1 << SPEAKER_PIN);
	PORTB |= (1 << SPEAKER_PIN); // Turn ON speaker
	_delay_ms(2000);
	PORTB &= ~(1 << SPEAKER_PIN);
	_delay_ms(1000);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(500);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(500);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(500);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(500);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(500);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(500);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(500);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(500);
	PORTB |= (1 << GREEN_LED);
	_delay_ms(500);
	PORTB &= ~(1 << GREEN_LED);
	_delay_ms(1000);
	fprintf_P(fio_0, PSTR("\nSystem Ready. Press 'T' or 't' to trigger sensor.\n\r"));
	PORTB |= (1 << ORANGE_LED);
	
	//initADCSetup();
	//setupPWMOutput();
	setupTimerInterrupt();

	PORTB &= ~(1 << GREEN_LED);

    while (1) {
	    while (uart0_RxCount() == 0) {};  // Wait for user input
	    key = uart0_getc();  // Read the character

	    switch (key) {
		    case 13:
		    fprintf_P(fio_0, PSTR("\n\r"));
		    break;

		    case 'T':
		    case 't':
		    fprintf_P(fio_0, PSTR("Triggering Sensor...\n\r"));
		    Trigger();
		    
		    while (MIP == 1) {};  // Wait for measurement to complete
		    
		    // Print raw timer values
		    fprintf_P(fio_0, PSTR("Raw Timer Values: ECHOHigh = %d, ECHOLow = %d, MIP = %d\n\r"), ECHOHigh, ECHOLow, MIP);

		    if (ECHOHigh == 0 && ECHOLow == 0) {
			    fprintf_P(fio_0, PSTR("No echo detected! Check wiring or sensor.\n\r"));
			    PORTB |= (1 << RED_LED); // turn red LED on
			    PORTB &= ~(1 << GREEN_LED);
			    PORTB &= ~(1 << ORANGE_LED);
		    }
		    
		    else {
			    
			    unsigned int distance = CalculateDistance();

			    if (distance > MAX_DISTANCE) {
				    fprintf_P(fio_0, PSTR("Distance is over the 100cm threshold. Door will remain closed.\n\r"));
				    PORTB |= (1 << ORANGE_LED); // turn orange LED on
				    PORTB &= ~(1 << GREEN_LED);  // Turn green LED OFF
				    PORTB &= ~(1 << RED_LED);
					PORTB &= ~(1 << SPEAKER_PIN);
					distance = CalculateDistance();
				    break;
			    }
			    
			    else {
				    
				    fprintf_P(fio_0, PSTR("Calculated Distance: %d cm\n\r"), distance);
				    PORTB &= ~(1 << RED_LED);  // Turn Red LED OFF if motion is detected
				    PORTB &= ~(1 << ORANGE_LED);
					PORTB |= (1 << GREEN_LED); 
				    PORTB |= (1 << SPEAKER_PIN); // Turn ON speaker
				    distance = CalculateDistance();
					_delay_ms(500);
					PORTB &= ~(1 << SPEAKER_PIN);
				    
				    if (distance <= DISTANCE_THRESHOLD) {
					    fprintf_P(fio_0, PSTR("Distance is too close to the door. Door will remain closed.\n\r"));
					    PORTB &= ~(1 << GREEN_LED);  // Turn green LED OFF if motion is detected
					    PORTB |= (1 << RED_LED); // turn red LED on
					    PORTB &= ~(1 << SPEAKER_PIN);
					    break;
				    }
				    
					
				    else {
					    PORTB |= (1 << GREEN_LED);  // Keep LED ON otherwise
					    break;
				    }
					
				    
			    }
			 
			 case 'B':
			 case  'b':
				return 0;
			    break;
		    }
		    
		    fprintf_P(fio_0, PSTR("\n\r"));
		    
		    break;

		    default:
		    fprintf_P(fio_0, PSTR("Invalid command! Press 'T' or 't' to trigger.\n\r"));
		    PORTB |= (1 << RED_LED); // turn red LED on
		    PORTB &= ~(1 << GREEN_LED);
		    PORTB &= ~(1 << ORANGE_LED);
		    break;
	    }
    }

    return 0;
    }
