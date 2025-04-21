#define F_CPU 16000000UL  // 16 MHz clock speed
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define BUTTON_OPEN  PD3  // Pin 3 for door open
#define BUTTON_CLOSE PD4  // Pin 4 for door close
#define PHOTORESISTOR_PIN PC0  // ADC0 (Light sensor)
#define THRESHOLD 500  // Light threshold value
#define SPEAKER_PIN PB2  // Pin D10 for speaker (OC2A on Timer2)

void setupPWM()
{
	// Configure Timer1 for Fast PWM mode (servo control)
	TCCR1A |= (1 << COM1A1) | (1 << WGM11);
	TCCR1B |= (1 << WGM13) | (1 << WGM12) | (1 << CS11); // Prescaler = 8
	ICR1 = 40000; // (16MHz / 8) / 50Hz = 40000
	DDRB |= (1 << PB1);  // Set PB1 as output (Servo control)
}

void setServoAngle(int16_t speed)
{
	OCR1A = 2950 + speed; // 3000 - stop, >3000 - CCW , <3000 CW
}

void setupButtons()
{
	DDRD &= ~(1 << BUTTON_OPEN);  // Set PD2 as input
	DDRD &= ~(1 << BUTTON_CLOSE); // Set PD3 as input
	PORTD |= (1 << BUTTON_OPEN);  // Enable pull-up resistor on PD2
	PORTD |= (1 << BUTTON_CLOSE); // Enable pull-up resistor on PD3
}

void doorOpen()
{
	setServoAngle(200);  // Open door by rotating servo
	_delay_ms(450);      // Wait 1 second for the door to open

	setServoAngle(0);    // Stop the servo rotation
}

void doorClose()
{
	setServoAngle(-200);  // Close door by rotating servo in the opposite direction
	_delay_ms(650);      // Wait 1 second for the door to close

	setServoAngle(0);    // Stop the servo rotation
}

void initADCSetup(void)
{
	ADMUX = (1 << REFS0) | PHOTORESISTOR_PIN;  // AVcc as reference, select ADC0
	ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);  // Enable ADC, prescaler 128
}

uint16_t readADCValue(void)
{
	ADCSRA |= (1 << ADSC);  // Start conversion
	while (ADCSRA & (1 << ADSC));  // Wait for conversion to finish
	return ADC;
}

void setupTimerInterrupt(void)
{
	TCCR0A = (1 << WGM01); // CTC mode
	TCCR0B = (1 << CS02) | (1 << CS00); // Prescaler 1024
	OCR0A = 156; // Approx 100ms interrupt at 16MHz clock
	TIMSK0 = (1 << OCIE0A); // Enable compare match interrupt
	sei(); // Enable global interrupts
}

volatile uint8_t timerFlag = 0;

ISR(TIMER0_COMPA_vect)
{
	timerFlag = 1;
}

// Setup Timer2 for generating tone on D8 (PD6)
void setupTone()
{
	// Configure Timer2 in Fast PWM mode to generate tone
	TCCR2A |= (1 << WGM21) | (1 << WGM20) | (1 << COM2A0);  // Fast PWM mode, toggle OC2A on compare match
	TCCR2B |= (1 << CS21);  // Prescaler = 8

	// Set PD6 (OC2A) as output
	DDRD |= (1 << PD6);
}

void playTone(uint16_t frequency)
{
	// Calculate the OCR2A value based on the desired frequency
	uint16_t ocrValue = (F_CPU / (2 * 8 * frequency)) - 1;
	OCR2A = ocrValue;
}

void stopTone()
{
	// Stop PWM on OC2A (PD6)
	TCCR2A &= ~(1 << COM2A0);
}

int main(void)
{
	int doorState = 0;
	setupPWM();  // Initialize PWM for servo
	setupButtons();  // Initialize button inputs
	initADCSetup();  // Initialize ADC for light sensor
	setupTimerInterrupt();  // Set up timer interrupt
	setupTone();  // Set up tone generation for speaker

	while (1)
	{
		uint16_t lightLevel = readADCValue();  // Read light sensor value
		
		// Check if the PSR is covered (dark)
		if (lightLevel < THRESHOLD)
		{
			
			// If it's dark, prevent the door from opening
			if (!(PIND & (1 << BUTTON_OPEN))) // If button 1 (PD2) is pressed
			{
				_delay_ms(100); // Debounce
				if (!(PIND & (1 << BUTTON_OPEN)))
				{
			
					// Do nothing since the door cannot open when it's dark
				}
			}
			
		}
		if (lightLevel > THRESHOLD)
		{
			
			// If it's light, door can open
			if (!(PIND & (1 << BUTTON_OPEN))) // If button 1 (PD2) is pressed
			{
				_delay_ms(100);  // Debounce
				if (!(PIND & (1 << BUTTON_OPEN)) && doorState == 0)
				{
					doorOpen();
					doorState = 1;  // Set door state to open
					playTone(1000);  // Play 1000 Hz tone when door opens
					_delay_ms(1000);  // Wait for 1 second
					stopTone();  // Stop tone after 1 second
				}
			}
			if (!(PIND & (1 << BUTTON_CLOSE))) // If button 2 (PD3) is pressed
		{
			_delay_ms(100); // Debounce
			if (!(PIND & (1 << BUTTON_CLOSE)) && doorState == 1)
			{
				doorClose();
				doorState = 0;  // Set door state to closed
			}
		}
		}

		
	}

	return 0;
}