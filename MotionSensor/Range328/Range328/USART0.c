/*
 * USART0.c
 *
 * Created: 4/1/2016 2:52:20 PM
 * Author : Mark
 */ 

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdio.h>


#include "USART0.h"

#define UART_BUF_SIZE 64
#define UartEchoOn 0 
 
volatile unsigned char TxDBuff0[UART_BUF_SIZE];
volatile unsigned char RxDBuff0[UART_BUF_SIZE];
 
// Uart Globals 

volatile unsigned char  TxDCnt0;
volatile unsigned char  RxDCnt0;
volatile unsigned char  *TxDBuffWR0, *TxDBuffRD0;
volatile unsigned char  *RxDBuffWR0, *RxDBuffRD0;


// ******************************************************
//	Call This Function to initialize the Uart. For the BAUD
//	parameter select the U2X=0 column of UBRR setting for 
//	crystal frequency of your setup. 
//*****************************************************

static int putch0(char c, FILE *stream);
static int getch0(FILE *stream);

FILE  usart0_Stream = FDEV_SETUP_STREAM(putch0, getch0, _FDEV_SETUP_RW);


static int putch0(char c, FILE *stream) {
	while(uart0_write_buff_full() == 1);
	uart0_putc(c);
	return 0;
}

static int getch0(FILE *stream) {
	while(uart0_RxCount() == 0);
	return uart0_getc();
}


void init_uart0(unsigned int  BAUD) {// initialize uart 

	// Set baud rate
	UBRR0H = ((unsigned char) (BAUD>>8)) & ~0x80;
	UBRR0L = (unsigned char) BAUD;
	// Enable Receiver and Transmitter
	UCSR0B = (1<<RXEN0) | (1<<TXEN0) | (1<<RXCIE0);
	// Set frame format: 8data, 2stop bit
	UCSR0C = (1<<USBS0) | (3<<UCSZ00);
	
    DDRD |= (1<<DDD1);        				//  Set DDR for Txd Pin on PortD 
    DDRD &= ~(1<<DDD0);        				//  Set DDR for Rxd Pin on PortD 


    TxDBuffWR0  = TxDBuffRD0 = TxDBuff0;			// Init Queues 
    RxDBuffWR0  = RxDBuffRD0 = RxDBuff0;
    TxDCnt0 = 0;
    RxDCnt0 = 0;
//	stdout = &mystdout;	// Bind mine to theirs
//	stdin = &mystdin;
}

// ******************************************************
//	Handle for Tx Interrupt 
//******************************************************

ISR (USART_UDRE_vect)  {		// Handler for uart data buffer empty interrupt 

    if (TxDCnt0 > 0) {
        UDR0 = *TxDBuffRD0;           		// write byte to data buffer 
        if (++TxDBuffRD0 >= TxDBuff0 + UART_BUF_SIZE) // Wrap Pointer 
            TxDBuffRD0 = TxDBuff0;
        if(--TxDCnt0 == 0)             			// if buffer is empty: 
            UCSR0B&=~(1<<UDRIE0);                    // disable UDRIE int 
    }
}

// ******************************************************
//	Handle for Rx Interrupt 
//******************************************************

ISR (USART_RX_vect)  {  // Interrupt handler for receive complete interrupt 

	unsigned char key;
	 key = UDR0;            // Get UDR --> Buff 
	 *RxDBuffWR0 = key;
	if(UartEchoOn==1) {
		if(!uart0_write_buff_full())
			uart0_putc(key);
		}
    RxDCnt0++;
    if (++RxDBuffWR0 >= RxDBuff0 + UART_BUF_SIZE) // Wrap Pointer 
        RxDBuffWR0 = RxDBuff0;
}
// *****************************************************
//	Returns one Character form Rx Buffer
// *****************************************************

char  uart0_getc(void) {
    unsigned char  c;

    if(RxDCnt0 > 0){
	
		cli();
		RxDCnt0--;
		c = *RxDBuffRD0;              // Get Buff Char and Return  
		if (++RxDBuffRD0 >= RxDBuff0 + UART_BUF_SIZE)  // Wrap Pointer 
			RxDBuffRD0 = RxDBuff0;
		sei();
		return c;
		}
	else 
	return 0;
}

// ****************************************************
//	Places a character in the Tx buffer Initiates the transfer and lets the 
//	ISR do the rest.
// *****************************************************

unsigned char  uart0_putc(char  c) {
    if (TxDCnt0<UART_BUF_SIZE) {
        cli();
        TxDCnt0++;
        *TxDBuffWR0 = c;               		// put character into buffer 
        if (++TxDBuffWR0 >= TxDBuff0 + UART_BUF_SIZE) 
													// pointer wrapping 
            TxDBuffWR0 = TxDBuff0;
        UCSR0B|=(1<<UDRIE0);   				// enable UDRIE int 
        sei();
        return 1;
    } 
    else 
        return 0;                           // buffer is full 
}


// ************************************************
//	Call this function to check for the presence of a character in the 
//	Rx Buffer.  The function returns the character count.
// ************************************************

unsigned char  uart0_RxCount(void) {

return(RxDCnt0);

}

// ************************************************
//	Call this function to check for the Tx Buffer is full.  
//	The function returns 1 if full 0 if not..
// ************************************************
unsigned char  uart0_write_buff_full(void) {
if(TxDCnt0 >= UART_BUF_SIZE)
	return 1 ;
return 0;
}

// ******************************************************
//	Call this function to  transfer a string stored in RAM to the output buffer.  
//	Will loop until the entire string is transfered.
// ******************************************************

void uart0_puts(char  *s) { 
	unsigned char  i = 0;
	while ( s[i] != 0 ) {
		while(uart0_write_buff_full());
		uart0_putc(s[i++]);
		}

}

// ************************************************
//	Call this function to  transfer a string from program memory 
//	(FLASH) to the output buffer.  
//	Will loop until the entire string is transfered.
// ************************************************

void uart0_puts_P(PGM_P s) { 
	unsigned char  i = 0;
	while ( pgm_read_byte(&s[i]) != 0 ){
		while(uart0_write_buff_full());
		uart0_putc(pgm_read_byte(&s[i++]));
		}

}

