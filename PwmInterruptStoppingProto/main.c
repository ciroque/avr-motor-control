/*
 * PwmInterruptStoppingProto.c
 *
 * Created: 2016-03-06 17:40:14
 * Author : Steve Wagner
 *
 *	Prototyping the actual code to run the train.
 *	The basic mode of operation is that the PWM duty cycle is driven up to about 80% and stays there.
 *	When a triggering event happens the PWM duty is wound down to 0%, pause  for a period, and then driven back up to 80%;
 *
 *	This will simulate the train being started when the mcu is started and then rolls over one of the proximity sensors on the track.
 */ 

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

enum DIRECTION
{
	INCR = 0,
	DECR = 1
};

volatile double dutyCycle = 0;
volatile int direction = INCR;

const int CYCLE_DELAY = 100;
const int STATE_HOLD = 1000;
const int DIVISOR = 100;
const int STEP = 2;

inline double scaleDutyCycle()
{
	return (dutyCycle / DIVISOR) * 255.0;
}

void initInputs()
{
	// Port C as inputs
	DDRC = (0 << PORTC);
}

void initOutputs() 
{
	// Output to Port B pins 6 and 7; 7 will get the Timer 0 output, 6 will toggle on overflow
	DDRB |= (1 << PORTB7) | (1 << PORTB6);
	
	// Set Port B
}

void initPWM()
{
	// Mode should be 11, PWM, Phase Correct
	// ? Ordering of the flags, doc's show 0 to the right increasing to the left, is it standard to follow that in the code?
	// ? Is it preferable to be explicit and zero out bits as well as setting bits?
	TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
	
	// Enable the Interrupt Overflow for Timer 0
	TIMSK0 = (1 << TOIE0);
	
	// Set the TOP
	// ? Do I have the mode configured I think I do?
	OCR0A = scaleDutyCycle();

	// enable interrupts
	sei();

	// reset the prescaler and use the internal clock,
	TCCR0B = (1 << CS00) | (1 << CS02);
}

int main(void)
{
	initInputs();
	initOutputs();
	initPWM();
	
    while (1) 
    {
    }
}

// Timer 0 Interrupt Handler. Simply calculate and update the duty cycle.
ISR(TIMER0_OVF_vect)
{
	OCR0A = scaleDutyCycle();
	PORTB ^= (1 << PINB6);
}
