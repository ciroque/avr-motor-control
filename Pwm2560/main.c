/*
 * Pwm2560.c
 *
 * Created: 2016-03-05 17:21:28
 * Author : Steve Wagner
 *
 *	Based on the AVRFreaks tutorial at: http://www.avrfreaks.net/forum/tut-c-newbies-guide-avr-pwm-incomplete
 *
 *	Must use Phase Correct PWM instead of Fast PWM as the phase matters in motor control.
 *	
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

int main(void) 
{
	// Output to PORTB, PIN7
	DDRB |= (1 << PORTB7);

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
	
	while(1) 
	{
		// ramp the duty cycle up and down...
		_delay_ms(CYCLE_DELAY);
		if(direction == INCR) 
		{
			if(dutyCycle >= DIVISOR) 
			{
				_delay_ms(STATE_HOLD);
				direction = DECR;
				dutyCycle -= STEP;
			} 
			else 
			{
				dutyCycle += STEP;
			}
		} 
		else 
		{
			if(dutyCycle <= 0) 
			{
				_delay_ms(STATE_HOLD);
				direction = INCR;
				dutyCycle += STEP;
			} 
			else 
			{
				dutyCycle -= STEP;
			}
		}
	}
}

// Timer 0 Interrupt Handler. Simply calculate and update the duty cycle.
ISR(TIMER0_OVF_vect)
{
	OCR0A = scaleDutyCycle();
}
