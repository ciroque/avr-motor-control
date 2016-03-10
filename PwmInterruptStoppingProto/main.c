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

#define LED_ON		PORTB |= (1 << PORTB5)
#define LED_OFF		PORTB &= ~(1 << PORTB5)
#define LED_TOGGLE	PINB |= (1 << PINB5)

const int MAX_OUTPUT = 0xFFFF;

enum DIRECTION
{
	INCR = 0,
	DECR = 1
};

void calcFrequency(uint8_t freq);
void initClockMode();
void initInterrupts();
void initOutputs();
void initPWM();

ISR(TIMER0_COMPA_vect)
{
	uint16_t duty = OCR1B;
	if(duty < MAX_OUTPUT)
	{
		duty++;
	}
	else
	{
		duty = 0;
	}
	OCR1B = duty;
}

ISR(TIMER1_COMPB_vect)
{
	LED_ON;
}

ISR(TIMER1_COMPA_vect)
{
	LED_OFF;
}

int main(void)
{
	initOutputs();
	initClockMode();
	initInterrupts();
	calcFrequency(2);
	sei();
	
    while(1) {;}
}

void calcFrequency(uint8_t freq)
{
	OCR1A = MAX_OUTPUT;
	OCR1B = 60000;
	
	OCR0A = 30 * 7.8125 - 1;
}

void initClockMode()
{
	// Timer 1; No Prescaler, 
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	
	// Timer 0
	TCCR0A |= (1 << WGM01);						// CTC mode
	TCCR0B |= (1 << CS02) || (1 << CS00);		// 1024 prescaler
}

void initInterrupts()
{
	// Timer 1 Output Compare Interrupts A and B
	TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1A); // Compare Interrupts A & B
	
	// Timer 0 Output Compare Interrupt A
	TIMSK0 |= (1 << OCIE0A);
}

void initOutputs()
{
	DDRB |= (1 << DDB5) | (1 << DDB4);
}
