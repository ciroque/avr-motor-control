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

#define VOLTAGE_CONTROL_ON		PORTB |= (1 << PORTB5)
#define VOLTAGE_CONTROL_OFF		PORTB &= ~(1 << PORTB5)
#define VOLTAGE_CONTROL_TOGGLE	PINB |= (1 << PINB5)

#define DIRECTION_TOGGLE		PINB |= (1 << PINB4)

#define HOLDING_INDICATOR_OFF	PINB |= (1 << PINB3)
#define HOLDING_INDICATOR_ON	PINB |= (0 << PINB3)

enum DIRECTION
{
	HOLD = -1,
	INCR = 0,
	DECR = 1
};

volatile int direction = INCR;

const int MIN_POWER = 0x00;
const int MAX_POWER = 0xFFFE;
const int DUTY_STEP = 1;

void calcFrequency(uint8_t freq);
void initClockMode();
void initInputs();
void initInterrupts();
void initOutputs();
void initPWM();
void scalePwmDuty();

ISR(TIMER3_COMPA_vect)
{
	if(direction == HOLD)
	{
		 // noop, but soon will have timing logic to restart the train.
		 
		 // Set holding indicator off when 
	}
	else
	{
		uint16_t duty = OCR1B;
	
		if(direction == INCR)
		{
			if(duty >= MAX_POWER)
			{
				duty = 65535;
				direction = HOLD;
				DIRECTION_TOGGLE;
				HOLDING_INDICATOR_ON;
			}
			else
			{
				duty += DUTY_STEP;
			}
		}
		else
		{
			if(duty <= MIN_POWER)
			{
				direction = INCR;
				duty += DUTY_STEP;
				DIRECTION_TOGGLE;
			}
			else
			{
				duty -= DUTY_STEP;
			}
		}
	
		OCR1B = duty;
	}
}

ISR(TIMER1_COMPB_vect)
{
	VOLTAGE_CONTROL_ON;
}

ISR(TIMER1_COMPA_vect)
{
	VOLTAGE_CONTROL_OFF;
}

int main()
{
	initInputs();
	initOutputs();
	initClockMode();
	initInterrupts();
	calcFrequency(2);
	sei();
	
    while(1) {;}
}

void calcFrequency(uint8_t freq)
{
	OCR1A = MAX_POWER;
	OCR1B = MIN_POWER;
	OCR3A = ((F_CPU / 1000) / 8) / 7;
}

void initClockMode()
{
	// Timer 1; No Prescaler, 
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	
	// Timer 3
	TCCR3A |= (1 << WGM32);															// CTC mode
	TCCR3B |= (1 << WGM32) | (0 << CS32) | (1 << CS31) | (0 << CS30);				// prescaler 8
}

void initInputs()
{
	
}

void initInterrupts()
{
	// Timer 1 Output Compare Interrupts A and B
	TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1A);
	
	// Timer 0 Output Compare Interrupt A
	TIMSK3 |= (1 << OCIE3A);
}

void initOutputs()
{
	DDRB |= (1 << DDB5) | (1 << DDB4) | (1 << DDB3);
	HOLDING_INDICATOR_OFF;
}
