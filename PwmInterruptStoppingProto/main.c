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


#define VOLTAGE_CONTROL_ON		PORTC |= (1 << PORTC5)
#define VOLTAGE_CONTROL_OFF		PORTC &= ~(1 << PORTC5)

#define DIRECTION_CONTROL_OFF	PORTC |= (1 << PORTC4)
#define DIRECTION_CONTROL_ON	PORTC &= ~(1 << PORTC4)

#define HOLDING_INDICATOR_OFF	PORTC |= (1 << PORTC3)
#define HOLDING_INDICATOR_ON	PORTC &= ~(1 << PORTC3)

#define EXTERNAL_INTERRUPT_INDICATOR_OFF	PORTC |= (1 << PORTC0)
#define EXTERNAL_INTERRUPT_INDICATOR_ON		PORTC &= ~(1 << PORTC0)

enum MOTION_TRANSITIONS
{
	HOLD = -1,
	ACCELERATE = 0,
	DECELERATE = 1,
	DECELERATE_TO_HOLD = 2,
	HOLD_TO_ACCELERATE = 3
};

volatile int motion_transitions = ACCELERATE;
volatile int timer3_overflow_count = 0;

const int MIN_POWER = 0x00;
const int MAX_POWER = 0xFFFE;
const int DUTY_STEP = 1;

void establishBaseFrequencies(uint8_t freq);
uint16_t handleAccelerate(uint16_t duty);
uint16_t handleDecelerate(uint16_t duty);
void holdForMillis(uint16_t millis);
void initTimers();
void initInputs();
void initInterrupts();
void initOutputs();
void initPWM();
void scalePwmDuty();

ISR(TIMER3_COMPA_vect)
{
	timer3_overflow_count++;
	
	switch(motion_transitions)
	{
		case HOLD:
		{
			HOLDING_INDICATOR_ON;
			DIRECTION_CONTROL_OFF;
			break;
		}
		
		case ACCELERATE: 
		{
			HOLDING_INDICATOR_OFF;
			DIRECTION_CONTROL_ON;
			uint16_t duty = OCR1B;
			OCR1B = handleAccelerate(duty);
			break;
		}
		
		case DECELERATE: 
		{
			HOLDING_INDICATOR_OFF;
			DIRECTION_CONTROL_OFF;
			uint16_t duty = OCR1B;
			OCR1B = handleDecelerate(duty);
			timer3_overflow_count = 0;
			break;
		}
		
		case HOLD_TO_ACCELERATE:
		{
			HOLDING_INDICATOR_ON;
			DIRECTION_CONTROL_OFF;
			holdForMillis(65535);
			break;
		}
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

ISR(PCINT0_vect)
{
	PINC |= (1 << PINC0);
	if(motion_transitions == HOLD)
	{
		motion_transitions = DECELERATE;
	}
}

int main()
{
	initInputs();
	initOutputs();
	initTimers();
	initInterrupts();
	establishBaseFrequencies(7);
	sei();
	
    while(1) {;}
}

void establishBaseFrequencies(uint8_t freq)
{
	OCR1A = MAX_POWER;
	OCR1B = MIN_POWER;
	OCR3A = ((F_CPU / 1000) / 8) / freq;
}

 // NOTE: Needs work to make the acceleration curve match that of a real locomotive...
uint16_t handleAccelerate(uint16_t duty)
{
	if(duty >= MAX_POWER)
	{
		duty = 65535;
		motion_transitions = HOLD;
	}
	else
	{
		duty += DUTY_STEP;
	}
	
	return duty;
}

 // NOTE: Needs work to be able to control the rate of deceleration to land the locomotive where we want it...
uint16_t handleDecelerate(uint16_t duty)
{
	if(duty <= MIN_POWER)
	{
		duty = MIN_POWER;
		motion_transitions = HOLD_TO_ACCELERATE;
	}
	else
	{
		duty -= DUTY_STEP;
	}
	
	return duty;
}

 // NOTE: This needs serious work to be able to finely control the delay...
void holdForMillis(uint16_t millis)
{
	if(timer3_overflow_count >= millis)
	{
		motion_transitions = ACCELERATE;
	}
}

void initTimers()
{
	// Timer 1; No Prescaler, 
	TCCR1B |= (1 << CS10) | (1 << WGM12);
	
	// Timer 3
	TCCR3A |= (1 << WGM32);															// CTC mode
	TCCR3B |= (1 << WGM32) | (0 << CS32) | (1 << CS31) | (0 << CS30);				// prescaler 8
}

void initInputs()
{
	DDRA |= (0 << DDA0);		// pin 1 as input
}

void initInterrupts()
{
	// Timer 1 Output Compare Interrupts A and B
	TIMSK1 |= (1 << OCIE1B) | (1 << OCIE1A);
	
	// Timer 0 Output Compare Interrupt A
	TIMSK3 |= (1 << OCIE3A);
	
	// external interrupt Port A, Pin 1
	PCIFR |= (1 << PCIF0);
	PCMSK0 |= (1 << PCINT0);
	PCICR |= (1 << PCIE0);
}

void initOutputs()
{
	DDRC |= (1 << DDC5) | (1 << DDC4) | (1 << DDC3) | (1 << DDC0);
	HOLDING_INDICATOR_OFF;
	DIRECTION_CONTROL_ON;
	EXTERNAL_INTERRUPT_INDICATOR_OFF;
}
