#ifndef _HCSR04_H_
#define _HCSR04_H_

/*  HC_SR04 ULTRASONIC SENSOR using Atmega2560
 *  Trigger pin = PC4 / Digital pin 33
 *  Echo pin    = ICP4 / Digital pin 49
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define TRIGGER_DDR			DDRC
#define ECHO_DDR			DDRL
#define TRIGGER				PINC4
#define ECHO				PINL0
#define TRIGGER_PORT		PORTC

class HCSR04
{
	public:
		HCSR04(const unsigned long);
		static HCSR04* HCSR04_pointer;
	
	private:
		volatile unsigned char triggerSent;
		volatile unsigned int risingEdge;
		volatile unsigned int fallingEdge;
		volatile unsigned int	distance;
		unsigned char usPerTick;
	
	public:
		inline void handle_interruptICR(void);
		inline void handle_interruptTIMER(void);
		void TIMER1_init(void);
		unsigned int getDistance(void);
};


#endif
