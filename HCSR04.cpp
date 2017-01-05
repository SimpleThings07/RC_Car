#include "HCSR04.h"

/* ICP4 (Input Capture Interrupt) , DIGITAL PIN 49
   *   ISR(TIMER4_CAPT_vect) when the signal on the ICP pin goes from 0 to 1 (raising edge).
   *   The method contains a if stament to see if it was called during a raising edge or falling edge.
   *   On a raising edge it switches the ICP to falling edge detection and then stores the start time.
   *   On a falling edge it switches the ICP to raising edge detection and gets the end time to calculate the distance in millimeters.
   */

HCSR04* HCSR04::HCSR04_pointer;

HCSR04::HCSR04(const unsigned long FCPU)
{
	HCSR04_pointer = this;
	if (FCPU == 1000000UL)
		usPerTick = 8;
	
	ECHO_DDR &= ~(1 << ECHO);
	TRIGGER_DDR |= (1 << TRIGGER);
	TIMER1_init();
}

void HCSR04::TIMER1_init()
{
	 /**
   * 
   * https://s28.postimg.org/o8hzag019/Waveform_Generation_Mode_Bit_Descriptioon.jpg
   */
  TCCR4B |= (1 << WGM42) | (1 << ICNC4);  //Input Capture Noise Canceler
   /**
   * https://s27.postimg.org/k2tu225pf/Clock_Select_Bit_Description.jpg
   **/
  TCCR4B |= 1 << CS41 | 1 << CS40; // prescaler 64

  TIMSK4 |= 1 << OCIE4A | 1 << ICIE4;
  /**
   *  ICIE: Input Capture Interrupt Enable
      When this bit is written to '1', and the I-flag in the Status Register is set (interrupts globally enabled), the
      Timer/Counter1 Input Capture interrupt is enabled. The corresponding Interrupt Vector is executed when
      the ICF Flag, located in TIFR1, is set.

      When a capture is triggered according to the ICESn setting, the counter value is copied into the Input Capture Register
      (ICRn). The event will also set the Input Capture Flag (ICFn), and this can be used to cause an Input Capture
      Interrupt, if this interrupt is enabled.

      When the ICRn is used as TOP value (see description of the WGMn3:0 bits located in the TCCRnA and the
      TCCRnB Register), the ICPn is disconnected and consequently the input capture function is disabled.
   */

  /* ICP4 (Input Capture Interrupt) , DIGITAL PIN 49
   *   ISR(TIMER4_CAPT_vect) when the signal on the ICP pin goes from 0 to 1 (raising edge).
   *   The method contains a if stament to see if it was called during a raising edge or falling edge.
   *   On a raising edge it switches the ICP to falling edge detection and then stores the start time.
   *   On a falling edge it switches the ICP to raising edge detection and gets the end time to calculate the distance in millimeters.
   */
  TCCR4B |= 1 << ICES4; // Input capture on rising edge
  /**
   * This bit selects which edge on the Input Capture pin (ICP1) that is used to trigger a capture event. When
    the ICES1 bit is written to zero, a falling (negative) edge is used as trigger, and when the ICES1 bit is
    written to '1', a rising (positive) edge will trigger the capture.
    When a capture is triggered according to the ICES1 setting, the counter value is copied into the Input
    Capture Register (ICR1). The event will also set the Input Capture Flag (ICF1), and this can be used to
    cause an Input Capture Interrupt, if this interrupt is enabled.
    When the ICR1 is used as TOP value (see description of the WGM1[3:0] bits located in the TCCR1A and
    the TCCR1B Register), the ICP1 is disconnected and consequently the Input Capture function is
    disabled.
   */
  OCR4A = 17500;
  // "we suggest to use over 60ms measurement cycle, in order to prevent trigger signal to the echo signal."
  // source: http://www.robosoftsystems.co.in/wikidocs/index.php?title=Ultrasonic_Sensor_(HC-SR04)
  // 70ms cycle: 16MHz/64 = 125000 counts/second => 125000/10 = 12500 counts/100ms => 12500/100*70 = 8750 counts / 70ms
  sei();
  HCSR04::usPerTick = 4; // 16MHz / 64 = 250 000 counts/second => 1/250 000 = 4 
}

unsigned int HCSR04::getDistance(void)
{
	return distance;
}


inline void HCSR04::handle_interruptICR()
{
	/*  Input Capture Unit
   * https://s23.postimg.org/gumgo0sln/Input_Capture_Unit.jpg
   */
  if (TCCR4B & (1<<ICES4)) // On rising edge
  {
    TCCR4B &= ~(1<<ICES4); // Next time detect falling edge
    risingEdge = ICR4; // Save current count
  }
  else // On falling edge
  {
    TCCR4B |= (1<<ICES4); // Next time detect falling edge
    fallingEdge = ICR4; // Save current count
    HCSR04::distance = (((unsigned int)HCSR04::fallingEdge - (unsigned int)HCSR04::risingEdge) * HCSR04::usPerTick ) / 58;
    HCSR04::triggerSent = 0;
	TCNT4 = 0;
  }
}

inline void HCSR04::handle_interruptTIMER()
{
	TRIGGER_PORT |= (1 << TRIGGER);
	_delay_us(10);
	TRIGGER_PORT &= ~(1 << TRIGGER);
	HCSR04::triggerSent = 1;
}

ISR(TIMER4_COMPA_vect)
{
	HCSR04::HCSR04_pointer->handle_interruptTIMER();
}

ISR(TIMER4_CAPT_vect)
{
	HCSR04::HCSR04_pointer->handle_interruptICR();
}




