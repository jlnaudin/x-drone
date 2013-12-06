#include "AP_AnalogSource_PIRATES.h"

extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
}

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WConstants.h"
#endif

volatile char sonar_meas=0;
volatile unsigned int sonar_data=0, sonar_data_start=0, pre_sonar_data=0; // Variables for calculating length of Echo impulse
volatile uint8_t sonar_error_cnt=0;
volatile char sonar_skip=0;

// Sonar read interrupts
ISR(TIMER5_COMPA_vect) // This event occurs when counter = 65510
{
	if (sonar_skip == 0){
		if (sonar_meas == 0) // sonar_meas=1 if we not found Echo pulse, so skip this measurement
			sonar_data = 0;
		sonar_meas=0; // Clean "Measurement finished" flag
		PORTH|=B01000000; // set Sonar TX pin to 1 and after ~12us set it to 0 (below) to start new measurement
		sonar_skip = 6; // next line will decrease it, 5*65535 = 163ms - sonar measurement cycle 
	}
	sonar_skip--; 
} 

ISR(TIMER5_OVF_vect) // Counter overflowed, 12us elapsed
{
	PORTH&=B10111111; // set TX pin to 0, and wait for 1 on Echo pin (below) 
}

ISR(PCINT0_vect)
{
	if (sonar_meas==0) {
		if (PINB & B00010000) { 
			sonar_data_start = TCNT5; // We got 1 on Echo pin, remeber current counter value
		} else {
			sonar_data=(uint32_t)TCNT5+((uint32_t)(5-sonar_skip)*65535)-sonar_data_start; // We got 0 on Echo pin, calculate impulse length in counter ticks
			sonar_meas=1; // Set "Measurement finished" flag
		}
	} 
}
void AP_AnalogSource_PIRATES::init(void)
{
	// Sonar INIT
	//=======================
	PORTH&=B10111111; // H6 -d9  - sonar TX
	DDRH |=B01000000;

	PORTB&=B11101111; // B4 -d10 - sonar Echo
	DDRB &=B11101111;

	// div64 = 0.5 us/bit
	// Using timer5, warning! Timer5 also share with RC PPM decoder
	TCCR5A = 0; //standard mode with overflow at A and OC B and C interrupts
	TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us
	TIMSK5 = B00000111; // ints: overflow, capture, compareA
	OCR5A = 65510; // approx 10m limit, 33ms period
	OCR5B = 3000;
}

float AP_AnalogSource_PIRATES::read(void)
{
		float result;
		if (((sonar_data < 590) || (sonar_data > 59000)) && (pre_sonar_data > 0) ) {	//value must be 5cm > X < 5m
			if (sonar_error_cnt > 50) {
				result = 65490; // set as maximum value 5.55m - fallback to Baro (in arducopter.pde)
			} else {
				sonar_error_cnt++;
				result = pre_sonar_data; // use previous valid data
			}
		} else {
			sonar_error_cnt = 0; // Valid data received, reset counter
			pre_sonar_data = sonar_data;
			result = sonar_data;
		}
		return(result/118);
}
