/*
	APM_RC.cpp - Radio Control Library for ArduPirates Arduino Mega with IPWM
	
	Total rewritten by Syberian
	
	Methods:
		Init() : Initialization of interrupts an Timers
		OutpuCh(ch,pwm) : Output value to servos (range : 900-2100us) ch=0..10
		InputCh(ch) : Read a channel input value.  ch=0..7
		GetState() : Returns the state of the input. 1 => New radio frame to process
		             Automatically resets when we call InputCh to read channels
		
*/

#include "APM_RC_PIRATES.h"

#include <avr/interrupt.h>

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


#if !defined(__AVR_ATmega1280__) && !defined(__AVR_ATmega2560__)
# error Please check the Tools/Board menu to ensure you have selected Arduino Mega as your target.
#else

// Variable definition for Input Capture interrupt
volatile uint8_t radio_status=0;
volatile bool use_ppm = 0;
volatile bool bv_mode;
uint8_t *pinRcChannel;

// failsafe counter
volatile uint8_t failsafeCnt=0;

// ******************
// rc functions split channels
// ******************
volatile uint16_t rcPinValue[NUM_CHANNELS]; // Default RC values

// Configure each rc pin for PCINT
void configureReceiver() {
	// PCINT activated only for specific pin inside [A8-A15]
	DDRK = 0;  // defined PORTK as a digital port ([A8-A15] are consired as digital PINs and not analogical)
	PORTK   = (1<<0) | (1<<1) | (1<<2) | (1<<3) | (1<<4) | (1<<5) | (1<<6) | (1<<7); //enable internal pull ups on the PINs of PORTK
	#ifdef SERIAL_SUM
		PCMSK2=1;	// Enable int for pin A8
	#else
		PCMSK2 = 255; // 
	#endif
	PCMSK0 = B00010000; // sonar port B4 - d10 echo
	PCICR = B101; // PCINT activated only for PORTK dealing with [A8-A15] PINs
}

ISR(PCINT2_vect) { //this ISR is common to every receiver channel, it is call everytime a change state occurs on a digital pin [D2-D7]
static  uint8_t mask;
static  uint8_t pin;
static  uint16_t cTime,dTime;
static uint16_t edgeTime[8];
static uint8_t PCintLast;

  cTime = TCNT5;         // from sonar
  pin = PINK;             // PINK indicates the state of each PIN for the arduino port dealing with [A8-A15] digital pins (8 bits variable)
	mask = pin ^ PCintLast;   // doing a ^ between the current interruption and the last one indicates wich pin changed
  sei();                    // re enable other interrupts at this point, the rest of this interrupt is not so time critical and can be interrupted safely
  PCintLast = pin;          // we memorize the current state of all PINs [D0-D7]

	if (use_ppm) {
		static uint8_t pps_num=0;
		static uint16_t pps_etime=0;
	
		if (pin & 1) { // Rising edge detection
			if (cTime < pps_etime) // Timer overflow detection
				dTime = (0xFFFF-pps_etime)+cTime;
			else
				dTime = cTime-pps_etime; 
			if (dTime < 4400) {
				rcPinValue[pps_num] = dTime>>1;
				pps_num++;
				pps_num&=7; // upto 8 packets in slot
			} else 
				pps_num=0; 
		 	pps_etime = cTime; // Save edge time
		 }
	} else {
		// generic split PPM  
	  // mask is pins [D0-D7] that have changed // the principle is the same on the MEGA for PORTK and [A8-A15] PINs
	  // chan = pin sequence of the port. chan begins at D2 and ends at D7
	  if (mask & 1<<0)
	    if (!(pin & 1<<0)) {
	      dTime = cTime-edgeTime[0]; if (1600<dTime && dTime<4400) rcPinValue[0] = dTime>>1;
	    } else edgeTime[0] = cTime;
	  if (mask & 1<<1)
	    if (!(pin & 1<<1)) {
	      dTime = cTime-edgeTime[1]; if (1600<dTime && dTime<4400) rcPinValue[1] = dTime>>1;
	    } else edgeTime[1] = cTime;
	  if (mask & 1<<2) 
	    if (!(pin & 1<<2)) {
	      dTime = cTime-edgeTime[2]; if (1600<dTime && dTime<4400) rcPinValue[2] = dTime>>1;
	    } else edgeTime[2] = cTime;
	  if (mask & 1<<3)
	    if (!(pin & 1<<3)) {
	      dTime = cTime-edgeTime[3]; if (1600<dTime && dTime<4400) rcPinValue[3] = dTime>>1;
	    } else edgeTime[3] = cTime;
	  if (mask & 1<<4) 
	    if (!(pin & 1<<4)) {
	      dTime = cTime-edgeTime[4]; if (1600<dTime && dTime<4400) rcPinValue[4] = dTime>>1;
	    } else edgeTime[4] = cTime;
	  if (mask & 1<<5)
	    if (!(pin & 1<<5)) {
	      dTime = cTime-edgeTime[5]; if (1600<dTime && dTime<4400) rcPinValue[5] = dTime>>1;
	    } else edgeTime[5] = cTime;
	  if (mask & 1<<6)
	    if (!(pin & 1<<6)) {
	      dTime = cTime-edgeTime[6]; if (1600<dTime && dTime<4400) rcPinValue[6] = dTime>>1;
	    } else edgeTime[6] = cTime;
	  if (mask & 1<<7)
	    if (!(pin & 1<<7)) {
	      dTime = cTime-edgeTime[7]; if (1600<dTime && dTime<4400) rcPinValue[7] = dTime>>1;
	    } else edgeTime[7] = cTime;
	  
	  // failsafe counter must be zero if all ok  
	  if (mask & 1<<pinRcChannel[2]) {    // If pulse present on THROTTLE pin, clear FailSafe counter  - added by MIS fow multiwii (copy by SovGVD to megapirateNG)
        failsafeCnt = 0;
	  }

	}
}

uint16_t readRawRC(uint8_t chan) {
  uint16_t data;
  uint8_t oldSREG;
  oldSREG = SREG;
  cli(); // Let's disable interrupts
  data = rcPinValue[pinRcChannel[chan]]; // Let's copy the data Atomically
  SREG = oldSREG;
  sei();// Let's enable the interrupts
  return data; // We return the value correctly copied when the IRQ's where disabled
}
  
//######################### END RC split channels

// Constructors ////////////////////////////////////////////////////////////////

APM_RC_PIRATES::APM_RC_PIRATES(int _use_ppm, int _bv_mode, uint8_t *_pin_map)
{
	use_ppm = _use_ppm; // Use serial sum (PPM)
	bv_mode = _bv_mode; // BlackVortex mode
	pinRcChannel = _pin_map; // Channel mapping
	// Fill default RC values array, set 900 for Throttle channel and 1500 for others
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		if (_pin_map[i] == 2) {
			rcPinValue[i] = 900;
		} else {
			rcPinValue[i] = 1500;
		}
	}
}

// Public Methods //////////////////////////////////////////////////////////////

void APM_RC_PIRATES::Init( Arduino_Mega_ISR_Registry * isr_reg )
{
	//We are using JUST 1 timer1 for 16 PPM outputs!!! (Syberian)
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  if (bv_mode) {
		pinMode(32,OUTPUT);//cam roll L5
		pinMode(33,OUTPUT);// cam pitch L4
	} else {
		pinMode(44,OUTPUT);//cam roll L5
		pinMode(45,OUTPUT);// cam pitch L4
	}
  
  //general servo
  TCCR5A =0; //standard mode with overflow at A and OC B and C interrupts
  TCCR5B = (1<<CS11); //Prescaler set to 8, resolution of 0.5us
  TIMSK5=B00000111; // ints: overflow, capture, compareA
  OCR5A=65510; 
  OCR5B=3000;
  
	//motors
  OCR1A = 1800; 
  OCR1B = 1800; 
  ICR1 = 40000; //50hz freq...Datasheet says  (system_freq/prescaler)/target frequency. So (16000000hz/8)/50hz=40000,
  TCCR1A = (1<<WGM31); 
  TCCR1B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  TIMSK1 = 1;
  
  OCR3A = 1800; 
  OCR3B = 1800; 
  OCR3C = 1800; 
  ICR3 = 40000; //50hz freq
  TCCR3A = (1<<WGM31);
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  TIMSK3 = 1;
  
  OCR4A = 1800; 
  OCR4B = 1800; 
  OCR4C = 1800; 
  ICR4 = 40000; //50hz freq
  TCCR4A = (1<<WGM31);
  TCCR4B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);
  TIMSK4 = 1;
  
  configureReceiver();
}



uint16_t OCRxx1[8]={1800,1800,1800,1800,1800,1800,1800,1800,};
char OCRstate = 7;
/*
D	Port PWM
2	e4	0 3B
3	e5	1 3C
4	g5	2
5	e3	3 3A
6	h3	4 4A
7	h4	5 4B
8	h5	6 5C
9	h6	7
//2nd group
22	a0	8
23	a1	9
24	a2	10
25	a3	11
26	a4	12
27	a5	13
28	a6	14
29	a7	15
*/
ISR(TIMER5_COMPB_vect)
{ // set the corresponding pin to 1
	OCRstate++;
	OCRstate&=15;
	if (bv_mode) {
		switch (OCRstate>>1)
		{
			case 0: if(OCRstate&1)PORTC&=(1<<5)^255; else PORTC|=(1<<5);break;	//d32, cam roll
			case 1: if(OCRstate&1)PORTC&=(1<<4)^255; else PORTC|=(1<<4);break;	//d33, cam pitch	
		}
	} else {
		switch (OCRstate>>1)
		{
			case 0:	if(OCRstate&1)PORTL&=(1<<5)^255; else PORTL|=(1<<5);break;	//d44, cam Roll
			case 1:	if(OCRstate&1)PORTL&=(1<<4)^255; else PORTL|=(1<<4);break;	//d45, cam Pitch
		}
	}
	if(OCRstate&1)OCR5B+=5000-OCRxx1[OCRstate>>1]; else OCR5B+=OCRxx1[OCRstate>>1];
}

/*
ch			3		4		1		2		7		8		10		11
=======================================================================
Pin			D2	D3	D5	D6	D7	D8	D11		D12
=======================================================================

For motor mapping, see release_notes.txt
*/
int ocr_tbl[8];
ISR(TIMER4_OVF_vect)
{
	OCR4A = ocr_tbl[1];
	OCR4B = ocr_tbl[4];
	OCR4C = ocr_tbl[5];
}

ISR(TIMER3_OVF_vect)
{
	OCR3A = ocr_tbl[0];
	OCR3B = ocr_tbl[2];
	OCR3C = ocr_tbl[3];
}

ISR(TIMER1_OVF_vect)
{
	OCR1A = ocr_tbl[6];
	OCR1B = ocr_tbl[7];
}

void APM_RC_PIRATES::OutputCh(uint8_t ch, uint16_t pwm)
{
	pwm = constrain(pwm,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
	pwm <<= 1;   // pwm*2;
 
	switch(ch)
  {
    case 0:  ocr_tbl[0] = pwm; break; //5
    case 1:  ocr_tbl[1] = pwm; break; //6
    case 2:  ocr_tbl[2] = pwm; break; //2
    case 3:  ocr_tbl[3] = pwm; break; //3
    case 4:  OCRxx1[1]  = pwm; break; //CAM PITCH
    case 5:  OCRxx1[0]  = pwm; break; //CAM ROLL
    case 6:  ocr_tbl[4] = pwm; break; //7
    case 7:  ocr_tbl[5] = pwm; break; //8

    case 9:  ocr_tbl[6] = pwm; break;// d11
    case 10: ocr_tbl[7] = pwm; break;// d12
  } 
}

uint16_t APM_RC_PIRATES::OutputCh_current(uint8_t ch)
{
	uint16_t pwm=0;
	switch(ch) {
	case 0:  pwm=OCR3A; break;  //ch1
	case 1:  pwm=OCR4A; break;  //ch2
	case 2:  pwm=OCR3B; break;  //ch3
	case 3:  pwm=OCR3C; break;  //ch4
	case 4:  pwm=OCRxx1[1]; break;  //ch5
	case 5:  pwm=OCRxx1[0]; break;  //ch6
	case 6:  pwm=OCR4B; break;  //ch7
	case 7:  pwm=OCR4C; break;  //ch8
//	case 8:  pwm=OCR5A; break;  //ch9
	case 9:  pwm=OCR1A; break;  //ch9
	case 10: pwm=OCR1B; break;  //ch10
	}
	return pwm>>1;
}

void APM_RC_PIRATES::enable_out(uint8_t ch)
{
  switch(ch) {
    case 0: TCCR3A |= (1<<COM3A1); break; // CH_1
    case 1: TCCR4A |= (1<<COM4A1); break; // CH_2
    case 2: TCCR3A |= (1<<COM3B1); break; // CH_3
    case 3: TCCR3A |= (1<<COM3C1); break; // CH_4
    	// 4,5
    case 6: TCCR4A |= (1<<COM4B1); break; // CH_7
    case 7: TCCR4A |= (1<<COM4C1); break; // CH_8
    case 9: TCCR1A |= (1<<COM1A1); break; // CH_9
    case 10: TCCR1A |= (1<<COM1B1); break; // CH_10
  }
}

void APM_RC_PIRATES::disable_out(uint8_t ch)
{
  switch(ch) {
    case 0: TCCR3A &= ~(1<<COM3A1); break; // CH_1
    case 1: TCCR4A &= ~(1<<COM4A1); break; // CH_2
    case 2: TCCR3A &= ~(1<<COM3B1); break; // CH_3
    case 3: TCCR3A &= ~(1<<COM3C1); break; // CH_4
    	// 4,5
    case 6: TCCR4A &= ~(1<<COM4B1); break; // CH_7
    case 7: TCCR4A &= ~(1<<COM4C1); break; // CH_8
    case 9: TCCR1A &= ~(1<<COM1A1); break; // CH_9
    case 10: TCCR1A &= ~(1<<COM1B1); break; // CH_10
  }
}
 
uint16_t APM_RC_PIRATES::InputCh(uint8_t ch)
{
  uint16_t result;
  uint16_t result2;
  
  // Because servo pulse variables are 16 bits and the interrupts are running values could be corrupted.
  // We dont want to stop interrupts to read radio channels so we have to do two readings to be sure that the value is correct...
	result=readRawRC(ch); 
  
  // Limit values to a valid range
  result = constrain(result,MIN_PULSEWIDTH,MAX_PULSEWIDTH);
  radio_status=1; // Radio channel read
  return(result);
}

uint8_t APM_RC_PIRATES::GetState(void)
{
	return(1);// always 1
}

uint8_t APM_RC_PIRATES::GetFailSafeState(void)
{
	failsafeCnt++;
	if(failsafeCnt > 20) failsafeCnt = 20;
	return(failsafeCnt);
}


// InstantPWM implementation
void APM_RC_PIRATES::Force_Out(void)
{
	Force_Out0_Out1();
}

// MPNG: Pirates has another channel map than AC, so we reset counters for all motors in single function

// This function forces the PWM output (reset PWM) on Out0 and Out1 (Timer3). For quadcopters use
void APM_RC_PIRATES::Force_Out0_Out1(void)
{
  if (TCNT3>5000)  // We take care that there are not a pulse in the output
    TCNT3=39990;   // This forces the PWM output to reset in 5us (10 counts of 0.5us). The counter resets at 40000
  if (TCNT4>5000)
    TCNT4=39990; 
  if (TCNT1>5000)
    TCNT1=39990; 
}
// This function forces the PWM output (reset PWM) on Out2 and Out3 (Timer4). For quadcopters use
void APM_RC_PIRATES::Force_Out2_Out3(void)
{
}
// This function forces the PWM output (reset PWM) on Out6 and Out7 (Timer1). For quadcopters use
void APM_RC_PIRATES::Force_Out6_Out7(void)
{
}

/* --------------------- OUTPUT SPEED CONTROL --------------------- */

void APM_RC_PIRATES::SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz)
{
	uint16_t icr = _map_speed(speed_hz);

	if ((chmask & ( _BV(CH_9) | _BV(CH_10))) != 0) {
		ICR1 = icr;
	}

	if ((chmask & ( _BV(CH_1) | _BV(CH_3) | _BV(CH_4))) != 0) {
		ICR3 = icr;
	}

	if ((chmask & ( _BV(CH_2) | _BV(CH_7) | _BV(CH_8))) != 0) {
		ICR4 = icr;
	}
} 

// allow HIL override of RC values
// A value of -1 means no change
// A value of 0 means no override, use the real RC values
bool APM_RC_PIRATES::setHIL(int16_t v[NUM_CHANNELS])
{
/*
	uint8_t sum = 0;
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		if (v[i] != -1) {
			_HIL_override[i] = v[i];
		}
		if (_HIL_override[i] != 0) {
			sum++;
		}
	}
	radio_status = 1;
	if (sum == 0) {
		return 0;
	} else {
		return 1;
	}
*/
	radio_status = 1;
	return 1;
}

void APM_RC_PIRATES::clearOverride(void)
{
	for (uint8_t i=0; i<NUM_CHANNELS; i++) {
		_HIL_override[i] = 0;
	}
}


#endif // defined(ATMega1280)
