#ifndef APM_RC_PIRATES_h
#define APM_RC_PIRATES_h

#define MIN_PULSEWIDTH 900
#define MAX_PULSEWIDTH 2100

#include "APM_RC.h"
#include "../Arduino_Mega_ISR_Registry/Arduino_Mega_ISR_Registry.h"

class APM_RC_PIRATES : public APM_RC_Class
{
  public:
	APM_RC_PIRATES(int _use_ppm, int _bv_mode, uint8_t *_pin_map);
	void Init( Arduino_Mega_ISR_Registry * isr_reg );
	void OutputCh(uint8_t ch, uint16_t pwm);
	uint16_t OutputCh_current(uint8_t ch);
	uint16_t InputCh(uint8_t ch);
	uint8_t GetState();
	uint8_t GetFailSafeState();
	bool setHIL(int16_t v[NUM_CHANNELS]);
	void clearOverride(void);
	void Force_Out(void);
	void SetFastOutputChannels(uint32_t chmask, uint16_t speed_hz = 400); 
	
	void enable_out(uint8_t);
	void disable_out(uint8_t); 
	
	void Force_Out0_Out1(void);
	void Force_Out2_Out3(void);
	void Force_Out6_Out7(void);

  private:
	int16_t _HIL_override[NUM_CHANNELS];
};

#endif
