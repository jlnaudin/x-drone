/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_BARO_MS5611_I2C_H__
#define __AP_BARO_MS5611_I2C_H__

#include "AP_Baro.h"

class AP_Baro_MS5611_I2C : public AP_Baro
{
  public:
  AP_Baro_MS5611_I2C() {}  // Constructor

  /* AP_Baro public interface: */
  bool init(AP_PeriodicProcess *scheduler);
  uint8_t read();
  int32_t get_pressure();     // in mbar*100 units
  int16_t get_temperature();  // in celsius degrees * 100 units
  float get_altitude();        // in meter units

  int32_t get_raw_pressure();
  int32_t get_raw_temp();

  private:
  /* Asynchronous handler functions: */
  void init_hardware(); 
  static void _update(uint32_t );
  /* Asynchronous state: */
  static bool _updated;
  static uint32_t _s_D1, _s_D2;
  static uint8_t _state;
  static uint32_t _timer;
  /* Gates access to asynchronous state: */
  static bool _sync_access;

  /* Serial wrapper functions: */
  static uint32_t _i2c_read_adc();
  static uint16_t _i2c_readPROM(uint8_t addr);

  void     _calculate();

  int16_t Temp;
  int32_t Press;
  int32_t Alt;

  int32_t _raw_press;
  int32_t _raw_temp;
  // Internal calibration registers
  uint16_t C1,C2,C3,C4,C5,C6;
  uint32_t D1,D2;
};

#endif //  __AP_BARO_MS5611_I2C_H__
