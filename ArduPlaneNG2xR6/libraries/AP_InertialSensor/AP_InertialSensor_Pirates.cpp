#include <FastSerial.h>

#include "AP_InertialSensor_Pirates.h"

#include <I2C.h>
#include <FastSerial.h>

// #define BMA_020 // do you have it?

// *********************
// I2C general functions
// *********************
#define I2C_PULLUPS_DISABLE        PORTD &= ~(1<<0); PORTD &= ~(1<<1); 
#define ITG3200_ADDRESS  0x68 // 0xD0

#define PIRATES_ALLINONE 0
#define PIRATES_FFIMU 1
#define PIRATES_FREEIMU 2
#define PIRATES_BLACKVORTEX 3 

// ITG-3200 14.375 LSB/degree/s
const float AP_InertialSensor_Pirates::_gyro_scale = 0.0012141421; // ToRad(1/14.375)
const float AP_InertialSensor_Pirates::_accel_scale = 9.81 / 2730.0;
uint8_t AP_InertialSensor_Pirates::_board_Type = PIRATES_ALLINONE;
int AP_InertialSensor_Pirates::accel_addr = 0;
const uint8_t AP_InertialSensor_Pirates::_temp_data_index = 3;
uint8_t AP_InertialSensor_Pirates::_gyro_data_index[3];
int8_t AP_InertialSensor_Pirates::_gyro_data_sign[3];

uint8_t AP_InertialSensor_Pirates::_accel_data_index[3];
int8_t AP_InertialSensor_Pirates::_accel_data_sign[3];

bool AP_InertialSensor_Pirates::healthy;

AP_InertialSensor_Pirates::AP_InertialSensor_Pirates(uint8_t brd)
{
  _gyro.x = 0;
  _gyro.y = 0;
  _gyro.z = 0;
  _accel.x = 0;
  _accel.y = 0;
  _accel.z = 0;
  _temp = 0;
  _initialised = 0;
  _board_Type = brd;
}

uint16_t AP_InertialSensor_Pirates::init( AP_PeriodicProcess * scheduler )
{
	if (_initialised) return _board_Type;
		
	if (_board_Type == PIRATES_ALLINONE || _board_Type == PIRATES_FREEIMU || _board_Type == PIRATES_BLACKVORTEX) {
		_gyro_data_index[0]  =  1;
		_gyro_data_index[1]  =  2;
		_gyro_data_index[2]  =  0;
		_gyro_data_sign[0]   = 1;
		_gyro_data_sign[1]   = 1;
		_gyro_data_sign[2]   = -1;
	
		_accel_data_index[0] = 4;
		_accel_data_index[1] = 5;
		_accel_data_index[2] = 6;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	} else if (_board_Type == PIRATES_FFIMU) {
		_gyro_data_index[0]  =  2;
		_gyro_data_index[1]  =  1;
		_gyro_data_index[2]  =  0;
		_gyro_data_sign[0]   = 1;
		_gyro_data_sign[1]   = -1;
		_gyro_data_sign[2]   = -1;
	
		_accel_data_index[0] = 5;
		_accel_data_index[1] = 4;
		_accel_data_index[2] = 6;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = -1;
		_accel_data_sign[2]  = -1;
	}

	if (_board_Type == PIRATES_ALLINONE || _board_Type == PIRATES_BLACKVORTEX) {
		accel_addr = 0x41;
	} else {
		accel_addr = 0x40;
	}

	delay(50);
	hardware_init();
	scheduler->register_process( &AP_InertialSensor_Pirates::read );
	_initialised = 1;
		
	return _board_Type;
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;

/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_Pirates::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;

	// wait for at least 1 sample
	while (_count == 0) ; // nop

	// disable interrupts for mininum time
	cli();
	for (int i=0; i<7; i++) {
		sum[i] = _sum[i];
		_sum[i] = 0;
	}
	count = _count;
	_count = 0;
	sei();

	count_scale = 1.0 / count;
	_gyro.x = _gyro_scale * _gyro_data_sign[0] * sum[_gyro_data_index[0]] * count_scale;
	_gyro.y = _gyro_scale * _gyro_data_sign[1] * sum[_gyro_data_index[1]] * count_scale;
	_gyro.z = _gyro_scale * _gyro_data_sign[2] * sum[_gyro_data_index[2]] * count_scale;

	_accel.x = _accel_scale * _accel_data_sign[0] * sum[_accel_data_index[0]] * count_scale;
	_accel.y = _accel_scale * _accel_data_sign[1] * sum[_accel_data_index[1]] * count_scale;
	_accel.z = _accel_scale * _accel_data_sign[2] * sum[_accel_data_index[2]] * count_scale;

	_temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

	return true;
}

bool AP_InertialSensor_Pirates::new_data_available( void )
{
    return _count != 0;
}

float AP_InertialSensor_Pirates::gx() { return _gyro.x; }
float AP_InertialSensor_Pirates::gy() { return _gyro.y; }
float AP_InertialSensor_Pirates::gz() { return _gyro.z; }

void AP_InertialSensor_Pirates::get_gyros( float * g )
{
  g[0] = _gyro.x;
  g[1] = _gyro.y;
  g[2] = _gyro.z;
}

float AP_InertialSensor_Pirates::ax() { return _accel.x; }
float AP_InertialSensor_Pirates::ay() { return _accel.y; }
float AP_InertialSensor_Pirates::az() { return _accel.z; }

void AP_InertialSensor_Pirates::get_accels( float * a )
{
  a[0] = _accel.x;
  a[1] = _accel.y;
  a[2] = _accel.z;
}

void AP_InertialSensor_Pirates::get_sensors( float * sensors )
{
  sensors[0] = _gyro.x;
  sensors[1] = _gyro.y;
  sensors[2] = _gyro.z;
  sensors[3] = _accel.x;
  sensors[4] = _accel.y;
  sensors[5] = _accel.z;
}

float AP_InertialSensor_Pirates::temperature() { return _temp; }

uint32_t AP_InertialSensor_Pirates::sample_time()
{
  uint32_t us = micros();
  /* XXX rollover creates a major bug */
  uint32_t delta = us - _last_sample_micros;
  reset_sample_time();
  return delta;
}

void AP_InertialSensor_Pirates::reset_sample_time()
{
    _last_sample_micros = micros();
}

/*================ HARDWARE FUNCTIONS ==================== */

void AP_InertialSensor_Pirates::read(uint32_t)
{
	static uint8_t i;
	uint8_t rawADC_ITG3200[8];
	uint8_t rawADC_BMA180[6];

	if (I2c.read(ITG3200_ADDRESS, 0X1B, 8, rawADC_ITG3200) != 0) {
		healthy = false;
		return;
	}
  
	_sum[3] += ((rawADC_ITG3200[0]<<8) | rawADC_ITG3200[1]); // temperature
	_sum[0] += ((rawADC_ITG3200[6]<<8) | rawADC_ITG3200[7]); //g yaw
	_sum[1] += ((rawADC_ITG3200[4]<<8) | rawADC_ITG3200[5]); //g roll
	_sum[2] += ((rawADC_ITG3200[2]<<8) | rawADC_ITG3200[3]); //g pitch

	if (I2c.read(accel_addr, 0x02, 6, rawADC_BMA180) != 0) {
		healthy = false;
		return;
	} 
	  
	_sum[4] += ((rawADC_BMA180[3]<<8) | (rawADC_BMA180[2])) >> 2; //a pitch
	_sum[5] += ((rawADC_BMA180[1]<<8) | (rawADC_BMA180[0])) >> 2; //a roll
	_sum[6] += ((rawADC_BMA180[5]<<8) | (rawADC_BMA180[4])) >> 2; //a yaw

  _count++;
  if (_count == 0) {
	  // rollover - v unlikely
	  memset((void*)_sum, 0, sizeof(_sum));
  }
}

void AP_InertialSensor_Pirates::hardware_init()
{
	int i;
	
	I2c.begin();
	I2c.setSpeed(true);// 400Hz
	I2c.pullup(true);
	
	// GYRO
	//=== ITG3200 INIT
	delay(10);  
	if (I2c.write(ITG3200_ADDRESS, 0x3E, 0x80) != 0) {	// Power Management register, reset device
		healthy = false;
		return;
	} 	
	delay(5);
	Serial.print("1");
	if (I2c.write(ITG3200_ADDRESS, 0x15, 0x3+1) != 0) {	// Sample Rate Divider, 1000Hz/(3+1) = 250Hz . 
		healthy = false;
		return;
	} 	
	delay(5);
	Serial.print("2");
	if (I2c.write(ITG3200_ADDRESS, 0x16, 0x18+4) != 0) {	// Internal Sample Rate 1kHz, Low pass filter: 1..6: 1=200hz, 2-100,3-50,4-20,5-10,6-5
		healthy = false;
		return;
	} 	
	delay(5);
	Serial.print("3");
	if (I2c.write(ITG3200_ADDRESS, 0x3E, 0x03) != 0) {	// PLL with Z Gyro reference
		healthy = false;
		return;
	} 	
	delay(100);
	Serial.print("4");

	delay(10);

	// ACCEL
	//===BMA180 INIT
	if (I2c.write(accel_addr, 0x0D, 1<<4) != 0) {	// ctrl_reg0, Set bit 4 to 1 to enable writing
		healthy = false;
		return;
	} 	
	if (I2c.write(accel_addr, 0x35, 3<<1) != 0) {	// range set to 3.  2730 1G raw data.  With /10 divisor on acc_ADC, more in line with other sensors and works with the GUI
		healthy = false;
		return;
	} 	
	if (I2c.write(accel_addr, 0x20, 0<<4) != 0) {	// bw_tcs reg: bits 4-7 to set bw, bw to 10Hz (low pass filter)
		healthy = false;
		return;
	} 	
	
	delay(10);  

	healthy = true;
}

float AP_InertialSensor_Pirates::_temp_to_celsius ( uint16_t regval )
{
	return (35.0 + ((float) (regval + 13200)) / 280);
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_Pirates::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}
