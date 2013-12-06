/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include "AP_InertialSensor_MPU6000_I2C.h"

#include <I2C.h>

#define PIRATES_FREEIMU_4 4
#define PIRATES_DROTEK_10DOF_MPU 5 
#define PIRATES_CRIUS_AIO_PRO_V1 6

// MPU 6000 registers
#define MPU6000_ADDR 0x68 //
#define MPUREG_WHOAMI 0x75 //
#define MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_INT_STATUS 0x3A
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 //
#define MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define MPUREG_PWR_MGMT_1 0x6B //
#define MPUREG_PWR_MGMT_2 0x6C //
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74
#define MPUREG_PRODUCT_ID 0x0C	// Product ID Register


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      0x10
#define BIT_RAW_RDY_EN        0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA   0x01

											// Product ID Description for MPU6000
											// high 4 bits 	low 4 bits
											// Product Name	Product Revision
#define MPU6000_REV_A4 			0x04 	// 0001			0100
#define MPU6000ES_REV_C4 			0x14 	// 0001			0100
#define MPU6000ES_REV_C5 			0x15 	// 0001			0101
#define MPU6000ES_REV_D6 			0x16	// 0001			0110
#define MPU6000ES_REV_D7 			0x17	// 0001			0111
#define MPU6000ES_REV_D8 			0x18	// 0001			1000	
#define MPU6000_REV_C4 				0x54	// 0101			0100 
#define MPU6000_REV_C5 				0x55	// 0101			0101
#define MPU6000_REV_D6 				0x56	// 0101			0110	
#define MPU6000_REV_D7 				0x57	// 0101			0111
#define MPU6000_REV_D8 				0x58	// 0101			1000
#define MPU6000_REV_D9 				0x59	// 0101			1001

/* 
   RS-MPU-6000A-00.pdf, page 33, section 4.25 lists LSB sensitivity of
   gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==3)
 */
const float AP_InertialSensor_MPU6000_I2C::_gyro_scale = (0.0174532 / 16.4);

/* 
   RS-MPU-6000A-00.pdf, page 31, section 4.23 lists LSB sensitivity of
   accel as 4096 LSB/mg at scale factor of +/- 8g (AFS_SEL==2)

   See note below about accel scaling of engineering sample MPU6k
   variants however
 */
const float AP_InertialSensor_MPU6000_I2C::_accel_scale = 9.81 / 4096.0;

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 */
const uint8_t AP_InertialSensor_MPU6000_I2C::_temp_data_index = 3;

static uint8_t _product_id;

uint8_t AP_InertialSensor_MPU6000_I2C::_board_Type = PIRATES_FREEIMU_4;
int AP_InertialSensor_MPU6000_I2C::mpu_addr = 0;

uint8_t AP_InertialSensor_MPU6000_I2C::_gyro_data_index[3];
int8_t AP_InertialSensor_MPU6000_I2C::_gyro_data_sign[3];

uint8_t AP_InertialSensor_MPU6000_I2C::_accel_data_index[3];
int8_t AP_InertialSensor_MPU6000_I2C::_accel_data_sign[3];

AP_InertialSensor_MPU6000_I2C::AP_InertialSensor_MPU6000_I2C(uint8_t addr, uint8_t brd)
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
	mpu_addr = addr; // Look at config.h for actual value

	if (_board_Type == PIRATES_FREEIMU_4) {
		_gyro_data_index[0]  = 4;		// X
		_gyro_data_index[1]  = 5;		// Y
		_gyro_data_index[2]  = 6;		// Z
		_gyro_data_sign[0]   = -1;	// -X
		_gyro_data_sign[1]   = 1;		// Y
		_gyro_data_sign[2]   = -1;	// -Z
	
		_accel_data_index[0] = 0;
		_accel_data_index[1] = 1;
		_accel_data_index[2] = 2;
		_accel_data_sign[0]  = -1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	} else if (_board_Type == PIRATES_DROTEK_10DOF_MPU || _board_Type == PIRATES_CRIUS_AIO_PRO_V1) {
		_gyro_data_index[0]  = 5;	// Y
		_gyro_data_index[1]  = 4; // X
		_gyro_data_index[2]  = 6;	// Z
		_gyro_data_sign[0]   = 1;	// Y
		_gyro_data_sign[1]   = 1;	// X
		_gyro_data_sign[2]   = -1;// -Z
	
		_accel_data_index[0] = 1;
		_accel_data_index[1] = 0;
		_accel_data_index[2] = 2;
		_accel_data_sign[0]  = 1;
		_accel_data_sign[1]  = 1;
		_accel_data_sign[2]  = -1;
	}
}

uint16_t AP_InertialSensor_MPU6000_I2C::init( AP_PeriodicProcess * scheduler )
{
	if (_initialised) return _product_id;
	_initialised = 1;
	delay(50);
	hardware_init();
	scheduler->register_process( &AP_InertialSensor_MPU6000_I2C::read );
	return _product_id;
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;


/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_MPU6000_I2C::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;

	// wait for at least 1 sample
	while (_count == 0) /* nop */;

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

bool AP_InertialSensor_MPU6000_I2C::new_data_available( void )
{
    return _count != 0;
}

float AP_InertialSensor_MPU6000_I2C::gx() { return _gyro.x; }
float AP_InertialSensor_MPU6000_I2C::gy() { return _gyro.y; }
float AP_InertialSensor_MPU6000_I2C::gz() { return _gyro.z; }

void AP_InertialSensor_MPU6000_I2C::get_gyros( float * g )
{
  g[0] = _gyro.x;
  g[1] = _gyro.y;
  g[2] = _gyro.z;
}

float AP_InertialSensor_MPU6000_I2C::ax() { return _accel.x; }
float AP_InertialSensor_MPU6000_I2C::ay() { return _accel.y; }
float AP_InertialSensor_MPU6000_I2C::az() { return _accel.z; }

void AP_InertialSensor_MPU6000_I2C::get_accels( float * a )
{
  a[0] = _accel.x;
  a[1] = _accel.y;
  a[2] = _accel.z;
}

void AP_InertialSensor_MPU6000_I2C::get_sensors( float * sensors )
{
  sensors[0] = _gyro.x;
  sensors[1] = _gyro.y;
  sensors[2] = _gyro.z;
  sensors[3] = _accel.x;
  sensors[4] = _accel.y;
  sensors[5] = _accel.z;
}

float AP_InertialSensor_MPU6000_I2C::temperature() { return _temp; }

uint32_t AP_InertialSensor_MPU6000_I2C::sample_time()
{
  uint32_t us = micros();
  uint32_t delta = us - _last_sample_micros;
  reset_sample_time();
  return delta;
}

void AP_InertialSensor_MPU6000_I2C::reset_sample_time()
{
    _last_sample_micros = micros();
}

/*================ HARDWARE FUNCTIONS ==================== */

/*
  this is called from a timer interrupt to read data from the MPU6000
  and add it to _sum[]
 */
static volatile long _ins_timer = 0;

void AP_InertialSensor_MPU6000_I2C::read(uint32_t tnow)
{
	if (tnow - _ins_timer < 5000) {
		return; // wait for more than 5ms
	}

	_ins_timer = tnow;
    
	// now read the data
	uint8_t rawMPU[14];
	
	if (I2c.read(mpu_addr, MPUREG_ACCEL_XOUT_H, 14, rawMPU) != 0) {
//		healthy = false;
		return;
	}
	
	_sum[0] += (((int16_t)rawMPU[0])<<8) | rawMPU[1]; // Accel X
	_sum[1] += (((int16_t)rawMPU[2])<<8) | rawMPU[3]; // Accel Y
	_sum[2] += (((int16_t)rawMPU[4])<<8) | rawMPU[5]; // Accel Z
	_sum[3] += (((int16_t)rawMPU[6])<<8) | rawMPU[7]; // Temperature
	_sum[4] += (((int16_t)rawMPU[8])<<8) | rawMPU[9]; // Gyro X
	_sum[5] += (((int16_t)rawMPU[10])<<8) | rawMPU[11]; // Gyro Y
	_sum[6] += (((int16_t)rawMPU[12])<<8) | rawMPU[13]; // Gyro Z

	_count++;
	if (_count == 0) {
        // rollover - v unlikely
		memset((void*)_sum, 0, sizeof(_sum));
	}
}

void AP_InertialSensor_MPU6000_I2C::hardware_init()
{
    // Chip reset
		if (I2c.write(mpu_addr, MPUREG_PWR_MGMT_1, BIT_H_RESET) != 0) {
			return;
		} 	
    delay(100);
    // Wake up device and select GyroZ clock (better performance)
		if (I2c.write(mpu_addr, MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ) != 0) {
			return;
		} 	
    delay(1);
		// Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
		if (I2c.write(mpu_addr, MPUREG_SMPLRT_DIV, 0x04) != 0) {
			return;
		} 	
    delay(1);
    // FS & DLPF   FS=2000º/s, DLPF = 98Hz (low pass filter)
		if (I2c.write(mpu_addr, MPUREG_CONFIG, BITS_DLPF_CFG_10HZ) != 0) {
			return;
		} 	
    delay(1);
		if (I2c.write(mpu_addr, MPUREG_GYRO_CONFIG, BITS_FS_2000DPS) != 0) { // Gyro scale 2000º/s
			return;
		} 	
    delay(1);
//		if (I2c.write(MPU6000_ADDR, MPUREG_ACCEL_CONFIG, 0x08) != 0) { // Accel scale 4g (4096LSB/g)
		// Set Accel sensivity AFS_SEL=2, 8g (4096LSB/g)

		if (I2c.read(mpu_addr, MPUREG_PRODUCT_ID, 1, &_product_id) != 0) { 
			return;
		}
		
		if ( (_product_id == MPU6000_REV_A4) || (_product_id == MPU6000ES_REV_C4) || (_product_id == MPU6000ES_REV_C5) ||
			(_product_id == MPU6000_REV_C4)   || (_product_id == MPU6000_REV_C5)){
			// Accel scale 8g (4096 LSB/g)
			// Rev C has different scaling than rev D
			if (I2c.write(mpu_addr, MPUREG_ACCEL_CONFIG, 1<<3) != 0) { // Accel scale 4g (4096LSB/g)
				return;
			} 	
		} else {
			// Accel scale 8g (4096 LSB/g)
			if (I2c.write(mpu_addr, MPUREG_ACCEL_CONFIG, 2<<3) != 0) { // Accel scale 4g (4096LSB/g)
				return;
			} 	
		}
			
    delay(1);

		// Enable I2C bypass mode, to work with Magnetometer 5883L
		// Disable I2C Master mode
		uint8_t user_ctrl;
		if (I2c.read(mpu_addr, MPUREG_USER_CTRL, 1, &user_ctrl) != 0) { 
			return;
		}
		user_ctrl = user_ctrl & ~(1 << 5); // reset I2C_MST_EN bit
		if (I2c.write(mpu_addr, MPUREG_USER_CTRL, &user_ctrl, 1) != 0) {
			return;
		} 	
    delay(1);
    
		// Enable I2C Bypass mode
		if (I2c.read(mpu_addr, MPUREG_INT_PIN_CFG, 1, &user_ctrl) != 0) { 
			return;
		}
		user_ctrl = user_ctrl | (1 << 1); // set I2C_BYPASS_EN bit
		if (I2c.write(mpu_addr, MPUREG_INT_PIN_CFG, &user_ctrl, 1) != 0) {
			return;
		} 	
}

float AP_InertialSensor_MPU6000_I2C::_temp_to_celsius ( uint16_t regval )
{
    /* TODO */
    return 20.0;
}

// return the MPU6k gyro drift rate in radian/s/s
// note that this is much better than the oilpan gyros
float AP_InertialSensor_MPU6000_I2C::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute
    return ToRad(0.5/60);
}
