// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// This file is just a placeholder for your configuration file.  If
// you wish to change any of the setup parameters from their default
// values, place the appropriate #define statements here.


// the following 2 defines control which APM board you have. The
// 'BETA' board is only if you are developer who received a
// pre-release APM2 board with the older barometer on it.

// # define CONFIG_APM_HARDWARE APM_HARDWARE_APM2
// # define APM2_BETA_HARDWARE

#define CONFIG_APM_HARDWARE  APM_HARDWARE_PIRATES

// Select your sensor board

#define PIRATES_SENSOR_BOARD PIRATES_CRIUS_AIO_PRO_V1
/*
	PIRATES_ALLINONE
	PIRATES_FFIMU
	PIRATES_FREEIMU
	PIRATES_BLACKVORTEX
	PIRATES_FREEIMU_4 					// New FreeIMU 0.4.1 with MPU6000, MS5611 and 5883L
	PIRATES_DROTEK_10DOF_MPU 		// MPU6000, MS5611 and 5883L
	PIRATES_CRIUS_AIO_PRO_V1    // Crius AllInOne Pro v1
*/

// RC configuration
// Uncomment if you uses PPM Sum signal from receiver
#define SERIAL_PPM DISABLED

#define CONFIG_ADC DISABLED
#define CONFIG_PITOT_SOURCE PITOT_SOURCE_ANALOG_PIN
#define MAGNETOMETER	    ENABLED
#define LOGGING_ENABLED	    DISABLED
#define MOUNT               DISABLED
#define PITOT_ENABLED       ENABLED
#define TRACE		    DISABLED

#define TX_CHANNEL_SET	TX_mwi
/*
	TX_set1							//Graupner/Spektrum												PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_standard					//standard  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL
	TX_set2							//some Hitec/Sanwa/others									PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
	TX_mwi							//MultiWii layout													ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
*/

// Select your baro sensor
#define CONFIG_BARO AP_BARO_MS5611_I2C  // For All In One Pro 1.0
/*
	AP_BARO_BMP085_PIRATES
	AP_BARO_MS5611_I2C  // For All In One Pro 1.0
*/

// Warning: COPTER_LEDS is not compatible with LED_SEQUENCER, so enable only one option
#define COPTER_LEDS DISABLED     // New feature coming from ArduCopter
#define LED_SEQUENCER DISABLED   // Old Oleg's LED Sequencer, see leds.pde for more info

#define MAX_SONAR_RANGE 400

#define OSD_PROTOCOL OSD_PROTOCOL_NONE
/*
	OSD_PROTOCOL_NONE
	OSD_PROTOCOL_SYBERIAN
	OSD_PROTOCOL_REMZIBI  // Read more at: http://www.rcgroups.com/forums/showthread.php?t=921467
*/

// For BlackVortex, just set PIRATES_SENSOR_BOARD as PIRATES_BLACKVORTEX, GPS will be selected automatically
#define GPS_PROTOCOL GPS_PROTOCOL_UBLOX   // for CRIUS CN-06 Ublox GPS   
/*
	GPS_PROTOCOL_NONE 	without GPS
	GPS_PROTOCOL_NMEA     
	GPS_PROTOCOL_SIRF    // for the EM406  GPS
	GPS_PROTOCOL_UBLOX   // for CRIUS CN-06 Ublox GPS
	GPS_PROTOCOL_IMU
	GPS_PROTOCOL_MTK
	GPS_PROTOCOL_HIL
	GPS_PROTOCOL_MTK16
	GPS_PROTOCOL_AUTO	auto select GPS  // OK for the MT3329 GPS
	GPS_PROTOCOL_UBLOX_I2C
	GPS_PROTOCOL_BLACKVORTEX
*/

#define SERIAL2_BAUD			38400	// setup for the MT3329 GPS or for CRIUS CN-06 Ublox GPS (normal speed)

// The following are the recommended settings for Xplane
// simulation. Remove the leading "/* and trailing "*/" to enable:

//#define HIL_MODE            HIL_MODE_DISABLED

/*
// HIL_MODE SELECTION
//
// Mavlink supports
// 1. HIL_MODE_ATTITUDE : simulated position, airspeed, and attitude
// 2. HIL_MODE_SENSORS: full sensor simulation
//#define HIL_MODE            HIL_MODE_ATTITUDE

*/

#include "APM_Config_Easyglider.h"

//#include "APM_Config_EasyStar2.h"
//#include "APM_Config_Cularis.h"
//#include "APM_Config_ASW27.h"
//#include "APM_Config_MAJA.h"
