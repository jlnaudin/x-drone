// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef _DEFINES_H
#define _DEFINES_H

#define ENABLED			1
#define DISABLED		0

// Internal defines, don't edit and expect things to work
// -------------------------------------------------------

#define TRUE 1
#define FALSE 0
#define ToRad(x) (x*0.01745329252)	// *pi/180
#define ToDeg(x) (x*57.2957795131)	// *180/pi

#define DEBUG 0
#define LOITER_RANGE 60 // for calculating power outside of loiter radius
#define SERVO_MAX 4500	// This value represents 45 degrees and is just an arbitrary representation of servo max travel.

// failsafe
// ----------------------
#define FAILSAFE_NONE	0
#define FAILSAFE_SHORT	1
#define FAILSAFE_LONG	2
#define FAILSAFE_GCS	3
#define	FAILSAFE_SHORT_TIME 1500	// Miliiseconds
#define	FAILSAFE_LONG_TIME  20000	// Miliiseconds


// active altitude sensor
// ----------------------
#define SONAR 0
#define BARO 1

#define PITOT_SOURCE_ADC 1
#define PITOT_SOURCE_ANALOG_PIN 2

#define T6 1000000
#define T7 10000000

// GPS type codes - use the names, not the numbers
#define GPS_PROTOCOL_NONE	-1
#define GPS_PROTOCOL_NMEA	0
#define GPS_PROTOCOL_SIRF	1
#define GPS_PROTOCOL_UBLOX	2
#define GPS_PROTOCOL_IMU	3
#define GPS_PROTOCOL_MTK	4
#define GPS_PROTOCOL_HIL	5
#define GPS_PROTOCOL_MTK16	6
#define GPS_PROTOCOL_AUTO	7
#define GPS_PROTOCOL_UBLOX_I2C 8
#define GPS_PROTOCOL_BLACKVORTEX 9

// SONAR types:
#define MAX_SONAR_UNKNOWN	0
#define MAX_SONAR_XL		1
#define SONAR_ME007		2

// OSD Types
#define	OSD_PROTOCOL_NONE 0
#define	OSD_PROTOCOL_SYBERIAN 1
#define	OSD_PROTOCOL_REMZIBI 2

#define CH_ROLL CH_1
#define CH_PITCH CH_2
#define CH_THROTTLE CH_3
#define CH_RUDDER CH_4
#define CH_YAW CH_4
#define CH_AIL1 CH_5
#define CH_AIL2 CH_6

// CH 7 control
#define CH7_DO_NOTHING 0
#define CH7_SAVE_WP 1
#define CH7_RTL 2

#define RC_CHANNEL_ANGLE 0
#define RC_CHANNEL_RANGE 1
#define RC_CHANNEL_ANGLE_RAW 2

// HIL enumerations
#define HIL_MODE_DISABLED			0
#define HIL_MODE_ATTITUDE			1
#define HIL_MODE_SENSORS			2

// Auto Pilot modes
// ----------------
#define MANUAL 0
#define CIRCLE 1			 // When flying sans GPS, and we loose the radio, just circle
#define STABILIZE 2

#define FLY_BY_WIRE_A 5		// Fly By Wire A has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical = manual throttle
#define FLY_BY_WIRE_B 6		// Fly By Wire B has left stick horizontal => desired roll angle, left stick vertical => desired pitch angle, right stick vertical => desired airspeed
#define FLY_BY_WIRE_C 7		// Fly By Wire C has left stick horizontal => desired roll angle, left stick vertical => desired climb rate, right stick vertical => desired airspeed
							// Fly By Wire B and Fly By Wire C require airspeed sensor
#define AUTO 10
#define RTL 11
#define LOITER 12
//#define TAKEOFF 13			// This is not used by APM.  It appears here for consistency with ACM
//#define LAND 14			// This is not used by APM.  It appears here for consistency with ACM
#define GUIDED 15
#define INITIALISING 16     // in startup routines
#define LAND         17   // Landing mode

// Commands - Note that APM now uses a subset of the MAVLink protocol commands.  See enum MAV_CMD in the GCS_Mavlink library
#define CMD_BLANK 0 // there is no command stored in the mem location requested
#define NO_COMMAND 0
#define WAIT_COMMAND 255

// Command/Waypoint/Location Options Bitmask
//--------------------
#define MASK_OPTIONS_RELATIVE_ALT	(1<<0)		// 1 = Relative altitude

//repeating events
#define NO_REPEAT 0
#define CH_5_TOGGLE 1
#define CH_6_TOGGLE 2
#define CH_7_TOGGLE 3
#define CH_8_TOGGLE 4
#define RELAY_TOGGLE 5
#define STOP_REPEAT 10

//  GCS Message ID's
/// NOTE: to ensure we never block on sending MAVLink messages
/// please keep each MSG_ to a single MAVLink message. If need be
/// create new MSG_ IDs for additional messages on the same
/// stream
enum ap_message {
    MSG_HEARTBEAT,
    MSG_ATTITUDE,
    MSG_LOCATION,
    MSG_EXTENDED_STATUS1,
    MSG_EXTENDED_STATUS2,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_CURRENT_WAYPOINT,
    MSG_VFR_HUD,
    MSG_RADIO_OUT,
    MSG_RADIO_IN,
    MSG_RAW_IMU1,
    MSG_RAW_IMU2,
    MSG_RAW_IMU3,
    MSG_GPS_STATUS,
    MSG_GPS_RAW,
    MSG_SERVO_OUT,
    MSG_NEXT_WAYPOINT,
    MSG_NEXT_PARAM,
    MSG_STATUSTEXT,
    MSG_FENCE_STATUS,
    MSG_AHRS,
    MSG_SIMSTATE,
    MSG_HWSTATUS,
    MSG_RETRY_DEFERRED // this must be last
};

enum gcs_severity {
    SEVERITY_LOW=1,
    SEVERITY_MEDIUM,
    SEVERITY_HIGH,
    SEVERITY_CRITICAL
};

//  Logging parameters
#define LOG_INDEX_MSG			0xF0
#define LOG_ATTITUDE_MSG		0x01
#define LOG_GPS_MSG				0x02
#define LOG_MODE_MSG			0X03
#define LOG_CONTROL_TUNING_MSG	0X04
#define LOG_NAV_TUNING_MSG		0X05
#define LOG_PERFORMANCE_MSG		0X06
#define LOG_RAW_MSG				0x07
#define LOG_CMD_MSG				0x08
#define LOG_CURRENT_MSG 		0x09
#define LOG_STARTUP_MSG 		0x0A
#define TYPE_AIRSTART_MSG		0x00
#define TYPE_GROUNDSTART_MSG	0x01
#define MAX_NUM_LOGS			100

#define MASK_LOG_ATTITUDE_FAST 	(1<<0)
#define MASK_LOG_ATTITUDE_MED 	(1<<1)
#define MASK_LOG_GPS 			(1<<2)
#define MASK_LOG_PM 			(1<<3)
#define MASK_LOG_CTUN 			(1<<4)
#define MASK_LOG_NTUN			(1<<5)
#define MASK_LOG_MODE			(1<<6)
#define MASK_LOG_RAW			(1<<7)
#define MASK_LOG_CMD			(1<<8)
#define MASK_LOG_CUR			(1<<9)

// Waypoint Modes
// ----------------
#define ABS_WP 0
#define REL_WP 1

// Command Queues
// ---------------
#define COMMAND_MUST 0
#define COMMAND_MAY 1
#define COMMAND_NOW 2

// Events
// ------
#define EVENT_WILL_REACH_WAYPOINT 1
#define EVENT_SET_NEW_COMMAND_INDEX 2
#define EVENT_LOADED_WAYPOINT 3
#define EVENT_LOOP 4

// Climb rate calculations
#define	ALTITUDE_HISTORY_LENGTH 8	//Number of (time,altitude) points to regress a climb rate from


#define BATTERY_VOLTAGE(x) (x*(g.input_voltage/1024.0))*g.volt_div_ratio
#define CURRENT_AMPS(x) ((x*(g.input_voltage/1024.0))-CURR_AMPS_OFFSET)*g.curr_amp_per_volt

#define RELAY_PIN 47


// sonar
#define MAX_SONAR_XL 0
#define MAX_SONAR_LV 1
#define SonarToCm(x) (x*1.26)   // Sonar raw value to centimeters
#define AN4			4
#define AN5			5

#define SPEEDFILT 400			// centimeters/second; the speed below which a groundstart will be triggered


// EEPROM addresses
#define EEPROM_MAX_ADDR		4096
// parameters get the first 1280 bytes of EEPROM, remainder is for waypoints
#define WP_START_BYTE 0x500 // where in memory home WP is stored + all other WP
#define WP_SIZE 15

// fence points are stored at the end of the EEPROM
#define MAX_FENCEPOINTS 20
#define FENCE_WP_SIZE sizeof(Vector2l)
#define FENCE_START_BYTE (EEPROM_MAX_ADDR-(MAX_FENCEPOINTS*FENCE_WP_SIZE))

#define MAX_WAYPOINTS  ((FENCE_START_BYTE - WP_START_BYTE) / WP_SIZE) - 1 // - 1 to be safe

#define ONBOARD_PARAM_NAME_LENGTH 15

// convert a boolean (0 or 1) to a sign for multiplying (0 maps to 1, 1 maps to -1)
#define BOOL_TO_SIGN(bvalue) ((bvalue)?-1:1)

// mark a function as not to be inlined
#define NOINLINE __attribute__((noinline))

#define CONFIG_IMU_OILPAN 1
#define CONFIG_IMU_MPU6000 2

#define APM_HARDWARE_APM1  1
#define APM_HARDWARE_APM2 2
#define APM_HARDWARE_PIRATES 3

// Pirates sensors boards
#define PIRATES_ALLINONE 0
#define PIRATES_FFIMU 1
#define PIRATES_FREEIMU 2
#define PIRATES_BLACKVORTEX 3
#define PIRATES_FREEIMU_4 4
#define PIRATES_DROTEK_10DOF_MPU 5
#define PIRATES_CRIUS_AIO_PRO_V1 6

#define AP_BARO_BMP085    1
#define AP_BARO_MS5611    2
#define AP_BARO_BMP085_PIRATES    3
#define AP_BARO_MS5611_I2C    4

/* ************************************************************** */
/* Expansion PIN's that people can use for various things. */

// AN0 - 7 are located at edge of IMU PCB "above" pressure sensor and Expansion port
// AN0 - 5 are also located next to voltage dividers and sliding SW2 switch
// AN0 - 3 has 10kOhm resistor in serial, include 3.9kOhm to make it as voltage divider
// AN4 - 5 are direct GPIO pins from atmega1280 and they are the latest pins next to SW2 switch
// Look more ArduCopter Wiki for voltage dividers and other ports
#define AN0  54  // resistor, vdiv use, divider 1 closest to relay
#define AN1  55  // resistor, vdiv use, divider 2
#define AN2  56  // resistor, vdiv use, divider 3
#define AN3  57  // resistor, vdiv use, divider 4 closest to SW2
#define AN4  58  // direct GPIO pin, default as analog input, next to SW2 switch
#define AN5  59  // direct GPIO pin, default as analog input, next to SW2 switch
#define AN6  60  // direct GPIO pin, default as analog input, close to Pressure sensor, Expansion Ports
#define AN7  61  // direct GPIO pin, default as analog input, close to Pressure sensor, Expansion Ports

// AN8 - 15 are located at edge of IMU PCB "above" pressure sensor and Expansion port
// AN8 - 15 PINs are not connected anywhere, they are located as last 8 pins on edge of the board above Expansion Ports
// even pins (8,10,12,14) are at edge of board, Odd pins (9,11,13,15) are on inner row
#define AN8  62  // NC
#define AN9  63  // NC
#define AN10  64 // NC
#define AN11  65 // NC
#define AN12  66 // NC
#define AN13  67 // NC
#define AN14  68 // NC
#define AN15  69 // NC

#define RELAY_PIN 47

#define PIEZO_PIN AN5           //Last pin on the back ADC connector

// Pirates Transmitter channel mappings
#define TX_set1	1							//Graupner/Spektrum												PITCH,YAW,THROTTLE,ROLL,AUX1,AUX2,CAMPITCH,CAMROLL
#define TX_standard	2					//standard  PPM layout Robbe/Hitec/Sanwa	ROLL,PITCH,THROTTLE,YAW,MODE,AUX2,CAMPITCH,CAMROLL
#define TX_standard_mode6	3		//standard, Mode channel is 6  						ROLL,PITCH,THROTTLE,YAW,AUX1,MODE,CAMPITCH,CAMROLL
#define TX_set2	4							//some Hitec/Sanwa/others									PITCH,ROLL,THROTTLE,YAW,AUX1,AUX2,CAMPITCH,CAMROLL
#define TX_mwi	5							//MultiWii layout													ROLL,THROTTLE,PITCH,YAW,AUX1,AUX2,CAMPITCH,CAMROLL

// Channel Config selection

#define CHANNEL_CONFIG_DEFAULT 1
#define CHANNEL_CONFIG_CUSTOM  2

#endif // _DEFINES_H
