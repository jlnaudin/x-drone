/// Config for the Multiplex Easyglider - Tested in flight on July 11, 2012 by Jean-Louis Naudin with the AIOP v1.0

#define E_GLIDER	    ENABLED

#define MSL_REF	       DISABLED

#define AOA	  	    0		// AOA in degree
#define CH7_OPTION	    CH7_SAVE_WP

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// FLIGHT AND NAVIGATION CONTROL
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// AIRSPEED_CRUISE
//
#define AIRSPEED_CRUISE		12 // 12 m/s

//////////////////////////////////////////////////////////////////////////////
// MIN_GNDSPEED
//
#define MIN_GNDSPEED			10 // m/s (0 disables)

//////////////////////////////////////////////////////////////////////////////
// FLY_BY_WIRE_B airspeed control
//
#define AIRSPEED_FBW_MIN		6
#define AIRSPEED_FBW_MAX		35

//////////////////////////////////////////////////////////////////////////////
// Servo Mapping
//
#define THROTTLE_MIN		0 // percent
#define THROTTLE_CRUISE		35
#define THROTTLE_MAX		60

//////////////////////////////////////////////////////////////////////////////
// Autopilot control limits
//
#define HEAD_MAX				35
#define PITCH_MAX				15
#define PITCH_MIN				-20

//////////////////////////////////////////////////////////////////////////////
// Attitude control gains
//
#define SERVO_ROLL_P         0.6
#define SERVO_ROLL_I         0.0
#define SERVO_ROLL_D         0.0
#define SERVO_ROLL_INT_MAX   5
#define ROLL_SLEW_LIMIT      0
#define SERVO_PITCH_P        0.6
#define SERVO_PITCH_I        0.0
#define SERVO_PITCH_D        0.0
#define SERVO_PITCH_INT_MAX  5
#define PITCH_COMP           0.18
#define SERVO_YAW_P          0.0
#define SERVO_YAW_I          0.0
#define SERVO_YAW_D          0.0
#define SERVO_YAW_INT_MAX    0.05
#define RUDDER_MIX           0.8

//////////////////////////////////////////////////////////////////////////////
// Navigation control gains
//
#define NAV_ROLL_P           0.7
#define NAV_ROLL_I           0.1
#define NAV_ROLL_D           0.02
#define NAV_ROLL_INT_MAX     5
#define NAV_PITCH_ASP_P      0.80
#define NAV_PITCH_ASP_I      0.0
#define NAV_PITCH_ASP_D      0.0
#define NAV_PITCH_ASP_INT_MAX 5
#define NAV_PITCH_ALT_P      0.75
#define NAV_PITCH_ALT_I      0.0
#define NAV_PITCH_ALT_D      0.0
#define NAV_PITCH_ALT_INT_MAX 5

#define CLOSED_LOOP_NAV     ENABLED    // set to ENABLED if closed loop navigation else set to DISABLED (Return To Lauch)
#define AUTO_WP_RADIUS      ENABLED

//////////////////////////////////////////////////////////////////////////////
// Energy/Altitude control gains
//
#define THROTTLE_TE_P        0.50
#define THROTTLE_TE_I        0.0
#define THROTTLE_TE_D        0.0
#define THROTTLE_TE_INT_MAX  20
#define THROTTLE_SLEW_LIMIT  0
#define P_TO_T               1.0
#define T_TO_P               0
#define PITCH_TARGET         0

//////////////////////////////////////////////////////////////////////////////
// Crosstrack compensation
//
#define XTRACK_GAIN          1 // deg/m
#define XTRACK_ENTRY_ANGLE   10 // deg

//////////////////////////////////////////////////////////////////////////////
// Navigation defaults
//
#define WP_RADIUS_DEFAULT	20

#define LOITER_RADIUS_DEFAULT   40

#define ALT_HOLD_HOME 150

