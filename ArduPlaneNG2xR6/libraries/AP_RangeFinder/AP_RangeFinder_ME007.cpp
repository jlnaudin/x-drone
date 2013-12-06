// -*- tab-width: 4; Mode: C++; c-basic-offset: 3; indent-tabs-mode: t -*-

// AVR LibC Includes
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WConstants.h"
#endif

#include "RangeFinder.h"
#include "AP_RangeFinder_ME007.h"

// Constructor //////////////////////////////////////////////////////////////

AP_RangeFinder_ME007::AP_RangeFinder_ME007(AP_AnalogSource *source,
                                                     FilterInt16 *filter) :
	RangeFinder(source, filter)
{
	max_distance = AP_RANGEFINDER_ME007_MAX_DISTANCE;
	min_distance = AP_RANGEFINDER_ME007_MIN_DISTANCE;
}

void AP_RangeFinder_ME007::init()
{
	((AP_AnalogSource_PIRATES*)_analog_source)->init();
}

// Read Sensor data - only the raw_value is filled in by this parent class
int AP_RangeFinder_ME007::read()
{
	raw_value = _analog_source->read();

	// convert analog value to distance in cm (using child implementation most likely)
	distance = raw_value;

	return distance;
}

// Public Methods //////////////////////////////////////////////////////////////
