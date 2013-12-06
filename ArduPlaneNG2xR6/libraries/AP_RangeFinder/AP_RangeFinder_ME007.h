#ifndef AP_RangeFinder_ME007_H
#define AP_RangeFinder_ME007_H

#include "RangeFinder.h"
#include "../AP_AnalogSource/AP_AnalogSource_PIRATES.h"

#define AP_RANGEFINDER_ME007_MIN_DISTANCE 2
#define AP_RANGEFINDER_ME007_MAX_DISTANCE 400

class AP_RangeFinder_ME007 : public RangeFinder
{
 // public:
  public:
	AP_RangeFinder_ME007(AP_AnalogSource *source, FilterInt16 *filter);
	void init();

	int read();
};
#endif
