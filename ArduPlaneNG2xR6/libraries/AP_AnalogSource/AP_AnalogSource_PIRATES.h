
#ifndef __AP_ANALOG_SOURCE_PIRATES_H__
#define __AP_ANALOG_SOURCE_PIRATES_H__

#include "AnalogSource.h"
#include <inttypes.h>

class AP_AnalogSource_PIRATES : public AP_AnalogSource
{
    public:
    AP_AnalogSource_PIRATES() {
    	first_call = 1;
    }
    float read(void);
    void init(void);
    
    private:
    int first_call;
};

#endif // __AP_ANALOG_SOURCE_PIRATES_H__
