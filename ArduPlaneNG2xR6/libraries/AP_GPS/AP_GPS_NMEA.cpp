// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// NMEA parser, adapted by Michael Smith + Greg Nash from TinyGPS v9:
//
// TinyGPS - a small GPS library for Arduino providing basic NMEA parsing
// Copyright (C) 2008-9 Mikal Hart
// All rights reserved.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//

/// @file	AP_GPS_NMEA.cpp
/// @brief	NMEA protocol parser
///
/// This is a lightweight NMEA parser, derived originally from the
/// TinyGPS parser by Mikal Hart.
///

#include <FastSerial.h>
#include <AP_Common.h>

#include <avr/pgmspace.h>
#include <ctype.h>
#include <stdint.h>

#include "AP_GPS_NMEA.h"

// SiRF init messages //////////////////////////////////////////////////////////
//
// Note that we will only see a SiRF in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any SiRF unit to binary mode as part of
// the autodetection process.
//
const prog_char AP_GPS_NMEA::_SiRF_init_string[] PROGMEM =
    "$PSRF103,0,0,1,1*25\r\n"	// GGA @ 1Hz
    "$PSRF103,1,0,0,1*25\r\n"	// GLL off
    "$PSRF103,2,0,0,1*26\r\n"	// GSA off
    "$PSRF103,3,0,0,1*27\r\n"	// GSV off
    "$PSRF103,4,0,1,1*20\r\n"	// RMC off
    "$PSRF103,5,0,1,1*20\r\n"	// VTG @ 1Hz
    "$PSRF103,6,0,0,1*22\r\n"	// MSS off
    "$PSRF103,8,0,0,1*2C\r\n"	// ZDA off
    "$PSRF151,1*3F\r\n"			// WAAS on (not always supported)
    "$PSRF106,21*0F\r\n"		// datum = WGS84
    "";

// MediaTek init messages //////////////////////////////////////////////////////
//
// Note that we may see a MediaTek in NMEA mode if we are connected to a non-DIYDrones
// MediaTek-based GPS.
//
const prog_char AP_GPS_NMEA::_MTK_init_string[] PROGMEM =
    "$PMTK314,0,0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"	// GGA & VTG once every fix
    "$PMTK330,0*2E\r\n"										// datum = WGS84
    "$PMTK313,1*2E\r\n"									// SBAS on
    "$PMTK301,2*2E\r\n"									// use SBAS data for DGPS
    "";

// ublox init messages /////////////////////////////////////////////////////////
//
// Note that we will only see a ublox in NMEA mode if we are explicitly configured
// for NMEA.  GPS_AUTO will try to set any ublox unit to binary mode as part of
// the autodetection process.
//
// We don't attempt to send $PUBX,41 as the unit must already be talking NMEA
// and we don't know the baudrate.
//
const prog_char AP_GPS_NMEA::_ublox_init_string[] PROGMEM =
    "$PUBX,40,gga,0,1,0,0,0,0*7B\r\n"	// GGA on at one per fix
    "$PUBX,40,vtg,0,1,0,0,0,0*7F\r\n"	// VTG on at one per fix
    "$PUBX,40,rmc,0,0,0,0,0,0*67\r\n"	// RMC off (XXX suppress other message types?)
    "";

// NMEA message identifiers ////////////////////////////////////////////////////
//
const char AP_GPS_NMEA::_gprmc_string[] PROGMEM = "RMC";
const char AP_GPS_NMEA::_gpgga_string[] PROGMEM = "GGA";
const char AP_GPS_NMEA::_gpvtg_string[] PROGMEM = "VTG";

// Convenience macros //////////////////////////////////////////////////////////
//
#define DIGIT_TO_VAL(_x)	(_x - '0')

// Constructors ////////////////////////////////////////////////////////////////
AP_GPS_NMEA::AP_GPS_NMEA(Stream *s) :
    GPS(s)
{
    FastSerial	*fs = (FastSerial *)_port;
    // Re-open the port with enough receive buffering for the messages we expect
    // and very little tx buffering, since we don't care about sending.
    // Leave the port speed alone as we don't actually know at what rate we're running...
    //
    fs->begin(0, 200, 16);
}

// Public Methods //////////////////////////////////////////////////////////////
void AP_GPS_NMEA::init(enum GPS_Engine_Setting nav_setting)
{
    BetterStream	*bs = (BetterStream *)_port;

    // send the SiRF init strings
    bs->print_P((const prog_char_t *)_SiRF_init_string);

    // send the MediaTek init strings
    bs->print_P((const prog_char_t *)_MTK_init_string);

    // send the ublox init strings
    bs->print_P((const prog_char_t *)_ublox_init_string);

    idleTimeout = 1200;
}

bool AP_GPS_NMEA::read(void)
{
    int numc;
    bool parsed = false;

    numc = _port->available();
    while (numc--) {
        if (_decode(_port->read())) {
            parsed = true;
        }
    }
    return parsed;
}

bool AP_GPS_NMEA::_decode(char c)
{
    bool valid_sentence = false;

    switch (c) {
    case ',': // term terminators
        _parity ^= c;
    case '\r':
    case '\n':
    case '*':
        if (_term_offset < sizeof(_term)) {
            _term[_term_offset] = 0;
            valid_sentence = _term_complete();
        }
        ++_term_number;
        _term_offset = 0;
        _is_checksum_term = c == '*';
        return valid_sentence;

    case '$': // sentence begin
        _term_number = _term_offset = 0;
        _parity = 0;
        _sentence_type = _GPS_SENTENCE_OTHER;
        _is_checksum_term = false;
        _gps_data_good = false;
        return valid_sentence;  // is false
    }

    // ordinary characters
    if (_term_offset < sizeof(_term) - 1)
        _term[_term_offset++] = c;
    if (!_is_checksum_term)
        _parity ^= c;
    
    return valid_sentence;  // is false
}

//
// internal utilities
//
int AP_GPS_NMEA::_from_hex(char a)
{
    if (a >= 'A' && a <= 'F')
        return a - 'A' + 10;
    else if (a >= 'a' && a <= 'f')
        return a - 'a' + 10;
    else
        return a - '0';
}

uint32_t AP_GPS_NMEA::_parse_decimal(char *p, uint8_t numdec)
{
	unsigned long ret = 0;
	byte ndec = 0;

	while (ndec==0 || ndec <= numdec) {
		if (*p == '.'){
			ndec = 1;
		}
		else {
			if (!isdigit(*p)) {
                // we hit the end of the number; ensure we have correct exponent
				while (ndec++ <= numdec) {
					ret *= 10;
				}
				return ret;
			}
			ret *= 10;
			ret += *p - '0';
			if (ndec > 0)
				ndec++;
		}
		p++;
	}
    return ret;
}

uint32_t AP_GPS_NMEA::_parse_degrees()
{
    char *p, *q;
    uint8_t deg = 0;
    uint32_t frac_min;
    
    // scan for decimal point or end of field
    for (p = _term; isdigit(*p); p++)
        ;
    q = _term;

    // convert degrees, leaving 2 digits for whole minutes
    while ((p - q) > 2) {
        if (deg)
            deg *= 10;
        deg += DIGIT_TO_VAL(*q++);
        //deg = (deg << 1) + (deg << 3) + DIGIT_TO_VAL(*q++);   //faster, more obscure
    }
    
    frac_min = _parse_decimal(q, 6);    // parse minutes * 10e6
    //return deg * 10000000UL + (frac_min / 6); // raise to 10e7, with minutes x 10 / 60
    
    // Alternate, faster math
    // To divide by 6 without using division routine,
    // halve, then multiply by 0x555555/0x1000000 (1/3 in hex)
    
    // Knowing frac_min < 59999999, we can multiply by 0x55 within a long, then repeat the fractional part
    // 0x55/0x100 is hex equivalent to, but not exactly equal to, decimal 0.33
    frac_min = ((frac_min >> 1) * 0x55) / 0x100;
    // Now repeat the digits. Equivalent to multiplying decimal 0.33 by 1.0101 to get 0.333333
    // (These divisions will be optimised into moves by the compiler)
    return (frac_min) + (frac_min / 0x100) + (frac_min / 0x10000) + (deg * 10000000UL);
}

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool AP_GPS_NMEA::_term_complete()
{
    // handle the last term in a message
    if (_is_checksum_term) {
        uint8_t checksum = 16 * _from_hex(_term[0]) + _from_hex(_term[1]);
        if (checksum == _parity) {
            if (_gps_data_good) {
                switch (_sentence_type) {
                case _GPS_SENTENCE_GPRMC:
                    time			= _new_time;
                    date			= _new_date;
                    latitude		= _new_latitude;	// degrees*10e7
                    longitude		= _new_longitude;	// degrees*10e7
                    ground_speed	= _new_speed;
                    ground_course	= _new_course;
                    fix				= true;
                    break;
                case _GPS_SENTENCE_GPGGA:
                    altitude		= _new_altitude;
                    time			= _new_time;
                    latitude		= _new_latitude;	// degrees*10e7
                    longitude		= _new_longitude;	// degrees*10e7
                    num_sats		= _new_satellite_count;
                    hdop			= _new_hdop;
                    fix				= true;
                    break;
                case _GPS_SENTENCE_GPVTG:
                    ground_speed	= _new_speed;
                    ground_course	= _new_course;
                    // VTG has no fix indicator, can't change fix status
                    break;
                }
            } else {    // Parity ok, but not _gps_data_good
                switch (_sentence_type) {
                case _GPS_SENTENCE_GPRMC:
                case _GPS_SENTENCE_GPGGA:
                    // Only these sentences must give us information about
                    // fix status.  (optional in GPVTG?)
                    fix = false;
                }
            }
            // we got a good message & checksum match
            return true;
        }
        // we got a checksum fail, ignore the message
        return false;
    }   // if _is_checksum_term

    // the first term contains talker ID (2 chars) plus sentence type (3 chars)
    // (or, P indicates custom message followed by 3 char company ID and 1 char custom message ID)
    if (_term_number == 0 && _term[0] != 'P') {
        if (!strcmp_P(_term+2, _gprmc_string)) {
            _sentence_type = _GPS_SENTENCE_GPRMC;
        } else if (!strcmp_P(_term+2, _gpgga_string)) {
            _sentence_type = _GPS_SENTENCE_GPGGA;
        } else if (!strcmp_P(_term+2, _gpvtg_string)) {
            _sentence_type = _GPS_SENTENCE_GPVTG;
            // VTG may not contain a data qualifier, presume the solution is good
            // unless it tells us otherwise.
            _gps_data_good = true;
        } else {
            _sentence_type = _GPS_SENTENCE_OTHER;
        }
        return false;
    }
    
    if (_sentence_type != _GPS_SENTENCE_OTHER && _term[0]) {
        switch (_sentence_type + _term_number) {
            // operational status
            //
        case _GPS_SENTENCE_GPRMC + 2: // validity (RMC) Autonomous | Differential | None | Estimated | Manual-input
            _gps_data_good = _term[0] == 'A' || _term[0] == 'D';
            break;
        case _GPS_SENTENCE_GPGGA + 6: // Fix data (GGA)
            _gps_data_good = _term[0] > '0';
                   // 0 = invalid, 1=GPS fix (SPS), 2=DGPS fix, 3=PPS fix, 4=RTK, 5=Float RTK, 6=estimated (dead reckoning), 7=Manual input, 8=Simulated
            break;
        case _GPS_SENTENCE_GPVTG + 9: // validity (VTG) (we may not see this field)
            _gps_data_good = _term[0] != 'N';
            break;
        case _GPS_SENTENCE_GPGGA + 7: // satellite count (GGA)
            _new_satellite_count = atol(_term);
            break;
        case _GPS_SENTENCE_GPGGA + 8: // HDOP (GGA)
            _new_hdop = _parse_decimal(_term, 2);
            break;

            // time and date
            //
        case _GPS_SENTENCE_GPRMC + 1: // Time (RMC)
        case _GPS_SENTENCE_GPGGA + 1: // Time (GGA)
            _new_time = _parse_decimal(_term, 2);
            break;
        case _GPS_SENTENCE_GPRMC + 9: // Date (GPRMC)
            _new_date = atol(_term);
            break;

            // location
            //
        case _GPS_SENTENCE_GPRMC + 3: // Latitude
        case _GPS_SENTENCE_GPGGA + 2:
            _new_latitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_GPRMC + 4: // N/S
        case _GPS_SENTENCE_GPGGA + 3:
            if (_term[0] == 'S')
                _new_latitude = -_new_latitude;
            break;
        case _GPS_SENTENCE_GPRMC + 5: // Longitude
        case _GPS_SENTENCE_GPGGA + 4:
            _new_longitude = _parse_degrees();
            break;
        case _GPS_SENTENCE_GPRMC + 6: // E/W
        case _GPS_SENTENCE_GPGGA + 5:
            if (_term[0] == 'W')
                _new_longitude = -_new_longitude;
            break;
        case _GPS_SENTENCE_GPGGA + 9: // Altitude (GPGGA)
            _new_altitude = _parse_decimal(_term, 2);
            break;

            // course and speed
            //
        case _GPS_SENTENCE_GPRMC + 7: // Speed (GPRMC)
        case _GPS_SENTENCE_GPVTG + 5: // Speed (VTG)
            _new_speed = (_parse_decimal(_term, 2) * 514) / 1000; 	// knots-> m/sec, approximiates * 0.514
            break;
        case _GPS_SENTENCE_GPRMC + 8: // Course (GPRMC)
        case _GPS_SENTENCE_GPVTG + 1: // Course (VTG)
            _new_course = _parse_decimal(_term, 2);
            break;
        
        }   // switch type + term number
    }   // if supported sentence type

    return false;   // Added a parm, but message not yet complete
}
