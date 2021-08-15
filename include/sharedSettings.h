/*
 * SharedSettings.h
 * 
 * header file for my sharedSettings data structures
 * that are used to communicate data over serial link between the FeatherMx and AGT.
 * 
 * Should be identical on FeatherMx and on AGT. As they need to know the structure of each others messages.
 */

#ifndef SHAREDSETTINGS_H
#define SHAREDSETTINGS_H

#include "Arduino.h"    // helps with the "types" used here.

//----------------------------------------------------
// Define the struct for all of my Feather originated settings, that I want to be able to share with the AGT.
// It is based heavily on myFeatherMxSettings.FMX_
// Whilst alot of it is duplicated from myFeatherMxSettings, I want a clean copy timestamped and ready to share with AGT.
// It is only ever written to by the Feather, and then shared with the AGT.

typedef struct // FeatherSharedSettings
{
    uint8_t MAGICNUM;    // sequence number incremented just before sending to AGT. Wraps at 240.
    
    // The following are populated by the sensors directly connected to the FeatherMx
    uint16_t BATTV;        // The battery (Batt socket LiPo) voltage in V * 10^-2
    uint16_t PRESS;        // The pressure in mbar
    int16_t TEMP;       // The air temperature in degrees C * 10^-2
    int16_t HUMID;         // The humidity in %RH * 10^-2
    int16_t WATERTEMP;     // The water temperature in degrees C * 10^-2
    int16_t AMBIENTLIGHT;  // Ambient light reading in lux
    
    // the time provided by the GPS when the GPS position was taken.
    uint16_t GPSYEAR;     // UTC year
    uint8_t  GPSMONTH;    // UTC month
    uint8_t  GPSDAY;      // UTC day
    uint8_t  GPSHOUR;     // UTC hour
    uint8_t  GPSMIN;      // UTC minute
    uint8_t  GPSSEC;      // UTC seconds
    uint16_t GPSMILLIS;   // UTC milliseconds

    // // the following relate to MAVLINK_MSG_ID_GPS_RAW_INT packets the Feather has received from the AutoPilot.
    int32_t LAT;           // Lat  in Degrees * 10^-7
    int32_t LON;           // Lon  in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    uint8_t SATS;          // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    uint8_t FIX;           // The gps fix type as defined in the u-blox PVT message
} FeatherSharedSettings;
//----------------------------------------------------


//----------------------------------------------------
// Define the struct for all of my AGT originated settings, that I want to be able to share with the FEATHER.
// It is based heavily on mytrackerSettings.
// Whilst alot of it is duplicated from mytrackerSettings, I want a clean copy timestamped and ready to share with FEATHER.
// It is only ever written to by the AGT, and then shared with the FEATHER.

typedef struct  // AgtSharedSettings
{
    uint8_t MAGICNUM;    // sequence number incremented just before sending to Feather. Wraps at 240.
    
    // The following are populated by the sensors directly connected to the AGT
    uint16_t BATTV;         // The battery (bus) voltage in V * 10^-2
    uint16_t PRESS;         // The pressure in mbar
    int16_t  TEMP;          // The temperature in degrees C * 10^-2
    uint16_t HUMID;         // The humidity in %RH * 10^-2

    // the time provided by the GPS when the GPS position was taken.
    uint16_t GPSYEAR;     // UTC year
    uint8_t  GPSMONTH;    // UTC month
    uint8_t  GPSDAY;      // UTC day
    uint8_t  GPSHOUR;     // UTC hour
    uint8_t  GPSMIN;      // UTC minute
    uint8_t  GPSSEC;      // UTC seconds
    uint16_t GPSMILLIS;      // UTC milliseconds

    // the following are populated by the AGT's onboard GPS
    int32_t LAT;           // Latitude in Degrees * 10^-7
    int32_t LON;           // Latitude in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    uint8_t SATS;          // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    uint8_t FIX;           // The gps fix type as defined in the u-blox PVT message
} AgtSharedSettings;
//----------------------------------------------------

#endif