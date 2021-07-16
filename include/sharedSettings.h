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
// It is based heavily on myFeatherSettings.
// Whilst alot of it is duplicated from myFeatherSettings, I want a clean copy timestamped and ready to share with AGT.
// It is only ever written to by the Feather, and then shared with the AGT.

typedef struct // FeatherSharedSettings
{
    // The following are populated by the sensors directly connected to the FeatherMx
    uint16_t BATTV;        // The battery (Batt socket LiPo) voltage in V * 10^-2
    uint16_t PRESS;        // The pressure in mbar
    int16_t AIRTEMP;          // The air temperature in degrees C * 10^-2
    int16_t HUMID;        // The humidity in %RH * 10^-2
    int16_t WATERTEMP;          // The water temperature in degrees C * 10^-2
    int16_t AMBIENTLIGHT; // Ambient light reading in lux
    
    // the following are derived from other data by the FeatherMx
    uint16_t YEAR; // UTC year
    uint16_t MONTH;    // UTC month
    uint16_t DAY;      // UTC day
    uint16_t HOUR;     // UTC hour
    uint16_t MIN;      // UTC minute
    uint16_t SEC;      // UTC seconds
    uint16_t MILLIS;       // UTC milliseconds

    // // the following relate to MAVLINK_MSG_ID_GPS_RAW_INT packets the Feather has received from the AutoPilot.
    uint64_t GPSTIMESTAMP;         /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    int32_t LAT;           // Lat  in Degrees * 10^-7
    int32_t LON;           // Lon  in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    uint16_t SATS;          // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    uint16_t FIX;                   // The gps fix type as defined in the u-blox PVT message
} FeatherSharedSettings;
//----------------------------------------------------


//----------------------------------------------------
// Define the struct for all of my AGT originated settings, that I want to be able to share with the FEATHER.
// It is based heavily on mytrackerSettings.
// Whilst alot of it is duplicated from mytrackerSettings, I want a clean copy timestamped and ready to share with FEATHER.
// It is only ever written to by the AGT, and then shared with the FEATHER.

typedef struct  // AgtSharedSettings
{
    // The following are populated by the sensors directly connected to the AGT
    uint16_t BATTV;         // The battery (bus) voltage in V * 10^-2
    uint16_t PRESS;         // The pressure in mbar
    int16_t  TEMP;          // The temperature in degrees C * 10^-2
    uint16_t HUMID;         // The humidity in %RH * 10^-2

    // the following is populated by the AGT, likely via time derived from its GPS.
    uint16_t YEAR; // UTC year
    byte MONTH;    // UTC month
    byte DAY;      // UTC day
    byte HOUR;     // UTC hour
    byte MIN;      // UTC minute
    byte SEC;      // UTC seconds
    uint16_t MILLIS;         // UTC milliseconds

    // the following are populated by the AGT's onboard GPS
    int32_t LAT;           // Latitude in Degrees * 10^-7
    int32_t LON;           // Latitude in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    byte SATS;                  // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    byte FIX;                   // The gps fix type as defined in the u-blox PVT message
} AgtSharedSettings;
//----------------------------------------------------

#endif