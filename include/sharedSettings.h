/*
 * SharedSettings.h
 * 
 * header file for my sharedSettings data structures
 * that are used to communicate data over serial link between the FeatherMx and AGT.
 * 
 */

#ifndef SHAREDSETTINGS_H
#define SHAREDSETTINGS_H

#include "Arduino.h"    // helps with the "types" used here.




// Define the struct for all of my Feather originated settings, that I want to be able to share with the AGT.
// It is based heavily on myFeatherSettings.
// Whilst alot of it is duplicated from myFeatherSettings, I want a clean copy timestamped and ready to share with AGT.
// It is only ever written to by the Feather, and then shared with the AGT.
typedef struct
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
    byte MONTH;    // UTC month
    byte DAY;      // UTC day
    byte HOUR;     // UTC hour
    byte MIN;      // UTC minute
    byte SEC;      // UTC seconds
    uint16_t MILLIS;       // UTC milliseconds

    // the following relate to MAVLINK_MSG_ID_GPS_RAW_INT packets
    uint64_t GPSTIMESTAMP;         /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    int32_t LAT;           // Latitude in Degrees * 10^-7
    int32_t LON;           // Latitude in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    byte    SATS;          // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    byte    FIX;                   // The gps fix type as defined in the u-blox PVT message

} FeatherSharedSettings;

// Define the struct for all of my AGT originated settings, that I want to be able to share with the FEATHER.
// It is based heavily on mytrackerSettings.
// Whilst alot of it is duplicated from mytrackerSettings, I want a clean copy timestamped and ready to share with FEATHER.
// It is only ever written to by the AGT, and then shared with the FEATHER.
typedef struct
{
    // The following are populated by the sensors directly connected to the AGT
    uint16_t BATTV;         // The battery (bus) voltage in V * 10^-2
    uint16_t PRESS;         // The pressure in mbar
    int16_t  TEMP;          // The temperature in degrees C * 10^-2
    uint16_t HUMID;         // The humidity in %RH * 10^-2
    // the following are populated by the AGT's onboard GPS
    int32_t LAT;           // Latitude in Degrees * 10^-7
    int32_t LON;           // Latitude in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    byte SATS;                  // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    byte FIX;                   // The gps fix type as defined in the u-blox PVT message
    // the following are populated by the AGT and may be updated from Land Systems over ISBD.
    byte USERVAL1;        // User value 1
    byte USERVAL2;        // User value 2
    uint16_t USERVAL3;     // User value 3
    uint16_t USERVAL4;     // User value 4
    uint32_t USERVAL5;     // User value 5
    uint32_t USERVAL6;     // User value 6
    float USERVAL7;       // User value 7
    float USERVAL8;       // User value 8
    // the following are populated by the AGT and may be updated from Land Systems over ISBD.
    uint16_t ALARMINT;     // The AGT alarm ISBD transmit interval in minutes
    uint16_t TXINT;        // The AGT message ISBD transmit interval in minutes
    // the following timestamp is populated by the AGT and marks the last time any parameter in this struct was writen to.
    uint16_t TIMESTAMP_YEAR; // UTC year
    byte TIMESTAMP_MONTH;    // UTC month
    byte TIMESTAMP_DAY;      // UTC day
    byte TIMESTAMP_HOUR;     // UTC hour
    byte TIMESTAMP_MIN;      // UTC minute
    byte TIMESTAMP_SEC;      // UTC seconds
    uint16_t MILLIS;       // UTC milliseconds
} AgtSharedSettings;


/* function pre defines */
bool sendSharedSettings_to_AGT(void);
void initFeatherSharedSettings(void);

#endif