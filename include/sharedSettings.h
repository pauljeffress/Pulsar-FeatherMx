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
// It is based heavily on myFeatherMxSettings.
// Whilst alot of it is duplicated from myFeatherMxSettings, I want a clean copy timestamped and ready to share with AGT.
// It is only ever written to by the FeatherMx, and then shared with the AGT.

typedef struct // FeatherSharedSettings
{
    uint8_t FMX_MAGICNUM;           // sequence number incremented just before sending to AGT. Wraps at 240.
    
    // The following are populated by the sensors directly connected to the FeatherMx
    uint16_t    FMX_BATT_V;         // The battery (Batt socket LiPo) voltage in V * 100
    int16_t     FMX_TEMP;           // The air temperature in degrees C * 100
    int16_t     FMX_RH;             // The relative humidity in %RH * 100
    int16_t     FMX_WATERTEMP;      // The water temperature in degrees C * 100
    int16_t     FMX_AMBIENTLIGHT;   // Ambient light reading in lux
    uint32_t    FMX_UPTIME_S;        // Uptime in seconds
    
    // PowerFeather data - received via CAN (all of this can be copied directly from PowerFeatherSettings.h)
    uint16_t    PF_BATT_V;         // The battery (Batt socket LiPo) voltage in V * 100
    int16_t     PF_TEMP;           // The air temperature in degrees C * 100
    int16_t     PF_RH;             // The relative humidity in %RH * 100
    uint32_t    PF_UPTIME_S;       // Uptime in seconds
    // CHARGER 1
    // storage for values read from charger
    uint16_t    PF_CHARGER1_PID;                // Victron Product ID (stored as 0xXXXX)
    uint16_t    PF_CHARGER1_FW;                 // Firmware version (16 bit)
    //char        PF_CHARGER1_SER[12];            // Serial number (string of 11 alpha + /0)    // Not using, too awkward and not needed.
    uint16_t    PF_CHARGER1_V;                  // Main or channel 1 battery voltage (mV)
    uint16_t    PF_CHARGER1_I;                  // Main or channel 1 battery current (mA)
    uint16_t    PF_CHARGER1_VPV;                // Panel voltage (mV)
    uint16_t    PF_CHARGER1_PPV;                // Panel power (W)
    uint8_t     PF_CHARGER1_CS;                 // State of operation (enum)
    uint8_t     PF_CHARGER1_ERR;                // Error code (enum)
    uint8_t     PF_CHARGER1_LOAD;               // Load output state (ON/OFF)
    uint16_t    PF_CHARGER1_IL;                 // Load current (mA)
    uint16_t    PF_CHARGER1_H19;                // Yield total, user resettable counter (0.01kWh)
    uint16_t    PF_CHARGER1_H20;                // Yield today (0.01kWh)
    uint16_t    PF_CHARGER1_H21;                // Maximum power today (W)
    uint16_t    PF_CHARGER1_H22;                // Yield yesterday (0.01kWh)
    uint16_t    PF_CHARGER1_H23;                // Maximum power yesterday (W)
    uint16_t    PF_CHARGER1_HSDS;               // Day sequence number (0..364)
    uint8_t     PF_CHARGER1_MPPT;               // Tracker operation mode (enum)
    // CHARGER 2
    // storage for values read from charger
    uint16_t    PF_CHARGER2_PID;                // Victron Product ID (stored as 0xXXXX)
    uint16_t    PF_CHARGER2_FW;                 // Firmware version (16 bit)
    //char        PF_CHARGER2_SER[12];            // Serial number (string of 11 alpha + /0)    // Not using, too awkward and not needed.
    uint16_t    PF_CHARGER2_V;                  // Main or channel 1 battery voltage (mV)
    uint16_t    PF_CHARGER2_I;                  // Main or channel 1 battery current (mA)
    uint16_t    PF_CHARGER2_VPV;                // Panel voltage (mV)
    uint16_t    PF_CHARGER2_PPV;                // Panel power (W)
    uint8_t     PF_CHARGER2_CS;                 // State of operation (enum)
    uint8_t     PF_CHARGER2_ERR;                // Error code (enum)
    uint8_t     PF_CHARGER2_LOAD;               // Load output state (ON/OFF)
    uint16_t    PF_CHARGER2_IL;                 // Load current (mA)
    uint16_t    PF_CHARGER2_H19;                // Yield total, user resettable counter (0.01kWh)
    uint16_t    PF_CHARGER2_H20;                // Yield today (0.01kWh)
    uint16_t    PF_CHARGER2_H21;                // Maximum power today (W)
    uint16_t    PF_CHARGER2_H22;                // Yield yesterday (0.01kWh)
    uint16_t    PF_CHARGER2_H23;                // Maximum power yesterday (W)
    uint16_t    PF_CHARGER2_HSDS;               // Day sequence number (0..364)
    uint8_t     PF_CHARGER2_MPPT;               // Tracker operation mode (enum)

    // position data from the MAVLINK_MSG_ID_GPS_RAW_INT packets the FeatherMx has received from the AutoPilot.
    uint64_t    AP_GPSTIMESTAMP;   // Fix taken time in Unix epoch time in mSec
    uint8_t     AP_FIX;            // The gps fix type as defined in the u-blox PVT message
    int32_t     AP_LAT;            // Lat  in Degrees * 10^7
    int32_t     AP_LON;            // Lon  in Degrees * 10^7
    int32_t     AP_SPEED;          // Ground speed in cm/s
    int32_t     AP_COG;            // The heading in Degrees * 100
    uint8_t     AP_SATS;           // The number of satellites (space vehicles) used in the solution
    // status data from the MAVLINK_MSG_ID_HEARTBEAT packets the FeatherMx has received from the AutoPilot.
    uint32_t    AP_CUSTOMMODE;      // A bitfield for use for autopilot-specific flags
    uint8_t     AP_SYSTEMSTATUS;     // Autopilot System status flag

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