/*
 * FeatherSettings.h
 * 
 * header file for my FeatherSettings data structures
 * that are used to store the state of the Feather itself
 * in both RAM and EEPROM.
 * 
 */

#ifndef FEATHERSETTINGS_H
#define FEATHERSETTINGS_H

#include "Arduino.h"    // helps with the "types" used here.

// Define the default value for each message field
#define DEF_STX           0x02  // See my AGT code for reasoning
#define DEF_SWVER         0x10  // Software version 1.0
#define DEF_BATTV         555   // 555 * 0.01V = 5.55V  This default is obvious and easy to identify.
#define DEF_PRESS         99    // used if we can't get a reading from the Feathers SHT31 sensor.
#define DEF_AIRTEMP       99    // used if we can't get a reading from the Feathers SHT31 sensor.
#define DEF_HUMID         1     // used if we can't get a reading from the Feathers SHT31 sensor.
#define DEF_WATERTEMP     99    // used if we can't get a reading from the Feathers DS18B20 sensor.
#define DEF_AMBIENTLIGHT  1     // used if we can't get a reading from the Feathers Light sensor.

#define DEF_GPSYEAR      1980
#define DEF_GPSMONTH     1
#define DEF_GPSDAY       1
#define DEF_GPSHOUR      1
#define DEF_GPSMIN       1
#define DEF_GPSSEC       1
#define DEF_GPSMILLIS    1

#define DEF_CUSTOMMODE    99    // unlikely and easy to spot
#define DEF_SYSTEMSTATUS  99    // unlikely and easy to spot

#define DEF_GPSTIMESTAMP 0
#define DEF_LAT       -891234599    // LAT (-90 to +90) min for LAT is -90deg so I picked just below that -89.1234599 as its easy to spot.
#define DEF_LON       1791234599    // LON (-180 to 180) max for LON is 180deg so I picked just below that 179.1234599 as its easy to spot.
#define DEF_ALT       99    // unlikely and easy to spot
#define DEF_SPEED     99    // unlikely and easy to spot
#define DEF_HEAD      99    // unlikely and easy to spot
#define DEF_SATS      99    // unlikely and easy to spot
#define DEF_PDOP      99    // unlikely and easy to spot
#define DEF_FIX       99    // possible but easy to spot


#define DEF_WAKEINT        60 // Seconds
#define DEF_TXAGTINT       15 // Seconds
#define DEF_RXAPINT        90 // Seconds
#define DEF_LOWBATT        330 // 330 * 0.01V = 3.3V
#define DEF_ETX       0x03


// Define the struct for _all_ of the global message fields (stored in RAM)
// I'm going to use this in case I want to write all of these globals to EEPROM like the AGT does.
// It also just helps me keep clear whats global and how to check on its value from time to time.
typedef struct
{
    byte STX;   // 0x02 - when written to EEPROM, helps indicate if EEPROM contains valid data
    byte SWVER; // Software version: bits 7-4 = major version; bits 3-0 = minor version
    // The following are populated by the sensors directly connected to the FeatherMx
    uint16_t BATTV;        // The battery (Batt socket LiPo) voltage in V * 10^-2
    uint16_t PRESS;        // The pressure in mbar
    int16_t AIRTEMP;          // The air temperature in degrees C * 10^-2
    int16_t HUMID;        // The humidity in %RH * 10^-2
    int16_t WATERTEMP;          // The water temperature in degrees C * 10^-2
    int16_t AMBIENTLIGHT; // Ambient light reading in lux
    
    // the following are derived from other data by the FeatherMx
    uint16_t GPSYEAR; // UTC year
    uint8_t  GPSMONTH;    // UTC month
    uint8_t  GPSDAY;      // UTC day
    uint8_t  GPSHOUR;     // UTC hour
    uint8_t  GPSMIN;      // UTC minute
    uint8_t  GPSSEC;      // UTC seconds
    uint16_t GPSMILLIS;       // UTC milliseconds
    
    // the following relate to MAVLINK_MSG_ID_HEARTBEAT packets
    uint32_t CUSTOMMODE;  /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t SYSTEMSTATUS; /*<  System status flag.*/
    
    // the following relate to MAVLINK_MSG_ID_GPS_RAW_INT packets
    uint64_t GPSTIMESTAMP;         /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    int32_t LAT;           // Latitude in Degrees * 10^-7
    int32_t LON;           // Latitude in Degrees * 10^-7
    int32_t ALT;           // Altitude above MSL in mm
    int32_t SPEED;         // Ground speed in mm/s
    int32_t HEAD;          // The heading in Degrees * 10^-7
    byte    SATS;          // The number of satellites (space vehicles) used in the solution
    uint16_t PDOP;         // The Positional Dilution of Precision in cm
    byte    FIX;           // The gps fix type as defined in the u-blox PVT message

    // will only use this if I implement sleep functionality
    uint32_t WAKEINT;   // Seconds - The wake-up interval in seconds
    uint32_t TXAGTINT;  // Seconds - The interval between periodic settings share to AGT.
    uint32_t RXAPINT;   // Seconds - The interval between periodic RX from AutoPilot.
    uint16_t LOWBATT;   // The low battery limit in V * 10^-2
    byte ETX;           // 0x03 - when written to EEPROM, helps indicate if EEPROM contains valid data
} FeatherSettings;


/* function pre defines */
bool sendSharedSettings_to_AGT(void);
void initFeatherSettings(void);

#endif