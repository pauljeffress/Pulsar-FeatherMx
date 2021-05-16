/*
 * global.h
 * 
 * Overall header file for this project
 * 
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_HX8357.h"
#include <Fonts/FreeMonoBoldOblique12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
//#include <DFRobot_SHT3x.h>    // for my DFRobot weatherproof Temp & Humidity I2C sensor
#include "Adafruit_SHT31.h"
#include <DFRobot_B_LUX_V30B.h>
#include <OneWire.h>           // required by DallasTemperature library
#include <DallasTemperature.h> // required for DS18B20 temp sensor

#include <common/mavlink.h> // The Mavlink library

#include <TimeLib.h> // The PJRC TimeLib library to help me deal with Unix Epoch time from GPS.

#define TFT_CONNECTED // toggles code depending on whether I want to use teh TFT or not. \
                      // Will make it easier to test with/without it and ultimatetly disconnect before a voyage.
// defs for Adafruit 3.5" 480x320 TFT Featherwing - https://learn.adafruit.com/adafruit-3-5-tft-featherwing?view=all
// I have removed the pin defs for other boards.  See original example "graphicstest_featherwing.ino" for them.
// Anything else!
#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(__AVR_ATmega328P__) || \
    defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
//#define STMPE_CS 6   // Touchscreen overlay controller CS pin - not in use in this project
#define TFT_CS 14 // D14 = A0
#define TFT_DC 15 // D15 = A1
//#define SD_CS    5   // SD slot CS pin - not in use in this project
#endif
#define TFT_RST -1

// DS18B20 sensor - Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4

// Loop Steps - these are used by the switch/case in the main loop()
#define loop_init 0 // Send the welcome message, check the battery voltage

#define assess_situation 1 // Look at all available data e.g. flags, timers, last data from FeatherMx etc. \
                           // Decide if any action required, if so set additional flags and/or next state
#define zzz 2              // Turn everything off and put the processor into deep sleep
#define wake 3             // Wake from deep sleep, restore the processor clock speed

// Assess Steps - these are used by the switch/case in assess_situation()
#define check_power 1       // Check Feather's power source status.
#define read_sensors 2      // Read the various sensors attached to the Feather.
#define write_to_tft 3      // Update the tft display
#define rx_from_autopilot 4 // Read Mavlink stream from the Autopilot.
#define process_autopilot 5 // Review/action the recently received Mavlink data from the Autopilot.
#define tx_to_autopilot 6   // Send Mavlink data to the autopilot.
#define rx_from_agt 7       // Check if the AGT has sent us a datum,
#define process_agt 8       // process it if it has and set appropriate flags
#define tx_to_agt 9         // If we need to, send a datum to the AGT
#define tx_to_logger 10     // Decide and write to the Logger.
#define tickle_watchdog 11  // Tickle the watchdog so it knows we are ok.
#define sleep_yet 12        // Look at flags, should we be going to SLEEP?

#define FEATHER_VBAT_PIN A6  // Pin the battery monitor voltage divider circuit is connected to
#define FEATHER_VBAT_LOW 3.3 // When running on the Feathers battery, what do we consider low battery.

#define GOOD true
#define BAD false

/* define any enums */

// Define the struct for _all_ of the global message fields (stored in RAM)
// I'm going to use this in case I want to write all of these globals to EEPROM like the AGT does.
// It also just helps me keep clear whats global and how to check on its value from time to time.
typedef struct
{
    byte STX;           // 0x02 - when written to EEPROM, helps indicate if EEPROM contains valid data
    byte SWVER;         // Software version: bits 7-4 = major version; bits 3-0 = minor version
    // The following are populated by the sensors directly connected to the FeatherMx
    float BATTV;        // The battery (bus) voltage in V on the FeatherMx BAT socket.
    float AIRTEMP;      // The air temperature in degrees C
    float AIRHUMIDITY;  // The humidity in %RH
    float WATERTEMP;    // The water temp in deg C
    float AMBIENTLIGHT; // Ambient light reading in lux
    // the following relate to MAVLINK_MSG_ID_HEARTBEAT packets
    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t system_status; /*<  System status flag.*/
    // the following relate to MAVLINK_MSG_ID_GPS_RAW_INT packets
    uint64_t time_usec;         /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    int32_t lat;                /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
    int32_t lon;                /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
    uint16_t vel;               /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
    uint16_t cog;               /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    uint8_t fix_type;           /*<  GPS fix type.*/
    uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to 255*/
    // the following are derived from other data by the FeatherMx
    uint16_t YEAR;    // UTC year
    byte MONTH;       // UTC month
    byte DAY;         // UTC day
    byte HOUR;        // UTC hour
    byte MIN;         // UTC minute
    byte SEC;         // UTC seconds
    // will only use this if I implement sleep functionality
    uint32_t WAKEINT; // The wake-up interval in seconds
    byte ETX;         // 0x03 - when written to EEPROM, helps indicate if EEPROM contains valid data
} featherSettings;

/* extern my global vars */
extern Adafruit_HX8357 tft;
extern Adafruit_SHT31 sht31;
extern DFRobot_B_LUX_V30B myLux;
extern OneWire oneWire;
extern DallasTemperature sensors;

extern featherSettings myfeatherSettings;

extern long iterationCounter;

extern bool _printDebug;
extern Stream *_debugSerial;

extern volatile int loop_step;
extern int assess_step;
extern uint32_t assess_iterations_counter;
extern uint32_t assess_iterations_counter_last;

extern bool send_F2Ablob;
extern bool send_F2Pblob;

extern bool sensor_sht31_status;
extern bool sensor_ambientlight_status;
extern bool sensor_ds18b20_status;

/* function pre defines */
void tftSetup();
void tftdiags();
unsigned long testText();
void sensorsSetup();
void sensorsTest();
void enableDebugging(Stream &debugPort);
void disableDebugging(void);
void debugPrint(const char *message);
void debugPrintln(const char *message);
void debugPrintInt(int32_t number);
void debugPrintlnInt(int32_t number);
void debugPrintFlt(float number);
void debugPrintlnFlt(float number);
void setupPins();
void mavlink_receive();
void mavlink_request_datastream();
void case_loop_init();
void case_zzz();
void case_wake();
void case_assess_situation();
void case_check_power();
void case_read_sensors();
void case_write_to_tft();
void case_rx_from_autopilot();
void case_process_autopilot();
void case_tx_to_autopilot();
void case_rx_from_agt();
void case_process_agt();
void case_tx_to_agt();
void case_tx_to_logger();
void case_tickle_watchdog();
void case_sleep_yet();

String my64toString(uint64_t x);
#endif