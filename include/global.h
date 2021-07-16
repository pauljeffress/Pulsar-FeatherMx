/*
 * global.h
 * 
 * Overall header file for this project
 * 
 */

#ifndef GLOBAL_H
#define GLOBAL_H

#include <Arduino.h>
#include "wiring_private.h"    // for "pinPeripheral()". Only needed in PlatformIO, not required in Arduino IDE???
#include <Wire.h>
#include <SPI.h>
#include "Adafruit_GFX.h"                       // For TFT
#include "Adafruit_HX8357.h"                    // For TFT
#include <Fonts/FreeMonoBoldOblique12pt7b.h>    // For TFT
#include <Fonts/FreeSans18pt7b.h>               // For TFT
#include "Adafruit_SHT31.h"         // for my DFRobot weatherproof Temp & Humidity I2C sensor (I'm not using the DFRobot lib)
#include <DFRobot_B_LUX_V30B.h>     // DFRobot Visible Light sensor
#include <OneWire.h>                // required by DallasTemperature library
#include <DallasTemperature.h>      // required for DS18B20 temp sensor

#include <common/mavlink.h> // The Mavlink library

#include <TimeLib.h> // The PJRC TimeLib library to help me deal with Unix Epoch time from GPS.

#include <SerialTransfer.h> // Note: This library will complain about SPDR in the SPITransfer.cpp/h files at compile time.
                            //       Its a known problem, search my Evernotes. The solution as I am not using SPI under
                            //       SerialTransfer is to rename two of the source files in the SerilTransfer library so
                            //       they are not compiled.  The two files are;
                            //       * SPITransfer.h and SPITransfer.cpp
                            //       they live in .pio/libdeps/..../SerialTransfer/src/


// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
// xxxx #include "SAMDTimerInterrupt.h"             // https://github.com/khoih-prog/SAMD_TimerInterrupt
#define TIMER0_INTERVAL_MS        1000



#include "FeatherSettings.h" // header file for my FeatherSettings data structure
#include "SharedSettings.h" // header file for my SharedSettings data structures

#define TFT_CONNECTED // toggles code depending on whether I want to use teh TFT or not. 
                      // Will make it easier to test with/without it and ultimatetly disconnect before a voyage.
// defs for Adafruit 3.5" 480x320 TFT Featherwing - https://learn.adafruit.com/adafruit-3-5-tft-featherwing?view=all
// I have removed the pin defs for other boards.  See original example "graphicstest_featherwing.ino" for them.
// Anything else!
#if defined(__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(__AVR_ATmega328P__) || \
    defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
//#define STMPE_CS 6   // Touchscreen overlay controller CS pin - not in use in this project
#define TFT_CS 9 // Default TFT_CS = D9 (I was using D14/A0 previously)
#define TFT_DC 10 // Default TFT_DC = D10 (I was using D15/A1 previously)
//#define SD_CS    5   // SD slot CS pin - not in use in this project
#endif
#define TFT_RST -1

// DS18B20 sensor - Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4

// Loop Steps - these are used by the switch/case in the main loop()
#define loop_init 0 // Send the welcome message, check the battery voltage

#define assess_situation 1 // Look at all available data e.g. flags, timers, last data from FeatherMx etc. 
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

// xxx - most of the below can be removed once I get new HW serial/I2C connection to AGT.
// define pins for the additional HW serial port I have to configure.
#define SERIAL_TO_AGT_TX_PIN    18  
#define SERIAL_TO_AGT_RX_PIN    19 
// define GPIO pins for the extra Serial control wires that go between this Feather and AGT.
#define FEATHER_READY_TO_RX_PIN 9  // I was using 13 and having issues, I think its because thats the LED pin on the Feather.
#define AGT_WANTS_TO_TX_PIN 12
#define FEATHER_WAIT_FOR_AGT_TO_DROP 10 // seconds - amount of time to wait for AGT to drop AGT_WANTS_TO_TX_PIN.
#define SENDPERIODSECONDS 15  // seconds - how often to send the dummy packets



/* define any enums */


/* define any struct's */

typedef struct // my datum to tx/rx over SerialTransfer
{
    uint8_t i1;
    char c1;
    char c2;
    char c3;
    char c4;
} datum;

/* extern my global vars */
extern Adafruit_HX8357 tft;
extern Adafruit_SHT31 sht31;
extern DFRobot_B_LUX_V30B myLux;
extern OneWire oneWire;
extern DallasTemperature sensors;
extern Uart Serial2;
extern Uart Serial3;

extern SerialTransfer STdriverF2A;

// xxx extern SAMDTimer ITimer0(TIMER_TC3);

extern FeatherSettings myFeatherSettings;
extern FeatherSharedSettings myFeatherSharedSettings;
extern AgtSharedSettings myAgtSharedSettings;

extern volatile uint32_t mytimercounter;
extern uint32_t mytimercounter_last;

extern unsigned long oneSecCounter;
extern unsigned long oneSecCounter_last;

extern long iterationCounter;

extern bool _printDebug;
extern Stream *_debugSerial;

extern bool _printLog;
extern Stream *_logSerial;

extern volatile int loop_step;
extern int assess_step;
extern uint32_t assess_iterations_counter;
extern uint32_t assess_iterations_counter_last;

extern bool flag_do_agt_tx;

extern bool sensor_sht31_status;
extern bool sensor_ambientlight_status;
extern bool sensor_ds18b20_status;

extern datum STDatumTX, STDatumRX;
extern uint32_t lastsend;

extern volatile unsigned long seconds_since_reset_or_powercycle;
extern volatile unsigned long seconds_since_last_wake;
extern volatile unsigned long seconds_since_last_ap_tx;
extern volatile unsigned long seconds_since_last_ap_rx;
extern volatile unsigned long seconds_since_last_agt_tx;
extern volatile unsigned long seconds_since_last_agt_rx;
extern volatile unsigned long seconds_since_last_sensors_read;


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

void enableLogging(Stream &debugPort);
void disableLogging(void);
void logPrint(const char *message);
void logPrintln(const char *message);
void logPrintInt(int32_t number);
void logPrintlnInt(int32_t number);
void logPrintFlt(float number);
void logPrintlnFlt(float number);

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
void setFEATHER_READY_TO_RX_PINhigh();
void setFEATHER_READY_TO_RX_PINlow();
void serialSetup();
void timerSetup();
void timerIncrementer();
bool sendSharedSettings_to_AGT(void);
void initFeatherSharedSettings(void);
void preptosendFeatherSharedSettings(void);
int16_t int32_to_int16_a(int32_t input_int32);
uint16_t int32_to_int16_b(int32_t input_int32);
uint16_t int32_to_int16_c(int32_t input_int32);


String my64toString(uint64_t x);
#endif