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

// Below mavlink includes draw from the c_library_v1 liubrary I added to this PlatformIO project.
//#include <common/mavlink.h> // The Mavlink library for the "common" dialect  // Must remove this line now that I have the
                                                                               // ardupilotmega dialect below, as it pulls 
                                                                               // in common itself.
#include <ardupilotmega/mavlink.h> // The Mavlink library for the "ardupilotmega" dialect

#include <TimeLib.h> // The PJRC TimeLib library to help me deal with Unix Epoch time from GPS.

#include <SerialTransfer.h> // Note: This library will complain about SPDR in the SPITransfer.cpp/h files at compile time.
                            //       Its a known problem, search my Evernotes. The solution as I am not using SPI under
                            //       SerialTransfer is to rename two of the source files in the SerilTransfer library so
                            //       they are not compiled.  The two files are;
                            //       * SPITransfer.h and SPITransfer.cpp
                            //       they live in .pio/libdeps/..../SerialTransfer/src/

#include "SharedSettings.h" // header file for my SharedSettings data structures

#include <PulsarCommon.h>

#define HOST_IS_FEATHERMX    // used to select what CAN/CBP packets to decode

// These define's must be placed at the beginning before #include "SAMDTimerInterrupt.h"
// _TIMERINTERRUPT_LOGLEVEL_ from 0 to 4
// Don't define _TIMERINTERRUPT_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
// Don't define TIMER_INTERRUPT_DEBUG > 2. Only for special ISR debugging only. Can hang the system.
#define TIMER_INTERRUPT_DEBUG         0
#define _TIMERINTERRUPT_LOGLEVEL_     0
// xxxx #include "SAMDTimerInterrupt.h"             // https://github.com/khoih-prog/SAMD_TimerInterrupt
#define TIMER0_INTERVAL_MS        1000



// #define TFT_CONNECTED // toggles code depending on whether I want to use teh TFT or not. 
//                       // Will make it easier to test with/without it and ultimatetly disconnect before a voyage.
// // defs for Adafruit 3.5" 480x320 TFT Featherwing - https://learn.adafruit.com/adafruit-3-5-tft-featherwing?view=all
// // I have removed the pin defs for other boards.  See original example "graphicstest_featherwing.ino" for them.
// // Anything else!
// #if defined(__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined(__AVR_ATmega328P__) || \
//     defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
// //#define STMPE_CS 6   // Touchscreen overlay controller CS pin - not in use in this project
// #define TFT_CS 9 // Default TFT_CS = D9 (I was using D14/A0 previously)
// #define TFT_DC 10 // Default TFT_DC = D10 (I was using D15/A1 previously)
// //#define SD_CS    5   // SD slot CS pin - not in use in this project
// #endif
// #define TFT_RST -1

// DS18B20 sensor - Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4

// 12v switched power outputs
#define PI_PWR_PIN              23 // The pin the FMX controls the power supply to the Pi on.
#define STROBE_LIGHT_PWR_PIN    24 // The pin the FMX controls the power supply to the Strobe light on.
#define POWER_FEATHER_PWR_PIN   25 // The pin the FMX controls the power supply to the Power Feather on.

// Loop Steps - these are used by the switch/case in the main loop()
#define loop_init           0   // Send the welcome message, check the battery voltage
#define assess_situation    1   // Look at all available data e.g. flags, timers, last data from FeatherMx etc. 
                                    // Decide if any action required, if so set additional flags and/or next state
#define zzz                 2   // Turn everything off and put the processor into deep sleep
#define wake                3   // Wake from deep sleep, restore the processor clock speed

// Assess Steps - these are used by the switch/case in assess_situation()
#define check_power             1   // Check Feather's power source status.
#define read_sensors            2   // Read the various sensors attached to the Feather.
#define tx_to_CANbus            3   // send data out onto CANbus
#define check_CANbus            4   // check if any packets received via CAN bus.
#define write_to_tft            5   // Update the tft display
#define heartbeat_to_autopilot  6   // Send a MAVLink HEARTBEAT to the AutoPilot.
#define rx_from_autopilot       7   // Read Mavlink stream from the Autopilot.
#define process_autopilot       8   // Review/action the recently received Mavlink data from the Autopilot.
#define tx_to_autopilot         9   // Send Mavlink data to the autopilot.
#define rx_from_agt             10   // Check if the AGT has sent us a datum,
#define process_agt             11  // process it if it has and set appropriate flags
#define tx_to_agt               12  // If we need to, send a datum to the AGT

#define tx_to_logger            20  // Decide and write to the Logger.
#define tickle_watchdog         21  // Tickle the watchdog so it knows we are ok.
#define sleep_yet               22  // Look at flags, should we be going to SLEEP?

// Various Timers
#define SENSORPERIODSECONDS             120 // seconds - how often should we read the sensors?
#define MAVLINKHEARTBEATPERIODSECONDS   20  // seconds - how often should we send a MAVLink HEARTBEAT to the AutoPilot
#define TX_TO_AP_PERIOD_SECONDS         120  // seconds - how often should we do periodic TX to AutoPilot

// MAVLink stuff
// MAVLink IDs - https://ardupilot.org/dev/docs/mavlink-basics.html
#define FMX_SYS_ID  1       // MAVLink System ID of this device. For example a companion computer (Arduino, RasPi etc) which is sending a heartbeat.
                            // I am setting this to 1, as all electronics on the drone itself should have the same SystemID as
                            // the AutoPilot, and it has sysID = 1.
#define FMX_COMP_ID  100    // MAVLink Component ID of this device. For example a companion computer (Arduino, RasPi etc) which is sending a heartbeat.
                            // I am setting this to 100, as it can be from 1 to 255 but the AutoPilot has compID = 1 so can't use that.
#define AP_SYS_ID  1        // MAVLink System ID of the autopilot.
#define AP_COMP_ID  100     // MAVLink Component ID of the autopilot.


/* define any enums */


/* define any struct's */


/* extern my global vars */
extern Adafruit_HX8357 tft;
extern Adafruit_SHT31 sht31;
extern DFRobot_B_LUX_V30B myLux;
extern OneWire oneWire;
extern DallasTemperature sensors;
extern Uart Serial2;
extern Uart Serial3;
extern SerialTransfer STdriverF2A;
extern bool sensor_sht31_status;
extern bool sensor_ambientlight_status;
extern bool sensor_ds18b20_status;

// xxx extern SAMDTimer ITimer0(TIMER_TC3);

extern FeatherSharedSettings myFeatherSharedSettings;
extern AgtSharedSettings myAgtSharedSettings;

extern bool flag_do_agt_tx;

extern bool _printLog;
extern Stream *_logSerial;



/* function pre defines */
void actuatorsSetup();
void sensorsSetup();
void sensorsTest();

void setupPins();

void serialSetup();

void mavlink_fmx_send_heartbeat_to_ap();
void mavlink_receive();
void mavlink_request_datastream();
void mavlink_unrequest_datastream();
void mavlink_request_streaming_params_from_ap();
void mavlink_unrequest_streaming_params_from_ap();

void request_one_param_from_ap();
void set_one_param_from_ap(); 
void set_arm_ap();
void set_disarm_ap();

void case_loop_init();
void case_zzz();
void case_wake();
void case_assess_situation();
void case_read_sensors();
void case_tx_to_CANbus();
void case_write_to_tft();
void case_heartbeat_to_autopilot();
void case_rx_from_autopilot();
void case_process_autopilot();
void case_tx_to_autopilot();
void case_rx_from_agt();
void case_process_agt();
void case_tx_to_agt();
void case_tx_to_logger();
void case_tickle_watchdog();
void case_sleep_yet();

bool sendSharedSettings_to_AGT(void);
void initFeatherSharedSettings(void);
void preptosendFeatherSharedSettings(void);

void enableLogging(Stream &logPort);
void disableLogging(void);
void logPrintStamp(void);
void logPrint(const char *message);
void logPrintln(const char *message);
void logPrintInt(int32_t number);
void logPrintlnInt(int32_t number);
void logPrintFlt(float number);
void logPrintlnFlt(float number);

int16_t int32_to_int16_a(int32_t input_int32);
uint16_t int32_to_int16_b(int32_t input_int32);
uint16_t int32_to_int16_c(int32_t input_int32);

String my64toString(uint64_t x);

void actuatorStrobeOn();
void actuatorStrobeOff();
void actuatorPiOn();
void actuatorPiOff();
void actuatorPowerFeatherOn();
void actuatorPowerFeatherOff();


#endif