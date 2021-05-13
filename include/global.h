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
#include <OneWire.h>             // required by DallasTemperature library
#include <DallasTemperature.h>   // required for DS18B20 temp sensor

#include <common/mavlink.h>


// defs for Adafruit 3.5" 480x320 TFT Featherwing - https://learn.adafruit.com/adafruit-3-5-tft-featherwing?view=all
// I have removed the pin defs for other boards.  See original example "graphicstest_featherwing.ino" for them.
// Anything else!
#if defined (__AVR_ATmega32U4__) || defined(ARDUINO_SAMD_FEATHER_M0) || defined (__AVR_ATmega328P__) || \
defined(ARDUINO_SAMD_ZERO) || defined(__SAMD51__) || defined(__SAM3X8E__) || defined(ARDUINO_NRF52840_FEATHER)
   //#define STMPE_CS 6   // Touchscreen overlay controller CS pin - not in use in this project
   #define TFT_CS   14  // D14 = A0 
   #define TFT_DC   15  // D15 = A1
   //#define SD_CS    5   // SD slot CS pin - not in use in this project
#endif
#define TFT_RST -1

// DS18B20 sensor - Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 4


/* define any enums */


/* extern my global vars */
extern Adafruit_HX8357 tft;
extern Adafruit_SHT31 sht31;
extern DFRobot_B_LUX_V30B myLux;  
extern OneWire oneWire;
extern DallasTemperature sensors;

/* function pre defines */
void tftSetup();
void tftdiags();
unsigned long testText();
void sensorsSetup();
void sensorsTest();

#endif